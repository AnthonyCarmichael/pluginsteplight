#include "stl_stepcreategrid3d.h"
#include "ct_global/ct_context.h"
#include "loginterface.h"
#include "stl_grid3d.h"
#include "stl_grid3dbeamvisitor.h"
#include "stl_grid3dwootraversalalgorithm.h"
#include <omp.h>
#include <thread>
#include <future>

STL_STEPCreateGrid3D::STL_STEPCreateGrid3D(): SuperClass()
{
    _grid_resolution = 0.2f;
}

QString STL_STEPCreateGrid3D::description() const
{
    return tr("STL: 1 - Créer une grille 3D à partir d'un nuage de points");
}

// Step detailled description
QString STL_STEPCreateGrid3D::getStepDetailledDescription() const
{
    return tr("Créer et remplir une grille 3D.");
}

CT_VirtualAbstractStep* STL_STEPCreateGrid3D::createNewInstance() const
{
    return new STL_STEPCreateGrid3D();
}

//////////////////// PROTECTED METHODS //////////////////

void STL_STEPCreateGrid3D::declareInputModels(CT_StepInModelStructureManager& manager)
{
    manager.addResult(_inResult, tr("Scène(s)"));
    manager.setZeroOrMoreRootGroup(_inResult, _inZeroOrMoreRootGroup);
    manager.addGroup(_inZeroOrMoreRootGroup, _inGroup);
    manager.addItem(_inGroup, _in_point_cloud, tr("Point cloud"));
    manager.addItem(_inGroup, _in_normal_cloud, tr("Normal cloud"));
}

void STL_STEPCreateGrid3D::declareOutputModels(CT_StepOutModelStructureManager& manager)
{
    manager.addResultCopy(_inResult);
    manager.addItem(_inGroup, _outSTLGrid3D, tr("Computed STL 3D Grid"));
    manager.addItem(_inGroup, _outSTLGridRayLength, tr("Ray length for each cells"));
}

void STL_STEPCreateGrid3D::fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog)
{
    postInputConfigDialog->addDouble(tr("Spatial resolution"),
                                     tr("[m]"),
                                     0.0001,
                                     2.0,
                                     4,
                                     _grid_resolution,
                                     1.0,
                                     tr("3d Grid: spatial resolution")
                                     );

}

void STL_STEPCreateGrid3D::compute()
{
    using Vec3d                 = Eigen::Vector3d;
    using PointCloudConst       = const CT_AbstractItemDrawableWithPointCloud;
    using PointCloudConstPtr    = PointCloudConst*;
    using NormalCloudConst      = const CT_PointsAttributesNormal;
    using NormalCloudConstPtr   = NormalCloudConst*;

    setProgress(0);

    for (CT_StandardItemGroup* group : _inGroup.iterateOutputs(_inResult))
    {
        if( isStopped() )
        {
            return;
        }

        // -----------------------------------------------------------------------------------------------------------------
        // Get point cloud and normal cloud from computree previous step
        PointCloudConstPtr  inPointCloud = group->singularItem(_in_point_cloud);
        NormalCloudConstPtr inNormalCloud = group->singularItem(_in_normal_cloud);

        // -----------------------------------------------------------------------------------------------------------------
        // Initialize empty Hough space according to the bounding box of the input point cloud
        Vec3d bbox_bot;
        Vec3d bbox_top;

        inPointCloud->boundingBox(bbox_bot, bbox_top);

        // -----------------------------------------------------------------------------------------------------------------
        // Loop through all points and normals of the input point cloud and start raytracing inside Hough space

        size_t n_points = inPointCloud->pointCloudIndex()->size();
        CT_PointIterator itPoint(inPointCloud->pointCloudIndex());

        // Multithreading
        std::atomic<size_t> i_point = 0;
        const unsigned int numThreads = std::thread::hardware_concurrency();
        const size_t pointsPerThread = n_points / numThreads;

        // Vecteur de résultat d'opération asynch. Permets de récupérer les grilles générées par chacun des threads.
        struct Result {
            STL_Grid3D<int>* _grid_3d;
            STL_Grid3D<float>* _grid_ray_length;
        };

        std::vector<std::future<Result>> futures;

        for (unsigned int i = 0; i < numThreads; ++i) {
            futures.push_back(std::async(std::launch::async, [this,&i_point,n_points, pointsPerThread, i, inPointCloud, inNormalCloud, bbox_bot, bbox_top]() mutable -> Result {
                STL_Grid3D<int>* grid_3d = STL_Grid3D<int>::createGrid3DFromXYZCoords(bbox_bot[0],bbox_bot[1],bbox_bot[2],
                                                                                      bbox_top[0],bbox_top[1],bbox_top[2],
                                                                                      _grid_resolution,
                                                                                      std::numeric_limits<int>::max(),
                                                                                      0);
                STL_Grid3D<float>* grid_ray_length = STL_Grid3D<float>::createGrid3DFromXYZCoords(bbox_bot[0],bbox_bot[1],bbox_bot[2],
                                                                                                bbox_top[0],bbox_top[1],bbox_top[2],
                                                                                                _grid_resolution,
                                                                                                std::numeric_limits<float>::max(),
                                                                                                0);

                STL_Grid3DBeamVisitor*  visitor =  new STL_Grid3DBeamVisitor(grid_3d);
                QList<CT_AbstractGrid3DBeamVisitor*> visitorArr;
                visitorArr.push_back(visitor);
                STL_Grid3DWooTraversalAlgorithm woo(grid_3d,true,visitorArr);

                size_t  beginIndex = i * pointsPerThread;
                CT_PointIterator itPoint(inPointCloud->pointCloudIndex());
                itPoint.jump(beginIndex);
                for(size_t i = beginIndex; i < (beginIndex + pointsPerThread); i++)
                {
                    // Gestion de la bar de progress multithread safe
                    if( i_point.fetch_add(1) % 100 == 0 )
                    {
                        setProgress(static_cast<float>(i_point) * 100.0f / static_cast<float>(n_points));
                    }

                    // Trouver une façon d'arrêter les threads et d'arrêter le compute
                    // if (isStopped())
                    // {
                    //     return;
                    // }

                    itPoint.next();
                    CT_Point currentPoint = itPoint.currentPoint();
                    const CT_Normal& currentCTNormal    = inNormalCloud->constNormalAt(i);
                    Eigen::Vector3d  currentNormal      = currentCTNormal.head(3).cast<double>();

                    float normalLenght = currentNormal.norm();

                    if( normalLenght != 0.0 )
                    {
                        currentNormal /= normalLenght;

                        CT_Beam beam_01(currentPoint, currentNormal);
                        CT_Beam beam_02(currentPoint, -currentNormal);

                        Eigen::Vector3d* endPoint1 = new Eigen::Vector3d(currentPoint + currentNormal * 1.5);
                        Eigen::Vector3d* endPoint2 = new Eigen::Vector3d(currentPoint - currentNormal * 1.5);

                        woo.compute(beam_01, endPoint1,grid_ray_length);
                        woo.compute(beam_02, endPoint2,grid_ray_length);

                        delete endPoint1;
                        delete endPoint2;
                    }
                }

                delete visitor;

                return Result{grid_3d,grid_ray_length};
            }));
        }


        STL_Grid3D<int>* grid_3d = nullptr;
        STL_Grid3D<float>* grid_ray = nullptr;
        for (auto &t : futures) {
            Result result = t.get();
            STL_Grid3D<int>* grid = result._grid_3d;

            if (grid_3d) {
                *grid_3d += *grid;
                delete grid;
            } else {
                grid_3d = grid;
            }

            // On refait la même chose pour les grilles de rayons
            STL_Grid3D<float>* g_ray = result._grid_ray_length;

            if (grid_ray) {
                *grid_ray += *g_ray;
                delete g_ray;
            } else {
                grid_ray = g_ray;
            }
        }

        grid_3d->setPointCloudPtr(inPointCloud,inNormalCloud);
        grid_3d->setBotTop(bbox_bot,bbox_top);
        grid_3d->setGridRayLength(grid_ray);
        grid_3d->setRealRayValueDivadedByVisit();

        grid_3d->computeMinMax();
        grid_3d->getGridRayLength()->computeMinMax();

        PS_LOG->addInfoMessage(LogInterface::error, tr("Min value %1").arg(grid_3d->dataMin()));
        PS_LOG->addInfoMessage(LogInterface::error, tr("Max value %1").arg(grid_3d->dataMax()));


        PS_LOG->addInfoMessage(LogInterface::error, tr("Rayon min value %1").arg(grid_3d->getGridRayLength()->dataMin()));
        PS_LOG->addInfoMessage(LogInterface::error, tr("Rayon max value %1").arg(grid_3d->getGridRayLength()->dataMax()));

        // -----------------------------------------------------------------------------------------------------------------
        // Add computed Hough space to the step's output(s)
        group->addSingularItem(_outSTLGrid3D, grid_3d);
        group->addSingularItem(_outSTLGridRayLength, grid_3d->getGridRayLength());
    }

    setProgress(100);
}
