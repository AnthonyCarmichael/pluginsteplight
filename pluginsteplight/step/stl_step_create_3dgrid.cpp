#include "stl_step_create_3d_grid.h"
#include "ct_global/ct_context.h"
#include "ct_itemdrawable/tools/gridtools/ct_grid3dwootraversalalgorithm.h"
#include "loginterface.h"
#include "stl_3dgrid.h"
#include "stl_grid3dbeamvisitor.h"
#include <omp.h>

STL_STEP_Create_3D_Grid::STL_STEP_Create_3D_Grid(): SuperClass()
{
    _grid_resolution = 0.2f;
}

QString STL_STEP_Create_3D_Grid::description() const
{
    return tr("STL: 1 - Créer une grille 3D à partir d'un nuage de points");
}

// Step detailled description
QString STL_STEP_Create_3D_Grid::getStepDetailledDescription() const
{
    return tr("Créer et remplir une grille 3D.");
}

CT_VirtualAbstractStep* STL_STEP_Create_3D_Grid::createNewInstance() const
{
    return new STL_STEP_Create_3D_Grid();
}

//////////////////// PROTECTED METHODS //////////////////

void STL_STEP_Create_3D_Grid::declareInputModels(CT_StepInModelStructureManager& manager)
{
    manager.addResult(_inResult, tr("Scène(s)"));
    manager.setZeroOrMoreRootGroup(_inResult, _inZeroOrMoreRootGroup);
    manager.addGroup(_inZeroOrMoreRootGroup, _inGroup);
    manager.addItem(_inGroup, _in_point_cloud, tr("Point cloud"));
    manager.addItem(_inGroup, _in_normal_cloud, tr("Normal cloud"));
}

void STL_STEP_Create_3D_Grid::declareOutputModels(CT_StepOutModelStructureManager& manager)
{
    manager.addResultCopy(_inResult);
    manager.addItem(_inGroup, _outSTLGrid3D, tr("Computed STL 3D Grid"));
}

void STL_STEP_Create_3D_Grid::fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog)
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

void STL_STEP_Create_3D_Grid::compute()
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

        STL_3DGrid<int>* grid_3d = STL_3DGrid<int>::createGrid3DFromXYZCoords(bbox_bot[0],bbox_bot[1],bbox_bot[2],
                                                                            bbox_top[0],bbox_top[1],bbox_top[2],
                                                                            _grid_resolution,
                                                                            std::numeric_limits<int>::max(),
                                                                            0);
        STL_Grid3DBeamVisitor*  visitor =  new STL_Grid3DBeamVisitor(grid_3d);
        QList<CT_AbstractGrid3DBeamVisitor*> visitorArr;
        visitorArr.push_back(visitor);

        CT_Grid3DWooTraversalAlgorithm woo(grid_3d,true,visitorArr);

        // -----------------------------------------------------------------------------------------------------------------
        // Loop through all points and normals of the input point cloud and start raytracing inside Hough space
        size_t i_point = 0;
        size_t n_points = inPointCloud->pointCloudIndex()->size();
        CT_PointIterator itPoint(inPointCloud->pointCloudIndex());

        //int* ptrVect = grid_3d->get_data();
        //int i;
        //#pragma omp parallel for reduction(+:ptrVect[0:n_points])
        for(  CT_PointIterator itPoint(inPointCloud->pointCloudIndex());itPoint.hasNext();i_point++ )
        {
            if( i_point % 100 == 0 )
            {
                setProgress( static_cast<float>(i_point) * 100.0f / static_cast<float>(n_points) );
            }

            if( isStopped() )
            {
                return ;
            }


            const CT_Point&  currentPoint       = itPoint.next().currentPoint();
            const CT_Normal& currentCTNormal    = inNormalCloud->constNormalAt(i_point);
            Vec3d      currentNormal      = currentCTNormal.head(3).cast<double>();

            float normalLenght = currentNormal.norm();

            if( normalLenght != 0.0 )
            {
                currentNormal /= normalLenght;
                CT_Beam beam_01( currentPoint, currentNormal );
                CT_Beam beam_02( currentPoint, -currentNormal );

                woo.compute(beam_01);
                woo.compute(beam_02);
            }
        }

        delete visitor;
        grid_3d->computeMinMax();



        PS_LOG->addInfoMessage(LogInterface::error, tr("Min value %1").arg(grid_3d->dataMin()));
        PS_LOG->addInfoMessage(LogInterface::error, tr("Max value %1").arg(grid_3d->dataMax()));

        // -----------------------------------------------------------------------------------------------------------------
        // Add computed Hough space to the step's output(s)
        group->addSingularItem(_outSTLGrid3D, grid_3d);
    }

    setProgress(100);
}
