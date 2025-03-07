#ifndef STL_STEPCREATEGRID3D_H
#define STL_STEPCREATEGRID3D_H

#include "ct_itemdrawable/ct_grid3d.h"
#include "ct_itemdrawable/tools/gridtools/ct_grid3dwootraversalalgorithm.h"
#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "stl_grid3d.h"

class STL_STEPCreateGrid3D : public CT_AbstractStep
{
    Q_OBJECT
    using SuperClass = CT_AbstractStep;

public:
    STL_STEPCreateGrid3D();
    QString description() const override;
    QString getStepDetailledDescription() const;
    CT_VirtualAbstractStep* createNewInstance() const override;

protected:
    void declareInputModels(CT_StepInModelStructureManager& manager) final;
    void declareOutputModels(CT_StepOutModelStructureManager& manager) final;
    void fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog) final;
    void compute() final;

private:
    void multithreadCompute(size_t pointsPerThread,const size_t threadNum, const CT_AbstractItemDrawableWithPointCloud* inPointCloud,const CT_PointsAttributesNormal* inNormalCloud, CT_Grid3DWooTraversalAlgorithm& woo);


protected:
    CT_HandleInResultGroupCopy<>                                    _inResult;
    CT_HandleInStdZeroOrMoreGroup                                   _inZeroOrMoreRootGroup;
    CT_HandleInStdGroup<>                                           _inGroup;
    CT_HandleInSingularItem<CT_AbstractItemDrawableWithPointCloud>  _in_point_cloud;
    CT_HandleInSingularItem<CT_PointsAttributesNormal>              _in_normal_cloud;
    CT_HandleOutSingularItem<STL_Grid3D<int>>                        _outSTLGrid3D;

    double _grid_resolution;
};

#endif // STL_STEPCREATEGRID3D_H
