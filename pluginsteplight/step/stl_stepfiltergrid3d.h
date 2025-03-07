#ifndef STL_STEPFILTERGRID3D_H
#define STL_STEPFILTERGRID3D_H

#include "stl_grid3d.h"
#include "ct_step/abstract/ct_abstractstep.h"

class STL_Step_Filter_Grid3D: public CT_AbstractStep
{
    Q_OBJECT
    using SuperClass = CT_AbstractStep;

    using Vec3d = Eigen::Vector3d;
    using Vec3f = Eigen::Vector3f;

public:
    STL_Step_Filter_Grid3D();
    QString description() const override;
    QString getStepDetailledDescription() const;
    CT_VirtualAbstractStep* createNewInstance() const override;

protected:
    void declareInputModels(CT_StepInModelStructureManager& manager) final;
    void declareOutputModels(CT_StepOutModelStructureManager& manager) final;
    void fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog) final;
    void compute() final;

protected:
    CT_HandleInResultGroupCopy<>            _inResult;
    CT_HandleInStdZeroOrMoreGroup           _inZeroOrMoreRootGroup;
    CT_HandleInStdGroup<>                   _inGroup;

    CT_HandleInSingularItem<STL_Grid3D<int>>     _in_grid3d;
    CT_HandleOutSingularItem<STL_Grid3D<int>>    _out_grid3d;

    double                                  _ratio_thresh;
};

#endif // STL_STEPFILTERGRID3D_H
