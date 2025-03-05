#ifndef STL_STEP_FILTER_3DGRID_H
#define STL_STEP_FILTER_3DGRID_H

#include "stl_3dgrid.h"
#include "ct_step/abstract/ct_abstractstep.h"

class STL_Step_Filter_3DGrid: public CT_AbstractStep
{
    Q_OBJECT
    using SuperClass = CT_AbstractStep;

    using Vec3d = Eigen::Vector3d;
    using Vec3f = Eigen::Vector3f;

public:
    STL_Step_Filter_3DGrid();
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

    CT_HandleInSingularItem<STL_3DGrid<int>>     _in_3dgrid;
    CT_HandleOutSingularItem<STL_3DGrid<int>>    _out_3dgrid;

    double                                  _ratio_thresh;
};

#endif // STL_STEP_FILTER_3DGRID_H
