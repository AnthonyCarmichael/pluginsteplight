#ifndef STL_STEPFILTERGRID3DBYVALUE_H
#define STL_STEPFILTERGRID3DBYVALUE_H

#include "ct_step/abstract/ct_abstractstep.h"
#include "stl_grid3d.h"
class STL_StepFilterGrid3DByValue : public CT_AbstractStep
{
public:
    STL_StepFilterGrid3DByValue();
    QString description() const override;
    QString getStepDetailledDescription() const;
    CT_VirtualAbstractStep* createNewInstance() const override;

protected:
    void declareInputModels(CT_StepInModelStructureManager& manager) final;
    void declareOutputModels(CT_StepOutModelStructureManager& manager) final;
    void fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog) final;
    void compute() final;

protected:
    CT_HandleInResultGroupCopy<>                _inResult;
    CT_HandleInStdZeroOrMoreGroup               _inZeroOrMoreRootGroup;
    CT_HandleInStdGroup<>                       _inGroup;
    CT_HandleInSingularItem<STL_Grid3D<int>>    _in_grid3d;
    CT_HandleOutSingularItem<STL_Grid3D<int>>   _out_grid3d;
    int                                         _thresh;

};

#endif // STL_STEPFILTERGRID3DBYVALUE_H
