#ifndef STL_STEPFILTERBYRATIO_H
#define STL_STEPFILTERBYRATIO_H

#include "stl_grid3d.h"
#include "ct_step/abstract/ct_abstractstep.h"

class STL_StepFilterByRatio : public CT_AbstractStep
{
    Q_OBJECT
    using SuperClass = CT_AbstractStep;

    using Grid3DValueType         = int;
    using Grid3D                  = STL_Grid3D<Grid3DValueType>;
    using Grid3DPtr               = Grid3D*;
    using Grid3DPtrConst          = Grid3DPtr const;
    using ConstGrid3D             = const Grid3D;
    using ConstGrid3DPtr          = ConstGrid3D*;
    using ConstGrid3DPtrConst     = ConstGrid3DPtr const;
    using Vec3d = Eigen::Vector3d;
    using Vec3f = Eigen::Vector3f;

public:
    STL_StepFilterByRatio();
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
    CT_HandleInSingularItem<Grid3D>         _in_grid_3d;
    CT_HandleOutSingularItem<Grid3D>        _out_grid_3d;

    double                                  _ratio_thresh;
};

#endif // STL_STEPFILTERBYRATIO_H




