#ifndef STL_STEP_CREATE_3D_GRID_H
#define STL_STEP_CREATE_3D_GRID_H

#include "ct_itemdrawable/ct_grid3d.h"
#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "stl_3dgrid.h"

class STL_STEP_Create_3D_Grid : public CT_AbstractStep
{
    Q_OBJECT
    using SuperClass = CT_AbstractStep;

public:
    STL_STEP_Create_3D_Grid();
    QString description() const override;
    QString getStepDetailledDescription() const;
    CT_VirtualAbstractStep* createNewInstance() const override;

protected:
    void declareInputModels(CT_StepInModelStructureManager& manager) final;
    void declareOutputModels(CT_StepOutModelStructureManager& manager) final;
    void fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog) final;
    void compute() final;

protected:
    CT_HandleInResultGroupCopy<>                                    _inResult;
    CT_HandleInStdZeroOrMoreGroup                                   _inZeroOrMoreRootGroup;
    CT_HandleInStdGroup<>                                           _inGroup;
    CT_HandleInSingularItem<CT_AbstractItemDrawableWithPointCloud>  _in_point_cloud;
    CT_HandleInSingularItem<CT_PointsAttributesNormal>              _in_normal_cloud;
    CT_HandleOutSingularItem<STL_3DGrid<int>>                        _outSTLGrid3D;

    template< class DataT >
    friend STL_3DGrid<DataT> operator+(STL_3DGrid<DataT>& leftGrid,const STL_3DGrid<DataT>& rightGrid );



    double _grid_resolution;


};

#endif // STL_STEP_Create_3D_Grid_H
