#ifndef ST_STEPEXTRACTCURVESFROMGRID3D_H
#define ST_STEPEXTRACTCURVESFROMGRID3D_H

#include "ct_itemdrawable/ct_circle.h"
#include "ct_step/abstract/ct_abstractstep.h"

class ST_StepExtractCurvesFromGrid3D : public CT_AbstractStep
{
    Q_OBJECT
    using SuperClass = CT_AbstractStep;

    using Vec3d = Eigen::Vector3d;
    using Vec4d = Eigen::Vector4d;
    using Vec4i = Eigen::Vector4i;

    using Circle                    = CT_Circle;
    using CirclePtr                 = Circle*;

    //using Snake     = ST_OpenActiveContours<HoughSpaceValueType>;
    //using SnakePtr  = Snake*;
public:
    ST_StepExtractCurvesFromGrid3D();
};

#endif // ST_STEPEXTRACTCURVESFROMGRID3D_H
