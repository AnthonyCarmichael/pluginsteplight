#ifndef STL_STEPEXTRACTCURVESFROMGRID3D_H
#define STL_STEPEXTRACTCURVESFROMGRID3D_H

#include "stl_grid3d.h"
#include "stl_openactivecontours.h"

#include "ct_itemdrawable/ct_circle.h"
#include "ct_step/abstract/ct_abstractstep.h"

class STL_StepExtractCurvesFromGrid3D : public CT_AbstractStep
{
    Q_OBJECT
    using RepulseImage              = STL_Grid3D<int>;
    using RepulseImagePtr           = RepulseImage*;
    using RepulseImagePtrConst      = RepulseImagePtr const;
    using Circle                    = CT_Circle;
    using CirclePtr                 = Circle*;

public:
    STL_StepExtractCurvesFromGrid3D();
    QString description() const override;
    QString getStepDetailledDescription() const;
    CT_VirtualAbstractStep* createNewInstance() const override;

protected:
    void declareInputModels(CT_StepInModelStructureManager& manager) final;
    void declareOutputModels(CT_StepOutModelStructureManager& manager) final;
    void fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog) final;
    void compute() final;

protected:
    CT_HandleInResultGroupCopy<>        _inResult;
    CT_HandleInStdZeroOrMoreGroup       _inZeroOrMoreRootGroup;
    CT_HandleInStdGroup<>               _inGroup;
    CT_HandleInSingularItem<STL_Grid3D<int>> _grid3D;

    CT_HandleOutStdGroup                _outGroupOfSnakes;
    CT_HandleOutStdGroup                _outGroupSingleSnake;
    CT_HandleOutStdGroup                _outGroupSingleCircle;
    CT_HandleOutSingularItem<Circle>    _outCircle;

private:
    // Step parameters
    int       _nIterMaxOptim;                       /*!<  */
    double    _treeHeightMaximum;                   /*!<  */
    double    _growCoeff;                           /*!<  */
    double    _timeStep;                            /*!<  */
    double    _alpha;                               /*!   */
    double    _beta;                                /*!   */
    double    _gama;                                /*!   */
    int       _minValueForGrid3DMaxima;             /*!<  */
    int       _grid3DMaximaNeighbourhoodSize;       /*!<  */
    double    _angleConeRecherche;                  /*!<  */
    int       _tailleConeRecherche;                 /*!<  */
    double    _tailleConeRechercheCm;               /*!<  */
    double    _seuilSigmaL1;                        /*!<  */
    double    _seuilSigmaL4;                        /*!<  */
    double    _threshGradMove;                      /*!<  */
    double    _threshGradLength;                    /*!<  */
    double    _longueurMin;                         /*!<  */
    double    _minHeightForMaximumSearch;           /*!<  */
    double    _maxHeightForMaximumSearch;           /*!<  */
    int       _minValueForMaximumSearch;            /*!<  */
    double    _movementThresh;                      /*!<  */
    bool      _forkSearchActive;                    /*!<  */
    int       _nForkLevels;                         /*!<  */
    int       _nSnakesMax;
};

#endif // STL_STEPEXTRACTCURVESFROMGRID3D_H
