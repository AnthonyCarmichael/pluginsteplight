#ifndef STL_GRID3DWOOTRAVERSALALGORITHM_H
#define STL_GRID3DWOOTRAVERSALALGORITHM_H

#include "ct_itemdrawable/ct_beam.h"
#include "ct_itemdrawable/ct_grid3d.h"
#include "ct_itemdrawable/tools/gridtools/ct_abstractgrid3dbeamvisitor.h"


class STL_Grid3DWooTraversalAlgorithm
{
public:

    STL_Grid3DWooTraversalAlgorithm();
    /*!
    *  \brief Constructor
    *
    *  Constructor of the class
    *
    */
    STL_Grid3DWooTraversalAlgorithm(const CT_AbstractGrid3D* grid, bool keepFirst, QList<CT_AbstractGrid3DBeamVisitor* > &list);

    STL_Grid3DWooTraversalAlgorithm(const CT_AbstractGrid3D *grid, bool keepFirst); // version sans visitors (ne pas utiliser Compute)


    /*!
     * \brief compute method of the algorithm
     */
    void compute(CT_Beam &data, Eigen::Vector3d* endPoint = nullptr, CT_Grid3D<float>* rayLengthGrid = nullptr);

    // Pour utilisation pas à pas, sans visitors
    bool init(CT_Beam &data, size_t returnedIndex);
    bool getNextIndex(size_t returnedIndex);

private:
    const CT_AbstractGrid3D*                _calibrationGrid; /*!< Calibration grid */
    Eigen::Vector3d                         _gridBottom;      /*!< bottom left coordinates of the calibration grid*/
    Eigen::Vector3d                         _gridTop;         /*!< upper right coordinates of the calibration grid*/
    double                                   _gridRes;         /*!< Resolution of the calibration grid*/
    bool                                    _keepFirst;       /*! Should be te cell containing the beam origin be kept*/
    QList<CT_AbstractGrid3DBeamVisitor* >   _visitorList;
    int                                     _numberOfVisitors;

    // Working variables
    Eigen::Vector3d     _start;
    Eigen::Vector3d     _end;
    Eigen::Vector3d     _boundary;
    int                 _currentCol;   /*!< current voxel column all along the algorithm (grid coordinate system)*/
    int                 _currentLin;   /*!< current voxel row all along the algorithm (grid coordinate system)*/
    int                 _currentLevz;  /*!< current voxel z level all along the algorithm (grid coordinate system)*/
    Eigen::Vector3d     _stepAxis;     /*!< indicates for each axis wether the ray goes forward (in the same direction than the base vector => 1) or backward (the opposite direction => -1)*/
    Eigen::Vector3d     _tMax;         /*!< "the value of t at which the ray crosses the first voxel boundary (along each direction)"*/
    Eigen::Vector3d     _tDel;         /*!< "how far along the ray we must move (in units of t)" for each component "of such a movement to equal the width of a voxel"*/
    bool        _intersects;   /*!< When creating or resetting the algorithm, this value indicates wether the ray intersects the box or not. If not, the run method won't do anything.*/
    int         _chooseAxis[8];
    int         _nextStepAxis;

    int                 _endPtCol;
    int                 _endPtLin;
    int                 _endPtLevz;



};

#endif // STL_GRID3DWOOTRAVERSALALGORITHM_H
