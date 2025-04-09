#ifndef STL_OPENACTIVECONTOURS_H
#define STL_OPENACTIVECONTOURS_H

#include "ct_itemdrawable/ct_circle.h"
#include <ct_log/ct_logmanager.h>
#include "stl_grid3d.h"

template< class DataT >
class STL_OpenActiveContours
{
public:
    STL_OpenActiveContours(const STL_Grid3D<int>* const grid ,
                          const STL_Grid3D<int>* const repulse_image,
                          const Eigen::Vector3i& start_pixel,
                          int search_cone_size);

    void get_dir_and_score_inside_bbox(const Eigen::Vector3i& center_pixel,
                                       const Eigen::Vector3d& reference_point,
                                       int search_bbox_size_spatial,
                                       int search_bbox_size_radius,
                                       int n_max_dir_to_keep,
                                       DataT min_value_to_consider,
                                       bool sort_crescent_order,
                                       std::vector< std::pair< Eigen::Vector3d, DataT> >& out_dir_and_scores);

    std::vector< CT_Circle* > get_raw_circles() const;

    void append_raw_circles_to_vector( std::vector< CT_Circle* >& in_out_circle_vector ) const;

    inline int get_n_points() const { return _points.rows(); }

    void markRepulsion( double repulseFactor );

    void resample( double sampleRes );

    double length3D() const;

    void grow(int nIterMax,
              double growCoeff,
              double coneAngleMaxDegres,
              double coneSizeMaxPixels,
              double seuilSigmaL1);

    void updatePoints();

    void getGrowingDirections(double coneAngleDegres,
                              int coneSizePixels,
                              double seuilSigmaL1,
                              double&outSigmaL1Head,
                              double&outSigmaL1Back,
                              bool& outHasGrownHead,
                              bool& outHasGrownBack,
                              Eigen::Vector3d& outGrowDirHead,
                              Eigen::Vector3d& outGrowDirBack ) const;

    void get_dir_and_score_inside_cone(const Eigen::Vector3i& center_pixel,
                                       const Eigen::Vector3d& reference_point,
                                       const Eigen::Vector3d& cone_direction,
                                       double cone_angle_deg,
                                       int search_bbox_size_spatial,
                                       int search_bbox_size_radius,
                                       int n_max_dir_to_keep,
                                       DataT min_value_to_consider,
                                       bool sort_crescent_order,
                                       std::vector<  std::pair< Eigen::Vector3d, DataT> >& out_dir_and_scores) const;

    Eigen::Vector3d getGradientEnergyGrowProportionalResolution(const Eigen::Vector3d& snakeExtremityPoint,
                                                      const Eigen::Vector3i& snakeExtremityPix,
                                                      const Eigen::Vector3d &snakeTangentAtExtremity,
                                                      double coneAngleDegres,
                                                      int coneSizePixels,
                                                      double& out_sigma_l1,
                                                      bool& outHasGrown ) const;

    void getGeometricMatrix(double alpha, double beta, Eigen::MatrixXd& outGeometricMatrix) const;
    void getAPlusRhoIInverse(double alpha, double beta, double timeStep, Eigen::MatrixXd& outAPlusRhoIInverse ) const;
    void updatePointsAndGetAverageAndStDevMovement(const Eigen::MatrixXd &newPoints,
                                                   double& outAverageMovemet,
                                                   double& outStdDevMovement );

    void relax(int nIterMax,
               double alpha, double beta, double gama,
               double globalWeight,
               double timeStep,
               double threshAverageMovement3D );

    Eigen::MatrixXd* getGradientEnergyOrthogonalImageMatrixMultipliedByTangentNorm( double globalWeight ) const;
    Eigen::MatrixXd* getGradientEnergyOrthogonalImageMatrix( double globalWeight ) const;
    Eigen::Vector3d gradientEnergyImageInterpolatedAtPoint(int indexPoint, double globalWeight) const;
    double partialDerivativeEnergyImageAtPixelInImage(const Eigen::Vector3i &p, double globalWeight, int coordToDerive) const;
    double partialDerivativeEnergyImageAtPixelOutsideImage(const Eigen::Vector3i& p, double globalWeight, int coordToDerive) const;

    inline double partialDerivativeEnergyImageAtPixel(const Eigen::Vector3i& p, double globalWeight, int coordToDerive) const
    {
        double partialDerivate;
        // Si p est dans l'image, calculer sa derivee partielle est toujours possible :
        // p est dans le centre de l'image,on fait une difference finie centree
        // p est un bord de l'image, on peut faire une difference finie a droite/gauche
        if( _grid3d->isPixelIn(p) )
        {
            partialDerivate = partialDerivativeEnergyImageAtPixelInImage( p, globalWeight, coordToDerive );
        }

        // Si p est en dehors de l'image il faut calculer la derivee partielle autrement
        // Pour le coup je prend le mirroir
        // i.e. si p est a gauche de l'image on prend l'oppose de la derivee partielle en image(1)
        //      si p est a droite de l'image on prend l'oppose de la derivee partielle en image( dim-1 )
        else
        {
            partialDerivate = partialDerivativeEnergyImageAtPixelOutsideImage( p, globalWeight, coordToDerive );
        }

        return partialDerivate;
    }

    double energyImageLocalAtPixel(const Eigen::Vector3i& p) const;

    inline double energyImageGlobalAtPixel(const Eigen::Vector3i& p) const
    {
        return static_cast<double>( _grid3d->dataMin() - _grid3d->value(p[0], p[1], p[2]) ) / static_cast<double>( _grid3d->dataMax() - _grid3d->dataMin() );
    }

    inline double energyImageAtPixel(const Eigen::Vector3i &p, double globalWeight) const
    {
        return ( globalWeight * energyImageGlobalAtPixel(p) ) + ( ( 1.0 - globalWeight ) * energyImageLocalAtPixel(p) );
    }

    Eigen::Vector3d directionContoursAtPoint(int indexPoint) const;

    Eigen::MatrixXd* getSecondDifferentialOnTangentDividedByTangentNormMultiplyedByImageEnergy( double globalWeight ) const;

    Eigen::Vector3d getTangentAtPoint( int i, bool normalize ) const;
    inline Eigen::Vector3d getTangentAtHead(bool normalize) const { return getTangentAtPoint(0, normalize); }
    inline Eigen::Vector3d getTangentAtTail(bool normalize) const { return getTangentAtPoint( get_n_points() - 1, normalize ); }

    inline Eigen::Vector3d getSecondDifferentialAtPoint( int indexPoint ) const
    {
        if( indexPoint == 0 || indexPoint == get_n_points()-1 )
        {
            return Eigen::Vector3d(0,0,0);
        }

        Eigen::Vector3d prevPoint = _points.row( indexPoint - 1 );
        Eigen::Vector3d currPoint = _points.row( indexPoint );
        Eigen::Vector3d nextPoint = _points.row( indexPoint + 1 );

        return ( prevPoint - (2*currPoint) + nextPoint );
    }



private :
    const STL_Grid3D<int>*  const _grid3d;   /*!< Un contours actif se deplace sur une image */
    const STL_Grid3D<int>*  const _repulse_image; /*!< Un contours actif ne peut pas se deplacer vers des pixels qui sont marques comme repulsifs */
    Eigen::MatrixXd         _points;        /*!< Points du contours places dans une matrice n lignes, 4 colonnes */

};

template< class DataT >
void uncenteredPCA(const std::vector< std::pair< Eigen::Vector3d, DataT> >& dir_score_pairs,
                   Eigen::Vector3d& out_v1, Eigen::Vector3d& out_v2, Eigen::Vector3d& out_v3,
                   double& out_l1, double& out_l2, double& out_l3);

namespace StepTools
{
template< typename DataT >
inline DataT linearInterpolation( DataT xa, DataT xb, DataT ya, DataT yb, DataT valToInterpolate )
{
    if( xa == xb )
    {
        return ya;
    }

    double a = (ya-yb)/((double)(xa-xb));
    return ( ( a * valToInterpolate ) + ( ya - (a*xa) ) );
}
}

#include "stl_openactivecontours.hpp"

#endif // STL_OPENACTIVECONTOURS_H
