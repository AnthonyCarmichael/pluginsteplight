#ifndef STL_GRID3D_H
#define STL_GRID3D_H

#include "ct_itemdrawable/ct_grid3d.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_step/abstract/ct_abstractstep.h"

template< class DataT >
class STL_Grid3D : public CT_Grid3D<DataT>
{

    using SuperClass = CT_Grid3D<DataT>;

    using PointCloudConst       = const CT_AbstractItemDrawableWithPointCloud;
    using PointCloudConstPtr    = PointCloudConst*;
    using NormalCloudConst      = const CT_PointsAttributesNormal;
    using NormalCloudConstPtr   = NormalCloudConst*;

    using Vec3i                 = Eigen::Vector3i;
    using Vec3f                 = Eigen::Vector3f;
    using Vec3d                 = Eigen::Vector3d;
public:
    //**********************************************//
    //           Constructors/Destructors           //
    //**********************************************//
    /**
      * \brief Default constructor
      *  Each attribute will be set to 0, nullptr or will be cleared
      */
    STL_Grid3D();
    STL_Grid3D(const STL_Grid3D& otherGrid);

    /*!
     * \brief Initialisation constructor
     *
     * Grid is created thanks to bottom left point of bounding box (3D) and number of cells along each dimension.
     *
     * \param model Item model for creation
     * \param result Result containing the item
     * \param xmin Minimum X coordinate (bottom left corner)
     * \param ymin Minimum Y coordinate (bottom left corner)
     * \param zmin Minimum Z coordinate (bottom left corner)
     * \param dimx Number of colums
     * \param dimy Number of rows
     * \param dimz Number of zlevels
     * \param res resolution
     * \param na Value used to code NA
     * \param initValue Initialisation value for grid cells
     */
    STL_Grid3D(double xmin,
               double ymin,
               double zmin,
               size_t dimx,
               size_t dimy,
               size_t dimz,
               double res,
               DataT na,
               DataT initValue);


    /*!
     * \brief Initialisation constructor
     *
     * Grid is created thanks to the bounding box (3D) of the grid
     *
     * \param model Item model for creation
     * \param result Result containing the item
     * \param xmin Minimum X coordinate (bottom left corner)
     * \param ymin Minimum Y coordinate (bottom left corner)
     * \param zmin Minimum Z coordinate (bottom left corner)
     * \param xmax Maximum X coordinate (top right corner)
     * \param ymax Maximum Y coordinate (top right corner)
     * \param zmax Maximum Z coordinate (top right corner)
     * \param res resolution
     * \param na Value used to code NA
     * \param initValue Initialisation value for grid cells
     * \param coordConstructor Not used, only to ensure constructor different signatures
     */
    STL_Grid3D(double xmin,
               double ymin,
               double zmin,
               double xmax,
               double ymax,
               double zmax,
               double res,
               DataT na,
               DataT initValue);

    static STL_Grid3D<DataT>* createGrid3DFromXYZCoords(double xmin,
                                                                double ymin,
                                                                double zmin,
                                                                double xmax,
                                                                double ymax,
                                                                double zmax,
                                                                double resolution,
                                                                DataT na,
                                                                DataT initValue,
                                                                bool extends = true);
public:
    /*!
     * \brief Destructor
     */
    ~STL_Grid3D() override;

    inline double getResolutionsGrid() const { return _res; }


    STL_Grid3D<DataT>* get_filtered_grid_by_neigbhours(int neighbours,
                                                            CT_AbstractStep* step_ptr = nullptr) const;

    STL_Grid3D<DataT>* get_filtered_grid_using_fixed_threshold(DataT fixed_threshold,
                                                                CT_AbstractStep* step_ptr = nullptr) const;

    STL_Grid3D<DataT>* get_filtered_grid3d_using_fast_filter(double ratio_thresh,
                                                            CT_AbstractStep* step_ptr = nullptr) const;

    void get_local_maximas(int nei_size,
                           std::vector<Vec3i>& out_local_maximas,
                           bool sort_descending_order) const;

    void get_local_maximas_within_height_range(float zmin, float zmax, int nei_size,
                                               std::vector<Vec3i>& out_local_maximas,
                                               bool sort_descending_order) const;

    void get_local_maximas_in_bbox(const Vec3i& bot, const Vec3i& top, int nei_size,
                                   std::vector<Vec3i>& out_local_maximas,
                                   bool sort_descending_order) const;

    DataT* get_data();


    bool is_pixel_local_maxima( const Vec3i& pix, int nei_size ) const;

    inline DataT value(const Vec3i& pix) const
    {
        return CT_Grid3D<DataT>::value(pix.x(), pix.y(), pix.z());
    }

    inline DataT value(int levx, int levy, int levz) const //override
    {
        return CT_Grid3D<DataT>::value(levx, levy, levz);
    }

    inline bool is_pixel_in_bbox( const Vec3i& pixel, const Vec3i& bot, const Vec3i& top ) const
    {
        if( pixel.x() < bot.x() ||
            pixel.y() < bot.y() ||
            pixel.z() < bot.z() ||
            pixel.x() > top.x() ||
            pixel.y() > top.y() ||
            pixel.z() > top.z() )
        {
            return false;
        }

        return true;
    }

    inline double pixelToCartesianX( int levx ) const
    {
        return ( minX() + ( levx + 0.5 ) * _res );
    }

    inline double pixelToCartesianY( int levy ) const
    {
        return ( minY() + ( levy + 0.5 ) * _res );
    }

    inline double pixelToCartesianZ( int levz ) const
    {
        return ( minZ() + ( levz + 0.5 ) * _res );
    }

    inline Vec3d pixel_to_cartesian(const Vec3i& pix)  const
    {
        return pixel_to_cartesian(pix[0], pix[1], pix[2]);
    }

    inline Vec3d pixel_to_cartesian(int levx, int levy, int levz) const
    {
        return Vec3d(pixelToCartesianX(levx),
                     pixelToCartesianY(levy),
                     pixelToCartesianZ(levz));
    }

    inline int cartesianToPixelX( double x ) const
    {
        if( x == _top[0] )
        {
            return _dimx - 1;
        }

        else
        {
            return floor( ( x - _bot[0] ) / _res );
        }
    }

    inline int cartesianToPixelY( double y ) const
    {
        if( y == _top[1] )
        {
            return _dimy - 1;
        }

        else
        {
            return floor( ( y - _bot[1] ) / _res );
        }
    }

    inline int cartesianToPixelZ( double z ) const
    {
        if( z == _top[2] )
        {
            return _dimz - 1;
        }

        else
        {
            return floor( ( z - _bot[2] ) / _res );
        }
    }

    inline Vec3i cartesian_to_pixel(const Vec3d& point)  const
    {
        return cartesian_to_pixel(point[0], point[1], point[2]);
    }

    inline Vec3i cartesian_to_pixel(double x, double y, double z) const
    {
        return Vec3i(cartesianToPixelX(x),
                     cartesianToPixelY(y),
                     cartesianToPixelZ(z));
    }

    bool isPixelIn( const Vec3i& pix ) const
    {
        return (pix[0] >= 0 && pix[0] < xdim() &&
                pix[1] >= 0 && pix[1] < ydim() &&
                pix[2] >= 0 && pix[2] < zdim() );
    }


    inline Vec3i dim() const
    {
        return Vec3i(_dimx, _dimy, _dimz);
    }

    inline void getMinMaxInNeighbourhood(const Vec3i& pixel, DataT& outMin, DataT& outMax, int size) const
    {
        outMin =  std::numeric_limits<DataT>::max();
        outMax = -std::numeric_limits<DataT>::max();

        DataT neiValue;
        Vec3i nei;

        for( nei[0] = pixel.x()-size ; nei[0] <= pixel.x()+size ; nei[0]++ )
        {
            for( nei[1] = pixel.y()-size ; nei[1] <= pixel.y()+size ; nei[1]++ )
            {
                for( nei[2] = pixel.z()-size ; nei[2] <= pixel.z()+size ; nei[2]++ )
                {
                    if( isPixelIn(nei) )
                    {
                        neiValue = value(nei[0], nei[1], nei[2]);

                        if ( neiValue < outMin )
                        {
                            outMin = neiValue;
                        }

                        if ( neiValue > outMax )
                        {
                            outMax = neiValue;
                        }
                    }
                }
            }
        }
    }

    Eigen::Vector3d getBot() const {
       return _bot;
    }

    Eigen::Vector3d getTop() const {
        return _top;
    }

    CT_Grid3D<float>* getGridRayLength() const {
        return _gridRayLength;
    }


    friend STL_Grid3D<DataT> operator+(const STL_Grid3D<DataT>& leftGrid,const STL_Grid3D<DataT>& rightGrid );


    STL_Grid3D<DataT>& operator+=(const STL_Grid3D<DataT>& other);

    void setPointCloudPtr(PointCloudConstPtr point_cloud_const_ptr, NormalCloudConstPtr normal_cloud_const_ptr);
    void setGridRayLength(CT_Grid3D<float> *gridRayLenght);
    void setBotTop(Vec3d _bot, Vec3d _top);
    void setRealRayValueDivadedByVisit();

protected:
    // -------------------------------------------------------
    // Attributes
    // -------------------------------------------------------
    using SuperClass::nCells;
    using SuperClass::NA;
    using SuperClass::id;
    using SuperClass::alternativeDrawManager;
    Vec3d _top;
    Vec3d _bot;
    using SuperClass::_dimx;
    using SuperClass::_dimy;
    using SuperClass::_dimz;
    using SuperClass::_res;
    using SuperClass::_NAdata;
    using SuperClass::_dataMin;
    using SuperClass::_dataMax;
    using SuperClass::_data;

    PointCloudConstPtr  _point_cloud_const_ptr;
    NormalCloudConstPtr _normal_cloud_const_ptr;

    CT_Grid3D<float> *_gridRayLength;
};

#include "stl_grid3d.hpp"

#endif // STL_GRID3D_H
