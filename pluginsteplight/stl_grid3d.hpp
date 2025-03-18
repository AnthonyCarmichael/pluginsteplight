#ifndef STL_GRID3D_HPP
#define STL_GRID3D_HPP


#include "stl_grid3d.h"
#include "opencv2/core/mat.hpp"
#include "ct_log/ct_logmanager.h"
 #include <numeric>

template< class DataT >
STL_Grid3D<DataT>::STL_Grid3D() :
    SuperClass(),
    _point_cloud_const_ptr( nullptr ),
    _normal_cloud_const_ptr( nullptr )
{
}

template< class DataT >
STL_Grid3D<DataT>::STL_Grid3D(const STL_Grid3D<DataT>& other) :
    SuperClass( other ),
    _point_cloud_const_ptr( other._point_cloud_const_ptr ),
    _normal_cloud_const_ptr( other._normal_cloud_const_ptr )
{
}

template< class DataT >
STL_Grid3D<DataT>::STL_Grid3D(double xmin,
                              double ymin,
                              double zmin,
                              size_t dimx,
                              size_t dimy,
                              size_t dimz,
                              double res,
                              DataT na,
                              DataT initValue) :
    SuperClass(xmin, ymin, zmin,
               static_cast<int>(dimx),static_cast<int>(dimy),static_cast<int>(dimz),
               res,
               na, initValue),
    _point_cloud_const_ptr( nullptr ),
    _normal_cloud_const_ptr( nullptr )
{
}

template< class DataT >
STL_Grid3D<DataT>::STL_Grid3D(double xmin,
                              double ymin,
                              double zmin,
                              double xmax,
                              double ymax,
                              double zmax,
                              double res,
                              DataT na,
                              DataT initValue) :
    SuperClass(xmin, ymin, zmin,
               xmax, ymax, zmax,
               res,
               na, initValue),
    _point_cloud_const_ptr( nullptr ),
    _normal_cloud_const_ptr( nullptr )
{
}

template< class DataT >
STL_Grid3D<DataT>::~STL_Grid3D()
{}

template< typename DataT>
STL_Grid3D<DataT>* STL_Grid3D<DataT>::createGrid3DFromXYZCoords(double xmin,
                                                              double ymin,
                                                              double zmin,
                                                              double xmax,
                                                              double ymax,
                                                              double zmax,
                                                              double resolution,
                                                              DataT na,
                                                              DataT initValue,
                                                              bool extends)
{
    size_t dimx = size_t(ceil((xmax - xmin)/resolution));
    size_t dimy = size_t(ceil((ymax - ymin)/resolution));
    size_t dimz = size_t(ceil((zmax - zmin)/resolution));

    if (extends)
    {
        // to ensure a point exactly on a maximum limit of the grid will be included in the grid
        while (xmax >= (xmin + dimx * resolution))
        {
            dimx++;
        }

        while (ymax >= (ymin + dimy * resolution))
        {
            dimy++;
        }

        while (zmax >= (zmin + dimz * resolution))
        {
            dimz++;
        }
    }


    return new STL_Grid3D<DataT>(xmin, ymin, zmin, dimx, dimy, dimz, resolution, na, initValue);
}


template<class DataT>
STL_Grid3D<DataT>* STL_Grid3D<DataT>::get_filtered_grid_using_ratio_thresh(double ratio_thresh, CT_AbstractStep *step_ptr) const
{
    STL_Grid3D<DataT>* filtered_grid = new STL_Grid3D<DataT>(*this);

    if (_data.empty()) {
        return filtered_grid;
    }

    for (int z = 0; z < _dimz; z++) {
        int maxVal = 0;

        // Détermine un pointeur qui représente le début de chaque tranche de z
        DataT* slice_start = &filtered_grid->_data[z * _dimx * _dimy];

        // Cherche la val max de la tranche
        for (int y = 0; y < _dimy; y++) {
            for (int x = 0; x < _dimx; x++) {
                DataT* val_ptr = slice_start + (y * _dimx) + x;
                if (*val_ptr > maxVal) {
                    maxVal = *val_ptr;
                }
            }
        }

        // Paramétrer le filtre avec le ratio
        maxVal *= 1-(ratio_thresh/100);

        // Filtre tout ce qui est inférieur de la valeur max
        for (int y = 0; y < _dimy; y++) {
            for (int x = 0; x < _dimx; x++) {
                DataT* val_ptr = slice_start + (y * _dimx) + x;
                if (*val_ptr < maxVal) {
                    *val_ptr = 0;
                }
            }
        }
    }

    return filtered_grid;
}

template<class DataT>
STL_Grid3D<DataT>* STL_Grid3D<DataT>::get_filtered_grid_using_fixed_threshold(DataT fixed_threshold, CT_AbstractStep *step_ptr) const
{
    STL_Grid3D<DataT>* filtered_grid = new STL_Grid3D<DataT>( *this );

    auto pixel_it = filtered_grid->_data.begin();
    auto pixel_it_end = filtered_grid->_data.end();

    for ( ; pixel_it != pixel_it_end ; ++pixel_it )
    {
        if (*pixel_it < fixed_threshold)
        {
            *pixel_it = 0;
        }
    }

    return filtered_grid;
}

template< class DataT >
void STL_Grid3D<DataT>::get_local_maximas(int nei_size,
                                             std::vector<Vec3i>& out_local_maximas,
                                             bool sort_descending_order) const
{
    out_local_maximas.clear();

    Vec3i bot(0,0,0);
    Vec3i top(_dimx, _dimy, _dimz );

    get_local_maximas_in_bbox( bot, top, nei_size, out_local_maximas, sort_descending_order );
}

template< class DataT >
void STL_Grid3D<DataT>::get_local_maximas_within_height_range(float zmin, float zmax, int nei_size,
                                                                 std::vector<Vec3i>& out_local_maximas,
                                                                 bool sort_descending_order) const
{
    out_local_maximas.clear();

    // Calcul du niveau z correspondant a minz a partir du sol
    int minLevZ;
    if( !levZ( minZ() + zmin , minLevZ) )
    {
        PS_LOG->addErrorMessage(LogInterface::error, QString("La grille ne contient pas la hauteur min %1").arg(zmin));
        return;
    }

    // Calcul du niveau z correspondant a maxz
    int maxLevZ;
    if( !levZ( minZ() + zmax , maxLevZ  ) )
    {
        maxLevZ = zdim();
    }

    // On limite la bbox de recherche par ces deux valeurs
    Vec3i bot( 0, 0, minLevZ );
    Vec3i top(xdim(), ydim(), maxLevZ );

    get_local_maximas_in_bbox(bot, top, nei_size, out_local_maximas, sort_descending_order);
}

template< class DataT >
void STL_Grid3D<DataT>::get_local_maximas_in_bbox(const Vec3i& bot, const Vec3i& top, int nei_size,
                                                     std::vector<Vec3i>& out_local_maximas,
                                                     bool sort_descending_order) const
{
    out_local_maximas.clear();

    // On met a jour la bbox pour etre sur de ne pas aller de la grille 3D (perte de temps dans le parcours car beaucoup de tests potentiels de cellules hors de la grille)
    Vec3i top_bbox = top;
    Vec3i bot_bbox = bot;
    Vec3i grid_dim   = dim();

    for( int axe = 0 ; axe < 3 ; axe++ )
    {
        if( bot_bbox[axe] < 0 )
        {
            bot_bbox[axe] = 0;
        }

        if( bot_bbox[axe] >= grid_dim[axe] )
        {
            bot_bbox[axe] = grid_dim[axe] - 1;
        }

        if( top_bbox[axe] < 0 )
        {
            top_bbox[axe] = 0;
        }

        if( top_bbox[axe] >= grid_dim[axe] )
        {
            top_bbox[axe] = grid_dim[axe] - 1;
        }
    }

    auto pixel_it = _data.begin();
    auto pixel_it_end = _data.end();

    //cv::SparseMatConstIterator_<DataT> pixel_it = _data.begin();
    //cv::SparseMatConstIterator_<DataT> pixel_it_end = _data.end();


    for( ; pixel_it != pixel_it_end ; ++pixel_it )
    {
        //cv::SparseMat::Node* curr_pixel_node = pixel_it.node();
        //DataT curr_val = pixel_it.template value<DataT>();

        if( *pixel_it > 0 )
        {
            // Vérifier comment avoir le x,y,z du point
            // Vec3i pix( pixel_it->idx[0], curr_pixel_node->idx[1], curr_pixel_node->idx[2]);

            // if( is_pixel_in_bbox(pix, bot_bbox, top_bbox) )
            // {
            //     if( is_pixel_local_maxima( pix, nei_size ) )
            //     {
            //         out_local_maximas.push_back( pix );
            //     }
            // }
        }
    }

    if( sort_descending_order )
    {
        std::sort(out_local_maximas.begin(), out_local_maximas.end(),
                  [&out_local_maximas, this](const Vec3i& pix1, const Vec3i& pix2) -> bool { return value(pix1[0], pix1[1], pix1[2]) > value(pix2[0], pix2[1], pix2[2]); } );
    }
}

template< class DataT >
DataT* STL_Grid3D<DataT>::get_data() {
    return _data.data();
}

template< class DataT >
bool STL_Grid3D<DataT>::is_pixel_local_maxima( const Vec3i& pix, int nei_size ) const
{
    DataT curr_value = value( pix );
    Vec3i nei;
    Vec3i dimensions = dim();
    Vec3i size ( nei_size, nei_size, nei_size, nei_size );
    Vec3i bot = pix - size;
    Vec3i top = pix + size;

    // On met a jour la bbox pour etre sur de ne pas aller de la grille 3D (perte de temps dans le parcours car beaucoup de tests potentiels de cellules hors de la grille)
    for( int axe = 0 ; axe < 3 ; axe++ )
    {
        if( bot[axe] < 0 )
        {
            bot[axe] = 0;
        }

        if( bot[axe] >= dimensions[axe] )
        {
            bot[axe] = dimensions[axe] - 1;
        }

        if( top[axe] < 0 )
        {
            top[axe] = 0;
        }

        if( top[axe] >= dimensions[axe] )
        {
            top[axe] = dimensions[axe] - 1;
        }
    }

    for ( nei.x() = bot.x(); nei.x() <= top.x() ; nei.x()++ )
    {
        for ( nei.y() = bot.y() ; nei.y() <= top.y() ; nei.y()++ )
        {
            for ( nei.z() = bot.z() ; nei.z() <= top.z(); nei.z()++ )
            {
                if ( nei != pix )
                {
                    if ( value(nei) > curr_value )
                    {
                        return false;
                    }
                }
            }
        }
    }

    return true;
}

template< class DataT >
STL_Grid3D<DataT> operator+(const STL_Grid3D<DataT>& leftGrid,const STL_Grid3D<DataT>& rightGrid )
{
    if (leftGrid._dimx != rightGrid._dimx ||
        leftGrid._dimy != rightGrid._dimy ||
        leftGrid._dimz != rightGrid._dimz) {
        throw std::invalid_argument("Les grilles doivent avoir la même taille pour être additionnées.");
    }

    STL_Grid3D<DataT> rslt(leftGrid._bot[0],
                          leftGrid._bot[1],
                          leftGrid._bot[2],
                          static_cast<size_t>(leftGrid._dimx),
                          static_cast<size_t>(leftGrid._dimy),
                          static_cast<size_t>(leftGrid._dimz),
                          leftGrid._res,
                          leftGrid._NAdata,
                          0);

    std::transform(leftGrid._data.begin(), leftGrid._data.end(), rightGrid._data.begin(), rslt._data.begin(), std::plus<DataT>());

    return rslt;
}


template< class DataT >
STL_Grid3D<DataT>& STL_Grid3D<DataT>::operator+=(const STL_Grid3D<DataT>& other)
{
    if (_dimx != other._dimx ||
        _dimy != other._dimy ||
        _dimz != other._dimz) {
        throw std::invalid_argument("Les grilles doivent avoir la même taille pour être additionnées.");
    }

    std::transform(_data.begin(), _data.end(), other._data.begin(), _data.begin(), std::plus<DataT>());

    return *this;
}

#endif // STL_GRID3D_HPP
