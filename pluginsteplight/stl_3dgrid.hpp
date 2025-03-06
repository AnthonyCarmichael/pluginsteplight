#ifndef STL_3DGRID_HPP
#define STL_3DGRID_HPP


#include "stl_3dgrid.h"
#include "opencv2/core/mat.hpp"
#include "ct_log/ct_logmanager.h"
 #include <numeric>

template< class DataT >
STL_3DGrid<DataT>::STL_3DGrid() :
    SuperClass(),
    _point_cloud_const_ptr( nullptr ),
    _normal_cloud_const_ptr( nullptr )
{
}

template< class DataT >
STL_3DGrid<DataT>::STL_3DGrid(const STL_3DGrid<DataT>& other) :
    SuperClass( other ),
    _point_cloud_const_ptr( other._point_cloud_const_ptr ),
    _normal_cloud_const_ptr( other._normal_cloud_const_ptr )
{
}

template< class DataT >
STL_3DGrid<DataT>::STL_3DGrid(double xmin,
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
STL_3DGrid<DataT>::STL_3DGrid(double xmin,
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
STL_3DGrid<DataT>::~STL_3DGrid()
{}

template< typename DataT>
STL_3DGrid<DataT>* STL_3DGrid<DataT>::createGrid3DFromXYZCoords(double xmin,
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


    return new STL_3DGrid<DataT>(xmin, ymin, zmin, dimx, dimy, dimz, resolution, na, initValue);
}



template< class DataT >
STL_3DGrid<DataT>* STL_3DGrid<DataT>::get_filtered_grid_using_fast_filter(double ratio_thresh,
                                                                              CT_AbstractStep* step_ptr) const
{

    STL_3DGrid<DataT>* filtered_grid  = new STL_3DGrid<DataT>( *this );
    /*
    STL_Grid3DBeamVisitor*  filter_visitor =  new STL_Grid3DBeamVisitor(this);
    QList<STL_Grid3DBeamVisitor<DataT>*> filter_visitors_list;

    filter_visitors_list.push_back( filter_visitor );

    CT_Grid3DWooTraversalAlgorithm woo(filtered_grid,true,filter_visitors_list);

    // On declare tout ce qui est necessaire pour faire le raytracing 4d
    ST_VisitorGrid4DFastFilter<DataT>* filter_visitor = new ST_VisitorGrid4DFastFilter<DataT>( this );
    QList< ST_AbstractVisitorGrid4D<DataT>* > filter_visitors_list;
    filter_visitors_list.push_back( filter_visitor );

    ST_VisitorGrid4DSetValue<DataT>* set_value_visitor = new ST_VisitorGrid4DSetValue<DataT>(filtered_grid , static_cast<DataT>(0) );
    QList< ST_AbstractVisitorGrid4D<DataT>* > set_value_visitors_list;
    set_value_visitors_list.push_back( set_value_visitor );

    // On declare un algorithme de raytracing 4D
    ST_Grid4DWooTraversalAlgorithm<DataT> traversal_algo_accumulate( this, true, filter_visitors_list );
    ST_Grid4DWooTraversalAlgorithm<DataT> traversal_algo_set_zero( filtered_grid , false, set_value_visitors_list );

    // -----------------------------------------------------------------------------------------------------------------
    // Loop through all points and normals of the input point cloud and start raytracing inside Hough space
    size_t i_point = 0;
    size_t n_points = _point_cloud_const_ptr->pointCloudIndex()->size();
    CT_PointIterator itPoint(_point_cloud_const_ptr->pointCloudIndex());
    for( CT_PointIterator itPoint(_point_cloud_const_ptr->pointCloudIndex()) ; itPoint.hasNext() ; i_point++ )
    {
        if( step_ptr != nullptr )
        {
            if( i_point % 100 == 0 )
            {
                // step_ptr->setProgress( static_cast<float>(i_point) * 100.0f / static_cast<float>(n_points) );
            }

            if( step_ptr->isStopped() )
            {
                return filtered_grid ;
            }
        }

        const CT_Point&  currentPoint       = itPoint.next().currentPoint();
        const CT_Normal& currentCTNormal    = _normal_cloud_const_ptr->constNormalAt(i_point);
        const Vec3d      currentNormal      = currentCTNormal.head(3).cast<double>();

        float normalLenght = currentNormal.norm();

        if( normalLenght != 0.0 )
        {

            CT_Beam beam_01( currentPoint, currentNormal );
            CT_Beam beam_02( currentPoint, -currentNormal );
            
            filter_visitor->setSumOfVisitedVotes( 0 );
            traversal_algo_accumulate.compute(beam_01);
            int n_votes_beam_01 = filter_visitor->sumOfVisitedVotes();

            filter_visitor->setSumOfVisitedVotes( 0 );
            traversal_algo_accumulate.compute(beam_02);
            int n_votes_beam_02 = filter_visitor->sumOfVisitedVotes();

            bool  beam_01_is_max = n_votes_beam_01 > n_votes_beam_02;
            float n_votes_max    = static_cast<float>( std::max( n_votes_beam_01, n_votes_beam_02 ) );
            float n_votes_min    = static_cast<float>( std::min( n_votes_beam_01, n_votes_beam_02 ) );
            float ratio          = n_votes_max / n_votes_min;

            if( ratio < ratio_thresh )
            {
                // Filtrer les deux directions
                traversal_algo_set_zero.compute(beam_01);
                traversal_algo_set_zero.compute(beam_02);
            }
            else if( beam_01_is_max )
            {
                // Filtre direction de beam 02
                traversal_algo_set_zero.compute(beam_02);
            }
            else
            {
                // Filtre direction de beam 01
                traversal_algo_set_zero.compute(beam_01);
            }
        }
    }

    delete filter_visitor;
    delete set_value_visitor;

    filtered_grid ->computeMinMax();
*/

    return filtered_grid ;
}



template<class DataT>
STL_3DGrid<DataT>* STL_3DGrid<DataT>::get_filtered_grid_using_fixed_threshold(DataT fixed_threshold, CT_AbstractStep *step_ptr) const
{
    STL_3DGrid<DataT>* filtered_grid = new STL_3DGrid<DataT>( *this );

    cv::SparseMatConstIterator_<DataT> pixel_it = filtered_grid->_data.begin();
    cv::SparseMatConstIterator_<DataT> pixel_it_end = filtered_grid->_data.end();

    for( ; pixel_it != pixel_it_end ; ++pixel_it )
    {
        const cv::SparseMat::Node* curr_pixel_node = pixel_it.node();
        DataT curr_val = pixel_it.template value<DataT>();
        if( curr_val < fixed_threshold )
        {
            filtered_grid->setValue(curr_pixel_node->idx[0],
                                  curr_pixel_node->idx[1],
                                  curr_pixel_node->idx[2],
                                  curr_pixel_node->idx[3],
                                  0);
        }
    }

    return filtered_grid;
}


template< class DataT >
void STL_3DGrid<DataT>::get_local_maximas(int nei_size,
                                             std::vector<Vec3i>& out_local_maximas,
                                             bool sort_descending_order) const
{
    out_local_maximas.clear();

    Vec3i bot(0,0,0,0);
    Vec3i top(_dimx, _dimy, _dimz );

    get_local_maximas_in_bbox( bot, top, nei_size, out_local_maximas, sort_descending_order );
}

template< class DataT >
void STL_3DGrid<DataT>::get_local_maximas_within_height_range(float zmin, float zmax, int nei_size,
                                                                 std::vector<Vec3i>& out_local_maximas,
                                                                 bool sort_descending_order) const
{
    out_local_maximas.clear();

    // Calcul du niveau z correspondant a minz a partir du sol (bot de l'espace de Hough)
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
void STL_3DGrid<DataT>::get_local_maximas_in_bbox(const Vec3i& bot, const Vec3i& top, int nei_size,
                                                     std::vector<Vec3i>& out_local_maximas,
                                                     bool sort_descending_order) const
{
    out_local_maximas.clear();

    // On met a jour la bbox pour etre sur de ne pas aller hors de l'espace de Hough (perte de temps dans le parcours car beaucoup de tests potentiels de cellules hors de l'espace)
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

    cv::SparseMatConstIterator_<DataT> pixel_it = _data.begin();
    cv::SparseMatConstIterator_<DataT> pixel_it_end = _data.end();

    for( ; pixel_it != pixel_it_end ; ++pixel_it )
    {
        const cv::SparseMat::Node* curr_pixel_node = pixel_it.node();
        DataT curr_val = pixel_it.template value<DataT>();
        if( curr_val > 0 )
        {
            Vec3i pix( curr_pixel_node->idx[0], curr_pixel_node->idx[1], curr_pixel_node->idx[2]);

            if( is_pixel_in_bbox(pix, bot_bbox, top_bbox) )
            {
                if( is_pixel_local_maxima( pix, nei_size ) )
                {
                    out_local_maximas.push_back( pix );
                }
            }
        }
    }

    if( sort_descending_order )
    {
        std::sort(out_local_maximas.begin(), out_local_maximas.end(),
                  [&out_local_maximas, this](const Vec3i& pix1, const Vec3i& pix2) -> bool { return value(pix1[0], pix1[1], pix1[2], pix1[3]) > value(pix2[0], pix2[1], pix2[2], pix2[3]); } );
    }
}

template< class DataT >
DataT* STL_3DGrid<DataT>::get_data() {
    return _data.data();
}

template< class DataT >
bool STL_3DGrid<DataT>::is_pixel_local_maxima( const Vec3i& pix, int nei_size ) const
{
    DataT curr_value = value( pix );
    Vec3i nei;
    Vec3i dimensions = dim();
    Vec3i size ( nei_size, nei_size, nei_size, nei_size );
    Vec3i bot = pix - size;
    Vec3i top = pix + size;

    // On met a jour la bbox pour etre sur de ne pas aller hors de l'espace de Hough (perte de temps dans le parcours car beaucoup de tests potentiels de cellules hors de l'espace)
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
STL_3DGrid<DataT> operator+(const STL_3DGrid<DataT>& leftGrid,const STL_3DGrid<DataT>& rightGrid )
{
    if (leftGrid._dimx != rightGrid._dimx ||
        leftGrid._dimy != rightGrid._dimy ||
        leftGrid._dimz != rightGrid._dimz) {
        throw std::invalid_argument("Les grilles doivent avoir la même taille pour être additionnées.");
    }

    STL_3DGrid<DataT> rslt(leftGrid._bot[0],
                          leftGrid._bot[1],
                          leftGrid._bot[2],
                          static_cast<size_t>(leftGrid._dimx),
                          static_cast<size_t>(leftGrid._dimy),
                          static_cast<size_t>(leftGrid._dimz),
                          leftGrid._res,
                          leftGrid._NAdata,
                          0);

    std::transform(leftGrid._data.begin(), leftGrid._data.end(), rightGrid._data.begin(), rslt._data.begin(), std::plus<DataT>());
    //std::transform(rslt._data.begin(), rslt._data.end(), rightGrid._data.begin(), rightGrid._data.begin(), std::plus<int>());

    return rslt;
}

#endif // STL_3DGRID_HPP
