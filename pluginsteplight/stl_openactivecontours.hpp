#ifndef STL_OPENACTIVECONTOURS_HPP
#define STL_OPENACTIVECONTOURS_HPP

#include "stl_openactivecontours.h"
#include <ct_log/ct_logmanager.h>

#include <QObject>
#include <QtConcurrent/QtConcurrent>

template< class DataT >
STL_OpenActiveContours<DataT>::STL_OpenActiveContours(const STL_Grid3D<int>* const grid ,
                                                      const STL_Grid3D<int>* const repulse_image,
                                                      const Eigen::Vector3i& start_pixel,
                                                      int search_cone_size) :
    _grid3d(grid),
    _repulse_image(repulse_image)
{
    Eigen::Vector3d head_point = _grid3d->pixel_to_cartesian( start_pixel );

    // Calcule la bbox correspondant a un cube centre autour de l'extremite du snake et de taille coneSizePix
    Eigen::Vector3i search_cone_size_vec( search_cone_size, search_cone_size, search_cone_size);
    Eigen::Vector3i cone_bbox_bot = start_pixel - search_cone_size_vec;
    Eigen::Vector3i cone_bbox_top = start_pixel + search_cone_size_vec;
    Eigen::Vector3i grid_dim = _grid3d->dim();

    // On met a jour la bbox pour etre sur de ne pas aller hors de la grille (perte de temps dans le parcours car beaucoup de tests potentiels de cellules hors de l'espace)
    for( int axe = 0 ; axe < 3 ; axe++ )
    {
        if( cone_bbox_bot[axe] < 0 )
        {
            cone_bbox_bot[axe] = 0;
        }

        if( cone_bbox_bot[axe] >= grid_dim[axe] )
        {
            cone_bbox_bot[axe] = grid_dim[axe] - 1;
        }

        if( cone_bbox_top[axe] < 0 )
        {
            cone_bbox_top[axe] = 0;
        }

        if( cone_bbox_top[axe] >= grid_dim[axe] )
        {
            cone_bbox_top[axe] = grid_dim[axe] - 1;
        }
    }

    // Il ne reste plus qu'a regarder dans la boite pour faire une acp des directions ponderees
    // On parcour donc toute la bbox et on recupere les directions de croissance potentielles
    // Avec leur scores dans l'espace de Hough
    std::vector<std::pair< Eigen::Vector3d,DataT>> potential_grow_dirs_and_scores;
    get_dir_and_score_inside_bbox(start_pixel,
                                  head_point,
                                  search_cone_size,
                                  0,
                                  40,
                                  static_cast<DataT>(10),
                                  true,
                                  potential_grow_dirs_and_scores);

    // Effectue l'ACP 3D non centree des directions de croissance potentielles
    Eigen::Vector3d  v1, v2, v3;
    double l1, l2, l3;

    uncenteredPCA(potential_grow_dirs_and_scores,
                  v1, v2, v3,
                  l1, l2, l3);

    // On normalise la direction v1
    if( v1.norm() == 0 )
    {
        PS_LOG->addErrorMessage(LogInterface::error, QObject::tr("STL_OpenActiveContours constructor: after PCA, direction has a norm of 0! %1 %2 %3").arg(v1[0]).arg(v1[1]).arg(v1[2]));
        exit( EXIT_FAILURE );
    }
    v1 *= _grid3d->resolution();

    // On ajoute la tete et la queue comme (head + v1)
    // On ajoute les deux points (transformes en coordonnees cartesiennes) dans la matrice
    _points.resize( 2, 3 );
    _points.row(0) = head_point - v1;
    _points.row(1) = head_point + v1;

    // Et on resample pour en avoir quatre
    resample( length3D() / 3.0 );
}

template< class DataT >
void STL_OpenActiveContours<DataT>::get_dir_and_score_inside_bbox(const Eigen::Vector3i& center_pixel,
                                                                 const Eigen::Vector3d& reference_point,
                                                                 int search_bbox_size_spatial,
                                                                 int search_bbox_size_radius,
                                                                 int n_max_dir_to_keep,
                                                                 DataT min_value_to_consider,
                                                                 bool sort_crescent_order,
                                                                 std::vector< std::pair< Eigen::Vector3d,DataT> >& out_dir_and_scores)
{
    Eigen::Vector3i search_bbox_size_vec(search_bbox_size_spatial, search_bbox_size_spatial, search_bbox_size_spatial);
    Eigen::Vector3i bbox_bot = center_pixel - search_bbox_size_vec;
    Eigen::Vector3i bbox_top = center_pixel + search_bbox_size_vec;

    // Il ne reste plus qu'a regarder dans la boite pour faire une acp des directions ponderees
    // On parcour donc toute la bbox et on recupere les directions de croissance potentielles
    // Avec leur scores dans la grille
    Eigen::Vector3i curr_pixel_in_bbox;
    for ( curr_pixel_in_bbox[0] = bbox_bot[0] ; curr_pixel_in_bbox[0] <= bbox_top[0] ; curr_pixel_in_bbox[0]++ )
    {
        for ( curr_pixel_in_bbox[1] = bbox_bot[1]; curr_pixel_in_bbox[1] <= bbox_top[1] ; curr_pixel_in_bbox[1]++ )
        {
            for ( curr_pixel_in_bbox[2] = bbox_bot[2] ; curr_pixel_in_bbox[2] <= bbox_top[2] ; curr_pixel_in_bbox[2]++ )
            {
                DataT cur_pixel_value = _grid3d->value(curr_pixel_in_bbox[0], curr_pixel_in_bbox[1], curr_pixel_in_bbox[2]);

                if( cur_pixel_value > min_value_to_consider && center_pixel != curr_pixel_in_bbox )
                {
                    Eigen::Vector3d curr_point_in_bbox = _grid3d->pixel_to_cartesian(curr_pixel_in_bbox);

                    // Calcul de la direction potentielle de croissance
                    Eigen::Vector3d  potential_grow_dir = curr_point_in_bbox - reference_point;
                    double potential_grow_dir_norm_3d = potential_grow_dir.tail(3).norm();

                    Eigen::Vector3d weighted_direction = potential_grow_dir * static_cast<double>(cur_pixel_value) * ((static_cast<double>(search_bbox_size_spatial) * _grid3d->resolution() ) - potential_grow_dir_norm_3d);
                    out_dir_and_scores.push_back( std::make_pair(weighted_direction, cur_pixel_value) );
                }
            }
        }
    }

    if( sort_crescent_order )
    {
        // Trie les directions ponderees par ordre croissant de score dans la grille
        std::sort(out_dir_and_scores.begin(),
                  out_dir_and_scores.end(),
                  [](const std::pair< Eigen::Vector3d,DataT>& pair1, const std::pair< Eigen::Vector3d,DataT>& pair2) -> bool{ return pair1.second < pair2.second ; } );
    }

    if( out_dir_and_scores.size() > n_max_dir_to_keep )
    {
        out_dir_and_scores.erase(out_dir_and_scores.begin()+n_max_dir_to_keep, out_dir_and_scores.end());
    }
}

template< class DataT >
void STL_OpenActiveContours<DataT>::get_dir_and_score_inside_cone(const Eigen::Vector3i& center_pixel,
                                                                 const Eigen::Vector3d& reference_point,
                                                                 const Eigen::Vector3d& cone_direction,
                                                                 double cone_angle_deg,
                                                                 int search_bbox_size_spatial,
                                                                 int search_bbox_size_radius,
                                                                 int n_max_dir_to_keep,
                                                                 DataT min_value_to_consider,
                                                                 bool sort_crescent_order,
                                                                 std::vector< std::pair< Eigen::Vector3d,DataT> >& out_dir_and_scores) const
{
    Eigen::Vector3i search_bbox_size_vec( search_bbox_size_spatial, search_bbox_size_spatial, search_bbox_size_spatial);
    Eigen::Vector3i bbox_bot = center_pixel - search_bbox_size_vec;
    Eigen::Vector3i bbox_top = center_pixel + search_bbox_size_vec;
    Eigen::Vector3d cone_direction_spatial = cone_direction.tail(3).normalized();

    // Il ne reste plus qu'a regarder dans la boite pour faire une acp des directions ponderees
    // On parcour donc toute la bbox et on recupere les directions de croissance potentielles
    // Avec leur scores dans l'espace de Hough
    Eigen::Vector3i curr_pixel_in_bbox;
    for ( curr_pixel_in_bbox[0] = bbox_bot[0] ; curr_pixel_in_bbox[0] <= bbox_top[0] ; curr_pixel_in_bbox[0]++ )
    {
        for ( curr_pixel_in_bbox[1] = bbox_bot[1]; curr_pixel_in_bbox[1] <= bbox_top[1] ; curr_pixel_in_bbox[1]++ )
        {
            for ( curr_pixel_in_bbox[2] = bbox_bot[2] ; curr_pixel_in_bbox[2] <= bbox_top[2] ; curr_pixel_in_bbox[2]++ )
            {
                DataT cur_pixel_value = _grid3d->value(curr_pixel_in_bbox[0], curr_pixel_in_bbox[1], curr_pixel_in_bbox[2]);

                if( cur_pixel_value > min_value_to_consider && center_pixel != curr_pixel_in_bbox )
                {
                    Eigen::Vector3d curr_point_in_bbox = _grid3d->pixel_to_cartesian(curr_pixel_in_bbox);

                    // Calcul de la direction potentielle de croissance
                    Eigen::Vector3d  potential_grow_dir         = curr_point_in_bbox - reference_point;
                    Eigen::Vector3d  potential_grow_dir_spatial = potential_grow_dir.tail(3);
                    double potential_grow_dir_norm_3d = potential_grow_dir_spatial.norm();

                    // --------------------------------------
                    // Test de l'angle en 3D
                    double scalar_product = potential_grow_dir_spatial.normalized().dot(cone_direction_spatial);

                    // Il faut que les deux vecteurs soient de meme sens
                    if( scalar_product >= 0 )
                    {
                        double angle = (180.0/M_PI) * acos( scalar_product ); // Pas besoin de diviser par la norme car potentielGrowDirPoint et tangentNormalized ont une norme de 1
                        if( angle <= cone_angle_deg )
                        {
                            Eigen::Vector3d weighted_direction = potential_grow_dir * static_cast<double>(cur_pixel_value) ;//* ((static_cast<double>(search_bbox_size_spatial) * _grid3d->resolution() ) - potential_grow_dir_norm_3d);
                            out_dir_and_scores.push_back( std::make_pair(weighted_direction, cur_pixel_value) );
                        }
                    }
                }
            }
        }
    }

    if( sort_crescent_order )
    {
        // Trie les directions ponderees par ordre croissant de score dans l'espace de Hough
        std::sort(out_dir_and_scores.begin(),
                  out_dir_and_scores.end(),
                  [](const std::pair< Eigen::Vector3d,DataT>& pair1, const std::pair< Eigen::Vector3d,DataT>& pair2) -> bool{ return pair1.second < pair2.second ; } );
    }

    if( out_dir_and_scores.size() > n_max_dir_to_keep )
    {
        out_dir_and_scores.erase(out_dir_and_scores.begin()+n_max_dir_to_keep, out_dir_and_scores.end());
    }
}

template< class DataT >
std::vector< CT_Circle* > STL_OpenActiveContours<DataT>::get_raw_circles() const
{
    std::vector< CT_Circle* > rslt;

    append_raw_circles_to_vector(rslt);

    return rslt;
}

template< class DataT >
void STL_OpenActiveContours<DataT>::append_raw_circles_to_vector( std::vector< CT_Circle* >& in_out_circle_vector) const
{
    int n_points = get_n_points();

    for( int i = 0 ; i < n_points ; i++ )
    {
        const Eigen::Vector3d& point = _points.row(i);
        in_out_circle_vector.push_back( new CT_Circle( new CT_CircleData(point.tail(3), Eigen::Vector3d::UnitZ(), _grid3d->getGridRayLength()->valueAtXYZ(point.x(),point.y(),point.z()))));
    }
}

template< class DataT >
void STL_OpenActiveContours<DataT>::markRepulsion( double repulse_factor )
{
    // //int dimw = _repulse_image->wdim();
    // Eigen::Vector3i grid_dim = _grid3d->dim();

    // // Pour chacun des pixels du contour on marque sa repulsion
    // //
    // // Les elements qui intersectent curPix verifient : r_p + r_elt < dist( p, elt )
    // // On a une bbox des elements qui intersectent pour chaque rayon
    // // Elle est donnee par : (p+(r_elt+r_p)) et (p-(r_elt+r_p))
    // int n_points = get_n_points();
    // for( int i = 0 ; i < n_points ; i++ )
    // {
    //     const Eigen::Vector3d& curr_point = _points.row(i);
    //     const double curr_radius = curr_point[0];

    //     // Si son pixel n'est pas deja marque alors on fait la marque autour, sinon on ne fait rien
    //     Eigen::Vector3i currPix = _grid3d->cartesian_to_pixel(curr_point);

    //     // À revoir!
    //     // Pour chaque rayon de l'espace de Hough on calcule la bbox des elements qui intersectent le cercle represente par curr_point
    //     for ( int currW = 0 ; currW < dimw ; currW++ )
    //     {
    //         // On calcule le rayon cartesien des elements de ce rayon
    //         double r_elt = _grid3d->pixelToCartesianW( currW );

    //         // Ainsi on a acces a (r_elt+r_p) et on peut avoir la bbox en cartesien
    //         // i.e. curr_point - (r_elt+r_p) et curr_point + (r_elt+r_p)
    //         double sum_radius = r_elt + curr_radius;
    //         double sum_radius_times_repulse_factor = repulse_factor*sum_radius;

    //         Eigen::Vector3d bott(r_elt,
    //                    curr_point[1] - sum_radius_times_repulse_factor,
    //                    curr_point[2] - sum_radius_times_repulse_factor,
    //                    curr_point[3] - sum_radius_times_repulse_factor);
    //         Eigen::Vector3d topp(r_elt,
    //                    curr_point[1] + sum_radius_times_repulse_factor,
    //                    curr_point[2] + sum_radius_times_repulse_factor,
    //                    curr_point[3] + sum_radius_times_repulse_factor);

    //         // On transforme la bbox cartesienne en bbox de la grille
    //         // On elargi la bbox d'un facteur du rayon de repulsion
    //         Eigen::Vector3i bot = _grid3d->cartesian_to_pixel(r_elt,
    //                                                      curr_point[1] - sum_radius_times_repulse_factor,
    //                                                      curr_point[2] - sum_radius_times_repulse_factor,
    //                                                      curr_point[3] - sum_radius_times_repulse_factor);
    //         Eigen::Vector3i top = _grid3d->cartesian_to_pixel(r_elt,
    //                                                      curr_point[1] + sum_radius_times_repulse_factor,
    //                                                      curr_point[2] + sum_radius_times_repulse_factor,
    //                                                      curr_point[3] + sum_radius_times_repulse_factor);

    //         // On met a jour la bbox pour etre sur de ne pas aller hors de l'espace de Hough (perte de temps dans le parcours car beaucoup de tests potentiels de cellules hors de l'espace)
    //         for( int axe = 0 ; axe < 4 ; axe++ )
    //         {
    //             if( bot[axe] < 0 )
    //             {
    //                 bot[axe] = 0;
    //             }
    //             if( bot[axe] >= grid_dim[axe] )
    //             {
    //                 bot[axe] = grid_dim[axe] - 1;
    //             }
    //             if( top[axe] < 0 )
    //             {
    //                 top[axe] = 0;
    //             }
    //             if( top[axe] >= grid_dim[axe] )
    //             {
    //                 top[axe] = grid_dim[axe] - 1;
    //             }
    //         }

    //         Eigen::Vector3i curInBBox;
    //         // Il ne reste plus qu'a tester les intersections des pixels a l'interieur de la bbox
    //         // On parcour donc toute la bbox
    //         for ( curInBBox[0] = bot[0] ; curInBBox[0] <= top[0] ; curInBBox[0]++ )
    //         {
    //             for ( curInBBox[1] = bot[1]; curInBBox[1] <= top[1] ; curInBBox[1]++ )
    //             {
    //                 for ( curInBBox[2] = bot[2] ; curInBBox[2] <= top[2] ; curInBBox[2]++ )
    //                 {
    //                     for ( curInBBox[3] = bot[3] ; curInBBox[3] <= top[3]; curInBBox[3]++ )
    //                     {
    //                         _repulse_image->setValueHough( curInBBox[0], curInBBox[1], curInBBox[2], curInBBox[3], 1 );
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }
}

template< class DataT >
void STL_OpenActiveContours<DataT>::resample( double sampleRes )
{
    // On recupere la resolution 3D de l'image sur laquelle se deplace le contour
    // C'est cette resolution que l'on cherche a atteindre
    double totalLength = length3D();

    if( totalLength <= sampleRes )
    {
        return;
    }

    double currLength = 0;
    double currSegmentLength = 0;
    double nextPointLength = sampleRes;

    // On copie la matrice de points du snake avant resampling
    int noldpts = get_n_points();
    Eigen::MatrixXd oldpts = _points;

    // On redimensionne la matrice pour le resampling
    int nbpointsresampled = floor(totalLength / sampleRes) + 1;    // +1 pour la queue qu'on rajoute toujours
    int indexNewPoint = 1;

    _points.resize( nbpointsresampled, 3 );

    int currPointIndex = 0;
    Eigen::Vector3d curr_point;

    int nextPointIndex = 1;
    Eigen::Vector3d next_point;

    // On ajoute la tete qui ne bouge pas
    _points.row(0) = oldpts.row(0);

    // Tant qu'il reste de la place dans laquelle ajouter un (des) point(s)
    while( nextPointLength < totalLength && nextPointIndex < noldpts )
    {
        // On avance jusqu'a trouver le segment dans lequel inserer un (ou plusieurs) point(s)
        while( nextPointLength > currLength )
        {
            curr_point = oldpts.row( currPointIndex );
            next_point = oldpts.row( nextPointIndex );
            currPointIndex++;
            nextPointIndex++;
            currSegmentLength = (next_point-curr_point).tail(3).norm();
            currLength += currSegmentLength;
        }

        // On insere autant de points que necessaire dans ce segment
        while( currLength >= nextPointLength )
        {
            // Inserer un point au bon endroit sur le segment
            if( indexNewPoint < _points.rows() )
            {
                double distToAddToSegmentEnd = currLength - nextPointLength;
                double ratio = distToAddToSegmentEnd / currSegmentLength;

                // On prend la direction du segment
                Eigen::Vector3d segmentDir = curr_point - next_point;

                // Ajout dans la matrice de points
                _points.row( indexNewPoint ) = next_point + ratio * segmentDir;

                indexNewPoint++;
            }

            // Avancer l'abscisse du point a inserer
            nextPointLength += sampleRes;
        }
    }

    // On ajoute la queue
    _points.row(nbpointsresampled-1) = oldpts.row( noldpts-1 );
}

template< class DataT >
double STL_OpenActiveContours<DataT>::length3D() const
{
    double length = 0;
    int n_points = get_n_points();

    for( int i = 0 ; i < n_points-1 ; i++ )
    {
        length += ( _points.row(i) - _points.row(i+1)).tail(3).norm();
    }

    return length;
}

template< class DataT >
void STL_OpenActiveContours< DataT >::grow(int nIterMax,
                                        double growCoeff,
                                        double coneAngleMaxDegres,
                                        double coneSizeMaxPixels,
                                        double seuilSigmaL1)
{
    // const double minZAxisLevel = 0.35;

    // Remet les points dans l'image
    updatePoints();

    // bool firstheadmsg = true;
    // bool firstbackmsg = true;

    // On cree une matrice pour stocker la coordonnee des points a l'iteration suivante
    double sigmaL1Head = std::numeric_limits<double>::max();
    double sigmaL1Back = std::numeric_limits<double>::max();

    bool hasGrownHead = true;
    bool hasGrownBack = true;
    Eigen::Vector3d growDirHead;
    Eigen::Vector3d growDirBack;

    int i = 0;
    for( i = 0 ; i < nIterMax && ( sigmaL1Head > seuilSigmaL1 || sigmaL1Back > seuilSigmaL1 ) && ( hasGrownBack || hasGrownHead ) ; i++ )
    {
        // //Giving a chance for the fork to get proper bearings on the first circle to prevent incremental recalculation
        // if( isFork() && i == 0 )
        // {
        //     // // Calcule la force de croissance
        //     // getGrowingDirections( repulseImage,
        //     //                      90,
        //     //                      coneSizeMaxPixels,
        //     //                      seuilSigmaL1,
        //     //                      sigmaL1Head,
        //     //                      sigmaL1Back,
        //     //                      hasGrownHead,
        //     //                      hasGrownBack,
        //     //                      growDirHead,
        //     //                      growDirBack );
        // }
        // else
        // {
        // Calcule la force de croissance
        getGrowingDirections(coneAngleMaxDegres,
                             coneSizeMaxPixels,
                             seuilSigmaL1,
                             sigmaL1Head,
                             sigmaL1Back,
                             hasGrownHead,
                             hasGrownBack,
                             growDirHead,
                             growDirBack );
        // }

        if( sigmaL1Head < seuilSigmaL1 )
        {
            hasGrownHead = false;
        }

        if( sigmaL1Back < seuilSigmaL1 )
        {
            hasGrownBack = false;
        }

        // Si le snake a pousse des deux cotes, il faut ajouter deux points aux extemites
        if( hasGrownBack && hasGrownHead )
        {
            Eigen::Vector3d head = _points.row(0);
            Eigen::Vector3d new_head = head  - ( growCoeff * growDirHead );
            Eigen::Vector3d tail = _points.row(get_n_points()-1);
            Eigen::Vector3d new_tail = tail - ( growCoeff * growDirBack );

            Eigen::MatrixXd new_points( get_n_points() + 2, 3 );
            new_points.row(0)                           = new_head;
            new_points.row(get_n_points()+1)            = new_tail;
            new_points.block(1, 0, get_n_points(), 3)   = _points.block(0, 0, get_n_points(), 3);

            _points = new_points;
            updatePoints();
        }
        else if (hasGrownHead)
        {
            Eigen::Vector3d head = _points.row(0);
            Eigen::Vector3d new_head = head  - ( growCoeff * growDirHead );

            Eigen::MatrixXd new_points( get_n_points() + 1, 3 );
            new_points.row(0)                           = new_head;
            new_points.block(1, 0, get_n_points(), 3)   = _points.block(0, 0, get_n_points(), 3);

            _points = new_points;
            updatePoints();
        }
        else if (hasGrownBack)
        {
            Eigen::Vector3d tail = _points.row(get_n_points()-1);
            Eigen::Vector3d new_tail = tail - ( growCoeff * growDirBack );

            Eigen::MatrixXd new_points( get_n_points() + 1, 3 );
            new_points.row(get_n_points())              = new_tail;
            new_points.block(0, 0, get_n_points(), 3)   = _points.block(0, 0, get_n_points(), 3);

            _points = new_points;
            updatePoints();
        }
        else
        {
            // PS_LOG->addWarningMessage(LogInterface::warning, QObject::tr("No growing at all"));
        }
    }

    PS_LOG->addWarningMessage(LogInterface::warning, QObject::tr("End growing after %1 iterations").arg(i));
}

template< class DataT >
void STL_OpenActiveContours< DataT >::getGrowingDirections(double coneAngleDegres,
                                                        int coneSizePixels,
                                                        double seuilSigmaL1,
                                                        double& outSigmaL1Head,
                                                        double& outSigmaL1Back,
                                                        bool& outHasGrownHead,
                                                        bool& outHasGrownBack,
                                                        Eigen::Vector3d& outGrowDirHead,
                                                        Eigen::Vector3d& outGrowDirBack ) const
{
    // Le gradient est nul partout sauf pour la tete et la queue du contour
    // Pour plus de precision, voir le calcul des directions de croissance d'un contour
    for( int i = 0 ; i < 3 ; i++ )
    {
        outGrowDirHead(i) = 0;
        outGrowDirBack(i) = 0;
    }

    // Si le snake avait grandi en tete la derniere iteration on le refait croitre
    // Sinon il s'etait arrete pour une bonne raison et on ne le fait plus croitre
    if( outSigmaL1Head > seuilSigmaL1 && outHasGrownHead )
    {
        // Calcul de la croissance en tete
        Eigen::Vector3d head_point = _points.row(0);
        Eigen::Vector3i head_pixel = _grid3d->cartesian_to_pixel(head_point[0], head_point[1], head_point[2]);
        Eigen::Vector3d head_tangent = getTangentAtHead(true);
        head_tangent *= -1;
        outGrowDirHead = getGradientEnergyGrowProportionalResolution(head_point,
                                                                     head_pixel,
                                                                     head_tangent,
                                                                     coneAngleDegres,
                                                                     coneSizePixels,
                                                                     outSigmaL1Head,
                                                                     outHasGrownHead );
    }
    // Si le snake avait grandi en queue la derniere iteration on le refait croitre
    // Sinon il s'etait arrete pour une bonne raison et on ne le fait plus croitre
    if( outSigmaL1Back > seuilSigmaL1 && outHasGrownBack )
    {
        // Calcul de la croissance en queue
        Eigen::Vector3d tail_point   = _points.row( get_n_points() - 1 );
        Eigen::Vector3i tail_pix     = _grid3d->cartesian_to_pixel(tail_point[0], tail_point[1], tail_point[2]);
        Eigen::Vector3d tail_tangent = getTangentAtTail(true);
        outGrowDirBack = getGradientEnergyGrowProportionalResolution(tail_point,
                                                                     tail_pix,
                                                                     tail_tangent,
                                                                     coneAngleDegres,
                                                                     coneSizePixels,
                                                                     outSigmaL1Back,
                                                                     outHasGrownBack );
    }
}

template< class DataT >
Eigen::Vector3d STL_OpenActiveContours< DataT >::getGradientEnergyGrowProportionalResolution(const Eigen::Vector3d& snakeExtremityPoint,
                                                                                          const Eigen::Vector3i& snakeExtremityPix,
                                                                                          const Eigen::Vector3d &snakeTangentAtExtremity,
                                                                                          double coneAngleDegres,
                                                                                          int coneSizePixels,
                                                                                          double& out_sigma_l1,
                                                                                          bool& outHasGrown ) const
{
    // Calcule la bbox correspondant a un cube centre autour de l'extremite du snake et de taille coneSizePix
    Eigen::Vector3i coneBBoxBot = snakeExtremityPix - Eigen::Vector3i( coneSizePixels, coneSizePixels, coneSizePixels);
    Eigen::Vector3i coneBBoxTop = snakeExtremityPix + Eigen::Vector3i( coneSizePixels, coneSizePixels, coneSizePixels);
    Eigen::Vector3i grid_dim = _grid3d->dim();

    for( int axe = 0 ; axe < 3 ; axe++ )
    {
        if( coneBBoxBot[axe] < 0 )
        {
            coneBBoxBot[axe] = 0;
        }
        if( coneBBoxBot[axe] >= grid_dim[axe] )
        {
            coneBBoxBot[axe] = grid_dim[axe] - 1;
        }
        if( coneBBoxTop[axe] < 0 )
        {
            coneBBoxTop[axe] = 0;
        }
        if( coneBBoxTop[axe] >= grid_dim[axe] )
        {
            coneBBoxTop[axe] = grid_dim[axe] - 1;
        }
    }

    // // Il ne reste plus qu'a tester les angles des directions a l'interieur de la bbox
    std::vector< std::pair< Eigen::Vector3d,DataT> > potential_grow_dirs_and_scores;
    get_dir_and_score_inside_cone(snakeExtremityPix,
                                  snakeExtremityPoint,
                                  snakeTangentAtExtremity,
                                  coneAngleDegres,
                                  coneSizePixels,
                                  5,
                                  100,
                                  static_cast<DataT>(10),
                                  true,
                                  potential_grow_dirs_and_scores);

    if( potential_grow_dirs_and_scores.size() < 5 )
    {
        outHasGrown = false;
        return Eigen::Vector3d::Zero();
    }

    // Effectue l'ACP 3D non centree des directions de croissance potentielles
    Eigen::Vector3d  v1, v2, v3;
    double l1, l2, l3;

    uncenteredPCA(potential_grow_dirs_and_scores,
                  v1, v2, v3,
                  l1, l2, l3);

    // Calcul du sigmaL1 pour cette ACP
    out_sigma_l1 = l1 / (l1+l2+l3);

    // On normalise la direction v1
    if( v1.norm() != 0 )
    {
        v1.normalize();
    }
    Eigen::Vector3d v1_normalized_3d = v1.tail(3).normalized();

    Eigen::Vector3d tangent_3d = snakeTangentAtExtremity.tail(3);

    // Produit scalaire entre la direction de croissance et la tangente au snake pour calculer la projection
    double scalr_prod = v1_normalized_3d.dot( tangent_3d );

    // Le vecteur v1 en sortie d'ACP est pas forcément orienté vers l'extremite du snake
    // On le réoriente en lui donnant la même orientation que la tangente au snake (verification du signe du produit scalaire)
    if( scalr_prod < 0 )
    {
        v1 *= -1;
    }

    // On rescale la direction de croissance conformement a la resolution de l'espace de Hough
    v1.normalize();
    v1 *= _grid3d->resolution();

    // Le gradient de l'energie de croissance est defini comme la projection de ce vecteur sur la tangente a la tete
    // Donc on multiplie cette direction par le produit scalaire avec la tangente
    Eigen::Vector3d grow_dir = v1 * fabs( scalr_prod );

    // Renvoie l'oppose de la direction de croissance
    return ( grow_dir * -1 );
}

template< class DataT >
Eigen::Vector3d STL_OpenActiveContours< DataT >::getTangentAtPoint( int i, bool normalize ) const
{
    Eigen::Vector3d rslt;
    Eigen::Vector3d curr = _points.row(i);

    if( i == 0 )
    {
        // Tangente par difference finie
        Eigen::Vector3d next = _points.row( i+1 );
        rslt = next - curr;
    }
    else if( i == get_n_points() - 1 )
    {
        // Tangente par difference finie
        Eigen::Vector3d prev = _points.row( i-1 );
        rslt = curr - prev;
    }
    else
    {
        // Tangente par difference finie des positions
        Eigen::Vector3d prev = _points.row( i-1 );
        Eigen::Vector3d next = _points.row( i+1 );
        rslt = next - prev;
    }

    // Normalise la tangente
    if( normalize )
    {
        rslt.normalize();
    }

    return rslt;
}

template< class DataT >
void STL_OpenActiveContours< DataT >::updatePoints()
{
    // On regarde le statut de chaque point
    int n_pts = get_n_points();
    Eigen::Vector3d bbox_bot = _grid3d->getBot();
    Eigen::Vector3d bbox_top = _grid3d->getTop();
    for( int i = 0 ; i < n_pts ; i++ )
    {
        for( int coord = 0 ; coord < 3 ; coord++ )
        {
            if( _points(i, coord) < bbox_bot[coord] )
            {
                _points(i, coord) = bbox_bot[coord] + 0.00001;
            }
            if( _points(i, coord) > bbox_top[coord] )
            {
                _points(i, coord) = bbox_top[coord] - 0.00001;
            }
        }
    }
}
template< class DataT >
void STL_OpenActiveContours< DataT >::getGeometricMatrix(double alpha, double beta, Eigen::MatrixXd& outGeometricMatrix) const
{
    int npts = get_n_points();

    outGeometricMatrix = Eigen::MatrixXd::Zero(npts, npts);

    // On remplit toutes les lignes sauf les deux de chaque bord (conditions aux bords)
    for( int i = 0 ; i < npts ; i++ )
    {
        if( i >= 2 && i < npts-2 )
        {
            if( i-2 >= 0 )
            {
                outGeometricMatrix(i,i-2) = beta;
            }
            if( i-1 >= 0 )
            {
                outGeometricMatrix(i,i-1) = -alpha - 3*beta;
            }

            outGeometricMatrix(i,i) = 2*alpha + 6*beta ;

            if( i+1 <= npts-1 )
            {
                outGeometricMatrix(i,i+1) = -alpha - 3*beta;
            }
            if( i+2 <= npts-1 )
            {
                outGeometricMatrix(i,i+2) = beta;
            }
        }
    }
}

template< class DataT >
void STL_OpenActiveContours< DataT >::getAPlusRhoIInverse(double alpha, double beta, double timeStep, Eigen::MatrixXd& outAPlusRhoIInverse ) const
{
    // Calcul de A+rho*I
    int npts    = get_n_points();
    double rho  = 1.0 / timeStep;

    // Matrice identite I
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(npts, npts);

    // Matrice geometrique A
    Eigen::MatrixXd matrixA;
    getGeometricMatrix( alpha, beta, matrixA );

    // Calcul de l'inverse voulu
    Eigen::MatrixXd aPlusRhoI = matrixA + ( rho * identity );

    outAPlusRhoIInverse = aPlusRhoI.inverse();
}

template< class DataT >
void STL_OpenActiveContours< DataT >::updatePointsAndGetAverageAndStDevMovement(const Eigen::MatrixXd &newPoints,
                                                                             double& outAverageMovemet,
                                                                             double& outStdDevMovement )
{
    int npts = get_n_points();

    Eigen::Vector3d bbox_bot = _grid3d->getBot();
    Eigen::Vector3d bbox_top = _grid3d->getTop();

    double movement;
    double movementSum = 0;
    double movementSquareSum = 0;

    // On regarde le statut de chaque point
    Eigen::Vector3d curPoint;
    Eigen::Vector3i curPix;
    for( int i = 0 ; i < npts ; i++ )
    {
        curPoint = newPoints.row(i);
        _grid3d->cartesian_to_pixel(curPoint[0], curPoint[1], curPoint[2]);

        movement = (Eigen::Vector3d(_points.row(i)) - curPoint).norm();
        movementSum += movement;
        movementSquareSum += movement*movement;

        if( _grid3d->isPixelIn(curPix) )
        {
            // La nouvelle position du point est dans l'image, on fait une simple copie
            _points.row(i) = newPoints.row(i);
        }
        else
        {
            for( int coord = 0 ; coord < 3 ; coord++ )
            {
                if( curPoint[coord] < bbox_bot[coord] )
                {
                    _points(i, coord) = bbox_bot[coord] + 0.00001;
                }
                else if( curPoint[coord] > bbox_top[coord] )
                {
                    _points(i, coord) = bbox_top[coord] - 0.00001;
                }
                else
                {
                    _points(i, coord) = curPoint[coord];
                }
            }
        }
    }

    outAverageMovemet = movementSum / npts;
    outStdDevMovement = ( movementSquareSum / npts ) - (outAverageMovemet*outAverageMovemet);
}

template< class DataT >
void STL_OpenActiveContours< DataT >::relax(int nIterMax,
                                         double alpha, double beta, double gama,
                                         double globalWeight,
                                         double timeStep,
                                         double threshaverageMovement3D )
{
    // On recupere le premier terme du systeme matriciel
    Eigen::MatrixXd aPlusRhoIInverse;
    getAPlusRhoIInverse( alpha, beta, timeStep, aPlusRhoIInverse);

    // On cree une matrice pour stocker la coordonnee des points a l'iteration suivante
    int npts = get_n_points();

    double rho = 1.0/timeStep;
    // À revoir (4 ou 3?)
    Eigen::MatrixXd pointsNextIteration( npts, 3 );

    double averageMovement3D = std::numeric_limits<double>::max();
    double stDevMovement3D = std::numeric_limits<double>::max();

    int iNIterMax = nIterMax+1;

    int relaxStep;
    for( relaxStep = 1 ; relaxStep < iNIterMax && averageMovement3D > threshaverageMovement3D ; relaxStep++ )
    {
        // Copy points at current iteration
        Eigen::MatrixXd savePoints =  _points;

        // Calcule les gradients d'energie image et d'energie de croissance
        QFuture<Eigen::MatrixXd*> futureGradientEnergyImage = QtConcurrent::run( this, &STL_OpenActiveContours< DataT >::getGradientEnergyOrthogonalImageMatrixMultipliedByTangentNorm, globalWeight );
        QFuture<Eigen::MatrixXd*> futureSecondDiffOnTangentDivByTanNorm = QtConcurrent::run( this, &STL_OpenActiveContours< DataT >::getSecondDifferentialOnTangentDividedByTangentNormMultiplyedByImageEnergy, globalWeight );
        futureGradientEnergyImage.waitForFinished();
        futureSecondDiffOnTangentDivByTanNorm.waitForFinished();

        Eigen::MatrixXd* gradientEnergyImage = futureGradientEnergyImage.result();
        Eigen::MatrixXd* secondDiffOnTangentDivByTanNorm = futureSecondDiffOnTangentDivByTanNorm.result();

        // Calcul de la position a la prochaine iteration
        pointsNextIteration = aPlusRhoIInverse * ( ( rho * _points ) - ( gama * (*gradientEnergyImage) ) + ( (*secondDiffOnTangentDivByTanNorm) ) );

        //        (*pointsNextIteration) = (*rhoIMoinsAInverse) * ( ( rho * _points ) + ( gama * (*gradientEnergyImage) ) - ( (*secondDiffOnTangentDivByTanNorm) ) );
        // On met a jour la position des points :
        // Pour lespoints dans l'image on fait une simple copie
        // Pour les points hors de l'image on les remet dans le dernier pixel traverse par la direction de mouvement du point
        updatePointsAndGetAverageAndStDevMovement(pointsNextIteration,
                                                  averageMovement3D,
                                                  stDevMovement3D );

        // Libere la memoire
        delete gradientEnergyImage;
        delete secondDiffOnTangentDivByTanNorm;
        //        // Toutes les 10 iterations on regarde si on doit resampler
        //        if( i%10 == 0 && hasToBeResampled() )
        //        {
        //            resample( _image->xres() );
        //            // Il faut donc mettre a jour la matrice de geometrie (les matrices des gradients sont mises a jour en debut de boucle)
        //            delete aPlusRhoIInverse;
        //            aPlusRhoIInverse = getAPlusRhoIInverse( alpha, beta, timeStep );
        //            // Il faut aussi mettre a jour la matrice des points a l'iteration suivante (adapter sa taille)
        //            pointsNextIteration->resize( npoints(), npoints() );
        //        }
    }

    updatePoints();
}

template< class DataT >
Eigen::MatrixXd* STL_OpenActiveContours< DataT >::getGradientEnergyOrthogonalImageMatrixMultipliedByTangentNorm( double globalWeight ) const
{
    int npts = get_n_points();

    Eigen::MatrixXd* rslt = getGradientEnergyOrthogonalImageMatrix( globalWeight );

    double curTangentNorm;

    for( int i = 0 ; i < npts; i++ )
    {
        curTangentNorm = getTangentAtPoint(i, false).norm();
        (*rslt)(i,0) *= curTangentNorm;
        (*rslt)(i,1) *= curTangentNorm;
        (*rslt)(i,2) *= curTangentNorm;
    }

    return rslt;
}

template< class DataT >
Eigen::MatrixXd* STL_OpenActiveContours< DataT >::getGradientEnergyOrthogonalImageMatrix( double globalWeight ) const
{
    int npts = get_n_points();
        // À revoir (4 ou 3?)
    Eigen::MatrixXd* rslt = new Eigen::MatrixXd( npts, 3 );

    // Calcul du gradient de l'energie d'image en chaque point du contours
    // Classiquement : Pour plus de precision ce gradient se calcule par interpolation lineaire
    //                 c.f. methode gradientEnergyImageAtPoint( int i, double globalWeight )
    // Maintenant : on ne considere que la partie de ce gradient orthogonale au snake
    for( int i = 0 ; i < npts ; i++ )
    {
        QFuture<Eigen::Vector3d> futureCurrGrad = QtConcurrent::run( this, &STL_OpenActiveContours< DataT >::gradientEnergyImageInterpolatedAtPoint, i, globalWeight );
        QFuture<Eigen::Vector3d> futureDirectionContours = QtConcurrent::run( this, &STL_OpenActiveContours< DataT >::directionContoursAtPoint, i );

        futureCurrGrad.waitForFinished();
        futureDirectionContours.waitForFinished();

        Eigen::Vector3d currGrad = futureCurrGrad.result();
        Eigen::Vector3d directionContour = futureDirectionContours.result().normalized();

        // On peut calculer la composante orthogonale de currGrad a directionContours
        currGrad = currGrad - ( directionContour * currGrad.dot( directionContour ) );

        for( int j = 0 ; j < 3 ; j++ )
        {
            (*rslt)( i, j ) = currGrad(j);
        }
    }

    return rslt;
}

template< class DataT >
Eigen::Vector3d STL_OpenActiveContours< DataT >::gradientEnergyImageInterpolatedAtPoint(int indexPoint, double globalWeight) const
{
    Eigen::Vector3d currPoint         = _points.row(indexPoint);                                                                  // ieme point du contour
    Eigen::Vector3i currPixel         = _grid3d->cartesian_to_pixel(currPoint[0], currPoint[1], currPoint[2]); // Pixel contenant le ieme point
    Eigen::Vector3d currPixelCenter   = _grid3d->pixel_to_cartesian(currPixel[0], currPixel[1], currPixel[2]); // Centre en cartesien du pixel contenant le ieme point

    // Pour plus de precision et de stabilite, le gradient de l'energie image est interpole lineairement entre deux valeurs :
    // - si le point courant est dans la moitie gauche du pixel on interpole le gradient entre
    //   Grad( prevPixel ) et Grad( currPixel )
    // - si il est dans la partie droite l'interpolation se fait entre
    //   Grad( currPixel ) et Grad( nextPixel )
    // Interpolation des gradients selon chaque coordonnee
    Eigen::Vector3d gradInterpolated;
    for( int i = 0 ; i < 3 ; i++ )
    {
        // On recupere la coordonnee i du gradient d'energie au pixel courant (on est sur que le pixel est dans l'image)
        double curPixelGrad = partialDerivativeEnergyImageAtPixel( currPixel, globalWeight, i );

        // On regarde si le point courrant est dans la partie gauche ou droite du pixel courant
        if( currPoint[i] < currPixelCenter[i] )
        {
            Eigen::Vector3i prevPixel         = currPixel; prevPixel[i] -= 1;
            Eigen::Vector3d prevPixelPoint    = _grid3d->pixel_to_cartesian(prevPixel[0], prevPixel[1], prevPixel[2]);
            double prevPixelCenter  = prevPixelPoint[i];

            // On calcule la coordonnee i du gradient d'image au pixel precedent
            double prevPixelGrad = partialDerivativeEnergyImageAtPixel( prevPixel, globalWeight, i );

            // Partie gauche, interpolation avec gradPrevPixel
            gradInterpolated[i] = StepTools::linearInterpolation( prevPixelCenter, currPixelCenter[i], prevPixelGrad, curPixelGrad, currPoint[i] );
        }
        else
        {
            Eigen::Vector3i nextPixel         = currPixel;  nextPixel[i] += 1;
            Eigen::Vector3d prevPixelPoint    = _grid3d->pixel_to_cartesian(nextPixel[0], nextPixel[1], nextPixel[2]);
            double nextPixelCenter  = prevPixelPoint[i];

            // On calcule la coordonnee i du gradient d'image au pixel precedent
            double nextPixelGrad = partialDerivativeEnergyImageAtPixel( nextPixel, globalWeight, i );

            // Partie droite, interpolation avec gradNextPixel
            gradInterpolated[i] = StepTools::linearInterpolation( currPixelCenter[i], nextPixelCenter, curPixelGrad, nextPixelGrad, currPoint[i] );
        }
    }
    return gradInterpolated;
}

template< class DataT >
double STL_OpenActiveContours< DataT >::partialDerivativeEnergyImageAtPixelInImage(const Eigen::Vector3i &p, double globalWeight, int coordToDerive) const
{
    // On es sur que le pixel p soit dans l'image (peut etre est ce un bord quand meme)
    double partialDerivate;

    // Calcul de la derivee partielle selon la variable demandee
    // On recupere le pixel precedent selon cette variable
    Eigen::Vector3i prevInDirection = p; prevInDirection[coordToDerive] -= 1;
    bool isPrevInImage = _grid3d->isPixelIn(prevInDirection);

    // On recupere le pixel suivant selon cette variable
    Eigen::Vector3i nextInDirection = p; nextInDirection( coordToDerive ) += 1;
    bool isNextInImage = _grid3d->isPixelIn(nextInDirection);

    // Si aucun des deux pixels (suivant et precedent) n'est dans l'image, on renvoie une derivee partielle nulle
    // i.e. l'image n'a qu'un seul pixel d'epaisseur selon cette variable
    if( !isPrevInImage && !isNextInImage )
    {
        partialDerivate = 0.0;
    }

    // Si les deux pixels (suivant et precedent) sont dans l'image, on calcule la derivee partielle par difference centree
    else if( isPrevInImage && isNextInImage )
    {
        partialDerivate = ( energyImageAtPixel( nextInDirection, globalWeight ) - energyImageAtPixel( prevInDirection, globalWeight ) ) / ( 2.0 );
    }

    // Si seul le pixel precedent est dans l'image (on a atteint le bord droit de l'image), on calcule la derivee partielle par difference a gauche (curr - prev)
    else if( isPrevInImage && !isNextInImage )
    {
        partialDerivate = ( energyImageAtPixel( p, globalWeight ) - energyImageAtPixel( prevInDirection, globalWeight ) );
    }

    // Si seul le pixel suivant est dans l'image (on a atteint le bord gauche de l'image), on calcule la derivee partielle par difference a droite (next - curr)
    else
    {
        partialDerivate = ( energyImageAtPixel( nextInDirection, globalWeight ) - energyImageAtPixel( p, globalWeight ) );
    }

    return partialDerivate;
}

template< class DataT >
double STL_OpenActiveContours< DataT >::partialDerivativeEnergyImageAtPixelOutsideImage(const Eigen::Vector3i& p, double globalWeight, int coordToDerive) const
{
    Eigen::Vector3i hough_space_dim = _grid3d->dim();

    if( hough_space_dim[coordToDerive] <= 2 )
    {
        PS_LOG->addFatalMessage(LogInterface::fatal, "Impossible de calculer une derivee sur une grille qui n'a qu'un élément. On fait planter computree intentionellement (TODO: il aurait fallu passer par des exceptions).");
        exit( EXIT_FAILURE );
    }

    // On est sur un pixel hors de l'image
    double partialDerivate = 0;

    // Ce cas ne devrait arriver que lorsqu'on essaie de calculer la derivee partielle d'un pixel au bord de l'image
    // avec le point P entre le centre du pixel et le bord
    // i.e. le point du snake est situe entre le centre du dernier pixel de l'image et le bord de l'image
    //
    // Dans ce cas, lorsqu'on va vouloir faire l'interpolation de la derivee partielle de l'energie d'image au pixel du point P
    // on va vouloir interpoler la valeur entre le pixel du point P et le pixel hors de l'image (celui qui est traite par cette methode)
    // C'est ce cas de figure que l'on traite ici
    // On traite ce cas en renvoyant un mirroir des derivees partielles de l'energie image
    // i.e. si le pixel est a gauche de l'image on renvoi la derivee partielle en 1
    //      si le pixel est a gauche de l'image on renvoi la derivee partielle en dim-2
    if( p[coordToDerive] < 0 )
    {
        // On est a gauche de l'image
        // On renvoit l'oppose de la derivee partielle en 1
        Eigen::Vector3i pixel1 = p;
        pixel1[coordToDerive] = 1;  // Ce pixel est obligatoirement sur l'image car le seul appel possible a cette methode est
            // si on demande la derivee partielle interpolee a un pixel dans l'image
        partialDerivate = - partialDerivativeEnergyImageAtPixelInImage( pixel1, globalWeight, coordToDerive );
    }
    else
    {
        // On est a droite de l'image
        // On renvoit l'oppose de la derivee partielle en dim-2
        Eigen::Vector3i pixelMinus2 = p;
        pixelMinus2[coordToDerive] = hough_space_dim[coordToDerive] - 2;  // Ce pixel est obligatoirement sur l'image car le seul appel possible a cette methode est
            // si on demande la derivee partielle interpolee a un pixel dans l'image
        partialDerivate = - partialDerivativeEnergyImageAtPixelInImage( pixelMinus2, globalWeight, coordToDerive );
    }

    return partialDerivate;
}

template< class DataT >
double STL_OpenActiveContours< DataT >::energyImageLocalAtPixel(const Eigen::Vector3i& p) const
{
    DataT min, max;

    // Recupere les min et max du voisinage
    _grid3d->getMinMaxInNeighbourhood(p, min, max, 1);

    if( min == max )
    {
        return -1;
    }

    // Renvoie l'energie image locale
    return static_cast<double>( min - _grid3d->value(p[0], p[1], p[2]) ) / static_cast<double>( max - min );
}

template< class DataT >
Eigen::MatrixXd* STL_OpenActiveContours< DataT >::getSecondDifferentialOnTangentDividedByTangentNormMultiplyedByImageEnergy( double globalWeight ) const
{
    int npts = get_n_points();

    Eigen::MatrixXd* rslt = new Eigen::MatrixXd( npts, 3 );
    Eigen::Vector3d curTangent;
    Eigen::Vector3d curSecondDiff;
    double tangentNorm;
    double scalProd;
    double scalProdNormalized;

    for( int i = 0 ; i < npts; i++ )
    {
        const Eigen::Vector3d& point  = _points.row(i);
        curTangent          = getTangentAtPoint(i, true);
        curSecondDiff       = getSecondDifferentialAtPoint(i);
        tangentNorm         = curTangent.norm();
        scalProd            = curTangent.dot(curSecondDiff);
        scalProdNormalized  = scalProd / ( tangentNorm * tangentNorm );

        for( int c = 0 ; c < 3 ; c++ )
        {
            Eigen::Vector3i pixel_at_i = _grid3d->cartesian_to_pixel(point[0], point[1], point[2]);
            (*rslt)(i, c) = ( ( curSecondDiff[c] - ( scalProdNormalized * curTangent[c] ) ) / tangentNorm ) * energyImageAtPixel( pixel_at_i, globalWeight );
        }
    }

    return rslt;
}

template< class DataT >
Eigen::Vector3d STL_OpenActiveContours< DataT >::directionContoursAtPoint(int indexPoint) const
{
    Eigen::Vector3d rslt;

    if( indexPoint == 0 )
    {
        rslt = getTangentAtHead(true);
    }

    else if ( indexPoint == get_n_points() - 1 )
    {
        rslt = getTangentAtTail(true);
    }

    else
    {
        rslt = _points.row( indexPoint+1 ) - _points.row( indexPoint-1 );
    }

    return rslt;
}

template< class DataT >
void uncenteredPCA(const std::vector< std::pair< Eigen::Vector3d, DataT> >& dir_score_pairs,
                   Eigen::Vector3d& out_v1, Eigen::Vector3d& out_v2, Eigen::Vector3d& out_v3,
                   double& out_l1, double& out_l2, double& out_l3)
{
    // Conserve que les n_max_dir_to_keep plus fortes directions
    if( dir_score_pairs.size() < 3 )
    {
        PS_LOG->addErrorMessage(LogInterface::error, QObject::tr("STL_OpenActiveContours::uncenteredPCA: not enough directions for pca"));
        exit( EXIT_FAILURE );
    }

    int n_dirs = dir_score_pairs.size();

    // On cree la matrice 3*n_dirs et on la remplit (conversion du tableau de points en matrice en fait)
    Eigen::MatrixXd dir_matrix( 3, n_dirs );
    for( int i = 0 ; i < n_dirs ; i++ )
    {
        dir_matrix.col(i) = dir_score_pairs[i].first;
    }

    // On calcule la matrice de covariance des points
    Eigen::Matrix3d cov = dir_matrix * dir_matrix.transpose();

    // On calcule les valeurs propres et les vecteurs propres de la matrice de covariance
    Eigen::EigenSolver< Eigen::Matrix3d > eigenSolver;
    eigenSolver.compute( cov );
    Eigen::Vector3d eigenValues = eigenSolver.eigenvalues().real();
    Eigen::Matrix3d eigenVectors = eigenSolver.eigenvectors().real();  // Les colonnes de cette matrice sont les vecteurs propres associes aux valeurs propres

    // Il faut trier les valeurs propres et permuter les vecteurs propres en accordance
    // On fait un tri decroissant, i.e. les plus grandes valeurs seront a gauche du tableau
    // On fait un tri brut, pour un vecteur de 3 valeurs ce n'est pas grave
    double max_value;
    double curr_value;
    double swap_value;
    int    max_id;
    Eigen::Vector3d swap_vec;

    // Pour chaque valeur
    for ( int i = 0 ; i < 3 ; i++ )
    {
        max_id = i;
        max_value = eigenValues(i);

        // On la compare a toutes les autres restantes
        for ( int j = i+1 ; j < 3 ; j++ )
        {
            curr_value = eigenValues(j);
            if( curr_value > max_value )
            {
                max_value = curr_value;
                max_id= j;
            }
        }

        swap_value = eigenValues(i);
        swap_vec = eigenVectors.col(i);
        eigenValues(i) = eigenValues(max_id);
        eigenVectors.col(i) = eigenVectors.col(max_id);
        eigenValues(max_id) = swap_value;
        eigenVectors.col(max_id) = swap_vec;
    }

    // On peut transformer le tout en pixels de sortie
    out_l1 = eigenValues[0];
    out_l2 = eigenValues[1];
    out_l3 = eigenValues[2];

    out_v1 << eigenVectors(0,0), eigenVectors(1,0), eigenVectors(2,0);
    out_v2 << eigenVectors(0,1), eigenVectors(1,1), eigenVectors(2,1);
    out_v3 << eigenVectors(0,2), eigenVectors(1,2), eigenVectors(2,2);
}

#endif // STL_OPENACTIVECONTOURS_HPP
