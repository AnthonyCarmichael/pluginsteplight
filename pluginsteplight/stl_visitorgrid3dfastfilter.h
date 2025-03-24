#ifndef STL_VISITORGRID3DFASTFILTER_H
#define STL_VISITORGRID3DFASTFILTER_H

#include "stl_abstractvisitorgrid3d.h"
#include "ct_itemdrawable/ct_beam.h"

template< class DataT >
class STL_Grid3D;

template< class DataT >
class STL_VisitorGrid3DFastFilter: public STL_AbstractVisitorGrid3D<DataT>
{
    using SuperClass                = STL_AbstractVisitorGrid3D<DataT>;
    using Grid3D                    = STL_Grid3D<DataT>;
    using Grid3DPtr                 = Grid3D*;
    using Grid3DPtrConst            = Grid3DPtr const;
    using ConstGrid3D               = const Grid3D;
    using ConstGrid3DPtr            = ConstGrid3D*;
    using ConstGrid3DPtrConst       = ConstGrid3DPtr const;

public:
    /*!
     * \brief STL_VisitorGrid3DFastFilter
     *
     * Constructeur
     *
     * \param grid : grille que le visiteur viste
     */
    STL_VisitorGrid3DFastFilter(ConstGrid3DPtrConst grid_3d_ptr);

    /*!
      * Destructeur (rien a faire, il ne doit pas liberer l'image qu'il visite!!)
      */
    virtual ~STL_VisitorGrid3DFastFilter();

    /*!
     * \brief visit
     *
     * \param levx : coordonnee du pixel a visiter
     * \param levy : coordonnee du pixel a visiter
     * \param levz : coordonnee du pixel a visiter
     * \param beam : rayon qui traverse la grille
     */
    virtual void visit(size_t levx, size_t levy, size_t levz, const CT_Beam* const beam) override;

    int sumOfVisitedVotes() const;

    void setSumOfVisitedVotes(int sumOfVisitedVotes);

protected :
    ConstGrid3DPtrConst     _grid_3d_ptr;
    int                     _sumOfVisitedVotes;
};

#include "stl_visitorgrid3dfastfilter.hpp"
#endif // STL_VISITORGRID3DFASTFILTER_H
