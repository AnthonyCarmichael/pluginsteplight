#ifndef STL_VISITORGRID3DFASTFILTER_H
#define STL_VISITORGRID3DFASTFILTER_H

// Inherits from abstract visitor
#include "ct_itemdrawable/ct_grid3d.h"
#include "ct_itemdrawable/ct_beam.h"
#include "ct_itemdrawable/tools/gridtools/ct_abstractgrid3dbeamvisitor.h"

class STL_VisitorGrid3DFastFilter: public CT_AbstractGrid3DBeamVisitor
{

public:
    /*!
     * \brief STL_VisitorGrid3DFastFilter
     *
     * Constructeur
     *
     * \param grid : grille que le visiteur viste
     */
    STL_VisitorGrid3DFastFilter(CT_Grid3D<int> *grid);

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
    virtual void visit(const size_t &index, const CT_Beam *beam);

    int sumOfVisitedVotes() const;

    void setSumOfVisitedVotes(int sumOfVisitedVotes);

private :
    CT_Grid3D<int>*         _grid;
    int                     _sumOfVisitedVotes;
};

#endif // STL_VISITORGRID3DFASTFILTER_H
