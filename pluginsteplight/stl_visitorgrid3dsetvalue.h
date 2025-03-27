#ifndef STL_VISITORGRID3DSETVALUE_H
#define STL_VISITORGRID3DSETVALUE_H

// Inherits from abstract visitor
#include "ct_itemdrawable/tools/gridtools/ct_abstractgrid3dbeamvisitor.h"
#include "ct_itemdrawable/ct_grid3d.h"

class STL_VisitorGrid3DSetValue : public CT_AbstractGrid3DBeamVisitor
{

public:
    /*!
     * \brief STL_VisitorGrid3DSetValue
     *
     * Constructeur
     *
     * \param grid : grille que le visiteur visite
     */
    STL_VisitorGrid3DSetValue(CT_Grid3D<int> *grid, int value_to_set);

    /*!
      * Destructeur (rien a faire, il ne doit pas liberer l'image qu'il visite!!)
      */
    virtual ~STL_VisitorGrid3DSetValue();

    /*!
     * \brief visit
     *
     * \param levx : coordonnee du pixel a visiter
     * \param levy : coordonnee du pixel a visiter
     * \param levz : coordonnee du pixel a visiter
     * \param beam : rayon qui traverse la grille
     */
    virtual void visit(const size_t &index, const CT_Beam *beam);

protected :
    CT_Grid3D<int>*     _grid;
    int                 _value_to_set;
};


#endif // STL_VISITORGRID3DSETVALUE_H
