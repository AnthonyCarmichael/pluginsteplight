#ifndef STL_ABSTRACTVISITORGRID3D_H
#define STL_ABSTRACTVISITORGRID3D_H

#include "ct_itemdrawable/ct_beam.h"

template< class DataT >
class STL_AbstractVisitorGrid3D
{
public:
    virtual void visit(size_t levx, size_t levy, size_t levz, const CT_Beam* const beam) = 0;
};

#endif // STL_ABSTRACTVISITORGRID3D_H
