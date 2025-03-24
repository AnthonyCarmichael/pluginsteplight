#ifndef STL_VISITORGRID3DFASTFILTER_HPP
#define STL_VISITORGRID3DFASTFILTER_HPP

#include "stl_visitorgrid3dfastfilter.h"

template< class DataT >
STL_VisitorGrid3DFastFilter<DataT>::STL_VisitorGrid3DFastFilter(ConstGrid3DPtrConst grid_3d_ptr) :
    SuperClass(),
    _sumOfVisitedVotes(0),
    _grid_3d_ptr(grid_3d_ptr)
{
}

template< class DataT >
STL_VisitorGrid3DFastFilter<DataT>::~STL_VisitorGrid3DFastFilter()
{
}

template< class DataT >
void STL_VisitorGrid3DFastFilter<DataT>::visit(size_t levx, size_t levy, size_t levz, const CT_Beam* const beam)
{
    Q_UNUSED(beam);
    _sumOfVisitedVotes += _grid_3d_ptr->value( static_cast<int>(levx), static_cast<int>(levy), static_cast<int>(levz) );
}

template< class DataT >
int STL_VisitorGrid3DFastFilter<DataT>::sumOfVisitedVotes() const
{
    return _sumOfVisitedVotes;
}

template< class DataT >
void STL_VisitorGrid3DFastFilter<DataT>::setSumOfVisitedVotes(int sumOfVisitedVotes)
{
    _sumOfVisitedVotes = sumOfVisitedVotes;
}

#endif // STL_VISITORGRID3DFASTFILTER_HPP
