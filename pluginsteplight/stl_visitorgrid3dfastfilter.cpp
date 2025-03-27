#include "stl_visitorgrid3dfastfilter.h"

STL_VisitorGrid3DFastFilter::STL_VisitorGrid3DFastFilter(CT_Grid3D<int> *grid) {
    _grid = grid;
    _sumOfVisitedVotes = 0;
}

STL_VisitorGrid3DFastFilter::~STL_VisitorGrid3DFastFilter()
{
}

void STL_VisitorGrid3DFastFilter::visit(const size_t &index, const CT_Beam *beam)
{
    Q_UNUSED(beam);
    _sumOfVisitedVotes += _grid->valueAtIndex(index);
}

int STL_VisitorGrid3DFastFilter::sumOfVisitedVotes() const
{
    return _sumOfVisitedVotes;
}

void STL_VisitorGrid3DFastFilter::setSumOfVisitedVotes(int sumOfVisitedVotes)
{
    _sumOfVisitedVotes = sumOfVisitedVotes;
}
