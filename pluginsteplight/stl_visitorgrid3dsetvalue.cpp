#include "stl_visitorgrid3dsetvalue.h"


STL_VisitorGrid3DSetValue::STL_VisitorGrid3DSetValue(CT_Grid3D<int> *grid, int value_to_set) {
    _grid = grid;
    _value_to_set = value_to_set;
}

STL_VisitorGrid3DSetValue::~STL_VisitorGrid3DSetValue()
{
}

void STL_VisitorGrid3DSetValue::visit(const size_t &index, const CT_Beam *beam)
{
    Q_UNUSED(beam);
    _grid->setValueAtIndex(index, _value_to_set);
}
