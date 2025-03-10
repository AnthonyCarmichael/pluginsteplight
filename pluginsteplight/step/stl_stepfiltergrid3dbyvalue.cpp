#include "stl_stepfiltergrid3dbyvalue.h"
#include <ct_log/ct_logmanager.h>

STL_StepFilterGrid3DByValue::STL_StepFilterGrid3DByValue() : CT_AbstractStep()
{
    _thresh = 10;
}

QString STL_StepFilterGrid3DByValue::description() const
{
    return tr("STL: 2 - Filtre une grille 3D selon un seuil fixe");
}

// Step detailled description
QString STL_StepFilterGrid3DByValue::getStepDetailledDescription() const
{
    return tr("Seuil fixe");
}

CT_VirtualAbstractStep* STL_StepFilterGrid3DByValue::createNewInstance() const
{
    return new STL_StepFilterGrid3DByValue();
}

//////////////////// PROTECTED METHODS //////////////////

void STL_StepFilterGrid3DByValue::declareInputModels(CT_StepInModelStructureManager& manager)
{
    manager.addResult(_inResult, tr("Scène(s)"));
    manager.setZeroOrMoreRootGroup(_inResult, _inZeroOrMoreRootGroup);
    manager.addGroup(_inZeroOrMoreRootGroup, _inGroup);
    manager.addItem(_inGroup, _in_grid3d,  tr("Input 3d grid"));
}

void STL_StepFilterGrid3DByValue::declareOutputModels(CT_StepOutModelStructureManager& manager)
{
    manager.addResultCopy(_inResult);
    manager.addItem(_inGroup, _out_grid3d, tr("Filtered 3D Grid with fixed threshold"));
}

void STL_StepFilterGrid3DByValue::fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog)
{
    postInputConfigDialog->addInt(tr("Fixed threshold"),
                                  tr(""),
                                  0,
                                  std::numeric_limits<int>::max(),
                                  _thresh,
                                  tr("Seuil fixe"));
}

void STL_StepFilterGrid3DByValue::compute()
{
    setProgress(0);

    for (CT_StandardItemGroup* group : _inGroup.iterateOutputs(_inResult))
    {
        if( isStopped() )
        {
            return;
        }

        const STL_Grid3D<int>* in_grid3d = group->singularItem(_in_grid3d);

        STL_Grid3D<int>* filtered_grid3d = in_grid3d->get_filtered_grid_using_fixed_threshold(_thresh, this);

        filtered_grid3d->computeMinMax();

        PS_LOG->addInfoMessage(LogInterface::error, tr("Min value %1").arg(filtered_grid3d->dataMin()));
        PS_LOG->addInfoMessage(LogInterface::error, tr("Max value %1").arg(filtered_grid3d->dataMax()));

        group->addSingularItem(_out_grid3d, filtered_grid3d);
    }

    setProgress(100);
}
