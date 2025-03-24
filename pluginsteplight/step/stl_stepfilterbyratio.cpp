#include "stl_stepfilterbyratio.h"
#include <ct_log/ct_logmanager.h>

STL_StepFilterByRatio::STL_StepFilterByRatio() : SuperClass()
{
    _ratio_thresh = 2.0;
}


QString STL_StepFilterByRatio::description() const
{
    return tr("2 - Filtre une grille 3D");
}

// Step detailled description
QString STL_StepFilterByRatio::getStepDetailledDescription() const
{
    return tr("Si l'algorithme qui crée la grille 3D");
}

CT_VirtualAbstractStep* STL_StepFilterByRatio::createNewInstance() const
{
    return new STL_StepFilterByRatio();
}

//////////////////// PROTECTED METHODS //////////////////

void STL_StepFilterByRatio::declareInputModels(CT_StepInModelStructureManager& manager)
{
    manager.addResult(_inResult, tr("Scène(s)"));
    manager.setZeroOrMoreRootGroup(_inResult, _inZeroOrMoreRootGroup);
    manager.addGroup(_inZeroOrMoreRootGroup, _inGroup);
    manager.addItem(_inGroup, _in_grid_3d,  tr("3D grid"));
}

void STL_StepFilterByRatio::declareOutputModels(CT_StepOutModelStructureManager& manager)
{
    manager.addResultCopy(_inResult);
    manager.addItem(_inGroup, _out_grid_3d, tr("Filtered 3D grid"));
}

void STL_StepFilterByRatio::fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog)
{
    postInputConfigDialog->addDouble(tr("Ratio threshold"),
                                     tr(""),
                                     0.0001,
                                     100.0,
                                     4,
                                     _ratio_thresh,
                                     1.0,
                                     tr("Si le ratio calculé est inférieur à celui-ci "
                                        "toutes les valeurs du point sont supprimées, sinon "
                                        "les valeurs ayant eu le moins de votes cumulés sont "
                                        "supprimées. Voir la description de l'étape pour plus "
                                        "d'explication.")
                                     );
}

void STL_StepFilterByRatio::compute()
{
    setProgress(0);

    for (CT_StandardItemGroup* group : _inGroup.iterateOutputs(_inResult))
    {
        if( isStopped() )
        {
            return;
        }

        ConstGrid3DPtr in_grid_3d = group->singularItem(_in_grid_3d);

        Grid3DPtr filtered_grid_3d = in_grid_3d->get_filtered_grid3d_using_fast_filter(_ratio_thresh, this);

        PS_LOG->addInfoMessage(LogInterface::error, tr("Min value %1").arg(filtered_grid_3d->dataMin()));
        PS_LOG->addInfoMessage(LogInterface::error, tr("Max value %1").arg(filtered_grid_3d->dataMax()));

        group->addSingularItem(_out_grid_3d, filtered_grid_3d);
    }

    setProgress(100);
}

