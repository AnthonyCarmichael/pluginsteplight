#include "stl_stepfiltergrid3d.h"
#include <ct_log/ct_logmanager.h>

STL_Step_Filter_Grid3D::STL_Step_Filter_Grid3D() : SuperClass()
{
    _ratio_thresh = 2.0;
}

QString STL_Step_Filter_Grid3D::description() const
{
    return tr("STL: 2 - Filtre une grille 3D");
}

// Step detailled description
QString STL_Step_Filter_Grid3D::getStepDetailledDescription() const
{
    return tr("Si l'algorithme qui crée une grille 3D suppose que les directions des normales "
              "des points ne sont pas forcément dans la bonne direction, il génère des cercles dans les deux directions. "
              "C'est pourquoi, si nous prenons l'exemple d'un tronc, il existe des valeurs élevées au centre du "
              "tronc mais aussi tout autour à une certaine distance. Ce filtre à pour but de supprimer les valeurs "
              "élevées en dehors du tronc. Pour chaque point du nuage l'algorithme va cumuler les valeurs dans la direction "
              "de la normale et cumuler dans une autre variable les valeurs dans le sens opposé de la normale. Le ratio "
              "calculé est max(cumul1, cumul2)/min(cumul1, cumul2). Si ce ratio est inférieur au ratio minimum les deux "
              "valeurs sont supprimer de la grille, sinon la valeur la moins élevée est supprimée.");
}

CT_VirtualAbstractStep* STL_Step_Filter_Grid3D::createNewInstance() const
{
    return new STL_Step_Filter_Grid3D();
}

//////////////////// PROTECTED METHODS //////////////////

void STL_Step_Filter_Grid3D::declareInputModels(CT_StepInModelStructureManager& manager)
{
    manager.addResult(_inResult, tr("Scène(s)"));
    manager.setZeroOrMoreRootGroup(_inResult, _inZeroOrMoreRootGroup);
    manager.addGroup(_inZeroOrMoreRootGroup, _inGroup);
    manager.addItem(_inGroup, _in_grid3d,  tr("3D grid"));
}

void STL_Step_Filter_Grid3D::declareOutputModels(CT_StepOutModelStructureManager& manager)
{
    manager.addResultCopy(_inResult);
    manager.addItem(_inGroup, _out_grid3d, tr("Filtered 3D grid"));
}

void STL_Step_Filter_Grid3D::fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog)
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

void STL_Step_Filter_Grid3D::compute()
{
    setProgress(0);

    for (CT_StandardItemGroup* group : _inGroup.iterateOutputs(_inResult))
    {
        if( isStopped() )
        {
            return;
        }

        const STL_Grid3D<int>* in_Grid3D = group->singularItem(_in_grid3d);

        STL_Grid3D<int>* filtered_Grid3D = in_Grid3D->get_filtered_grid_using_fast_filter(_ratio_thresh, this);

        PS_LOG->addInfoMessage(LogInterface::error, tr("Min value %1").arg(filtered_Grid3D->dataMin()));
        PS_LOG->addInfoMessage(LogInterface::error, tr("Max value %1").arg(filtered_Grid3D->dataMax()));

        group->addSingularItem(_out_grid3d, filtered_Grid3D);
    }

    setProgress(100);
}
