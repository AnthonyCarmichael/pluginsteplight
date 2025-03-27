#include "stl_stepfiltergrid3d.h"
#include <ct_log/ct_logmanager.h>

STL_StepFilterGrid3D::STL_StepFilterGrid3D() : SuperClass()
{
    // _ratio_thresh = 2.0;
    _neighbours = 1;
}

QString STL_StepFilterGrid3D::description() const
{
    return tr("STL: 2 - Filtre une grille 3D selon ses voisins");
}

// Step detailled description
QString STL_StepFilterGrid3D::getStepDetailledDescription() const
{
    return tr("Ce filtre compare chaque valeur des cellules avec ses voisins pour conserver seulement les maximas locaux.");
}

CT_VirtualAbstractStep* STL_StepFilterGrid3D::createNewInstance() const
{
    return new STL_StepFilterGrid3D();
}

//////////////////// PROTECTED METHODS //////////////////

void STL_StepFilterGrid3D::declareInputModels(CT_StepInModelStructureManager& manager)
{
    manager.addResult(_inResult, tr("Scène(s)"));
    manager.setZeroOrMoreRootGroup(_inResult, _inZeroOrMoreRootGroup);
    manager.addGroup(_inZeroOrMoreRootGroup, _inGroup);
    manager.addItem(_inGroup, _in_grid3d,  tr("3D grid"));
}

void STL_StepFilterGrid3D::declareOutputModels(CT_StepOutModelStructureManager& manager)
{
    manager.addResultCopy(_inResult);
    manager.addItem(_inGroup, _out_grid3d, tr("Filtered 3D grid"));
}

void STL_StepFilterGrid3D::fillPostInputConfigurationDialog(CT_StepConfigurableDialog* postInputConfigDialog)
{
    // postInputConfigDialog->addDouble(tr("Ratio threshold"),
    //                                  tr(""),
    //                                  0.0000,
    //                                  100.0,
    //                                  4,
    //                                  _ratio_thresh,
    //                                  1.0,
    //                                  tr("Si le ratio calculé est inférieur à celui-ci "
    //                                     "toutes les valeurs du point sont supprimées, sinon "
    //                                     "les valeurs ayant eu le moins de votes cumulés sont "
    //                                     "supprimées. Voir la description de l'étape pour plus "
    //                                     "d'explication.")
    //                                  );
    postInputConfigDialog->addInt(tr("Nombre ligne et colonne de voisin à comparer à la cellule courante"), "", 1, 100, _neighbours, tr("Nombre ligne et colonne de voisin à comparer à la cellule courante"));
}

void STL_StepFilterGrid3D::compute()
{
    setProgress(0);

    for (CT_StandardItemGroup* group : _inGroup.iterateOutputs(_inResult))
    {
        if( isStopped() )
        {
            return;
        }

        const STL_Grid3D<int>* in_Grid3D = group->singularItem(_in_grid3d);

        STL_Grid3D<int>* filtered_Grid3D = in_Grid3D->get_filtered_grid_by_neigbhours(_neighbours, this);

        filtered_Grid3D->computeMinMax();

        PS_LOG->addInfoMessage(LogInterface::error, tr("Min value %1").arg(filtered_Grid3D->dataMin()));
        PS_LOG->addInfoMessage(LogInterface::error, tr("Max value %1").arg(filtered_Grid3D->dataMax()));

        group->addSingularItem(_out_grid3d, filtered_Grid3D);
    }

    setProgress(100);
}
