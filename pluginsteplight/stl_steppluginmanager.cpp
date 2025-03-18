#include "stl_steppluginmanager.h"
#include "step/stl_stepcreategrid3d.h"
#include "step/stl_stepextractcurvesfromgrid3d.h"
#include "step/stl_stepfiltergrid3d.h"
#include "step/stl_stepfiltergrid3dbyvalue.h"



ST_STL_StepPluginManager::ST_STL_StepPluginManager() : CT_AbstractStepPlugin()
{
}

ST_STL_StepPluginManager::~ST_STL_StepPluginManager()
{
}

QString ST_STL_StepPluginManager::getPluginRISCitation() const
{
    return "TY  - COMP\n"
           "TI  - Plugin Step v6\n"
           "AU  - Piboule, Alexandre\n"
           "AU  - Ravaglia, Joris\n"
           "AU  - Krebs, Michael\n"
           "PB  - \n"
           "PY  - \n"
           "UR  - No url yet\n"
           "ER  - \n";
}


bool ST_STL_StepPluginManager::loadGenericsStep()
{
    addNewPointsStep<STL_STEPCreateGrid3D>(CT_StepsMenu::LP_Points);
    addNewPointsStep<STL_StepFilterGrid3D>(CT_StepsMenu::LP_Points);
    addNewPointsStep<STL_StepFilterGrid3DByValue>(CT_StepsMenu::LP_Points);
    addNewPointsStep<STL_StepExtractCurvesFromGrid3D>(CT_StepsMenu::LP_Points);
    return true;
}

bool ST_STL_StepPluginManager::loadOpenFileStep()
{
    return true;
}

bool ST_STL_StepPluginManager::loadCanBeAddedFirstStep()
{
    return true;
}

bool ST_STL_StepPluginManager::loadActions()
{
    return true;
}

bool ST_STL_StepPluginManager::loadExporters()
{
    return true;
}

bool ST_STL_StepPluginManager::loadReaders()
{
    return true;
}


