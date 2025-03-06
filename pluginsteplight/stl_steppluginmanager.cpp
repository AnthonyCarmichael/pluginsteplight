#include "stl_steppluginmanager.h"
#include "step/stl_stepcreate3dgrid.h"


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
    addNewPointsStep<STL_STEPCreate3DGrid>(CT_StepsMenu::LP_Points);
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


