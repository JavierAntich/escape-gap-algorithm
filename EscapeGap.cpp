
#include "EscapeGap.h"
#include <vector>
#include <wx/msgdlg.h>
#include "CAboutBox.h"

// Librerias para la creacion de numeros al azar
#include <ctime>
#include <cstdlib>

// Ficheros
#include <iostream>
#include <fstream>

// (*InternalHeaders(EscapeGap)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/tglbtn.h>
#include <wx/settings.h>
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
//*)

#include <mrpt/gui/WxUtils.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include "imgs/main_icon.xpm"
#include "../wx-common/mrpt_logo.xpm"

#include "../ReactiveNavigationDemo/DEFAULT_GRIDMAP_DATA.h"
#include <mrpt/nav.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/system/filesystem.h>


#include <stdio.h>
#include <math.h>
#include <time.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::nav;
using namespace mrpt::poses;
using namespace std;

#define minAngleGap 10

// Archivo de DEBUGG
ofstream debugg("debugging.txt", ios::app);
// Archivo donde se guardan los estados de cada posicion
ofstream planif("planificador.txt", ios::app);
ofstream dataPoints("dataPoints.txt", ios::app);

unsigned int currentCycle = 0; // DEBUGG

// Funcion utilizada principalmente para la realizacion del debugging,
// permitiendo obtener un registro con fecha y hora
void CHolonomicEG::getTime(void)
{
    time_t   t,i;
    char *p;
    i = time (&t);
    p = ctime (&i);
    debugg<<p<<": ";
}

// Constructor
CHolonomicEG::CHolonomicEG(unsigned int numRegions, CMyGLCanvas* m_plotScan, struct parametros *param)
{
    a_numRegions = numRegions;

    // Division de angulos para cada region
    division_reg=180.0/a_numRegions;

    layer = 0;
    environmentInfo.push_back(parametros());
    environmentInfo[0] = *param;

    srand(time(0));

    // Regiones libres
    a_gl_T2_regionsFree= mrpt::opengl::CSetOfLines::Create();
    a_gl_T2_regionsFree->setLineWidth(2);
    a_gl_T2_regionsFree->setColor_u8(TColor(255,0,0));
    a_gl_T2_regionsFree->clear();
    m_plotScan->m_openGLScene->insert(a_gl_T2_regionsFree);

    // Regiones ocupadas
    a_gl_T2_regionsOccup = mrpt::opengl::CSetOfLines::Create();
    a_gl_T2_regionsOccup->setLineWidth(2);
    a_gl_T2_regionsOccup->setColor_u8(TColor(78,59,49));
    a_gl_T2_regionsOccup->clear();
    m_plotScan->m_openGLScene->insert(a_gl_T2_regionsOccup);

    // Region libre + Target
    a_gl_T2_targetFree = mrpt::opengl::CSetOfLines::Create();
    a_gl_T2_targetFree->setLineWidth(2);
    a_gl_T2_targetFree->setColor_u8(TColor(18,10,143)); // Azul marino
    a_gl_T2_targetFree->clear();
    m_plotScan->m_openGLScene->insert(a_gl_T2_targetFree);

    // Region ocupada + Target
    a_gl_T2_targetOccup = mrpt::opengl::CSetOfLines::Create();
    a_gl_T2_targetOccup->setLineWidth(2);
    a_gl_T2_targetOccup->setColor_u8(TColor(135,206,255)); // Azul celeste
    a_gl_T2_targetOccup->clear();
    m_plotScan->m_openGLScene->insert(a_gl_T2_targetOccup);

    // Region de direccion
    a_gl_T2_direccion = mrpt::opengl::CSetOfLines::Create();
    a_gl_T2_direccion->setLineWidth(2);
    a_gl_T2_direccion->setColor_u8(TColor(255,255,0)); // Amarillo
    a_gl_T2_direccion->clear();
    m_plotScan->m_openGLScene->insert(a_gl_T2_direccion);

    dir=direccion(0);

    writeDebugg = false;

    cout<<"+++++++++++++++++++++chooseLocalObstacle por constructor++++++++++++++++++++++++"<<endl;
    chooseLocalObstacle=true;
}

// DIBUJAR

// Se utiliza este constructor para dibujar las regiones alrededor del robot, en lugar de
// dibujarlas en el radar
CHolonomicEG::CHolonomicEG(unsigned int numRegions, mrpt::opengl::CSetOfObjectsPtr m_plotScan, CMyGLCanvas* m_plotScan2D, struct parametros *param)
{
    a_numRegions = numRegions;

    // Division de angulos para cada region
    division_reg=180.0/a_numRegions;

    layer = 0;
    environmentInfo.push_back(parametros());
    environmentInfo[0] = *param;

    srand(time(0));

    // Regiones libres
    a_gl_T2_regionsFree= mrpt::opengl::CSetOfLines::Create();
    a_gl_T2_regionsFree->setLineWidth(2);
    a_gl_T2_regionsFree->setColor_u8(TColor(0,200,0));
    a_gl_T2_regionsFree->clear();
    m_plotScan->insert(a_gl_T2_regionsFree);
    m_plotScan2D->m_openGLScene->insert(a_gl_T2_regionsFree);

    // Regiones ocupadas
    a_gl_T2_regionsOccup = mrpt::opengl::CSetOfLines::Create();
    a_gl_T2_regionsOccup->setLineWidth(2);
    a_gl_T2_regionsOccup->setColor_u8(TColor(78,59,49));
    a_gl_T2_regionsOccup->clear();
    m_plotScan->insert(a_gl_T2_regionsOccup);
    m_plotScan2D->m_openGLScene->insert(a_gl_T2_regionsOccup);

    // Region libre + Target
    a_gl_T2_targetFree = mrpt::opengl::CSetOfLines::Create();
    a_gl_T2_targetFree->setLineWidth(2);
    a_gl_T2_targetFree->setColor_u8(TColor(18,10,143)); // Azul marino
    a_gl_T2_targetFree->clear();
    m_plotScan->insert(a_gl_T2_targetFree);
    m_plotScan2D->m_openGLScene->insert(a_gl_T2_targetFree);

    // Region ocupada + Target
    a_gl_T2_targetOccup = mrpt::opengl::CSetOfLines::Create();
    a_gl_T2_targetOccup->setLineWidth(2);
    a_gl_T2_targetOccup->setColor_u8(TColor(135,206,255)); // Azul celeste
    a_gl_T2_targetOccup->clear();
    m_plotScan->insert(a_gl_T2_targetOccup);
    m_plotScan2D->m_openGLScene->insert(a_gl_T2_targetOccup);

    // Region de direccion
    a_gl_T2_direccion = mrpt::opengl::CSetOfLines::Create();
    a_gl_T2_direccion->setLineWidth(2);
    a_gl_T2_direccion->setColor_u8(TColor(255,255,0)); // Amarillo
    a_gl_T2_direccion->clear();
    m_plotScan->insert(a_gl_T2_direccion);
    m_plotScan2D->m_openGLScene->insert(a_gl_T2_direccion);

    dir=direccion(0);

    writeDebugg = false;

    chooseLocalObstacle=true;
}


CHolonomicEG::~CHolonomicEG()
{ }


//****************************************************************************************
// Esta funcion se encarga de ir actualizando el comportamiento del algoritmo iND
//****************************************************************************************
std::string CHolonomicEG::update(const mrpt::math::TPoint2D &target, const std::vector<float> &scan,
                                 const mrpt::math::TPose2D &rPose, const float maxRange, mrpt::math::TPoint2D *realTarget, 
                                 bool *objetivof, std::vector<int> iniGap, std::vector<int> endGap, bool *regionStatePlan, double desiredDirection, int mainCycles, double timeElapsed)
{
    if (debuggLines) cout<<"Line 213"<<endl;

    //************ DATOS IMPRIMIBLES ************
    double memory = 0;
    if (printData){
        // Se suma el numero de datos que tiene en la memoria cada capa
        for (int i = 0; i <= layer; i++){
            memory += environmentInfo[i].ang.size();
        }
        dataPoints<<"-- cycles = "<<mainCycles<<" Memory = "<<memory<<" Layer = "<<layer<<" timeElapsed = "<<timeElapsed<<endl;
    }

    //***************************************

    if (debuggUpdate) debugg<<"L142 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    if (debuggUpdate) cout<<"L142 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;

    /************************ ELIMINACION DE GAPS ******************************/

    static unsigned int iterCycles = 0;
    iterCycles++;

    if (debuggUpdate) cout<<"---------------INICIO CYCLE "<<iterCycles<<"--------------------"<<endl;
    if (debuggUpdate) debugg<<"---------------INICIO CYCLE "<<iterCycles<<"--------------------"<<endl;
    if (debuggUpdate) cout<<"Size scan: "<<scan.size()<<endl;
   
    if (debuggLines) cout<<"Line 241"<<endl;
    // Con esta funcion se eliminan los pequeños gaps que nos dificultan el tratamiento
    // de la informacion
    deleteSmallGaps(iniGap, endGap);

    if (debuggFilter || debuggToFunction || fileDebugg) {
        if (!fileDebugg) cout<<"*******************INICIO: "<<iterCycles<<" ******************"<<endl<<"Los valores del nuevo vector son los siguientes fuera de funcion:"<<endl;
        if (fileDebugg) debugg<<"*******************INICIO: "<<iterCycles<<" ******************"<<endl<<"Los valores del nuevo vector son los siguientes fuera de funcion:"<<endl;
        for (size_t gh = 0; gh < Gaps.ini.size(); gh++){
            if (!fileDebugg) cout<<"Gaps.ini["<<gh<<"] = "<<Gaps.ini[gh]<<endl;
            if (!fileDebugg) cout<<"Gaps.end["<<gh<<"] = "<<Gaps.end[gh]<<endl;
            if (fileDebugg) debugg<<"Gaps.ini["<<gh<<"] = "<<Gaps.ini[gh]<<endl;
            if (fileDebugg) debugg<<"Gaps.end["<<gh<<"] = "<<Gaps.end[gh]<<endl;
        }
        cout<<"**************************************"<<endl;
        debugg<<"**************************************"<<endl;
    }

    if (sizeLayerDebugg) debugg<<"L168 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    if (sizeLayerDebugg) cout<<"L168 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    /********************************************************************/

    // Inicializacion de cada una de las lineas que forman las regiones del barrido
    // del sensor laser
    a_gl_T2_regionsFree ->clear(); // Regiones libres
    a_gl_T2_regionsFree ->appendLine(0,0,0,0,0,0);
    a_gl_T2_regionsOccup->clear(); // Regiones ocupadas
    a_gl_T2_regionsOccup->appendLine(0,0,0,0,0,0);
    a_gl_T2_targetFree  ->clear(); // Region libre en la direccion que se encuentra el destino
    a_gl_T2_targetFree  ->appendLine(0,0,0,0,0,0);
    a_gl_T2_targetOccup ->clear(); // Region ocupada en la direccion que se encuentra el destino
    a_gl_T2_targetOccup ->appendLine(0,0,0,0,0,0);
    a_gl_T2_direccion -> clear();  // Region que indica la direccion que debe seguir el robot
    a_gl_T2_direccion ->appendLine(0,0,0,0,0,0);

    // Vector en el que guardamos el estado en el que se encuentra cada region (libre, ocupada, etc.)
    char regionState[a_numRegions];
    for (size_t l=0; l<a_numRegions; l++) {
        regionState[l]=0;
        regionStatePlan[l]=0;
    }

    //************** DEBUGG REGION **********************
    if (debuggFilter || debuggRegionDirection || fileDebugg) {
        if (!fileDebugg) cout<<"Estado regiones en linea 194"<<endl;  // DEBUGG REGION
        if (fileDebugg) debugg<<"Estado regiones en linea 194"<<endl; // DEBUGG REGION

        for (size_t i = 0; i<20; i++) {
            if (!fileDebugg) cout<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;
            if (fileDebugg) debugg<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;        
        }
    }
    //************************************************

    // Vector que nos indica el estado actual de cada region
    char currBusyReg[a_numRegions];
    for (size_t l=0; l<a_numRegions; l++) {
        currBusyReg[l]=0;
    }
    
    double regions = (double)a_numRegions;

    // Variable de DEBUGGING
    static unsigned int numCycle = 0;
    numCycle++;
    if (debuggLines) cout<<"Line 303"<<endl;

    //************************ Variables para el apartado de deteccion del target ******************************************

    double alfa=atan2(target.y,target.x); // Calculo del angulo del destino respecto al robot
    if (debuggSpiral) cout<<"target = ("<<target.x<<", "<<target.y<<")"<<endl;
    double alfa_deg=((alfa+M_PI)*scan.size())/M_2PI; // Calculamos el angulo en el que se encuentra el 
                                                     // target respecto al robot y lo pasamos a grados
    int regTarget; // Region donde se encuentra el destino
    static int lastDirectionRegion=0; // Ultima region en la que se ha indicado donde se encuentra
                                      // la direccion a la que se debe dirigir el robot
    static int currentDirectionRegion=0; // Actual region donde se encuentra la direccion que debe
                                         // seguir el robot
    static int lastlastYellowRegion=0;
    static bool toTarget=false; // Indica si se ha avistado el destino para dirigirse de manera
                                // directa a el
    static int cycles=0;

    static char targetRegionState = 0; // Estado de la region en la que se encuentra el destino
    static char lastTargetRegionState = 0; // Ultimo estado de la region en la que se encuentra
                                           // el destino

    //******************************************* Fin de variables deteccion target *******************************************

    //******************************************** Ignorar obstaculos que no evitamos ********************************
    // Variables
    
    if (sizeLayerDebugg) debugg<<"L234 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    if (sizeLayerDebugg) cout<<"L234 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;

    static unsigned int iterDebug=0; // DEBUUGG!!!!

    // Cada vez que se pase del algoritmo ND al algoritmo iND, se realiza un reset de todos los datos y dibujos
    if (chooseLocalObstacle) {
        reset();
        middlePointInObstacle = false;
        middlePoint = (double)alfa_deg;
        if (debuggRegionDirection || debuggToFunction) cout<<"Volvemos a ir en direccion del target: "<<(double)alfa_deg<<" con size: "<<environmentInfo[layer].ang.size()<<endl;
    }

    if (sizeLayerDebugg) debugg<<"L249 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    if (sizeLayerDebugg) cout<<"L249 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;

    if (debuggLines) cout<<"Line 349 "<<endl;

    // Se realiza una inicializacion del filtro de informacion
    if (!middlePointInObstacle){
        cout<<"Se inicializan variables de nivel reactivo"<<endl;
        for (int i = 0; i < 181; i++) angleObstacle[i]=true;
        Obstacle.lowAngle = 0.0;
        Obstacle.highAngle = 180.0;
    }
    // En un principio se hace uso de los gaps que nos proporciona la libreria de MRPT, para
    // asi obtener un primer muestreo de datos del obstaculo al que se va a rodear
    if (!middlePointInObstacle){
        getLocalObstacleBoundary(false);
        if (debuggUpdate) cout<<"middlePointInObstacle in false : "<<middlePointInObstacle<<endl;
        if (debuggUpdate) debugg<<"middlePointInObstacle in false : "<<middlePointInObstacle<<endl;
    } else{
        if (debuggLines) cout<<"Line 357"<<endl;
        debugg<<endl<<"******************Cycle of ten = "<<iterCycles<<"****************"<<endl<<endl;
        // Se forma el filtro de informacion que realizara un filtrado de toda la informacion recogida
        // por el barrido del sensor laser
        getLocalObstacleBoundary_ten(scan, rPose, maxRange, desiredDirection, currentDirectionRegion, false);
        if (debuggUpdate) cout<<"middlePointInObstacle in true : "<<middlePointInObstacle<<endl;
        if (debuggUpdate) debugg<<"middlePointInObstacle in true : "<<middlePointInObstacle<<endl;
        if (debuggLines) cout<<"Line 362"<<endl;
    }

    if (debuggUpdate) cout<<"************ RANGO FILTRO DE OBSTACULO **************"<<endl;

    if (debuggUpdate||debuggLines) cout<<"Obstacle.lowAngle: "<<Obstacle.lowAngle<<" ; Obstacle.highAngle: "<<Obstacle.highAngle<<" ; Obstacle.falseGap: "<<(int)Obstacle.falseGap<<endl;
    if (debuggUpdate) debugg<<"Obstacle.lowAngle: "<<Obstacle.lowAngle<<" ; Obstacle.highAngle: "<<Obstacle.highAngle<<" ; Obstacle.falseGap: "<<(int)Obstacle.falseGap<<endl;

    if (debuggUpdate) cout<<"*************************************************"<<endl;

    if (sizeLayerDebugg) debugg<<"L274 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    if (sizeLayerDebugg) cout<<"L274 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;

    //************* DEBUGG REGION ***********************
    if (debuggFilter || debuggRegionDirection || fileDebugg) {
        if (!fileDebugg) cout<<"Estado regiones en linea 284"<<endl;  // DEBUGG REGION
        if (fileDebugg) debugg<<"Estado regiones en linea 284"<<endl; // DEBUGG REGION

        for (size_t i = 0; i<20; i++){
            if (!fileDebugg) cout<<"currBusyReg["<<i<<"] = "<<(int)currBusyReg[i]<<endl;
            if (fileDebugg) debugg<<"currBusyReg["<<i<<"] = "<<(int)currBusyReg[i]<<endl;        
        }
    }
    //**************************************************

    //******************************* Obtencion de puntos del sensor laser *****************************************************
 
    // Teniendo en cuenta los limites obtenidos en la funcion getLocalObstacleBoundary_ten
    // se obtienen y se almacenan en la memoria a corto plazo, todos los puntos
    // detectados mediante el uso del sensor laser

    // Se van creando posiciones en un vector dinamico, donde se introducen las distancias
    // de los obstaculos, su angulo en radianes, sus posiciones absolutas en ejes 
    // cartesianos (X e Y). Ademas se van creando las posiciones del vector que contendra las 
    // posiciones relativas de los obstaculos respecto al robot en todo momento

    if (debuggLines) cout<<"Line 404"<<endl;
    getSensorPoints(scan, currBusyReg, rPose, maxRange, false);

    if (TFGpoint) cout<<"***************** PUNTO TFG *************************"<<endl;
    if (TFGpoint) cout<<"Absolutas: Xobs = "<<environmentInfo[layer].Xobs[30]<<" Yobs = "<<environmentInfo[layer].Yobs[30]<<endl;
    if (TFGpoint) cout<<"Relativas: Xfin = "<<environmentInfo[layer].Xfin[30]<<" Yfin = "<<environmentInfo[layer].Yfin[30]<<endl;
    if (TFGpoint) cout<<"environmentInfo[layer].ang = "<<environmentInfo[layer].ang[30]<<endl;

    //************* DEBUGG REGION ***********************
    if (debuggFilter || debuggRegionDirection) {
        if (!fileDebugg) cout<<endl<<"currentDirectionRegion: "<<currentDirectionRegion<<endl;
        if (!fileDebugg) cout<<"Estado regiones en linea 315"<<endl;  // DEBUGG REGION
        if (fileDebugg) debugg<<endl<<"currentDirectionRegion: "<<currentDirectionRegion<<endl;
        if (fileDebugg) debugg<<"Estado regiones en linea 315"<<endl; // DEBUGG REGION

        for (size_t i = 0; i<20; i++) {
            cout<<"currBusyReg["<<i<<"] = "<<(int)currBusyReg[i]<<endl;
            debugg<<"currBusyReg["<<i<<"] = "<<(int)currBusyReg[i]<<endl;        
        }
    }
    //**************************************************

    if (sizeLayerDebugg) debugg<<"L314 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    if (sizeLayerDebugg) cout<<"L314 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    
    //**************** Obtencion de region posterior a la region de direccion actual *******************
  
    //********************************************************************************************
    // Se consiguen los angulos que delimitan la region siguiente a la region actual de direccion
    //********************************************************************************************
    if (debuggLines) cout<<"Line 446"<<endl;
    getRegionDirectionBoundary(currBusyReg, currentDirectionRegion, false);

     if (debuggFilter || debuggRegionDirection || fileDebugg) {
        if (!fileDebugg) cout<<"Estado regiones en linea 349"<<endl;  // DEBUGG REGION
        if (fileDebugg) debugg<<"Estado regiones en linea 349"<<endl; // DEBUGG REGION

        for (size_t i = 0; i<20; i++) {
            if (!fileDebugg) cout<<"currBusyReg["<<i<<"] = "<<(int)currBusyReg[i]<<endl;
            if (fileDebugg) debugg<<"currBusyReg["<<i<<"] = "<<(int)currBusyReg[i]<<endl;        
        }
    }

    if (sizeLayerDebugg) debugg<<"L346 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    if (sizeLayerDebugg) cout<<"L346 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    //**************** DEBUGG REGION ***********************
    // IMPRIMIR VARIABLES ANGLOWBOUND OBSTAC.LOW ....
    if (debuggFilter || debuggRegionDirection) {

        cout<<"************ VARIABLES DELIMITADORAS **************"<<endl;

        cout<<"angLowBound: "<<regionDirection.angLowBound<<" ; "<<"angHighBound: "<<regionDirection.angHighBound<<" ; "<<"Boundary: "<<regionDirection.boundary<<endl;
        cout<<"Obstacle.lowAngle: "<<Obstacle.lowAngle<<" ; Obstacle.highAngle: "<<Obstacle.highAngle<<" ; Obstacle.falseGap: "<<(int)Obstacle.falseGap<<endl;

        cout<<"*************************************************"<<endl;

        debugg<<"************ VARIABLES DELIMITADORAS **************"<<endl;

        debugg<<"angLowBound: "<<regionDirection.angLowBound<<" ; "<<"angHighBound: "<<regionDirection.angHighBound<<" ; "<<"Boundary: "<<regionDirection.boundary<<endl;
        debugg<<"Obstacle.lowAngle: "<<Obstacle.lowAngle<<" ; Obstacle.highAngle: "<<Obstacle.highAngle<<" ; Obstacle.falseGap: "<<(int)Obstacle.falseGap<<endl;

        debugg<<"*************************************************"<<endl;
    }
    //****************************************************

    //*************************** DEBUGG ***********************************

    if (debuggFilter || debuggRegionDirection || fileDebugg) {
        if (!fileDebugg) cout<<"Estado regiones en linea 387"<<endl;  // DEBUGG REGION
        if (fileDebugg) debugg<<"Estado regiones en linea 387"<<endl; // DEBUGG REGION
        for (size_t i = 0; i<20; i++) {
            if (!fileDebugg) cout<<"currBusyReg["<<i<<"] = "<<(int)currBusyReg[i]<<endl;
            if (fileDebugg) debugg<<"currBusyReg["<<i<<"] = "<<(int)currBusyReg[i]<<endl;
        }
    }
    
    if (sizeLayerDebugg) debugg<<"L385 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    for (int i = 0; i < a_numRegions; i++) regionState[i] = 0;
    if (sizeLayerDebugg) debugg<<"L387 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    if (sizeLayerDebugg) cout<<"L387 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;

    //*********************************************************************

    if (debuggFilter || debuggRegionDirection || fileDebugg) {
        if (!fileDebugg) cout<<"Estado regiones en linea 404"<<endl;  // DEBUGG REGION
        if (fileDebugg) debugg<<"Estado regiones en linea 404"<<endl; // DEBUGG REGION
        for (size_t i = 0; i<20; i++) {
            if (!fileDebugg) cout<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;
            if (fileDebugg) debugg<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;        
        }
    }
    
    if (debuggRegionDirection) cout<<"Size del vector de environment: "<<environmentInfo[layer].ang.size()<<endl;

    if (debuggLimit) {
        cout<<"Scans beforeUpdateEnvironment"<<endl;
        debugg<<"Scans beforeUpdateEnvironment"<<endl;
        for (int i = 0; i < 181; i++) {
            if (angleObstacle[i]) {
                double scanAngle = i*180.0/181.0;
                debugg<<"scanAngle["<<i<<"] = "<<scanAngle<<endl;
                cout<<"scanAngle["<<i<<"] = "<<scanAngle<<endl;
            }
        }
    }

    cout<<"Obstacle.falseGap = "<<Obstacle.falseGap<<" Obstacle.lowAngle = "<<Obstacle.lowAngle<<" Obstacle.highAngle = "<<Obstacle.highAngle<<endl;

    //*************************************************************************************
    // Se actualiza la informacion de los puntos guardados en la memoria a corto plazo
    //*************************************************************************************
    if (debuggLines) cout<<"Line 537"<<endl;
    updateEnvironment(rPose, currentDirectionRegion, false);

    if (TFGpoint) cout<<"***************** PUNTO TFG AFTER UPDATE ENVIRONMENT *************************"<<endl;
    if (TFGpoint) cout<<"Absolutas: Xobs = "<<environmentInfo[layer].Xobs[30]<<" Yobs = "<<environmentInfo[layer].Yobs[30]<<endl;
    if (TFGpoint) cout<<"Relativas: Xfin = "<<environmentInfo[layer].Xfin[30]<<" Yfin = "<<environmentInfo[layer].Yfin[30]<<endl;
    if (TFGpoint) cout<<"environmentInfo[layer].ang = "<<environmentInfo[layer].ang[30]<<endl;

    if (debuggLimit) {
        cout<<"Scans finales"<<endl;
        debugg<<"Scans finales"<<endl;
        for (int i = 0; i < 181; i++) {
            if (angleObstacle[i]){
                double scanAngle = i*180.0/181.0;
                debugg<<"scanAngle["<<i<<"] = "<<scanAngle<<endl;
                cout<<"scanAngle["<<i<<"] = "<<scanAngle<<endl;
            }
        }
    }


    if (sizeLayerDebugg) debugg<<"L398 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    if (sizeLayerDebugg) cout<<"L398 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;

    //************** DEBUGG REGION **********************
    if (debuggFilter || debuggRegionDirection || fileDebugg) {
        if (!fileDebugg) cout<<"Estado regiones en linea 447"<<endl;  // DEBUGG REGION
        if (fileDebugg) debugg<<"Estado regiones en linea 447"<<endl; // DEBUGG REGION
        for (size_t i = 0; i<20; i++) {
            if (!fileDebugg) cout<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;
            if (fileDebugg) debugg<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;        
        }
    }

    if (anglesDebugg) {
        debugg<<"Valores angleObstacle: "<<endl;
        for (unsigned int i = 0; i < scan.size(); i++){
            debugg<<"valor de "<<i<<": "<<angleObstacle[i]<<endl;
        }
    }

    int reg = 99;

    //******************************************************************************
    // Actualizacion del estado de cada una de las regiones en libre u ocupadas
    //******************************************************************************
    if (anglesDebugg) debugg<<"Asignacion de valores a las regiones"<<endl;
    if (anglesDebugg) debugg<<"division_reg = "<<division_reg<<endl;
    for (unsigned int ik=0;ik<scan.size();ik++) {
        double scanAngle = ik*180.0/scan.size();
        bool inFollowedObstacle = (Obstacle.falseGap && ((scanAngle >= Obstacle.lowAngle && scanAngle <= 180.0) ||
            (scanAngle < Obstacle.highAngle && scanAngle >= 0.0))) || (!Obstacle.falseGap && (scanAngle >= Obstacle.lowAngle && scanAngle < Obstacle.highAngle));
        if (((scanAngle < division_reg/2)||(scanAngle >= (180-division_reg/2))) && angleObstacle[ik]) {
            regionState[0]=1;
            debugg<<"Region 0 ocupada por angulo "<<scanAngle<<endl;
        } else if (scanAngle >= division_reg/2 && scanAngle < (180-division_reg/2) && angleObstacle[ik]) {
            reg=(int)trunc((scanAngle/division_reg)+0.5); // Añadimos 0.5 debido al desfase de angulos que hay
            regionState[reg]=1;
            if (anglesDebugg) debugg<<"angleObstacle["<<ik<<"] = "<<angleObstacle[ik]<<"Division: "<<angleObstacle[ik]/division_reg<<endl;
            if (anglesDebugg) debugg<<"Region "<<reg<<" ocupada por ANGULO "<<scanAngle<<endl;
        }
    }

    if (anglesDebugg) debugg<<"Fin de asignacion de valores a las regiones"<<endl;
    if (sizeLayerDebugg) debugg<<"L455 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;

    //************** DEBUGG REGION **********************
    if (debuggFilter || debuggRegionDirection || fileDebugg) {
        if (!fileDebugg) cout<<"Estado regiones en linea 485"<<endl;  // DEBUGG REGION
        if (fileDebugg) debugg<<"Estado regiones en linea 485"<<endl; // DEBUGG REGION
        for (size_t i = 0; i<20; i++) {
            if (!fileDebugg) cout<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;
            if (fileDebugg) debugg<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;
        }
    }

    //************* Debugg hibrido *****************
    cout<<"-------------Regions state en modo ocupado o no---------------"<<endl;
    for (int i = 0; i < 20 ; i++){
        cout<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;
    }
    //************************************************    

    //************************* DETECCION DE OBJETIVO *****************************
    cycles++;

    string navFiltState; // Indica el estado en el que se encuentra actualmente el robot:
                         // - MTG -> Motion To Goal
                         // - BFL -> Boundary Following Left
                         // - BFR -> Boundary Following Right 

  
    if (debuggLines) cout<<"Line 213"<<endl;

    if ((alfa_deg<division_reg/2)||(alfa_deg>=(180-division_reg/2))) {
        regTarget=0;
    } else {
        regTarget=(int)trunc((alfa_deg/division_reg)+0.5); // Añadimos 0.5 debido al desfase de angulos que hay
    }

    if (regionState[regTarget]==1) {
        regionState[regTarget]=2; // Region target ocupada
    } else {
        regionState[regTarget]=3; // Region target libre
    }

    //************* Debugg hibrido *****************
    cout<<"-------------Regions State para target--------------"<<endl;
    for (int i = 0; i < 20 ; i++){
        cout<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;
    }
    //************************************

    targetRegionState = regionState[regTarget];

    if (sizeLayerDebugg) debugg<<"L496 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;

    //************** DEBUGG REGION **********************
    if (debuggFilter || debuggRegionDirection || fileDebugg) {

        if (!fileDebugg) cout<<"Estado regiones en linea 527"<<endl;  // DEBUGG REGION
        if (fileDebugg) debugg<<"Estado regiones en linea 527"<<endl; // DEBUGG REGION
        for (size_t i = 0; i<20; i++){
            if (!fileDebugg) cout<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;
            if (fileDebugg) debugg<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;
        }
    }
    //************************************************

    //****************************** AÑADIENDO CAPAS NUEVAS *************************
    int layerState = 0;
    static bool sameRegionDirection = false;

    // Tenemos 3 estados: 1- que se cree una nueva capa; 2- que se elimine la ultima capa; 3- ninguna accion
    layerState = manageLayers(regionState, targetRegionState, lastTargetRegionState, lastlastYellowRegion, false);

    // Si no se ha añadido una capa nueva se guarda el estado actual de la region del objetivo
    if (layerState != layerAdded) {
        lastTargetRegionState = targetRegionState; 
    } else {
        lastTargetRegionState = 0;
        targetRegionState = 0;
    }

    if (sizeLayerDebugg) debugg<<"L524 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    if (sizeLayerDebugg) cout<<"L524 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    //*****************************************************************************

    // Obtenemos la region a la que se debe dirigir el robot
    if (layersDebugg) debugg<<"layerState = "<<layerState<<" sameRegionDirection = "<<sameRegionDirection<<endl;
    // Si se elimina la capa actual o en el ciclo anterior se ha eliminado
    if (layerState == layerDeleted || sameRegionDirection) {
        // Obtencion de la region de la direccion a la que va el robot
        int directionRegion = targetRegion(regTarget, regionState);
        currentDirectionRegion = regTarget;

        cout<<"____________lastlastYellowRegion = "<<lastlastYellowRegion<<" directionRegion = "<<directionRegion<<" currentDirectionRegion = "<<currentDirectionRegion<<endl;

        if (sameRegionDirection && directionRegion != lastlastYellowRegion && directionRegion != -1) sameRegionDirection = false;
        if (layerState == layerDeleted) sameRegionDirection = true;
    }   
    else {
        // Obtencion de la region de la direccion a la que va el robot
        currentDirectionRegion = targetRegion(regTarget, regionState);
    }

    if (debuggUpdate || layersDebugg) debugg<<"currentDirectionRegion = "<<currentDirectionRegion<<endl;

    if (debuggRegionDirection || debuggVector) {
        cout<<"Data to filterState -> currentDirectionRegion: "<<currentDirectionRegion<<" lastlastYellowRegion: "<<lastlastYellowRegion<<endl;
        cout<<"regTarget: "<<regTarget<<" regionState[regTarget]: "<<regionState[regTarget]<<" &toTarget: "<<toTarget<<endl;
        cout<<"lastDirectionRegion"<<lastDirectionRegion<<endl;
        cout<<"*objetivof: "<<*objetivof<<endl;
    }

    if (sizeLayerDebugg) debugg<<"L537 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    if (sizeLayerDebugg) cout<<"L537 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;

    // Se devuelve el estado actual en el que se encuentra el robot. Si el estado es MTG,
    // mediante el parametro toTarget, se indica que el robot debe dirigirse hacia su
    // objetivo de manera directa cambiando al algoritmo ND
    navFiltState = filterState(currentDirectionRegion,lastlastYellowRegion,regTarget,regionState[regTarget], &toTarget, layerState == layerDeleted);

    if (currentDirectionRegion >= 0) regionState[currentDirectionRegion] = 4;

    if (sizeLayerDebugg) debugg<<"L543 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;

    //************** DEBUGG REGION **********************
    if (debuggFilter || debuggRegionDirection || fileDebugg) {
        if (!fileDebugg) cout<<"Estado regiones en linea 588"<<endl;  // DEBUGG REGION
        if (fileDebugg) debugg<<"Estado regiones en linea 588"<<endl; // DEBUGG REGION
        for (size_t i = 0; i<20; i++) {
            if (!fileDebugg) cout<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;
            if (fileDebugg) debugg<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;        
        }
    }
    //************************************************
    // Fijando una direccion al robot para avanzar
    if (currentDirectionRegion >= 0) {
        const double sec = currentDirectionRegion*division_reg + (currentDirectionRegion*180/scan.size()/(regions-1));
        const double ang = M_PI *( -1 + 2*sec/((float)scan.size()) );
        const double d=1;
        realTarget->x = d*cos(ang);
        realTarget->y = d*sin(ang);
    }

    // Cuando toTarget se activa se le indicara al robot que se dirija de manera directa,
    // mediante el cambio al algoritmo ND
    if (toTarget) {
        cout<<"Vamos a goal"<<endl; // Debugg hibrido
        *objetivof = true;
        *realTarget = target;
        toTarget = false;
        cout<<"++++++++++++++++chooseLocalObstacle por toTarget"<<endl;
        chooseLocalObstacle = true;
        regionDirection.angHighBound = 0.0;
        regionDirection.angLowBound = 0.0;
    } else {
        cout<<"No vamos a goal"<<endl; // Debugg hibrido
        *objetivof = false;
    }

    if (currentDirectionRegion != lastDirectionRegion) {
        lastlastYellowRegion = currentDirectionRegion;
    }
  
    //************************ DECIDIR DIRECCION **********************************

    if (sizeLayerDebugg) debugg<<"L585 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    if (sizeLayerDebugg) cout<<"L585 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    
    //********************** ESTADO DE LAS REGIONES *******************************

    if (debuggFilter) cout<<"************* ESTADO DE LAS REGIONES ***********"<<endl;
    for (size_t rnum = 0; rnum < a_numRegions; rnum++) {
        if (regionState[rnum] == 1 || regionState[rnum] == 2) regionStatePlan[rnum] = true;
        else regionStatePlan[rnum] = false;
    }

    //************* Debugg hibrido *****************
    cout<<"-------------Regions State para direccion--------------"<<endl;
    for (int i = 0; i < 20 ; i++) {
        cout<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;
    }
    //************************************

    //********************* CREACION DE LAS REGIONES *****************************
    bool show_gaps=true; // Variable que determina si mostrar las regiones que determinan los gaps o no
    size_t r=1;

    if (sizeLayerDebugg) debugg<<"L602 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;
    if (sizeLayerDebugg) cout<<"L602 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;

    //************** DEBUGG REGION **********************
    if (debuggFilter || debuggRegionDirection || fileDebugg) {
        if (!fileDebugg) cout<<"Estado regiones en linea 648"<<endl;  // DEBUGG REGION
        if (fileDebugg) debugg<<"Estado regiones en linea 648"<<endl; // DEBUGG REGION
        for (size_t i = 0; i<20; i++) {
            if (!fileDebugg) cout<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;
            if (fileDebugg) debugg<<"regionState["<<i<<"] = "<<(int)regionState[i]<<endl;        
        }
    }
    //************************************************

    for (r=1; r<a_numRegions+1; r++) {
        const size_t N_STEPS = 5;              
        for (size_t j=0; j<N_STEPS; j++)
        {
            if (r==a_numRegions) {
                const double sec = r*division_reg-division_reg/2+2*180/scan.size()-180/scan.size()/(regions-1)+ j*(division_reg/2)/(static_cast<double>(N_STEPS-1));
                const double ang = M_PI *( -1 + 2*sec/((float)scan.size()) );
                const double d = 1.5;

                switch (regionState[0]) {

                    case 1:
                        a_gl_T2_regionsOccup->appendLineStrip(d*cos(ang),d*sin(ang),0);
                        break;

                    case 2:
                        a_gl_T2_targetOccup->appendLineStrip(d*cos(ang),d*sin(ang),0);
                        break;

                    case 3:
                        a_gl_T2_targetFree->appendLineStrip(d*cos(ang),d*sin(ang),0);
                        break;
                    case 4:
                        a_gl_T2_direccion->appendLineStrip(d*cos(ang),d*sin(ang),0);
                        break;
                    default:
                        if (show_gaps) {
                            a_gl_T2_regionsFree->appendLineStrip(d*cos(ang),d*sin(ang),0);
                        }
                }
            } else {
                const double sec = r*(division_reg)+(r*180/scan.size()/(regions-1))-division_reg/2+ j*(division_reg+2*180/scan.size()/(regions-1))/(static_cast<double>(N_STEPS-1));
                const double ang = M_PI *( -1 + 2*sec/((float)scan.size()) );
                const double d = 1.5;

                switch (regionState[r]) {

                    case 1:
                        a_gl_T2_regionsOccup->appendLineStrip(d*cos(ang),d*sin(ang),0);
                        break;

                    case 2:
                        a_gl_T2_targetOccup->appendLineStrip(d*cos(ang),d*sin(ang),0);
                        break;

                    case 3:
                        a_gl_T2_targetFree->appendLineStrip(d*cos(ang),d*sin(ang),0);
                        break;
                    case 4:
                        a_gl_T2_direccion->appendLineStrip(d*cos(ang),d*sin(ang),0);
                        break;
                    default:
                        if (show_gaps) {
                            a_gl_T2_regionsFree->appendLineStrip(d*cos(ang),d*sin(ang),0);
                        }
                }
            }
        }

        if (r!=a_numRegions) {
            switch (regionState[r]) {

                case 1:
                    a_gl_T2_regionsOccup->appendLineStrip(0,0,0);
                    break;

                case 2:
                    a_gl_T2_targetOccup->appendLineStrip(0,0,0);
                    break;

                case 3:
                    a_gl_T2_targetFree->appendLineStrip(0,0,0);
                    break;

                case 4:
                     a_gl_T2_direccion->appendLineStrip(0,0,0);
                     break;    

                default:
                    if (show_gaps) {
                        a_gl_T2_regionsFree->appendLineStrip(0,0,0);
                    }
            }
        }
    }

    r=0;
    const size_t N_STEPS = 5;
                
    for (size_t j=0; j<N_STEPS; j++)
    {
        const double sec = 180+180/scan.size()+180/scan.size()/(regions-1) + j*(division_reg+2*180/scan.size()-division_reg/2)/(static_cast<double>(N_STEPS-1));
        const double ang = M_PI *( -1 + 2*sec/((float)scan.size()) );
        const double d = 1.5;

        switch (regionState[r]) {

            case 1:
                a_gl_T2_regionsOccup->appendLineStrip(d*cos(ang),d*sin(ang),0);
                break;

            case 2:
                a_gl_T2_targetOccup->appendLineStrip(d*cos(ang),d*sin(ang),0);
                break;

            case 3:
                a_gl_T2_targetFree->appendLineStrip(d*cos(ang),d*sin(ang),0);
                break;

            case 4:
                a_gl_T2_direccion->appendLineStrip(d*cos(ang),d*sin(ang),0);
                break;    

            default:
                if (show_gaps) {
                    a_gl_T2_regionsFree->appendLineStrip(d*cos(ang),d*sin(ang),0);
                }
        }
    }

    switch (regionState[0]) {

        case 1:
            a_gl_T2_regionsOccup->appendLineStrip(0,0,0);
            break;

        case 2:
            a_gl_T2_targetOccup->appendLineStrip(0,0,0);
            break;

        case 3:
            a_gl_T2_targetFree->appendLineStrip(0,0,0);
            break;

        case 4:
            a_gl_T2_direccion->appendLineStrip(0,0,0);
            break;

        default:
            if (show_gaps) {
                a_gl_T2_regionsFree->appendLineStrip(0,0,0);
            }
    }

    // DEBUG
    if (sizeLayerDebugg) debugg<<"L761 -> environmentInfo["<<layer<<"].ang.size() = "<<environmentInfo[layer].ang.size()<<endl;

    string sLog = "Escape Gap DEBUG\n";
    sLog+=mrpt::format("endGap1: %f\t", (double)Gaps.end[0]);
    sLog+=mrpt::format("iniGap1: %f\n", (double)Gaps.ini[0]);
    sLog+=mrpt::format("endGap2: %f\t", (double)Gaps.end[1]);
    sLog+=mrpt::format("iniGap2: %f\n", (double)Gaps.ini[1]);
    sLog+=mrpt::format("endGap3: %f\t", (double)Gaps.end[2]);
    sLog+=mrpt::format("iniGap3: %f\n", (double)Gaps.ini[2]);
    sLog+=mrpt::format("middlePoint %f\t", (double)middlePoint);
    sLog+=mrpt::format("sizeParams %d\t", environmentInfo[layer].ang.size());
    sLog+=mrpt::format("Layer:  %d\n", layer);
    sLog+=mrpt::format("iterDebugFile: %d\t", iterCycles);
    sLog+=mrpt::format("chooseLocalObstacle: %d\n", chooseLocalObstacle);
    sLog+=mrpt::format("gaps_size: %f\n", (double)iniGap.size());
    sLog+=mrpt::format("Filter State: %s\t",navFiltState.c_str());

    cout<<"__________________________________"<<endl;
    return(sLog);
}

//*************************************************************************************
// Aqui se van actualizando constantemente la distancia de los obstaculos y sus 
// posiciones respecto al robot. Ademas, se va calculando el angulo en el que se 
// encuentran para posteriormente decidir en que region del radar se encuentran.
// Para saber exactamente en que angulo se encuentran los obstaculos, se calcula 
// su angulo teniendo en cuenta tanto el arcosinus como el arcsinus, de esta manera,
// si ambos coinciden, se puede asegurar de que angulo se trata. Sin realizar
// estas operaciones se podria dar el caso de que hubiera una equivocacion eligiendo un 
// angulo complementario equivocado
//*************************************************************************************

void CHolonomicEG::computeEnvironment(const mrpt::math::TPose2D &rPose, size_t it, bool debugging) {
    if (debuggFile) debugg<<it<<" - regionDirection.boundary(computeEnvironment beg): "<<regionDirection.boundary<<endl;

    double anguloProv[4];

    if (debugging) cout<<"Antes "<<it<<": "<<environmentInfo[layer].ang[it]<<endl;
    environmentInfo[layer].dist[it]=sqrt(pow(environmentInfo[layer].Xobs[it]-rPose.x,2)+pow(environmentInfo[layer].Yobs[it]-rPose.y,2));
    if (debuggFile) debugg<<"Dist "<<it<<": "<<environmentInfo[layer].dist[it]<<endl;
    environmentInfo[layer].Xfin[it]=environmentInfo[layer].Xobs[it]-rPose.x;
    if (debuggFile) debugg<<"Xobs "<<it<<": "<<environmentInfo[layer].Xobs[it]<<endl;
    if (debuggFile) debugg<<"Xfin "<<it<<": "<<environmentInfo[layer].Xfin[it]<<endl;
    environmentInfo[layer].Yfin[it]=environmentInfo[layer].Yobs[it]-rPose.y;
    if (debuggFile) debugg<<"Yobs "<<it<<": "<<environmentInfo[layer].Yobs[it]<<endl;
    if (debuggFile) debugg<<"Yfin "<<it<<": "<<environmentInfo[layer].Yfin[it]<<endl;
    anguloProv[0]=acos(environmentInfo[layer].Xfin[it]/environmentInfo[layer].dist[it]);
    anguloProv[1]=asin(environmentInfo[layer].Yfin[it]/environmentInfo[layer].dist[it]);
    anguloProv[2]=0-anguloProv[0];
    anguloProv[3]=M_PI-anguloProv[1];
    // Miramos entre que angulos esta el obstaculo para asegurar su angulo
    if (((anguloProv[0]<=anguloProv[1]+0.1)&&(anguloProv[0]>=anguloProv[1]-0.1))||((anguloProv[0]<=anguloProv[3]+0.1)&&(anguloProv[0]>=anguloProv[3]-0.1))) {
        environmentInfo[layer].ang[it]=((anguloProv[0]*(180/M_PI))+180)/2;
    } else {
        environmentInfo[layer].ang[it]=((anguloProv[2]*(180/M_PI))+180)/2;
    }

    int index = environmentInfo[layer].ang[it] < 0.5 || environmentInfo[layer].ang[it] > 179.0 ? 0 : (int)trunc(environmentInfo[layer].ang[it]) + 1;
    angleObstacle[index] = true;
    distObstacle[index]  = environmentInfo[layer].dist[it];

    if (debugging) cout<<"Creado "<<it<<": "<<environmentInfo[layer].ang[it]<<endl;
    if (debuggFile) debugg<<"angacos: "<<anguloProv[0]*180/M_PI<<" angasin:"<<anguloProv[1]*180/M_PI<<" 0-angacos:"<<anguloProv[2]*180/M_PI<<" M_PI-angasin:"<<anguloProv[3]*180/M_PI<<endl;
    if (debuggFile) debugg<<it<<" - regionDirection.boundary(computeEnvironment end): "<<regionDirection.boundary<<endl;
}

//*************************************************************************************************
// Si el angulo del obstaculo que se esta tratando actualmente esta entre los dos angulos 
// que delimitan la siguiente region cercana a la direccion que sigue el robot, se borraran 
// los datos de esta region, siempre y cuando este libre, pero esto se ha comprobado anteriormente 
// consiguiendo estos angulos delimitadores.
// Por otro lado se podria cumplir que la region cercana se tratara de la region 0.
// Si una de las condiciones se cumple, se borraran los datos y se decrementaran los iteradores. 
// Esta decrementacion es debida a que cuando se borran datos de un vector dinamico, esta posicion 
// que contenia estos datos, es sustituida por los datos que tienen la siguiente posicion, 
// produciendose asi un desplazamiento de todos los datos hacia la izquierda
//*************************************************************************************************

void CHolonomicEG::filterEnvironment(size_t &it, int currentDirectionRegion, bool debugging) {

    if (debuggFile || fileDebugg)
    {
        if (environmentInfo[layer].ang[it]>=regionDirection.angLowBound&&environmentInfo[layer].ang[it]<regionDirection.angHighBound) {
            if (debuggFile || debuggVector || fileDebugg) debugg<<"Se cumple condicion 1->"<<"regionDirection.boundary: "<<regionDirection.boundary<<" regionDirection.angLowBound: "<<regionDirection.angLowBound<<" regionDirection.angHighBound:"<<regionDirection.angHighBound<<endl;
        }
        if (regionDirection.boundary&&((environmentInfo[layer].ang[it]<regionDirection.angHighBound&&environmentInfo[layer].ang[it]>=0)||
                (environmentInfo[layer].ang[it]>=(regionDirection.angLowBound)&&environmentInfo[layer].ang[it]<=180))) {
            if (debuggFile || debuggVector || fileDebugg) debugg<<"Se cumple condicion 2->"<<"regionDirection.boundary: "<<regionDirection.boundary<<" regionDirection.angLowBound: "<<regionDirection.angLowBound<<" regionDirection.angHighBound:"<<regionDirection.angHighBound<<endl;
        }
    }

    if (((environmentInfo[layer].ang[it]>=regionDirection.angLowBound&&environmentInfo[layer].ang[it]<regionDirection.angHighBound)||(regionDirection.boundary&&((environmentInfo[layer].ang[it]<regionDirection.angHighBound&&environmentInfo[layer].ang[it]>=0)||
        (environmentInfo[layer].ang[it]>=(regionDirection.angLowBound)&&environmentInfo[layer].ang[it]<=180)))) && currentDirectionRegion > 0) {
        environmentInfo[layer].dist.erase(environmentInfo[layer].dist.begin()+it);
        debugg<<"Borrar "<<it<<": "<<environmentInfo[layer].ang[it]<<endl;
        if (debuggFile  || debuggVector || fileDebugg) debugg<<"   Borrar "<<it<<": "<<environmentInfo[layer].ang[it]<<endl;
        environmentInfo[layer].ang.erase(environmentInfo[layer].ang.begin()+it);
        environmentInfo[layer].Xobs.erase(environmentInfo[layer].Xobs.begin()+it);
        environmentInfo[layer].Yobs.erase(environmentInfo[layer].Yobs.begin()+it);
        environmentInfo[layer].Xfin.erase(environmentInfo[layer].Xfin.begin()+it);
        environmentInfo[layer].Yfin.erase(environmentInfo[layer].Yfin.begin()+it);
        environmentInfo[layer].belongToObstacle.erase(environmentInfo[layer].belongToObstacle.begin()+it);
        it--;
        environmentInfo[layer].iter--;
    } else {
        environmentInfo[layer].belongToObstacle[it] = true;
        if (debuggVector) debugg<<"belongToObstacle: angle["<<it<<"] = "<<environmentInfo[layer].ang[it]<<endl;
    }
}

//************************************************************************************************
// Esta funcion actualiza la informacion guardada en la memoria a corto plazo, tanto actualizando
// los datos de cada punto, como la distancia al robot, sus coordenadas y su angulo, como el
// borrado de puntos que cumplen un criterio para ser borrados
//************************************************************************************************

void CHolonomicEG::updateEnvironment(const mrpt::math::TPose2D &rPose, int currentDirectionRegion, bool debugging) {
    
    if (debugging) cout<<"*********************updateEnvironment*********************************"<<endl;

    if (debugging) cout<<"currentDirectionRegion: "<<currentDirectionRegion<<endl;

    debugg<<"regionDirection.angLowBound = "<<regionDirection.angLowBound<<" regionDirection.angHighBound = "<<regionDirection.angHighBound<<" regionDirection.boundary = "<<regionDirection.boundary<<endl;

    for (size_t i = 0; i < 180; i++) angleFrecuency[i] = 0;
    for (size_t i = 0; i < 181; i++) angleObstacle[i] = false;
    for (size_t i = 0; i < 181; i++) distObstacle[i] = 0.0;
    int iterDebugg = 0;

    if (debuggFile || debuggVector || debuggLimit) debugg<<endl<<"******************************* INICIO CICLO COMPUTE **************************************"<<endl;
    currentCycle++;

    for (size_t it=0;it<environmentInfo[layer].dist.size();it++) {

        if (debugging) cout<<"iterDebugg: "<<iterDebugg++<<endl;

        if (debuggFile || debuggVector) debugg<<"Ciclo actual: "<<currentCycle<<endl;
        
        // Actualizacion de datos de los puntos guardados en la memoria a corto plazo
        computeEnvironment(rPose, it, debugging);

        if (debuggFile) debugg<<it<<" - regionDirection.boundary: "<<regionDirection.boundary<<endl;
    
        // Borrado de datos en la region delimitada por los angulos obtenidos en getRegionDirectionBoundary
        filterEnvironment(it, currentDirectionRegion, debugging);

        if (debuggFile) debugg<<endl<<"____________________________________________________________"<<endl;

        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //Eliminacion de las regiones pertenecientes a un obstaculo ajeno al que estamos siguiendo
    }
}

//********************************************************************************************
// Dependiendo de la actual region de direccion, se decide la region continua a esta.

// A continuacion, cuando se habla de limite se refiere al paso de 0º a 360º, o de -pi a pi
// en el barrido del sensor laser.

// Si se va hacia la derecha, se analiza primero si se sale del limite en la siguiente region. 
// Si es el caso, se tiene en cuenta que la siguiente region sera la 0. En caso de que no se 
// pase del limite, se calcularia el limite superior e inferior del rango de angulos que 
// componen la region.

// Si se va hacia la izquierda, primero se comprueba que no se pase del limite. En el caso 
// que se pasara de este limite, se le asignaria la siguiente region a la ultima region.
// Se debe tener en cuenta tambien que si la region actual es la region 1, y por lo tanto 
// la siguiente es la 0 se debe tratar de manera diferente a las demas regiones, siendo 
// el angulo del limite inferior [360º - (angulo de cada region) / 2], y el angulo del limite
// superior de [(angulo de cada region) / 2]
//********************************************************************************************

void CHolonomicEG::getRegionDirectionBoundary(char busyRegions[], int currentDirectionRegion, bool debugging) {

    if (getLocalDebugg) cout<<"currentDirectionRegion: "<<currentDirectionRegion<<endl;
    if (dir==Izquierda){
        if (debugging) cout<<"Entramos en derecha"<<endl;   // DEBUGG
        if (debugging) debugg<<"Entramos en derecha"<<endl; // DEBUGG
        // Si la siguiente region se pasa del limite del sensor laser en sentido antihorario y 
        // esta region esta libre
        if ((currentDirectionRegion+1)==a_numRegions&&(busyRegions[0]==0||busyRegions[0]==3)) { // Region 0
            regionDirection.angLowBound=180-division_reg/2;
            regionDirection.angHighBound=division_reg/2;
            regionDirection.boundary=true;
            if (debugging) cout<<"Entramos en la primera condicion derecha. currentDirectionRegion = "<<currentDirectionRegion<<endl;   // DEBUGG
            if (debugging) debugg<<"Entramos en la primera condicion derecha. currentDirectionRegion = "<<currentDirectionRegion<<endl; // DEBUGG
        // Si la siguiente region esta libre
        } else if ((currentDirectionRegion+1)<a_numRegions&&(busyRegions[currentDirectionRegion+1]==0||busyRegions[currentDirectionRegion+1]==3)) { // Resto de regiones
            regionDirection.angHighBound=(currentDirectionRegion+2)*division_reg-division_reg/2;
            regionDirection.angLowBound=(currentDirectionRegion+1)*division_reg-division_reg/2;
            regionDirection.boundary=false;
            if (debugging) cout<<"Entramos en la segunda condicion derecha. currentDirectionRegion = "<<currentDirectionRegion<<endl;   // DEBUGG
            if (debugging) cout<<"busyRegions[20] = "<<(int)busyRegions[20]<<endl;   // DEBUGG
            if (debugging) debugg<<"Entramos en la segunda condicion derecha. currentDirectionRegion = "<<currentDirectionRegion<<endl; // DEBUGG
            if (debugging) debugg<<"busyRegions[20] = "<<(int)busyRegions[20]<<endl; // DEBUGG
        }
        else { // Si la siguiente region no esta libre
            if (debugging) cout<<"No entramos en ninguna condicion de la derecha"<<endl;      // DEBUGG
            if (debugging) cout<<"currentDirectionRegion = "<<currentDirectionRegion<<endl;   // DEBUGG
            if (debugging) debugg<<"No entramos en ninguna condicion de la derecha"<<endl;    // DEBUGG
            if (debugging) debugg<<"currentDirectionRegion = "<<currentDirectionRegion<<endl; // DEBUGG
            regionDirection.angHighBound=0.0;
            regionDirection.angLowBound=0.0;
            regionDirection.boundary=false;
        }
    } else if(dir==Derecha) {
        // Si la siguiente region se pasa del limite del sensor laser en sentido horario y 
        // las region esta libre
        if (debugging) cout<<"Entramos en izquierda"<<endl;   // DEBUGG
        if (debugging) debugg<<"Entramos en izquierda"<<endl; // DEBUGG
        if ((currentDirectionRegion-1)==-1) currentDirectionRegion=a_numRegions; // Ultima region
        if ((currentDirectionRegion-1)==0&&(busyRegions[currentDirectionRegion-1]==0||busyRegions[currentDirectionRegion-1]==3)) { // Region 0
            regionDirection.angLowBound=180-division_reg/2;
            regionDirection.angHighBound=division_reg/2;
            regionDirection.boundary=true;
            if (debugging) cout<<"Entramos en la primera condicion izquierda. currentDirectionRegion = "<<currentDirectionRegion<<endl;   // DEBUGG
            if (debugging) debugg<<"Entramos en la primera condicion izquierda. currentDirectionRegion = "<<currentDirectionRegion<<endl; // DEBUGG
        }
        else if ((currentDirectionRegion-1)>0&&(busyRegions[currentDirectionRegion-1]==0||busyRegions[currentDirectionRegion-1]==3)) { // Resto de regiones
            // Si la siguiente region esta libre
            regionDirection.angHighBound=(currentDirectionRegion)*division_reg-division_reg/2;
            regionDirection.angLowBound=(currentDirectionRegion-1)*division_reg-division_reg/2;
            regionDirection.boundary=false;
            if (debugging) cout<<"Entramos en la segunda condicion izquierda. currentDirectionRegion = "<<currentDirectionRegion<<endl;   // DEBUGG
            if (debugging) debugg<<"Entramos en la segunda condicion izquierda. currentDirectionRegion = "<<currentDirectionRegion<<endl; // DEBUGG
        }
        else { // Si la siguiente region no esta libre
            if (debugging) cout<<"No entramos en ninguna condicion de la izquierda"<<endl;
            if (debugging) debugg<<"No entramos en ninguna condicion de la izquierda"<<endl;
            regionDirection.angHighBound=0.0;
            regionDirection.angLowBound=0.0;
            regionDirection.boundary=false;
        }
    }
}

//*************************************************************************************
// Con esta funcion recogemos los datos de los obstaculos que detecta el sensor laser
// del robot. Estos datos pasan por un filtrado delimitado por los limites obtenidos
// con la ejecucion de la funcion getLocalObstacleBoundary_ten
//*************************************************************************************

void CHolonomicEG::getSensorPoints(const std::vector<float> &scan, char busyRegions[], const mrpt::math::TPose2D &rPose, const float maxRange, bool debugging) {

    int regInRatio;
    double currentAngle;

    if (debugging) cout<<"Valores angulos"<<endl;
    for (size_t i = 0; i < scan.size(); i++) {
        if (scan[i] < 0.98) {
            double anglePoint = i*180.0/scan.size();
            bool inFollowedObstacle = (Obstacle.falseGap && ((anglePoint >= Obstacle.lowAngle && anglePoint <= 180.0) ||
            (anglePoint < Obstacle.highAngle && anglePoint >= 0.0))) || (!Obstacle.falseGap && (anglePoint >= Obstacle.lowAngle &&
            anglePoint < Obstacle.highAngle)); // Podemos volver la condicion mas estricta teniendo tambien en cuenta el angulo maximo de 180 y minimo de 0
            
            if (inFollowedObstacle){
                environmentInfo[layer].belongToObstacle.push_back(false);

                // Distancia del punto
                environmentInfo[layer].dist.push_back(scan[i]*maxRange); 
                // Angulo en el que se encuentra el punto
                environmentInfo[layer].ang.push_back(i*180.0/scan.size()); 
                // Coordenada X de la posicion del punto en coordenadas absolutas
                environmentInfo[layer].Xobs.push_back(environmentInfo[layer].dist[environmentInfo[layer].iter]*
                    cos((environmentInfo[layer].ang[environmentInfo[layer].iter]*2 - 180)*(M_PI/180))+rPose.x);
                // Coordenada Y de la posicion del punto en coordenadas absolutas
                environmentInfo[layer].Yobs.push_back(environmentInfo[layer].dist[environmentInfo[layer].iter]*
                    sin((environmentInfo[layer].ang[environmentInfo[layer].iter]*2 - 180)*(M_PI/180))+rPose.y);
        
                environmentInfo[layer].Xfin.push_back(0.0);
                environmentInfo[layer].Yfin.push_back(0.0);
                if ((environmentInfo[layer].ang[environmentInfo[layer].iter] < division_reg/2)||(environmentInfo[layer].ang[environmentInfo[layer].iter] >= (180-division_reg/2))) {
                    busyRegions[0]=1; // Debido a que la region 0 esta en un rango de angulos especial, debemos tratarlo a parte
                } else if (environmentInfo[layer].ang[environmentInfo[layer].iter] >= division_reg/2) {
                    regInRatio=(int)trunc(((environmentInfo[layer].ang[environmentInfo[layer].iter])/division_reg)+0.5); // Añadimos 0.5 debido al desfase de angulos que hay
                    busyRegions[regInRatio]=1;
                }
                environmentInfo[layer].iter++; // El iterador va contando la cantidad de obstaculos detectados
            }
        }
    }
}

//*************************************************************************************
// Se realiza un reset de los datos guardados en la memoria a corto plazo del robot,
// y ademas, se limpian todas las lineas que dibujan las regiones que forman parte del
// barrido del sensor laser del robot
//*************************************************************************************

void CHolonomicEG::reset(void) {
    // Limpieza de todas las lineas que forman las regiones del barrido del sensor
    // laser del robot
    a_gl_T2_regionsFree ->clear();
    a_gl_T2_regionsFree ->appendLine(0,0,0,0,0,0);
    a_gl_T2_regionsOccup->clear();
    a_gl_T2_regionsOccup->appendLine(0,0,0,0,0,0);
    a_gl_T2_targetFree  ->clear();
    a_gl_T2_targetFree  ->appendLine(0,0,0,0,0,0);
    a_gl_T2_targetOccup ->clear();
    a_gl_T2_targetOccup ->appendLine(0,0,0,0,0,0);
    a_gl_T2_direccion -> clear();
    a_gl_T2_direccion ->appendLine(0,0,0,0,0,0);

    // Reseteo de los datos contenidos en la memoria a corto plazo del robot
    environmentInfo[layer].dist.clear();
    environmentInfo[layer].ang.clear();
    environmentInfo[layer].Xobs.clear();
    environmentInfo[layer].Yobs.clear();
    environmentInfo[layer].Xfin.clear();
    environmentInfo[layer].Yfin.clear();
    environmentInfo[layer].iter=0;
    debugg<<"Reseteado parametros, size: "<<environmentInfo[layer].ang.size()<<endl;
}

//**********************************************************************************
// Esta funcion devuelve la region en la que se encuentra la direccion que sigue
// el robot.
// En general la funcion diferencia dos casos.
// - Si la region del objetivo esta ocupado. La region de direccion vendra dada
// por la region siguiente a la region del objetivo, dependiendo si se esta
// siguiendo un sentido horario o un sentido antihorario.
//- Si la region del objetivo esta libre, primero se buscara una region ocupada
// dependiendo del sentido que se siga, y posteriormente se buscara una region libre
// en este mismo sentido
//**********************************************************************************

int CHolonomicEG::targetRegion(int m_regionTarget, char busyRegions[]) {

    if (directionDebugg) cout<<"m_regionTarget: "<<m_regionTarget<<endl;
    if (directionDebugg) debugg<<"m_regionTarget: "<<m_regionTarget<<endl;
    
    int directionTarget = -1;
    unsigned int n = 0;
        
    if (busyRegions[m_regionTarget] == 2){
        unsigned int iter = 0;
        n = m_regionTarget;    

        while (iter < a_numRegions + 1) {

            iter++;
            if (busyRegions[n] == 0 || busyRegions[n] == 3){
                directionTarget = n;
                break;
            }
            if (dir == Derecha) {
                if (n == (a_numRegions-1)) {
                    n = 0;
                }else {
                    n++;
                }
            } else {
                if (n == 0) {
                    n = a_numRegions - 1;
                } else {
                    n--;
                }
            }
        }
        if (directionDebugg) cout<<"Entramos en busqueda de una region libre. directionTarget: "<<directionTarget<<endl;   // DEBUGG
        if (directionDebugg) debugg<<"Entramos en busqueda de una region libre. directionTarget: "<<directionTarget<<endl; // DEBUGG

    } else {
        unsigned int iter = 0;
        n = m_regionTarget;
            
        while (iter < a_numRegions + 1) {
                
            iter++;
            if (directionDebugg) cout<<"busyRegions["<<n<<"]: "<<(int)busyRegions[n]<<endl;   // DEBUGG
            if (directionDebugg) debugg<<"busyRegions["<<n<<"]: "<<(int)busyRegions[n]<<endl; // DEBUGG

            if (busyRegions[n] == 1 || busyRegions[n] == 2) {
                directionTarget = n;
                break;
            }
            if (dir == Derecha) {
                if (n == (a_numRegions - 1)){
                    n = 0;
                }else{
                    n++;
                }  
            }
            else {
                if (n == 0) {
                    n = a_numRegions - 1;
                } else {
                    n--;
                }
            }
        }

        if (directionDebugg) cout<<"Cuando ya hemos encontrado region ocupada. directionTarget: "<<directionTarget<<endl;
        if (directionDebugg) debugg<<"Cuando ya hemos encontrado region ocupada. directionTarget: "<<directionTarget<<endl;

        iter = 0;
    
        // Buscar soluciones para las condiciones con "dir"

        while (iter<a_numRegions + 1) {

            iter++;
            if (busyRegions[n] == 0 || busyRegions[n] == 3) {
                directionTarget = n;
                break;
            }
            if (dir == Derecha) {
                if (n == (a_numRegions - 1)){
                    n = 0;
                } else {
                    n++;
                }  
            }
            else {
                if (n == 0) {
                    n = a_numRegions - 1;
                } else {
                    n--;
                }
            }
        }

        if (directionDebugg) cout<<"Entramos en busqueda de una region ocupada y luego libre. directionTarget: "<<directionTarget<<endl;   // DEBUGG
        if (directionDebugg) debugg<<"Entramos en busqueda de una region ocupada y luego libre. directionTarget: "<<directionTarget<<endl; // DEBUGG
    }

    return directionTarget;
}

//*****************************************************************************************
// Esta funcion se encarga de analizar en que estado se encuentra actualmente el 
// robot.
// Estos estados pueden ser los siguientes:
// - Boundary Following Right (BFR). Cuando el robot este rodeando el obstaculo encontrado
//   por la derecha.
// - Boundary Following Left (BFL). Cuando el robot este rodeando el obstaculo encontrado
//   por la izquierda.
// - Motion To Goal (MTG). Cuando el robot se dirija directamente hacia el objetivo
//*****************************************************************************************

std::string CHolonomicEG::filterState(int m_currentDirectionRegion, int m_lastDirectionRegion, int m_targetRegion, char m_targetState, bool *toTarget, bool lastLayerDeleted) {

    static string state;
    static int count=31; // Ponemos este contador a causa de que en caso de no ponerlo el robot se desbiaria de su objetivo
    cout<<endl<<endl;
    if (dir == Derecha) {
        if (directionDebugg) cout<<"Direccion derecha"<<endl;
        if (directionDebugg) debugg<<"Direccion derecha"<<endl;
        // En caso de que el objetivo este en la misma region que nuestra direccion y la region anterior
        // de nuestra direccion es justo la anterior, teniendo en cuenta un sentido horario. Iriamos hacia
        // el objetivo
        if (m_currentDirectionRegion == m_targetRegion && m_targetState == 3 && !lastLayerDeleted) {
            if (directionDebugg) cout<<"Ahora vamos a motionToGoal"<<endl;
            if (directionDebugg) debugg<<"Ahora vamos a motionToGoal"<<endl;
            state = "MTG";
            *toTarget = true;
            count = 0;
        } else if (m_currentDirectionRegion == a_numRegions - 1 && m_currentDirectionRegion == m_targetRegion && m_targetState == 3 && !lastLayerDeleted) {
            if (directionDebugg) cout<<"Ahora vamos a motionToGoal en borde"<<endl;
            if (directionDebugg) debugg<<"Ahora vamos a motionToGoal en borde"<<endl;
            state = "MTG";
            *toTarget = true;
            count = 0;
        } else {
            if (directionDebugg) cout<<"No vamos a motionToGoal"<<endl;
            if (directionDebugg) debugg<<"No vamos a motionToGoal"<<endl;
            if (count > 30) {
                state = "BFR";
                *toTarget = false;

                if (directionDebugg) if (directionDebugg) cout<<"Count mayor a 30"<<endl;
                if (directionDebugg) debugg<<"Count mayor a 30"<<endl;
            }
            count++;
        }

    } else if (dir == Izquierda) {
        if (directionDebugg) cout<<"Direccion izquierda"<<endl;
        if (directionDebugg) debugg<<"Direccion izquierda"<<endl;
        // En caso de que el objetivo este en la misma region que nuestra direccion y la region anterior
        // de nuestra direccion es justo la anterior, teniendo en cuenta un sentido antihorario. Iriamos hacia
        // el objetivo
        if (((m_currentDirectionRegion == m_targetRegion)) && m_targetState == 3 && !lastLayerDeleted) {
            if (directionDebugg) cout<<"Ahora vamos a motionToGoal"<<endl;
            if (directionDebugg) debugg<<"Ahora vamos a motionToGoal"<<endl;
            state = "MTG";
            *toTarget = true;
            count = 0;
        } else if (m_currentDirectionRegion == 0 && m_currentDirectionRegion == m_targetRegion && m_targetState == 3 && !lastLayerDeleted) {
            if (directionDebugg) cout<<"Ahora vamos a motionToGoal en borde"<<endl;
            if (directionDebugg) debugg<<"Ahora vamos a motionToGoal en borde"<<endl;
            state = "MTG";
            *toTarget = true;
            count = 0;
        } else {
            
            if (directionDebugg) cout<<"No vamos a motionToGoal"<<endl;
            if (directionDebugg) debugg<<"No vamos a motionToGoal"<<endl;
            if (count > 30){
                state = "BFL";
                *toTarget = false;

                if (directionDebugg) cout<<"Count mayor a 30"<<endl;
                debugg<<"Count mayor a 30"<<endl;
            }
            count++;
        }
    }

    if (directionDebugg) cout<<"Datos: "<<endl<<"m_lastDirectionRegion = "<<m_lastDirectionRegion<<"  m_currentDirectionRegion: "<<m_currentDirectionRegion<<endl;
    if (directionDebugg) debugg<<"Datos: "<<endl<<"m_lastDirectionRegion = "<<m_lastDirectionRegion<<"  m_currentDirectionRegion: "<<m_currentDirectionRegion<<endl;

    return state;
}

void CHolonomicEG::deleteSmallGaps(std::vector<int> m_iniGap, std::vector<int> m_endGap, bool debugg) {
     // MENOS BUCLE FOR QUE SE ENCARGA DE ELIMINAR LOS GAPS PEQUEÑOS
    if (debugg) {
        for (unsigned int i = 0; i < m_iniGap.size(); i++){
            cout<<"m_iniGap["<<i<<"]: "<<m_iniGap[i]<<"   m_endGap["<<i<<"]: "<<m_endGap[i]<<endl;
        }
    }

    int gapsDeleted = 0;
    int gapSize = m_iniGap.size();
    if (debugg) cout<<"¡¡¡¡Tienen un size de: "<<gapSize<<"!!!!!!!"<<endl; // DEBUGG
    for (int kgap = 0; kgap<gapSize; kgap++) {
        if (debugg) cout<<"Valores gap: m_endGap["<<kgap-gapsDeleted<<"] = "<<m_endGap[kgap-gapsDeleted]<<"   m_iniGap["<<kgap-gapsDeleted<<"] = "<<m_iniGap[kgap-gapsDeleted]<<endl; // DEBUGG
        if (((int)(m_endGap[kgap-gapsDeleted])-(int)(m_iniGap[kgap-gapsDeleted]))<=4.0) {
            m_iniGap.erase(m_iniGap.begin()+kgap-gapsDeleted);
            m_endGap.erase(m_endGap.begin()+kgap-gapsDeleted);
            if (debugg) cout<<"¡¡¡¡¡Gap numero "<<kgap-gapsDeleted<<" eliminado!!!!!"<<endl; // DEBUGG
            gapsDeleted++;
        }
    }

    Gaps.ini = m_iniGap;
    Gaps.end = m_endGap;

    // Ordenamos la estructura de gaps para facilitar su posterior tratamiento
    int auxIniGap;
    int auxEndGap;

    for (size_t n = 0; n < m_iniGap.size(); n++) {
        for (size_t m = 0; m < n; m++) {
            if (Gaps.ini[m]>Gaps.ini[n]) {
                auxIniGap = Gaps.ini[n];
                Gaps.ini[n] = Gaps.ini[m];
                Gaps.ini[m] = auxIniGap;
            }

            if (Gaps.end[m]>Gaps.end[n]) {
                auxEndGap = Gaps.end[n];
                Gaps.end[n] = Gaps.end[m];
                Gaps.end[m] = auxEndGap;
            }
        }
    }

    if (debugg) {
        cout<<"********************************"<<endl<<"Los valores del nuevo vector son los siguientes en funcion:"<<endl;
        for (size_t gh = 0; gh < Gaps.ini.size(); gh++){
            cout<<"Gaps.ini["<<gh<<"] = "<<Gaps.ini[gh]<<endl;
            cout<<"Gaps.end["<<gh<<"] = "<<Gaps.end[gh]<<endl;
        }
    }
}

//********************************************************************************************
// Con esta funcion se obtienen los datos de la memoria a corto plazo (angulos y distancias) y
// se ordenan los datos en funcion del valor de los angulos, de menor a mayor.
//********************************************************************************************

void CHolonomicEG::orderEnvironmentInfo(std::vector<double> &orderedAngles, std::vector<double> &orderedDistances, const std::vector<float> &scan, bool debugging) {

    if (debuggLimit) debugg<<endl<<"***********orderEnvironmentInfo_four*********************"<<endl<<endl;
    if (debuggLimit) debugg<<"Obstacle.lowAngle = "<<Obstacle.lowAngle<<" Obstacle.highAngle = "<<Obstacle.highAngle<<" Obstacle.falseGap = "<<Obstacle.falseGap<<endl<<endl;
    if (debuggLimit) cout<<"-----scans iniciales-----"<<endl;
    if (debuggLimit) cout<<"Obstacle.lowAngle = "<<Obstacle.lowAngle<<" Obstacle.highAngle = "<<Obstacle.highAngle<<" Obstacle.falseGap = "<<Obstacle.falseGap<<endl<<endl;
    
    //*************************************************************************************
    // Se comprueba que puntos estan guardados en la memoria a corto plazo y se obtiene su 
    // angulo equivalente al del sensor laser. Guardando asi cada punto contenido en la me-
    // moria a corto plazo de manera ordenada
    //*************************************************************************************

    for (int i = 0; i < 181; i++) {
        if (angleObstacle[i]){
            double scanAngle = i*180.0/181.0;
            if (debuggLimit) debugg<<"scanAngle["<<i<<"] = "<<scanAngle<<endl;
            if (debuggLimit) cout<<"scanAngle["<<i<<"] = "<<scanAngle<<" ";

            orderedAngles.push_back(scanAngle);
            orderedDistances.push_back(scan[i]);
        }
    }

    double aux;
    double distAux;
    bool centinela = true;

    if (debugging) cout<<"Entramos a ordenar el vector dinamico"<<endl;
    if (debugging) debugg<<"Entramos a ordenar el vector dinamico"<<endl;
    if (debugging) cout<<"Antes de ordenar: "<<endl;
    if (debugging) debugg<<"Antes de ordenar: "<<endl;

    if (debugging||debuggLimit){
        for (unsigned int i = 0; i < orderedAngles.size(); i++) {
            debugg<<"ang["<<i<<"] = "<<orderedAngles[i]<<"   dist["<<i<<"] = "<<orderedDistances[i]<<endl;
        } 
    } 

    //*************************************************************************************
    // En caso de que no se hayan ordenado completamente los datos el bucle anterior, se
    // acaban de ordenar de manera correcta los datos en este bucle
    //*************************************************************************************
    for (unsigned int i = 0; i < orderedAngles.size() && centinela == true; i++) {
        centinela = false;
        
        for (int j = 1; j < orderedAngles.size(); j++) {
            if (orderedAngles[j] < orderedAngles[j-1]) {
                aux = orderedAngles[j-1];
                orderedAngles[j-1] = orderedAngles[j];
                orderedAngles[j] = aux;

                distAux = orderedDistances[j-1];
                orderedDistances[j-1] = orderedDistances[j];
                orderedDistances[j] = distAux;

                centinela = true;
            }
        }
    }
}


bool CHolonomicEG::robotFits(const mrpt::math::TPose2D &rPose, double distanceA, double angleA, double distanceB, double angleB, bool debugging) {

    if (debugging) {
        debugg<<"Robot = ("<<rPose.x<<", "<<rPose.y<<") distanceA = "<<distanceA<<" angleA = "<<angleA<<" distanceB = "<<distanceB<<" angleB = "<<angleB<<endl;
    }

    double angRadA = (angleA*2.0-180.0) * (M_PI/180.0);
    double angRadB = (angleB*2.0-180.0) * (M_PI/180.0);
    double angDebuggA = (angleA*2-180) * (M_PI/180);
    double angDebuggB = (angleB*2-180) * (M_PI/180);

    if (debugging) debugg<<"angRadA = "<<angRadA<<" angRadB = "<<angRadB<<" angDebuggA = "<<angDebuggA<<" angDebuggB = "<<angDebuggB<<endl;

    double XpointA = distanceA * cos(angRadA);
    double YpointA = distanceA * sin(angRadA);
    double XpointB = distanceB * cos(angRadB);
    double YpointB = distanceB * sin(angRadB);

    double distance = sqrt(pow(XpointB-XpointA, 2) + pow(YpointB-YpointA, 2));

    if (debugging) debugg<<"XpointA = "<<XpointA<<" YpointA = "<<YpointA<<" XpointB = "<<XpointB<<" YpointB = "<<YpointB<<" Distance btw points = "<<distance;

    if (distance > (rDiameter/100.0)) {
        debugg<<" El robot SI pasa"<<endl;
        return true;
    } else {
        debugg<<" El robot NO pasa"<<endl;
        return false;
    }
}

//*************************************************************************************
// Esta funcion se utiliza para obtener los angulos que indican los limites del angulo
// que el robot esta rodeando en el momento
//*************************************************************************************

void CHolonomicEG::setAnglesLimit(std::vector<double> &orderedAngles, std::vector<double> &orderedDistances, double desiredDirection, const float maxRange, bool debugging) {

    if (debugging) debugg<<"desiredDirection = "<<desiredDirection<<endl;
    if (debuggLines) cout<<"Line 2051"<<endl;
    double desiredDirectionDeg = (180.0/M_2PI)*(desiredDirection + M_PI); // Direccion a la que se dirige el robot en grados
    double angleObstacleFollowed; // Perpendicular a la direccion que sigue el robot indicando
                                  // el angulo en el que esta el obstaculo que se esta rodeando

    if (debugging) debugg<<"desiredDirectionDeg = "<<desiredDirectionDeg<<endl;

    if (debuggLines) cout<<"Line 2060"<<endl;
    // Obtencion del angulo del obstaculo que se esta rodeando, dependiendo de
    // la direccion que actualmente estamos siguiendo
    if (dir == Derecha) {
        if (debuggLines) cout<<"Line 2064"<<endl;
        if (desiredDirectionDeg - 45 < 0) {
            angleObstacleFollowed = 180.0 - (45.0 - desiredDirectionDeg);
        } else {
            angleObstacleFollowed = desiredDirectionDeg - 45;
        }
    } else if (dir == Izquierda) {
        if (debuggLines) cout<<"Line 2071"<<endl;
        if (desiredDirectionDeg + 45 > 180) {
            angleObstacleFollowed = 45.0 - (180.0 - desiredDirectionDeg);
        } else {
            angleObstacleFollowed = desiredDirectionDeg + 45;
        }
    }

    if (debugging) debugg<<"angleObstacleFollowed = "<<angleObstacleFollowed<<" con dir = "<<dir<<endl;

    if (debuggLines) cout<<"Line 2085"<<endl;
    // Se busca entre los angulos guardados en la memoria dinamica, el angulo mas cercano
    // al angulo que indica donde se encuentra el obstaculo que estamos rodeando
    double closerDiffAngle = 180.0;
    int closerAngleIndex;
    for (int i = 0; i < orderedAngles.size(); i++) {
        if (abs(angleObstacleFollowed - orderedAngles[i]) < closerDiffAngle) {
            closerDiffAngle = abs(angleObstacleFollowed - orderedAngles[i]);
            closerAngleIndex = i;
        }
    }

    if (debugging) debugg<<"closerAngle = "<<orderedAngles[closerAngleIndex]<<" with index = "<<closerAngleIndex<<endl;
    if (debuggLines) cout<<"Line 2099"<<endl;
    int highIndex = orderedAngles.size() - 1;
    int lowIndex = 0;
    double distanceLimit = 0.996;
    bool lowIndexInLimit = false;
    bool highIndexInLimit = false;

    //******************************************************************************************
    // Se busca el angulo que indica el limite superior del obstaculo rodeado. Este angulo
    // se encuentra comprobando que cada uno de los pares de puntos detectados en sentido 
    // antihorario son contiguos y el robot cabe entre ellos, es decir que la distancia entre 
    // los dos puntos analizados actualmente es menor que el diametro del robot. Cuando se
    // de el caso que hay dos puntos separados, marcaremos el menor de esos puntos como
    // limite superior. Por otro lado, tambien se puede encontrar el limite cuando dos puntos
    // son contiguos, pero su distancia es mayor a la del diametro del robot. Por ultimo,
    // el limite tambien puede ser encontrado cuando llegamos a los limites del radar (360º)
    //****************************************************************************************** 
    if (debuggLines) cout<<"Line 2116"<<endl;
    if (debugging) debugg<<"Limite superior ";
    for (int i = closerAngleIndex+1; i < orderedAngles.size(); i++) {
        if (abs(orderedAngles[i-1] - orderedAngles[i]) < 2.0) {
            if (debuggLines) cout<<"Line 2121"<<endl;
            double angRadA = (orderedAngles[i-1]*2.0-180.0) * (M_PI/180.0);
            double angRadB = (orderedAngles[i]*2.0-180.0) * (M_PI/180.0); 
            double XpointA = orderedDistances[i-1] * maxRange * cos(angRadA);
            double YpointA = orderedDistances[i-1] * maxRange * sin(angRadA);
            double XpointB = orderedDistances[i] * maxRange * cos(angRadB);
            double YpointB = orderedDistances[i] * maxRange * sin(angRadB);
            if (sqrt(pow(XpointB - XpointA , 2)+pow(YpointB - YpointA, 2)) >= rDiameter/100.0) {
                highIndex = i-1;
                if (debugging) debugg<<"Parado por distancia entre index "<<i-1<<" y "<<i<<endl;
                break;
            } else if (i == orderedAngles.size() - 1) {
                highIndex = orderedAngles.size() - 1;
                if (debugging) debugg<<"Parado por final bucle entre index "<<i-1<<" y "<<i<<endl;
                break;
            }
        } else {
            if (debuggLines) cout<<"Line 2140"<<endl;
            highIndex = i-1;
            if (debugging) debugg<<"Parado entre index "<<i-1<<" y "<<i<<endl;
            break;
        }
    }
    if (debugging) debugg<<"Ultima posicion de orderedAngles = "<<orderedAngles[orderedAngles.size()-1];
    if (debugging) debugg<<" Distances = "<<orderedDistances[orderedDistances.size()-1]<<endl;
    if (debugging) debugg<<"Primera posicion de orderedAngles = "<<orderedAngles[0];
    if (debugging) debugg<<" Distances = "<<orderedDistances[0]<<endl;

    if (debuggLines) cout<<"Line 2152"<<endl;
    if (debuggLines) cout<<"highIndex = "<<highIndex<<" lowIndex = "<<lowIndex<<endl;
    if (debuggLines) cout<<"orderedAngles.size() = "<<orderedAngles.size()<<endl;
    //**********************************************************************************************
    // Cuando el limite indicado forma parte de unos de los limites propios del radar, se comprueba
    // si el obstaculo contiene un limite que pasa de ahi
    //**********************************************************************************************
    if (orderedAngles.size() > 0) {
        if (highIndex == orderedAngles.size()-1 && orderedAngles[0]<2.0 && orderedDistances[0] < distanceLimit) {
            if (debuggLines) cout<<"Line 2158"<<endl;
            if (debugging) debugg<<"El campo pasa el limite superior"<<endl;
            highIndexInLimit = true;
        }
        if (lowIndex == 0 && orderedAngles[orderedAngles.size()-1]>178.0 && orderedDistances[orderedDistances.size()-1] < distanceLimit) {
            if (debuggLines) cout<<"Line 2163"<<endl;
            if (debugging) debugg<<"El campo pasa el limite inferior"<<endl;
            lowIndexInLimit = true;
        }
    }

    if (debuggLines) cout<<"Line 2168"<<endl;
    //******************************************************************************************
    // Se busca el angulo que indica el limite inferior del obstaculo rodeado. Este angulo
    // se encuentra comprobando que cada uno de los pares de puntos detectados en sentido 
    // horario son contiguos y el robot cabe entre ellos, es decir que la distancia entre 
    // los dos puntos analizados actualmente es menor que el diametro del robot. Cuando se
    // de el caso que hay dos puntos separados, marcaremos el mayor de esos puntos como
    // limite inferior. Por otro lado, tambien se puede encontrar el limite cuando dos puntos
    // son contiguos, pero su distancia es mayor a la del diametro del robot. Por ultimo,
    // el limite tambien puede ser encontrado cuando llegamos a los limites del radar (0º)
    //******************************************************************************************
    if (debugging) debugg<<"Limite inferior ";
    if (orderedAngles.size() > 0) {
        for (int i = closerAngleIndex-1; i >= 0; i--) {
            if (abs(orderedAngles[i+1] - orderedAngles[i]) < 2.0) {
                double angRadA = (orderedAngles[i+1]*2.0-180.0) * (M_PI/180.0);
                double angRadB = (orderedAngles[i]*2.0-180.0) * (M_PI/180.0); 
                double XpointA = orderedDistances[i+1] * maxRange * cos(angRadA);
                double YpointA = orderedDistances[i+1] * maxRange * sin(angRadA);
                double XpointB = orderedDistances[i] * maxRange * cos(angRadB);
                double YpointB = orderedDistances[i] * maxRange * sin(angRadB);
                if (sqrt(pow(XpointB - XpointA , 2)+pow(YpointB - YpointA, 2)) >= rDiameter/100.0) {
                    lowIndex = i+1;
                    if (debugging) debugg<<"Parado por distancia entre index "<<i+1<<" y "<<i<<endl;
                    break;
                } else if(i == 0) {
                    lowIndex = 0;
                    if (debugging) debugg<<"Parado por final de bucle entre index "<<i+1<<" y "<<i<<endl;
                    break;
                }
            } else {
                lowIndex = i+1;
                if (debugging) debugg<<"Parado entre index "<<i+1<<" y "<<i<<endl;
                break;
            }
        }
    }

    if (debuggLines) cout<<"Line 2211"<<endl;

    for (int i = 0; i < orderedAngles.size(); i++) {
        if (debugging) debugg<<"orderedAngles["<<i<<"] = "<<orderedAngles[i]<<"  orderedDistances["<<i<<"] = "<<orderedDistances[i]<<endl;
    }

    int orderedAnglesSize = orderedAngles.size();

    if (debugging) debugg<<"size = "<<orderedAngles.size()<<" lowIndex = "<<lowIndex<<" highIndex = "<<highIndex<<endl;
    if (debugging) debugg<<"Orden lowIndex"<<endl;
    if (debuggLines) cout<<"Line 2226"<<endl;
    //******************************************************************************************
    // Si el obstaculo no sobrepasa el angulo que limita la zona superior, se borraran
    // todos los puntos de la memoria desde el angulo 0º hasta el limite inferior del obstaculo
    //******************************************************************************************
    if (!highIndexInLimit) {
        if (debuggLines) cout<<"Line 2232"<<endl;
        for (int i = 0; i < lowIndex; i++) {
            if (debugging) debugg<<"Eliminamos orderedAngles["<<0<<"] y orderedDistances["<<0<<"]"<<endl;
            orderedAngles.erase(orderedAngles.begin());
            orderedDistances.erase(orderedDistances.begin());
            if (debugging) cout<<"Eliminados"<<endl;
            if (debugging) debugg<<"Eliminados"<<endl;
        }

    } else {
        if (debuggLines) cout<<"Line 2243"<<endl;
        if (debugging) debugg<<"NO ELIMINAMOS POR highIndexInLimit"<<endl;
    }

    if (debugging) debugg<<"Orden highIndex"<<endl;
    if (debugging) debugg<<"Calculo: "<<orderedAnglesSize - 1 - highIndex<<" orderedAngles.size() = "<<orderedAngles.size()<<" highIndex = "<<highIndex<<" orderedAnglesSize = "<<orderedAnglesSize<<endl;
    if (debuggLines) cout<<"Line 2253"<<endl;
    //********************************************************************************************
    // Si el obstaculo no sobrepasa el angulo que limita la zona inferior, se borraran
    // todos los puntos de la memoria desde el angulo 360º hasta el limite superior del obstaculo
    //********************************************************************************************
    if (!lowIndexInLimit) {
        if (debuggLines) cout<<"Line 2259"<<endl;
        for (int i = 0; i < (orderedAnglesSize - 1 - highIndex); i++) {
            if (debugging) debugg<<"Eliminamos orderedAngles["<<orderedAngles.size()-1<<"] y orderedDistances["<<orderedDistances.size()-1<<"]"<<endl;
            orderedAngles.erase(orderedAngles.end()-1);
            orderedDistances.erase(orderedDistances.end()-1);
            if (debugging) debugg<<"Eliminados"<<endl;
        }

    } else {
        if (debuggLines) cout<<"Line 2270"<<endl;
        if (debugging) debugg<<"NO ELIMINAMOS POR lowIndexInLimit"<<endl;
    }

    if (debuggLines) cout<<"Line 2275"<<endl;
    for (int i = 0; i < orderedAngles.size(); i++) {
        if (debugging) debugg<<"orderedAngles["<<i<<"] = "<<orderedAngles[i]<<"  orderedDistances["<<i<<"] = "<<orderedDistances[i]<<endl;
    }
}

//*****************************************************************************************************
// Mediante el uso de esta funcion obtenemos los limites del obstaculo que estamos siguiendo, paso asi
// unicamente aceptar datos provenientes de este obstaculo, ignorando todos los demas que se nos
// presenten
//*****************************************************************************************************

void CHolonomicEG::getLocalObstacleBoundary_ten(const std::vector<float> &scan, const mrpt::math::TPose2D &rPose, const float maxRange, double desiredDirection, int currentDirectionRegion, bool debugging) {
    
    std::vector<double> orderedAngles;    // Se guardan los datos de los angulos de manera ordenada
    std::vector<double> orderedDistances; // Se guardan las distancias de los puntos captados de manera ordenada
    std::vector<double> lowAngles;
    std::vector<double> highAngles;
    std::vector<bool> falseGaps;

    if (debugging) debugg<<"Entramos en getLocalObstacleBoundary_five totalObstacle"<<endl;
    if (debuggLines) cout<<"Line 2281"<<endl;

    // Ordenamos los datos recogidos por la memoria dinamica del robot
    orderEnvironmentInfo(orderedAngles, orderedDistances, scan, debugging);

    //******************* DEBBUGG *********************
    if (debuggSmallGap) {
        debugg<<"------------ESTADO INICIAL-------------"<<endl;
        cout<<"------------ESTADO INICIAL-------------"<<endl;
        for (int i = 0; i < orderedAngles.size(); i++) {
            debugg<<"orderedAngles["<<i<<"] = "<<orderedAngles[i]<<"  orderedDistances["<<i<<"] = "<<orderedDistances[i]<<endl;
            cout<<"orderedAngles["<<i<<"] = "<<orderedAngles[i]<<"  orderedDistances["<<i<<"] = "<<orderedDistances[i]<<endl;
        }
    }
    //***********************************************
    if (debuggLines) cout<<"Line 2295"<<endl;

    // Obtenemos los angulos que indican los limites del obstaculo que se rodea
    setAnglesLimit(orderedAngles, orderedDistances, desiredDirection, maxRange, false);

    //******************* DEBBUGG *********************
    if (debuggSmallGap) {
        debugg<<"------------ESTADO LIMITADO-------------"<<endl;
        cout<<"------------ESTADO LIMITADO-------------"<<endl;
        for (int i = 0; i < orderedAngles.size(); i++) {
            debugg<<"orderedAngles["<<i<<"] = "<<orderedAngles[i]<<"  orderedDistances["<<i<<"] = "<<orderedDistances[i]<<endl;
            cout<<"orderedAngles["<<i<<"] = "<<orderedAngles[i]<<"  orderedDistances["<<i<<"] = "<<orderedDistances[i]<<endl;
        }
    }
    //***********************************************
    
    if (debugging) {
        for (unsigned int i = 0; i < orderedAngles.size(); i++){
            debugg<<"ang["<<i<<"] = "<<orderedAngles[i]<<endl;
        } 
    }

    float distanceLimit = 0.996; // Cualquier punto con una distancia menor a esta sera considerado
                                 // como detectado
    bool obstAtBeg = false; // Indica que el obstaculo contiene los dos limites del sensor laser
                            // (0º y 360º)
    bool pointsToAdd = false;
    int iterWithFalseGap; // Se utiliza este iterador para insertar valores al vector en ciertas posiciones

    if (debugging) debugg<<"distanceLimit = "<<distanceLimit<<endl;

    if (debugging) {
        for (unsigned int i = 0; i < scan.size(); i++) {
            if (scan[i] < distanceLimit) {
                debugg<<"scan["<<i<<"]: "<<i*180.0/scan.size()<<"   "<<endl;
            }
        }
    }

    if (debuggLines) cout<<"Line 2342"<<endl;
    if (orderedAngles.size() > 0) {
        //*************************************************************************************
        // Se añaden angulos en sentido antihorario
        //*************************************************************************************
        if (dir == Derecha) {
            if (debuggLines) cout<<"Line 2348"<<endl;
            if (debugging) debugg<<"Vamos a ordenar higher angles"<<endl;

            // Buscamos puntos de obstaculo que pertenezcan al mismo obstaculo en sentido antihorario

            //**************************************************************************
            // El angulo que se esta analizando actualmente sera añadido como parte del
            // ostaculo que se esta rodeando, si es mayor que el limite superior
            // del obstaculo, y ademas, su angulo es contiguo y la distancia
            // entre estos dos puntos es menor al diametro del robot
            //**************************************************************************

            for (unsigned int i = 0; i < scan.size(); i++) {
                if (scan[i] < distanceLimit){ // Si obstaculo dentro del rango de deteccion
                    double scanAngle = i*180.0/scan.size();
                    if (scanAngle > orderedAngles[orderedAngles.size()-1] && abs(scanAngle - orderedAngles[orderedAngles.size()-1]) < 2.0) {
                        if (!robotFits(rPose, orderedDistances[orderedDistances.size()-1]*maxRange, orderedAngles[orderedAngles.size()-1], scan[i]*maxRange, scanAngle, false)) {
                            orderedAngles.push_back(scanAngle);
                            orderedDistances.push_back(scan[i]);
                            if (debugging) debugg<<"Se cumple robot NO fits A"<<endl;
                            if (debugging) debugg<<"El angulo "<<scanAngle<<" cumple la condicion high con distancia "<<scan[i]<<endl;
                        }
                    }
                }
            }

            //******************* DEBBUGG *********************
            if (debuggSmallGap) {
                debugg<<"------------CASO A-------------"<<endl;
                for (int i = 0; i < orderedAngles.size(); i++) {
                    debugg<<"orderedAngles["<<i<<"] = "<<orderedAngles[i]<<"  orderedDistances["<<i<<"] = "<<orderedDistances[i]<<endl;
                }
            }
            //***********************************************

        //*************************************************************************************
        // Se añaden angulos en sentido horario
        //*************************************************************************************
        } else if (dir == Izquierda) {
            if (debuggLines) cout<<"Line 2389"<<endl;
            if (debugging) debugg<<"Vamos a ordenar lower angles"<<endl;

            //**************************************************************************
            // El angulo que se esta analizando actualmente sera añadido como parte del
            // ostaculo que se esta rodeando, si es mayor que el limite superior
            // del obstaculo, y ademas, su angulo es contiguo y la distancia
            // entre estos dos puntos es menor al diametro del robot
            //**************************************************************************

            for (int i = scan.size()-1; i >= 0 ; i--) {
                if (scan[i] < distanceLimit) { // Si obstaculo dentro del rango
                    double scanAngle = i*180.0/scan.size();
                    if (scanAngle < orderedAngles[0] && abs(scanAngle - orderedAngles[0]) < 2.0 ) {
                        if (!robotFits(rPose, orderedDistances[0]*maxRange, orderedAngles[0], scan[i]*maxRange, scanAngle, false)) {    
                            orderedDistances.insert(orderedDistances.begin(), scan[i]);
                            orderedAngles.insert(orderedAngles.begin(), scanAngle);
                            if (debugging) debugg<<"Se cumple robot NO fits B"<<endl;
                            if (debugging) debugg<<"El angulo "<<scanAngle<<" cumple la condicion low"<<endl;
                        }
                    }
                }
            }

            //******************* DEBBUGG *********************
            if (debuggSmallGap) {
                debugg<<"------------CASO B-------------"<<endl;
                for (int i = 0; i < orderedAngles.size(); i++) {
                    debugg<<"orderedAngles["<<i<<"] = "<<orderedAngles[i]<<"  orderedDistances["<<i<<"] = "<<orderedDistances[i]<<endl;
                }
            }
            //***********************************************
        }

        if (debuggLines) cout<<"Line 2429"<<endl;
        //****************************************************************************
        // En este trozo de codigo se utiliza para definir cuando el obstaculo que se
        // esta rodeando contiene los dos limites del sensor laser (0º y 360º)
        //****************************************************************************
        bool higherPlaceLimit_SA = false;
        bool lowerPlaceLimit_SA = false;
        bool higherPlaceLimit_OA = false;
        bool lowerPlaceLimit_OA = false;

        for (int j = 0; j < scan.size() && !obstAtBeg; j++) {
            if (scan[j] < distanceLimit) {
                double scanAngle = j*180.0/scan.size();
                if (scanAngle <= 180.0 && scanAngle > 175.5) higherPlaceLimit_SA = true;
                if (scanAngle >= 0.0 && scanAngle < 4.5) lowerPlaceLimit_SA = true;
            }
        }

        for (unsigned int i = 0; i < orderedAngles.size() && !obstAtBeg; i++) {
            if (orderedAngles[i] <= 180.0 && orderedAngles[i] > 175.5) higherPlaceLimit_OA = true;
            if (orderedAngles[i] >= 0.0 && orderedAngles[i] < 4.5) lowerPlaceLimit_OA = true;
        }

        if (((higherPlaceLimit_OA || higherPlaceLimit_SA ) && (lowerPlaceLimit_OA || lowerPlaceLimit_SA) && (lowerPlaceLimit_OA || higherPlaceLimit_OA)) || (lowerPlaceLimit_OA && higherPlaceLimit_OA)) {
            obstAtBeg = true;
            if (debugging) debugg<<"obstAtBeg = "<<obstAtBeg<<" with values: 1->"<<higherPlaceLimit_OA<<" 2->"<<higherPlaceLimit_SA<<" 3->"<<lowerPlaceLimit_OA<<" 4->"<<lowerPlaceLimit_SA<<endl;
            if (higherPlaceLimit_OA && lowerPlaceLimit_SA) iterWithFalseGap = 0;
            else if (lowerPlaceLimit_OA && higherPlaceLimit_SA) iterWithFalseGap = orderedAngles.size() - 1;
        }

        if (obstAtBeg == false) if (debugging) debugg<<"obstAtBeg = "<<obstAtBeg<<" No se cumplen condiciones"<<endl;
        debugg<<"obstAtBeg = "<<obstAtBeg<<endl;
        debugg<<"Obstacle.lowAngle = "<<Obstacle.lowAngle<<" Obstacle.highAngle = "<<Obstacle.highAngle<<" Obstacle.falseGap = "<<Obstacle.falseGap<<endl;

        //***********************************************

        if (debuggLines) cout<<"Line 2475"<<endl;
        //*************************************************************************************
        // Al poder darse la posibilidad de que el obstaculo que se esta rodeando contenga
        // los dos limites del sensor laser, se debe tener en cuenta, tanto para la adicion
        // de datos en sentido horario como en sentido antihorario
        //*************************************************************************************
        if (obstAtBeg == true) {
            if (debuggLines) cout<<"Line 2482"<<endl;
            if (debugging) debugg<<"Insertamos nuevos valores de puntos de obstaculos al vector que seguimos"<<endl;
            if (debugging) debugg<<"orderedAngles.size() = "<<orderedAngles.size()<<endl;

            //*****************************************************************************************
            // En sentido horario, se empiezan a añadir datos de puntos detectados por el sensor laser
            // desde el limite superior en sentido horario
            //*****************************************************************************************
            if (dir == Izquierda) {
                if (debuggLines) cout<<"Line 2495"<<endl;
                if (debugging) debugg<<"Rellenamos desde el final en el caso del sentido izquierdo (horario) con obstAtBeg"<<endl;
                for (int i = scan.size()-1; i >= 0 ; i--) {
                    int pointsAdded = 0;
                    if (scan[i] < distanceLimit) { // Si obstaculo dentro del rango
                        double scanAngle = i*180.0/scan.size();
                        if (abs(scanAngle - orderedAngles[orderedAngles.size()-1]) < 2.0 || abs(scanAngle - 180) < 2.0){
                            if (!robotFits(rPose, orderedDistances[orderedDistances.size()-1]*maxRange, orderedAngles[orderedAngles.size()-1], scan[i]*maxRange, scanAngle, false) || 
                                !robotFits(rPose, scan[scan.size()-1]*maxRange, 180.0, scan[i]*maxRange, scanAngle, false)) {
                                orderedDistances.insert(orderedDistances.end() - pointsAdded, scan[i]);
                                orderedAngles.insert(orderedAngles.end() - pointsAdded, scanAngle);
                                pointsAdded++;
                                debugg<<"Se cumple robot NO fits C1"<<endl;
                                if (debugging) debugg<<"El angulo "<<scanAngle<<" cumple la condicion low para izquierda"<<endl;
                            }
                        }
                    }
                }

                //******************* DEBBUGG *********************
                if (debuggSmallGap) {
                    debugg<<"------------CASO C1-------------"<<endl;
                    for (int i = 0; i < orderedAngles.size(); i++){
                        debugg<<"orderedAngles["<<i<<"] = "<<orderedAngles[i]<<"  orderedDistances["<<i<<"] = "<<orderedDistances[i]<<endl;
                    }
                }
                //***********************************************

            //*********************************************************************************************
            // En sentido antihorario, se empiezan a añadir datos de puntos detectados por el sensor laser
            // desde el limite inferior en sentido antihorario
            //*********************************************************************************************
            } else if (dir == Derecha) {
                if (debuggLines) cout<<"Line 2528"<<endl;
                if (debugging) debugg<<"Rellenamos desde el inicio en el caso del sentido derecho (antihorario) con obstAtBeg"<<endl;
                for (int i = 0; i < scan.size() ; i++) {
                    int pointsAdded = 0;
                    if (scan[i] < distanceLimit) { // Si obstaculo dentro del rango
                        double scanAngle = i*180.0/scan.size();
                        if (abs(scanAngle - orderedAngles[pointsAdded]) < 2.0 || abs(scanAngle) < 2.0){ //sin false gap
                            if (!robotFits(rPose, orderedDistances[pointsAdded]*maxRange, orderedAngles[pointsAdded], scan[i]*maxRange, scanAngle, false) || 
                                !robotFits(rPose, scan[0]*maxRange, 0.0, scan[i]*maxRange, scanAngle, false)) {
                                if (debugging) debugg<<"Step 1"<<endl;
                                orderedDistances.insert(orderedDistances.begin() + pointsAdded, scan[i]);
                                if (debugging) debugg<<"Step 2"<<endl;
                                orderedAngles.insert(orderedAngles.begin() + pointsAdded, scanAngle);
                                pointsAdded++;
                                if (debugging) debugg<<"Se cumple robot NO fits C2"<<endl;
                                if (debugging) debugg<<"El angulo "<<scanAngle<<" cumple la condicion high para derecha"<<endl;
                            }
                        }
                    }
                }

                //******************* DEBBUGG *********************
                if (debuggSmallGap){
                    debugg<<"------------CASO C2-------------"<<endl;
                    for (int i = 0; i < orderedAngles.size(); i++){
                        debugg<<"orderedAngles["<<i<<"] = "<<orderedAngles[i]<<"  orderedDistances["<<i<<"] = "<<orderedDistances[i]<<endl;
                    }
                }
                //***********************************************
            }

            for (unsigned int i = 1; i < orderedAngles.size(); i++) {
                if (abs(orderedAngles[i] - orderedAngles[i-1]) > 9.0) {
                    
                    iterWithFalseGap = i;
                    if (debugging) debugg<<"Salida por break"<<endl;
                    break;
                }
                if (debugging) debugg<<"i: "<<i<<"orderedAngles[i] = "<<orderedAngles[i]<<"  orderedAngles[i-1] = "<<orderedAngles[i-1]<<endl;
            }

            if (debugging) debugg<<"Salto en iterWithFalseGap: "<<iterWithFalseGap<<endl;
            if (debuggLines) cout<<"Line 2575"<<endl;

            if (dir == Derecha) {
                if (debuggLines) cout<<"Line 2581"<<endl;
                if (debugging) debugg<<"Vamos a ordenar higher angles con obstAtBeg"<<endl;

                // Buscamos puntos de obstaculo que pertenezcan al mismo obstaculo en sentido antihorario
                
                for (unsigned int i = 0; i < scan.size(); i++) {
                    if (scan[i] < distanceLimit) { // Si obstaculo dentro del rango
                        double scanAngle = i*180.0/scan.size();
                        if (scanAngle > orderedAngles[iterWithFalseGap-1] && abs(scanAngle - orderedAngles[iterWithFalseGap-1]) < 2.0 ) {
                            if (!robotFits(rPose, orderedDistances[iterWithFalseGap-1]*maxRange, orderedAngles[iterWithFalseGap-1], scan[i]*maxRange, scanAngle, false)) {
                                orderedDistances.insert(orderedDistances.begin() + iterWithFalseGap, scan[i]);
                                orderedAngles.insert(orderedAngles.begin() + iterWithFalseGap, scanAngle);
                                iterWithFalseGap++;
                                debugg<<"Se cumple robot NO fits D"<<endl;
                                if (debugging) debugg<<"El angulo "<<scanAngle<<" cumple la condicion high"<<endl;
                            }
                        }
                    }
                }

                //******************* DEBBUGG *********************
                if (debuggSmallGap) {
                    debugg<<"------------CASO D-------------"<<endl;
                    for (int i = 0; i < orderedAngles.size(); i++){
                        debugg<<"orderedAngles["<<i<<"] = "<<orderedAngles[i]<<"  orderedDistances["<<i<<"] = "<<orderedDistances[i]<<endl;
                    }
                }
                //***********************************************

            } else if (dir = Izquierda) {
                if (debuggLines) cout<<"Line 2618"<<endl;
                if (debugging) debugg<<"Vamos a ordenar lower angles con obstAtBeg"<<endl;

                // Buscamos puntos de obstaculo que pertenezcan al mismo obstaculo en sentido horario
                if (debuggLines) cout<<"scan.size() = "<<scan.size()<<" orderedAngles.size() = "<<orderedAngles.size()<<endl;

                for (int i = scan.size()-1; i >= 0 ; i--) {
                    if (debuggLines) cout<<"i = "<<i<<" ";
                    if (scan[i] < distanceLimit) { // Si obstaculo dentro del rango
                        double scanAngle = i*180.0/scan.size();
                        if (debuggLines) cout<<"i(in) = "<<i<<" ";
                        if (scanAngle < orderedAngles[iterWithFalseGap] && abs(scanAngle - orderedAngles[iterWithFalseGap]) < 2.0 ){ //sin false gap
                            if (debuggLines) cout<<"firsCond  ";
                            if (!robotFits(rPose, orderedDistances[iterWithFalseGap]*maxRange, orderedAngles[iterWithFalseGap], scan[i]*maxRange, scanAngle, false)) {
                                orderedDistances.insert(orderedDistances.begin() + iterWithFalseGap, scan[i]);
                                orderedAngles.insert(orderedAngles.begin() + iterWithFalseGap, scanAngle);
                                debugg<<"Se cumple robot NO fits E"<<endl;
                                if (debugging) debugg<<"El angulo "<<scanAngle<<" cumple la condicion low"<<endl;
                                if (debuggLines) cout<<"robotNoFits  ";
                            }
                        }
                    }
                }

                if (debuggLines) cout<<"Line 2670"<<endl;
                //*******************DEBBUGG*********************
                if (debuggSmallGap) {
                    debugg<<"------------CASO E-------------"<<endl;
                    for (int i = 0; i < orderedAngles.size(); i++) {
                        debugg<<"orderedAngles["<<i<<"] = "<<orderedAngles[i]<<"  orderedDistances["<<i<<"] = "<<orderedDistances[i]<<endl;
                    }
                }
                //***********************************************
            }

        }
        //*************************************************

        if (debugging) debugg<<"Impresion de puntos: "<<endl;
        if (debugging) {
            for (unsigned int i = 0; i < orderedAngles.size(); i++) {
                cout<<orderedAngles[i]<<endl;
                debugg<<orderedAngles[i]<<endl;
            }
        }

        if (debuggLines) cout<<"Line 2661"<<endl;

        //*************************************************************************************
        // Dependiendo de si el obstaculo contiene ambos limites del sensor laser, o no,
        // se encontraran los limites de diferente manera. Si el obstaculo no esta en los dos
        // limites del sensor laser, simplemente se escogeran los valores menor y mayor del vector
        // que contiene la informacion de los angulos la memoria a corto plazo. En caso contrario
        // se escogeran los dos angulos que esten mas separados entre si, siendo estos los
        // limites del obstaculo que se esta rodeando
        //*************************************************************************************
        if (debugging) debugg<<"obstAtBeg final = "<<obstAtBeg<<endl;
        if (obstAtBeg == true) {
            if (debuggLines) cout<<"Line 2672"<<endl;
            if (debugging) debugg<<"False gap at the end"<<endl;
            for (unsigned int i = 1; i < orderedAngles.size(); i++) {
                if (abs(orderedAngles[i] - orderedAngles[i-1]) > 9.0) {
                    cout<<"Paso de angulo "<<orderedAngles[i-1]<<" a angulo "<<orderedAngles[i]<<endl;
                    if (debugging) debugg<<"Paso de angulo "<<orderedAngles[i-1]<<" a angulo "<<orderedAngles[i]<<endl;
                    lowAngles.push_back(orderedAngles[i]);
                    highAngles.push_back(orderedAngles[i-1]);
                    falseGaps.push_back(true);
                }
            }
        } else {
            if (debuggLines) cout<<"Line 2687"<<endl;
            if (debugging) debugg<<"NO False gap"<<endl;
            lowAngles.push_back(orderedAngles[0]);
            highAngles.push_back(orderedAngles[orderedAngles.size()-1]);
            falseGaps.push_back(false);
        }

        //****************** DECIDIR QUE LIMITES COGER ********************
        int boundaryIndex = 0;

        if (debugging) debugg<<"lowAngles.size(): "<<lowAngles.size()<<endl;
        
        //*************************************************************************************
        // Se buscan los limites mas separados entre si y se seleccionan, finalmente como los
        // angulos que representan los limites del obstaculo que se esta rodeando
        //*************************************************************************************
        for (unsigned int i = 1; i < lowAngles.size(); i++) {
            double longerAngleDiff = abs(highAngles[boundaryIndex] - lowAngles[boundaryIndex]);
            double currentAngleDiff = abs(highAngles[i] - lowAngles[i]);
            if (currentAngleDiff > longerAngleDiff) boundaryIndex = i;
        }

        if (lowAngles.size() > 0) {
            Obstacle.lowAngle = lowAngles[boundaryIndex];
            Obstacle.highAngle = highAngles[boundaryIndex];
            Obstacle.falseGap = falseGaps[boundaryIndex];
        }

        //**************************************************************

        cout<<"FINAL: Obstacle.lowAngle: "<<Obstacle.lowAngle<<" Obstacle.highAngle: "<<Obstacle.highAngle<<" Obstacle.falseGap: "<<Obstacle.falseGap<<endl;
        if (debugging) debugg<<"FINAL: Obstacle.lowAngle: "<<Obstacle.lowAngle<<" Obstacle.highAngle: "<<Obstacle.highAngle<<" Obstacle.falseGap: "<<Obstacle.falseGap<<endl;
        
        //*************** DEBUGG *******************
        if (debuggSmallGap){
            debugg<<"----------Final result of orderedAngles-------------"<<endl;
            cout<<"----------Final result of orderedAngles-------------"<<endl;
            for (int i = 0; i < orderedAngles.size(); i++){
                debugg<<"orderedAngles["<<i<<"] = "<<orderedAngles[i]<<"  orderedDistances["<<i<<"] = "<<orderedDistances[i]<<endl;
                cout<<"orderedAngles["<<i<<"] = "<<orderedAngles[i]<<"  orderedDistances["<<i<<"] = "<<orderedDistances[i]<<endl;
            }
        }
        //****************************************
    }
}

void CHolonomicEG::getLocalObstacleBoundary(bool debugging) {

    // Variable de DEBUGGING
    static unsigned int numCycle = 0;
    numCycle++;

    static unsigned int iterDebug=0; // DEBUG

    if (debugging) cout<<"Size gap: "<<Gaps.ini.size()<<endl; // DEBUG

    // Con este bucle for obtenemos los angulos de cada obstaculo
    for (unsigned int i=0; i<Gaps.ini.size(); i++) {
        // Si solo hay un gap
        if (Gaps.ini.size()==1) {

            middlePointInObstacle = true;
            chooseLocalObstacle = false;

            //********** DEBUGG *********
            getTime();
            if (debugging) debugg<<"Cycle: "<<numCycle<<"Entramos en zona de un gap AAA"<<endl;
            if (debugging) printf("Cycle: %d Entramos en zona de un gap AAA. middlePoint: %f\n", numCycle, (float)middlePoint);
            //*************************

            Obstacle.highAngle=Gaps.ini[0];
            Obstacle.lowAngle=Gaps.end[0];
            Obstacle.falseGap=true;
            middlePoint=((180-Obstacle.highAngle)+Obstacle.lowAngle)/2.0;
            if (middlePoint>=(180-Obstacle.highAngle)) middlePoint=middlePoint-(180-Obstacle.highAngle);
            else middlePoint=Obstacle.highAngle+middlePoint;

            if (debugging) printf("highAngle: %d lowAngle: %d \n", Obstacle.highAngle, Obstacle.lowAngle);
            if (debugging) printf("middlePoint NOW para %d de %f \n", i, (float)middlePoint);
            
        } else {
            // Preguntar: en el caso de que un gap este entre el principio y el final, se toma la separacion
            // como obstaculo o mejor no tenerlo en cuenta?. Ahora mismo esta tomado como obstaculo
            if ((i==Gaps.ini.size()-1)&&((middlePoint>=Gaps.end[i]&&middlePoint<=180)||(middlePoint<=Gaps.ini[0]&&middlePoint>=0))) 
            { 
                middlePointInObstacle = true;
                chooseLocalObstacle = false;
                //********** DEBUGG *********
                getTime();
                if (debugging) debugg<<"Cycle: "<<numCycle<<"Entramos en zona de mas de un gap en extremos BBB"<<endl;
                if (debugging) printf("Cycle: %d for %d Entramos en zona de mas de un gap en extremos BBB. middlePoint: %f\n", numCycle, i, (float)middlePoint);
                if (debugging) printf("endGap: %d Gaps.ini: %d \n", Gaps.end[i], Gaps.ini[0]);
                //*************************

                Obstacle.highAngle=Gaps.ini[0];
                Obstacle.lowAngle=Gaps.end[i];
                Obstacle.falseGap=true;
                middlePoint=((180-Obstacle.highAngle)+Obstacle.lowAngle)/2.0;
                if (middlePoint>=(180-Obstacle.highAngle)) middlePoint=middlePoint-(180-Obstacle.highAngle);
                else middlePoint=Obstacle.highAngle+middlePoint;

                if (debugging) printf("highAngle: %d lowAngle: %d \n", Obstacle.highAngle, Obstacle.lowAngle);
                if (debugging) printf("middlePoint NOW para %d de %f \n", i, (float)middlePoint); // DEBUG
            }
            else {
                if (middlePoint<=Gaps.ini[i+1]&&middlePoint>=Gaps.end[i]) {

                    middlePointInObstacle = true;
                    chooseLocalObstacle = false;
                    //********** DEBUGG *********
                    getTime();
                    if (debugging) debugg<<"Cycle: "<<numCycle<<"Entramos en zona de mas de un gap, obstaculo entre gaps DDD"<<endl;
                    if (debugging) printf("Cycle: %d for %d Entramos en una zona de mas de un gap, obstaculo entre gaps DDD. middlePoint: %f\n", numCycle, i, (float)middlePoint);
                    //*************************

                    Obstacle.highAngle=Gaps.ini[i+1];
                    Obstacle.lowAngle=Gaps.end[i];
                    Obstacle.falseGap=false;
                    middlePoint=(Obstacle.highAngle-Obstacle.lowAngle)/2.0+Obstacle.lowAngle;

                    if (debugging) printf("highAngle: %d lowAngle: %d \n", Obstacle.highAngle, Obstacle.lowAngle);
                    if (debugging) printf("middlePoint NOW para %d de %f \n", i, (float)middlePoint); // DEBUG
                }
                else {
                    getTime();
                    if (debugging) debugg<<"Cycle: "<<numCycle<<"Entramos en una opcion sin nada EEE"<<endl;
                    if (debugging) printf("Cycle: %d for %d Entramos en una opcion sin nada EEE\n", numCycle, i);
                    if (debugging) printf("Gaps.ini: %d  Gaps.end: %d  middlePoint: %f\n", Gaps.ini[i+1], Gaps.end[i], (float)middlePoint);
                    if (debugging) printf("highAngle: %d lowAngle: %d \n", Obstacle.highAngle, Obstacle.lowAngle);
                }
            }
        }
        iterDebug=i;
    }
}

//*******************************************************************************************
// Esta funcion se encarga de añadir nuevas capas de memoria a corto plazo, siempre y cuando
// no exista ninguna region libre
//*******************************************************************************************

bool CHolonomicEG::addNewLayer(char busyRegions[], bool debugging) {

    if (layersDebugg) debugg<<"***************addNewLayer****************"<<endl;
    bool freeRegions = false;
    for (int i = 0; i < a_numRegions; i++) {
        if (busyRegions[i] == 0 || busyRegions[i] == 3 || busyRegions[i] == 4) freeRegions = true;
    }

    if (freeRegions == true) {
        if (layersDebugg) debugg<<"Aun quedan regiones libres"<<endl;
        return false;
    } else {
        if (layersDebugg) debugg<<"Ya no hay mas regiones libres"<<endl;
        environmentInfo.push_back(parametros());
        layer++;
        if (layersDebugg) debugg<<"environmentInfo.size() = "<<environmentInfo.size()<<endl;
        if (layersDebugg) debugg<<"layerValue = "<<layer<<endl;
        return true;
    }
}

//*******************************************************************************************
// Esta funcion elimina una capa y reduce el indice que indica el numero de capas existentes
//*******************************************************************************************

void CHolonomicEG::deleteLayer(bool debugging) {

    environmentInfo.erase(environmentInfo.begin() + layer);
    layer--;
    if (layersDebugg) debugg<<"Capa superior eliminada, layer = "<<layer<<endl;
}

//*************************************************************************************
// Esta funcion se encarga de añadir o de eliminar capas de la memoria a corto plazo
//*************************************************************************************

int CHolonomicEG::manageLayers(char busyRegions[], char m_targetRegionState, char m_lastTargetRegionState, int lastDirectionRegion, bool debugging) {
    bool newLayerAdded;
    bool lastLayerDeleted = false;
    char aux;

    aux = busyRegions[lastDirectionRegion];
    busyRegions[lastDirectionRegion] = 4;
    
    // Se añade una nueva capa si todas las regiones estan ocupadas
    newLayerAdded = addNewLayer(busyRegions);

    busyRegions[lastDirectionRegion] = aux;

    if (layersDebugg) debugg<<"Valores de regionTarget:"<<endl;
    if (layersDebugg) debugg<<"m_lastTargetRegionState = "<<(int)m_lastTargetRegionState<<endl;
    if (layersDebugg) debugg<<"m_targetRegionState = "<<(int)m_targetRegionState<<endl;

    // Si no se ha añadido una nueva capa, en el ciclo anterior la region del target estaba ocupada,
    // la region del target actual esta ocupada y existen 2 capas o mas, se borrara la capa actual
    if (!newLayerAdded && m_lastTargetRegionState == 2 && m_targetRegionState == 3 && layer > 0){
        if (layersDebugg) debugg<<"Capa actual debe ser eliminada"<<endl;
        deleteLayer();
        lastLayerDeleted = true;
    }

    // La funcion devuelve un determinado valor dependiendo si:
    // se añade una nueva capa (1), se borra la ultima capa (2) o no ocurre ninguna
    // de estas situaciones (0)
    if (newLayerAdded) {
        if (layersDebugg) debugg<<"Devolvemos 1"<<endl;
        return 1;
    } else if (lastLayerDeleted) {
        if (layersDebugg) debugg<<"Devolvemos 2"<<endl;
        return 2;
    } else {
        if (layersDebugg) debugg<<"Devolvemos 0"<<endl;
        return 0;
    }
}

//*******************************************************************************************
// Dependiendo del valor pasado por parametro, el robot cambia de direccion hacia un sentido
// horario o un sentido antihorario
//*******************************************************************************************

void CHolonomicEG::setDirection(int choice) {

    if (choice == 0) cout<<"Seleccionado un sentido antihorario (derecha)"<<endl;
    else if (choice == 1) cout<<"Seleccionado un sentido horario (izquierda)"<<endl;
    else cout<<"La opcion seleccionada no es valida"<<endl;
    dir = direccion(choice);
}

void CHolonomicEG::getPointCoord(double &X, double &Y) {
    
    X = environmentInfo[layer].Xobs[30];
    Y = environmentInfo[layer].Yobs[30];
}