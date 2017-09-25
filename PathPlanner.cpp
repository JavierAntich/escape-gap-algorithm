#include <iostream>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <iomanip>
#include <string.h>
#include <stdio.h>
#include <cmath>
#include <unistd.h>
#include "PathPlanner.h"
#include <sbpl/headers.h>  

ofstream sbpl("SBPLDebugg.txt", ios::app);
ofstream debplan("debugging_planif.txt", ios::app);

using namespace std;

CPathPlanner::CPathPlanner(void){
	// Do nothing
}

CPathPlanner::~CPathPlanner(void){
	// Do nothing
}

// Se obtienen las coordenadas en formato del mapa
//*********************************************************************
void CPathPlanner::coordRobotToRelMap(int &X, int &Y) {
	X = X / mapAccuracy; // mapAccuracy --> 200
	Y = Y / mapAccuracy; // mapAccuracy --> 200
}

void CPathPlanner::coordRelMapToRobot(int &X, int &Y) {
	X = X * mapAccuracy;
	Y = Y * mapAccuracy;
}

void CPathPlanner::coordAbsMapToRelMap(int &X, int &Y){
	X = X + startAbsolute.X;
	Y = Y + startAbsolute.Y;	
}

void CPathPlanner::coordRelMapToAbsMap(int &X, int &Y){
	X = X - startAbsolute.X;
	Y = Y - startAbsolute.Y;
}

//***************************************************************************************
// Consigue informacion sobre el punto (0, 0) absoluto en el mapa del entorno
//***************************************************************************************
void CPathPlanner::setStartAbsolute(int X, int Y) {

	cout<<"**********setStartAbsolute***********"<<endl;
	startAbsolute.X = X;
	startAbsolute.Y = Y;
	cout<<"Valores de startAbsolute: X = "<<startAbsolute.X<<" Y = "<<startAbsolute.Y<<endl;
}

//*****************************************************************************************
// Consigue informacion sobre el parametro que fija la precision en la que se crea el mapa
//*****************************************************************************************
void CPathPlanner::setMapAccuracy(int accuracy) {
	cout<<"**********setMapAccuracy***************"<<endl;
	mapAccuracy = accuracy;
}

//***************************************************************************************
// Consigue informacion sobre las dimensiones que tiene el entorno
//***************************************************************************************
void CPathPlanner::setCellsInformation(int X, int Y) {
	cout<<"**********setCellsInformation***************"<<endl;
	cells.X = X;
	cells.Y = Y;
	cout<<"cells.X = "<<cells.X<<" cells.Y = "<<cells.Y<<endl;
}

//***************************************************************************************
// Se utiliza esta funcion para la obtencion del vector que se encarga de almacenar todos
// los puntos que posteriormente son escritos en el archivo que contiene el mapa
//***************************************************************************************
void CPathPlanner::setDataEnvironment(std::vector<int> dataEnvironment) {
	cout<<"**********setEnvironmentData***************"<<endl;
	data = dataEnvironment;
	cout<<"size dataEnvironment = "<<data.size()<<endl;
}

//***********************************************************************************************
// Esta funcion se encarga de leer las coordenadas del camino calculado por el planificador.
// En este caso con el algoritmo ARA*. Despues devuelve mediante los parametros pasados por 
// referencia, las coordenadas de cada uno de los puntos que forman el camino planificado
//**********************************************************************************************
void CPathPlanner::readSolution(std::vector<double> &Xsolution, std::vector<double> &Ysolution) {
	int aux_X;
	int aux_Y;
	std::vector<double>::iterator itX;
	std::vector<double>::iterator itY;

	optimusSolution.X.clear();
	optimusSolution.Y.clear();

	cout<<"********Solution:*********"<<endl;
	sol>>aux_X;
	sol>>aux_Y;
	while (!sol.eof()){
		optimusSolution.X.push_back(aux_X);
		optimusSolution.Y.push_back(aux_Y);
		// Pasamos de coordenadas del mapa a coordenadas reales
		coordAbsMapToRelMap(aux_X, aux_Y);
		coordRelMapToRobot(aux_X, aux_Y);
		aux_Y *= -1;
		Xsolution.push_back((double)(aux_X/1000.000));
		Ysolution.push_back((double)(aux_Y/1000.000));
		itX = Xsolution.end()-1;
		itY = Ysolution.end()-1;
		sol>>aux_X;
		sol>>aux_Y;
	}

	cout<<"Fin Solution"<<endl;
}

//********************* RESET GOALS ******************************
// Vuelve a empezar desde el primer subgoal. Esta funcion
// se suele utilizar cuando se vuelve a ejecutar la planificacion
//****************************************************************
void CPathPlanner::resetGoals(void) {
	cout<<"////**********Reseting goals************"<<endl;
	static bool firstTime = true;
	if (firstTime) {
		currentGoal = 0;
		firstTime = false;
	}
	else {
		currentGoal = stepSize;
	}
}

//************************* GO TO NEXT GOAL ********************************
// Proporciona las coordenadas del siguiente subgoal
//**************************************************************************
bool CPathPlanner::goToNextGoal(double rPositionX, double rPositionY, double &Xsol, double &Ysol, std::vector<double> Xsolution, std::vector<double> Ysolution) {

	cout<<"**********En goToNextGoal_two***************"<<endl;
	int indexNextGoal = stepSize;
	//******* DEBUGG ************
	int aux_X = (int)(Xsolution[currentGoal]*1000);
	int aux_Y = (int)(Ysolution[currentGoal]*1000);
	aux_Y *= -1;
	coordRobotToRelMap(aux_X, aux_Y);
	coordRelMapToAbsMap(aux_X, aux_Y);
	//*************************

	cout<<"currentGoal = "<<currentGoal<<" Xsolution.size() = "<<Xsolution.size()<<endl;
	cout<<"Posiciones en mapa -> X = "<<aux_X<<" Y = "<<aux_Y<<endl;
	cout<<"Posiciones actuales currentGoal = "<<currentGoal<<" Xsolution = "<<Xsolution[currentGoal]<<" Ysolution = "<<Ysolution[currentGoal]<<endl;

	double distanceToGoal = sqrt(pow(Xsolution[currentGoal] - rPositionX, 2) + pow(Ysolution[currentGoal] - rPositionY, 2));
	if (distanceToGoal < 1.0) {
		cout<<"Muy cerca del goal, cambio de goal"<<endl;
		if (currentGoal + indexNextGoal < Xsolution.size()) {
			currentGoal += indexNextGoal;
			cout<<"No nos pasamos del tamano del vector con currentGoal = "<<currentGoal<<endl;
		}
		else {
			currentGoal = Xsolution.size() - 1;
			cout<<"Nos pasamos del tamano del vector con currentGoal = "<<currentGoal<<endl;
		}
		Xsol = Xsolution[currentGoal];
		Ysol = Ysolution[currentGoal];
		cout<<"Soluciones -> X = "<<Xsol<<" Y = "<<Ysol<<endl;
		return true;
	}
	Xsol = Xsolution[currentGoal];
	Ysol = Ysolution[currentGoal];

	cout<<"Soluciones -> X = "<<Xsol<<" Y = "<<Ysol<<endl;
	return false;
}

//********************* CHECK OPTIMAL ROAD ***************************************
// Comprueba si el camino planificado sigue siendo correcto, en caso
// contrario, volvera a planificar. Comprueba que cada uno de los puntos
// proporcionados por la funcion "goToNextGoal" esten o no ocupados
//********************************************************************************
bool CPathPlanner::checkOptimusRoad(std::vector<double> Xsolution, std::vector<double> Ysolution) {

	cout<<"********En checkOptimusRoad**************"<<endl;
	cout<<"Cols = "<<cells.X<<" Rows = "<<cells.Y<<endl;
	int aux_X;
	int aux_Y;
	double dAux_X;
	double dAux_Y;
	int indexNextGoal = stepSize;
	int currentStep;
	if (currentGoal < indexNextGoal) currentStep = currentGoal;
	else currentStep = currentGoal - indexNextGoal;
	for (int i = currentStep; i < Xsolution.size(); i += indexNextGoal) {
		aux_X = (int)(Xsolution[i]*1000);
		aux_Y = (int)(Ysolution[i]*1000);
		aux_Y *= -1;
		coordRobotToRelMap(aux_X, aux_Y);
		coordRelMapToAbsMap(aux_X, aux_Y);
		int index = aux_Y*cells.X + aux_X;
		cout<<"Despues de transformacion X = "<<aux_X<<" Y = "<<aux_Y<<" INDEX = "<<index<<"data = "<<data[index]<<endl;
		if (data[index] == 1) {
			cout<<"El camino optimo esta ocupado en la posicion con i -> "<<i<<" X = "<<aux_X<<" Y = "<<aux_Y<<endl;
			coordAbsMapToRelMap(aux_X, aux_Y);
			coordRelMapToRobot(aux_X, aux_Y);
			aux_Y *= -1;
			dAux_X = aux_X/1000.000;
			dAux_Y = aux_Y/1000.000;
			cout<<"Coordenadas reales -> X = "<<dAux_X<<" Y = "<<dAux_Y<<endl;
			return true;
		}
	}
	cout<<"El camino optimo sigue siendo el mismo"<<endl;
	return false;
}


//********************* CHECK OPTIMAL ROAD ALTERNATIVE ****************************
// Comprueba si el camino planificado sigue siendo correcto, en caso
// contrario, volvera a planificar. Comprueba que cada uno de los puntos
// que forma el camino no este ocupado
//*******************************************************************************
bool CPathPlanner::checkOptimusRoad_alternative(std::vector<double> Xsolution, std::vector<double> Ysolution) {

	cout<<"********En checkOptimusRoad ALTERNATIVE**************"<<endl;
	cout<<"Cols = "<<cells.X<<" Rows = "<<cells.Y<<endl;
	debplan<<"********En checkOptimusRoad ALTERNATIVE**************"<<endl;
	debplan<<"Cols = "<<cells.X<<" Rows = "<<cells.Y<<endl;
	int aux_X;
	int aux_Y;
	double dAux_X;
	double dAux_Y;
	int indexNextGoal = stepSize;
	int currentStep;
	if (currentGoal < indexNextGoal) currentStep = currentGoal;
	else currentStep = currentGoal - indexNextGoal;
	for (int i = currentStep; i < Xsolution.size(); i++) {
		debplan<<"Estamos dentro del bucle en el ciclo "<<i<<endl;
		aux_X = (int)(Xsolution[i]*1000);
		aux_Y = (int)(Ysolution[i]*1000);
		debplan<<"Antes de transformacion X = "<<aux_X<<" Y = "<<aux_Y<<endl;
		aux_Y *= -1;
		coordRobotToRelMap(aux_X, aux_Y);
		coordRelMapToAbsMap(aux_X, aux_Y);
		int index = aux_Y*cells.X + aux_X;
		debplan<<"Despues de transformacion X = "<<aux_X<<" Y = "<<aux_Y<<"cells.X = "<<cells.X<<" cells.Y = "<<cells.Y<<" INDEX = "<<index<<endl;
		if (data[index] == 1) {
			cout<<"El camino optimo esta ocupado en la posicion con i -> "<<i<<" X = "<<aux_X<<" Y = "<<aux_Y<<endl;
			debplan<<"El camino optimo esta ocupado en la posicion con i -> "<<i<<" X = "<<aux_X<<" Y = "<<aux_Y<<endl;
			coordAbsMapToRelMap(aux_X, aux_Y);
			coordRelMapToRobot(aux_X, aux_Y);
			aux_Y *= -1;
			dAux_X = aux_X/1000.000;
			dAux_Y = aux_Y/1000.000;
			debplan<<"Coordenadas reales -> X = "<<dAux_X<<" Y = "<<dAux_Y<<endl;
			return true;
		}
	}
	cout<<"El camino optimo sigue siendo el mismo"<<endl;
	debplan<<"El camino optimo sigue siendo el mismo"<<endl;
	return false;
}

//***************************** OPEN SOL *********************************
// Abre el archivo que contiene las coordenadas de la planificacion
//************************************************************************
void CPathPlanner::openSol(void) {
	sol.open("sol.txt", ios::binary);
	if ( !sol)
	{
		cerr<< "No se pudo abrir el archivo" << endl;
		exit(1);
	}
}

//***************************** CLOSE FILE *********************************
// Cierra el archivo que contiene las coordenadas de la planificacion
//**************************************************************************
void CPathPlanner::closeSol(void) {
	sol.close();
}

//***************************** EXECUTE PLANIFICATION **********************
// Mediante esta funcion se llama al planificador para la obtencion del
// camino que debe seguir el robot, buscando asi el camino mas cercano
// al optimo posible
//**************************************************************************
void CPathPlanner::executePlanification(void) {
	// Parametros de planificacion
	int bRet = 0;
    double allocated_time_secs = 0.035;
    double initialEpsilon = 10.0;
    MDPConfig MDPCfg;
    bool bsearchuntilfirstsolution = false;
    bool bforwardsearch = true;

    // Inicializar entorno (debe ser llamado antes de inicializar nada mas)
    EnvironmentNAV2D environment_nav2D;

    if (!environment_nav2D.InitializeEnv("Map.txt")) {
        sbpl<<"ERROR: InitializeEnv failed\n"<<endl;
        if (debuggSBPL) printf("ERROR: InitializeEnv failed\n");
        throw new SBPL_Exception();
    }

    // Initialize MDP Info
    if (!environment_nav2D.InitializeMDPCfg(&MDPCfg)) {
        if (debuggSBPL) printf("ERROR: InitializeMDPCfg failed\n");
        throw new SBPL_Exception();
    }

    // Planear un camino
    vector<int> solution_stateIDs_V;
    SBPLPlanner* planner = NULL;

     if (debuggSBPL) printf("Initializing ARAPlanner...\n");
     sbpl<<"Initializing ARAPlanner...\n";
     planner = new ARAPlanner(&environment_nav2D, bforwardsearch);

     // Activar modo busqueda
    planner->set_search_mode(bsearchuntilfirstsolution);

    if (planner->set_start(MDPCfg.startstateid) == 0) {
        sbpl<<"ERROR: failed to set start state\n";
        if (debuggSBPL) printf("ERROR: failed to set start state\n");
        throw new SBPL_Exception();
    } else {
        sbpl<<"Success to set start state\n";
    }

    if (planner->set_goal(MDPCfg.goalstateid) == 0) {
        sbpl<<"ERROR: failed to set goal state\n";
        if (debuggSBPL) printf("ERROR: failed to set goal state\n");
        throw new SBPL_Exception();
    } else {
        sbpl<<"Success to set goal state\n";
    }

    planner->set_initialsolution_eps(initialEpsilon);

    if (debuggSBPL) printf("start planning...\n");
    sbpl<<"start planning...\n";
    bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V);
    if (debuggSBPL) printf("done planning\n");
    sbpl<<"done planning\n";
    std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

    environment_nav2D.PrintTimeStat(stdout);

    const char* sol = "sol.txt";
    FILE* fSol = fopen(sol, "w");
    if (fSol == NULL) {
        sbpl<<"ERROR: could not open solution file\n";
        if (debuggSBPL) printf("ERROR: could not open solution file\n");
        throw new SBPL_Exception();
    } else {
        sbpl<<"SUCCESS: solution file opened\n";
    }

    for (unsigned int i = 0; i < solution_stateIDs_V.size(); i++) {
        environment_nav2D.PrintState(solution_stateIDs_V[i], false, fSol);
    }
    fclose(fSol);

    environment_nav2D.PrintTimeStat(stdout);

     // Print a path
    if (bRet) {
        // Print the solution
        if (debuggSBPL) printf("Solution is found\n");
        sbpl<<"Solution is found\n";
        
    }
    else {
        if (debuggSBPL) printf("Solution does not exist\n");
        sbpl<<"Solution does not exist\n";
    }

    fflush(NULL);

    delete planner;
}

void CPathPlanner::getNearPoint(double &Xpoint, double &Ypoint, std::vector<double> Xsolution, std::vector<double> Ysolution) {
	int indexNextGoal = stepSize;
	Xpoint = Xsolution[currentGoal-indexNextGoal+2];
	Ypoint = Ysolution[currentGoal-indexNextGoal+2];
	cout<<"***************getNearPoint!!!*************"<<endl;
	cout<<"Xpoint = "<<Xpoint<<" Ypoint = "<<Ypoint<<endl;
}
