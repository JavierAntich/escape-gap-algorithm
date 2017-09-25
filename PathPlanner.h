#include <iostream>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <iomanip>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

using namespace std;

#define debuggSBPL 0

#define firstWayToCheckPath 1 // Seleccion del metodo de validacion: 0 -> Comprueba cada uno de los puntos del camino actual
							  // 									 1 -> Comprueba el estado del subgoal actual
#define planificatorActive 1  // Activacion del planificador

#define stepSize 10 // Distancia entre subgoals en numero de celdas

struct coordenadasInt {
	int X;
	int Y;
};

struct solCoord {
	vector<int>X;
	vector<int>Y;
};

class CPathPlanner
{
	// Atributos
private:
	ifstream sol; // Archivo que contiene el resultado de la planificacion
	
	struct coordenadasInt startAbsolute;
	struct coordenadasInt cells;
	struct solCoord optimusSolution;
	
	int mapAccuracy;

	vector<int> data;

	int currentGoal;

	// Metodos
public:
	CPathPlanner(void);
	~CPathPlanner(void);
    
public:
	void setStartAbsolute(int X, int Y);
	void setMapAccuracy(int accuracy);
	void setCellsInformation(int X, int Y);
	void setDataEnvironment(std::vector<int>);

	void coordRobotToRelMap(int &X, int &Y);
	void coordRelMapToRobot(int &X, int &Y);
	void coordAbsMapToRelMap(int &X, int &Y);
	void coordRelMapToAbsMap(int &X, int &Y);
	
	void readSolution(std::vector<double> &Xsolution, std::vector<double> &Ysolution);
	
	void resetGoals(void);
	
	bool goToNextGoal(double rPositionX, double rPositionY, double &Xsol, double &Ysol, std::vector<double> Xsolution, std::vector<double> Ysolution);
	
	bool checkOptimusRoad(std::vector<double> Xsolution, std::vector<double> Ysolution);
	bool checkOptimusRoad_alternative(std::vector<double> Xsolution, std::vector<double> Ysolution);

	void openSol(void);
	void closeSol(void);

	void executePlanification(void);

	void getNearPoint(double &Xpoint, double &Ypoint, std::vector<double> Xsolution, std::vector<double> Ysolution);	
};
