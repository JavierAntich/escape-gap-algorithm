#ifndef MAPGENERATOR_H
#define MAPGENERATOR_H

#include <iostream>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <iomanip>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#define Trunc 1
#define Normal 0

#define debuggMap 0
#define debuggSolution 0
#define debuggXlim 0
#define debuggYlim 0
#define debuggCoord 0
#define debuggObstacles 0

#define debuggObstaclesDetectionFail 0 

#define roadPlanificator 1 //Crear mapa
#define searchSolution 1 //Buscar camino óptimo 
#define createMap 1 //Crear mapa básico

#define m_Accuracy 1000 //Longitud de cada lado de las celdas en milímetros

using namespace std;

struct pos {
	long start;
	long end;
	long robot;
	unsigned int Xboundary;
	unsigned int Yboundary;
	unsigned int i;
	unsigned int j;
};

struct coordenadas{
	char *name;
	unsigned int X;
	unsigned int Y;
	long int pos;
};

struct coordenadasTwo{
	int X;
	int Y;
};

struct coordenadasRob{
	double X;
	double Y;
};

struct solutionCoord{
	vector<int>X;
	vector<int>Y;
};

struct obst{
	char *name;
	unsigned int value;
	long int pos;
};

struct environmentStruct{
	char *name;
	long int beginEnvironment;
	long int endEnvironment;
	vector<int>environment;
};

struct mapFile {
		struct coordenadas cells;
		struct obst obsthresh;
		struct coordenadas start;
		struct coordenadas end;
		struct environmentStruct environment;
	};

class MapGenerator
{
	
	//Attributes
private:
	fstream mapa;	//Archivo que contiene el entorno de planificacion
	//ifstream sol;	//Archivo que contiene el resultado de la planificacion
	
	vector<int> data; //Vector dinamico que contiene los datos del entorno

	
	
	struct pos position;
	struct coordenadasTwo startAbsolute; //Coordenadas que marcan el origen de las coordenadas en modo absoluto. 
						//Suele ser las coordenadas de menor valor que hayamos obtenido hasta ahora
	struct coordenadasTwo targetAbsoluteValue; //Valor del target en valores relativos al mapa !!CHANGE REL TO ABS
	struct coordenadasTwo robotAbsoluteValue; //Valor del robot en valores relativos al mapa !!CHANGE REL TO ABS	

	//struct solutionCoord solCoord;		//Estructura donde guardamos el conjunto de coordenadas que nos indican el camino
										//marcado por el planificador.
									     
	struct mapFile mapData;				//Aqui se guarda toda la informacion que contiene el fichero que se pasa al
										//planificador.
	int mapAccuracy;

public:

	unsigned int mapCycle;
	
	//struct coordenadasTwo startAbsolute; //Coordenadas que marcan el origen de las coordenadas en modo absoluto. 
						//Suele ser las coordenadas de menor valor que hayamos obtenido hasta ahora
	//int mapAccuracy;	

	MapGenerator(void);
	~MapGenerator(void);
    
    //Methods
private:
	
	void relativeToAbsolute(int &, int &);
	void setStartAndEndPosition(void);
	void printRobotCoords(int X, int Y);
    
public:
	
	void openFile(bool Mode);
	void closeFile();
	
	void initVariables(int accuracy, double startRobot_X, double startRobot_Y, double m_target_X, double m_target_Y);
	void readData(void);
	void writeData(void);
	void initData(void);
	long int wordPosition(const char *word);
	void mapDimension(void);
	void setCoord(int X, int Y, int value);
	void pruebaVector(vector<int> myvector);
	void readMap(void);
	void clearEnvironment();
	void showData(void);
	
	void coordRobotToRelMap(int &X, int &Y);
	void coordRelMapToRobot(int &X, int &Y);
	void coordAbsMapToRelMap(int &X, int &Y);
	void coordRelMapToAbsMap(int &X, int &Y);

	void printMapCoords(double X, double Y);
	
	void obstaclesDetection(double robotPosX, double robotPosY, const std::vector<float> &scan, float maxRange);

	void getStartAbsolute(int &X, int &Y);
	void getCellsInformation(int &X, int &Y);
	void getDataEnvironment(std::vector<int> &dataEnvironment);
	
};
#endif // MAPGENERATOR_H
