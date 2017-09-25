#ifndef ESCAPE_GAP_H
#define ESCAPE_GAP_H

//(*Headers(Escape Gap)
#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/radiobox.h>
#include <wx/menu.h>
#include <wx/textctrl.h>
#include <wx/things/toggle.h>
#include "MyGLCanvas.h"
#include <wx/panel.h>
#include <wx/frame.h>
#include <wx/timer.h>
#include <wx/statusbr.h>
#include <wx/string.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <mrpt/nav.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPlanarLaserScan.h> // It's in the lib mrpt-maps
#include <mrpt/maps/COccupancyGridMap2D.h>
//*)

// JLBC: Unix X headers have these funny things...
#ifdef Button1
#	undef Button1
#	undef Button2
#	undef Button3
#	undef Button4
#	undef Button5
#	undef Button6
#	undef Button7
#	undef Button8 // NEW!!!
#endif

#define getLocalDebugg 0
#define debuggUpdate 0
#define debuggTarget 0
#define debuggFunctions 0
#define debuggFilter 0
#define debuggRegionDirection 0
#define debuggToFunction 0
#define debuggFile 0
#define debuggVector 0
#define fileDebugg 0
#define directionDebugg 0
#define anglesDebugg 0
#define layersDebugg 0
#define sizeLayerDebugg 0
#define debuggSpiral 0
#define debuggSmallGap 0
#define debuggLimit 0
#define debuggLines 0
#define TFGpoint 0
#define printData 1
#define printDataInFile 1
#define layerAdded 1
#define layerDeleted 2
#define rDiameter 50 // Diametro del robot expresado en cm

struct m_Gaps{
	std::vector<int> ini;
	std::vector<int> end;
};

// Estructura de vectores donde guardaremos los valores de los angulos que delimitan los obstaculos
struct Obst{
	double highAngle;
	double lowAngle;
	bool falseGap;
	bool directionTarget;
};

struct parametros{
        // Creacion de objetos vector

        std::vector<double> dist; // Distancias actuales de los puntos
        std::vector<double> ang;  // Angulo del punto
        std::vector<double> Xobs; // Coordenadas X absolutas de los obstaculos
        std::vector<double> Yobs; // Coordenadas Y absolutas de los obstaculos
        std::vector<double> Xfin; // Coordenadas X relativas al robot de los obstaculos
        std::vector<double> Yfin; // Coordenadas Y relativas al robot de los obstaculos
		std::vector<bool> belongToObstacle;	// Variable booleana que indica si el dato actual pertenece al obstaculo que estamos siguiendo o no
        int iter; // Iterador que nos sirve para acceder a los vectores necesarios sin tener que salir del bucle
};

struct RegDirect{
	double angHighBound; // Limite superior del angulo de la siguiente region libre
	double angLowBound;  // Limite inferior del angulo de la siguiente region libre
	bool boundary;
};

class CHolonomicEG
{
    public:

        // Atributos
        unsigned int a_numRegions;
		bool writeDebugg;

		// Metodos

        	// Constuctores
        	CHolonomicEG(unsigned int numRegions, CMyGLCanvas* m_plotScan, struct parametros *param);
			CHolonomicEG(unsigned int numRegions, mrpt::opengl::CSetOfObjectsPtr m_plotScan, CMyGLCanvas* m_plotScan2D, struct parametros *param); // Solo para dibujar

        	std::string update(const mrpt::math::TPoint2D &target,const std::vector<float> &scan, const mrpt::math::TPose2D &rPose, 
			const float maxRange, mrpt::math::TPoint2D *realTarget, bool *objetivof, std::vector<int> iniGap,
             std::vector<int> endGap, bool *regionStatePlan, double desiredDirection, int mainCycles, double timeElapsed);

    		int targetRegion(int m_regionTarget, char busyRegions[]);

    		std::string filterState(int m_currentDirectionRegion, int m_lastDirectionRegion, int m_targetRegion, char m_targetState, bool *toTarget, bool lastLayerDeleted);

    		void reset(void);

			void setDirection(int choice);

			// Solo para conseguir datos para la memoria del TFG
			void getPointCoord(double &X, double &Y);

			// Destructor
        	virtual ~CHolonomicEG();

    private:

        // Atributos
		struct m_Gaps Gaps;
		struct Obst Obstacle;
		std::vector<parametros> environmentInfo;
		int layer;
		struct RegDirect regionDirection;
		double division_reg; // El valor en grados que tiene cada region
		double middlePoint;  // Nos indica el angulo de la mitad del obstaculo que estamos siguiendo
		int angleFrecuency[180];
		bool middlePointInObstacle;
		bool chooseLocalObstacle; // Variable que nos permite elegir por una sola vez que obstaculo elegir y
                                  // a partir de ahi nos guiamos por el punto medio del obstaculo calculado

		bool angleObstacle[181];  // Vector simplificado
		double distObstacle[181];

        mrpt::opengl::CSetOfLinesPtr a_gl_T2_regionsFree;  // Regiones libres
        mrpt::opengl::CSetOfLinesPtr a_gl_T2_regionsOccup; // Regiones ocupadas
        mrpt::opengl::CSetOfLinesPtr a_gl_T2_targetFree;   // Region libre   + Target
        mrpt::opengl::CSetOfLinesPtr a_gl_T2_targetOccup;  // Region ocupada + Target
    	mrpt::opengl::CSetOfLinesPtr a_gl_T2_direccion;	   // Region de direccion cuando el filtro esta activado
    	enum direccion{Derecha,Izquierda} dir;

        // Métodos
        void getObstacles(int scanSize, const std::vector<float> &scan, char *currentRegionState);
		void getTime(void);
		void deleteSmallGaps(std::vector<int> m_iniGap, std::vector<int> m_endGap, bool debugg = 0);

		void getLocalObstacleBoundary(bool debugging = 0);
		void getLocalObstacleBoundary_ten(const std::vector<float> &scan, const mrpt::math::TPose2D &rPose,const float maxRange, double desiredDirection, int currentDirectionRegion, bool debugging = 0);

		void getSensorPoints(const std::vector<float> &scan, char busyRegions[], const mrpt::math::TPose2D &rPose, const float maxRange, bool debugging = 0);	

		void getTotalObstacle(bool debugging = 0);

		void getRegionDirectionBoundary(char busyRegions[], int currentDirectionRegion, bool debugging = 0);

		void computeEnvironment(const mrpt::math::TPose2D &rPose, size_t it, bool debugging = 0);

		void filterEnvironment(size_t &it, int currentDirectionRegion, bool debugging = 0);
		void updateEnvironment(const mrpt::math::TPose2D &rPose, int currentDirectionRegion, bool debugging = 0);

		void orderEnvironmentInfo(std::vector<double> &orderedAngles, std::vector<double> &orderedDistances, const std::vector<float> &scan, bool debugging = 0);

		void setAnglesLimit(std::vector<double> &orderedAngles, std::vector<double> &orderedDistances, double desiredDirection, const float maxRange, bool debugging = 0);

		bool addNewLayer(char busyRegions[], bool debugging = 0);
		void deleteLayer(bool debugging = 0);
		int manageLayers(char busyRegions[], char m_targetRegionState, char m_lastTargetRegionState, int lastDirectionRegion, bool debugging = 0);
  
		// Asegurar que no nos haran falta las coordenadas del robot para ejecutar esta funcion
		bool robotFits(const mrpt::math::TPose2D &rPose, double distanceA, double angleA, double distanceB, double angleB, bool debugging = 0);
};

#endif // ESCAPE_GAP_H
