#include <iostream>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <iomanip>
#include <string.h>
#include <stdio.h>
#include <cmath>
#include <unistd.h>
#include "MapGenerator.h"

using namespace std;
#define MAX 50

#define begginingPoint 0
#define endPoint 1
#define intervals 10

ofstream debmap("debugging_map.txt", ios::app);

MapGenerator::MapGenerator(void){
	//Do nothing
	mapCycle = 0;
}

MapGenerator::~MapGenerator(void){
	//Do nothing
}

//****************************INIT VARIABLES******************************
//En esta función se realiza una inicialización de las características del mapa
//a crear. Como parámetros se indican la precisión que va a tener el mapa, donde
//se encuentra inicialmente el robot (coordenadas X,Y) y donde se encuentra
//el target (coordenadas X,Y).
//************************************************************************
void MapGenerator::initVariables(int accuracy, double startRobot_X, double startRobot_Y, double m_target_X, double m_target_Y){
	bool Xnegative = false; //Indica si la coordenada X de la posición del target es negativa.
	bool Ynegative = false; //Indica si la coordenada Y de la posición del target es negativa.
	bool XnegativeRob = false; //Indica si la coordenada X de la posición del robot es negativa.
	bool YnegativeRob = false; //Indica si la coordenada Y de la posición del robot es negativa.

	if (debuggMap) debmap<<"initVariables"<<endl;

	mapAccuracy = accuracy; //Unidad en milimetros.

	if (debuggMap) cout<<"MapAccuracy set to "<<mapAccuracy<<endl;
	if (debuggMap) cout<<"StartAbsoluteValues ("<<startAbsolute.X<<", "<<startAbsolute.Y<<")"<<endl;

	if (startRobot_X<0){
		XnegativeRob = true;
	}
	if (startRobot_Y>0){  		//Ponemos la condicion mayor que 0 para mantener el mismo sistema de coordenadas
		YnegativeRob = true;   //que el mapa del robot.
	}

	//Cambiamos el formato de las coordenadas iniciales del robot de metros a milímetros y
	//de variable double a variable integer. 
	startAbsolute.X = ((int)(abs(startRobot_X)*1000));
	startAbsolute.Y = ((int)(abs(startRobot_Y)*1000));

	// int robotX = ((int)(abs(startRobot_X)*1000));
	// int robotY = ((int)(abs(startRobot_Y)*1000));

	if (debuggMap) cout<<"**************************INICIO******************************"<<endl;

	if (debuggMap) cout<<"***Valor original***"<<endl;
	if (debuggMap) cout<<"Value startAbsolute.X: "<<startAbsolute.X<<endl<<"Value startAbsolute.Y: "<<startAbsolute.Y<<endl;

	coordRobotToRelMap(startAbsolute.X, startAbsolute.Y); //Pasamos los datos en referencia al
														  //mapa.
	// coordRobotToRelMap(robotX, robotY);

	// robotX *= XnegativeRob ? -1 : 1 ;
	// robotY *= YnegativeRob ? -1 : 1 ;

	//Si alguno de los componentes de la posición inicial del robot
	//era negativa, aquí se aplica su signo.
	startAbsolute.X *= XnegativeRob ? -1 : 1 ;
	startAbsolute.Y *= YnegativeRob ? -1 : 1 ;

	if (debuggMap) cout<<"***Valor ToRelMap***";
	if (debuggMap) cout<<"Value startAbsolute.X: "<<startAbsolute.X<<endl<<"Value startAbsolute.Y: "<<startAbsolute.Y<<endl;

	if (debuggMap) debmap<<"Value startAbsolute.X: "<<startAbsolute.X<<endl<<"Value startAbsolute.Y: "<<startAbsolute.Y<<endl;

	//Se guardan las coordenas iniciales del robot.
	robotAbsoluteValue.X = startAbsolute.X; 
	robotAbsoluteValue.Y = startAbsolute.Y;
	//*****

	//Se debe de tener en cuenta cuando el valor de las coordenadas del robot son negativas.
	//Si no lo tenemos en cuenta, pueda haber un conflicto entre tipo de variables.
	if (m_target_X<0){
		Xnegative = true;
	}
	if (m_target_Y>0){  	//Se utiliza la condicion mayor que 0 para mantener el mismo 
		Ynegative = true;   //sistema de coordenadas que el mapa del robot.
	}

	//Cambiamos el formato de las coordenadas del target de metros a milímetros y
	//de variable double a variable integer. 
	int target_X = ((int)(abs(m_target_X)*1000));
	int target_Y = ((int)(abs(m_target_Y)*1000));
	
	coordRobotToRelMap(target_X, target_Y); //Conversión en referencia al mapa.
	
	target_X *= Xnegative ? -1 : 1 ;
	target_Y *= Ynegative ? -1 : 1 ;

	targetAbsoluteValue.X = target_X;
	targetAbsoluteValue.Y = target_Y;

	if (debuggMap) cout<<"targetAbsoluteValue ("<<targetAbsoluteValue.X<<" ,"<<targetAbsoluteValue.Y<<")"<<endl;
	if (debuggMap) debmap<<"targetAbsoluteValue ("<<targetAbsoluteValue.X<<" ,"<<targetAbsoluteValue.Y<<")"<<endl;
	
	setCoord(target_X, target_Y, 0); //Se añade el valor 0 a las coordenadas
									 //(target_X, target_Y).
	
	//*****
	mapData.cells.name = "discretization(cells):";
	mapData.obsthresh.name = "obsthresh:";
	mapData.start.name = "start(cells):";
	mapData.end.name = "end(cells):";
	mapData.environment.name = "environment:";

	
	setStartAndEndPosition(); //Se guardan las posiciones de inicio y final que se 
							  //tendrán en cuenta a la hora de la planificación del camino.
	
//	cout<<"initVariables:"<<endl<<mapData.cells.name<<endl<<mapData.obsthresh.name<<endl<<mapData.start.name<<endl<<mapData.end.name<<endl<<endl;
}

//*****************************INIT DATA**********************************
//En esta función inicializamos el formato del archivo que contiene el mapa.
//************************************************************************
void MapGenerator::initData(void){
	mapa.clear();

	mapData.cells.name = "discretization(cells):";
	mapData.obsthresh.name = "obsthresh:";
	mapData.start.name = "start(cells):";
	mapData.end.name = "end(cells):";
	mapData.environment.name = "environment:";

	if (createMap){
		mapa<<mapData.cells.name<<' '<<15<<' '<<15<<endl;
		// mapa<<mapData.obsthresh.name<<' '<<1<<' '<<endl;
		mapa<<mapData.obsthresh.name<<' '<<1<<' '<<endl;
		mapa<<mapData.start.name<<' '<<0<<' '<<0<<endl;
		mapa<<mapData.end.name<<' '<<0<<' '<<0<<endl;
		mapa<<mapData.environment.name<<endl;
		for (int j = 0; j<15; j++){
			for (int i = 0; i<15; i++){
				mapa<<0<<' ';
			}
			mapa<<endl;
		}
	}

	mapa.clear();
	mapa.seekg(0);


}
//*****************************WRITE DATA*********************************
//Esta función se encarga de escribir en el archivo del mapa, utilizando
//el formato correcto.
//************************************************************************
void MapGenerator::writeData(void){
	int saltar = 0;
	unsigned int Xlim = (position.Xboundary-1)/2;

	
	//Se añade la estructura que da formato al archivo
	mapa<<mapData.cells.name<<' '<<mapData.cells.X<<' '<<mapData.cells.Y<<endl;
	mapa<<mapData.obsthresh.name<<' '<<mapData.obsthresh.value<<' '<<endl;
	mapa<<mapData.start.name<<' '<<mapData.start.X<<' '<<mapData.start.Y<<endl;
	mapa<<mapData.end.name<<' '<<mapData.end.X<<' '<<mapData.end.Y<<endl;
	mapa<<mapData.environment.name;
	mapa<<endl;

	if (debuggMap) debmap<<mapData.cells.name<<' '<<mapData.cells.X<<' '<<mapData.cells.Y<<endl;
	if (debuggMap) debmap<<mapData.obsthresh.name<<' '<<mapData.obsthresh.value<<' '<<endl;
	if (debuggMap) debmap<<mapData.start.name<<' '<<mapData.start.X<<' '<<mapData.start.Y<<endl;
	if (debuggMap) debmap<<mapData.end.name<<' '<<mapData.end.X<<' '<<mapData.end.Y<<endl;
	if (debuggMap) debmap<<mapData.environment.name;
	if (debuggMap) debmap<<endl;
	
//	cout<<"Xlim en write = "<<Xlim<<endl;
//	cout<<"Xboundary en write = "<<position.Xboundary<<endl;

	//Se añaden los datos del entorno
	for (vector<int>::iterator iter=data.begin(); iter<data.end(); iter++){
		saltar++;
//		cout<<*iter<<' ';
		mapa<<*iter<<' ';
		if (debuggMap) debmap<<*iter<<' ';
		if (saltar%(Xlim)== 0){
//			cout<<endl;
			mapa<<endl;
			if (debuggMap) debmap<<" | ";
		} 
	}
	
	if (debuggMap) debmap<<endl;
}

//*****************************READ DATA**********************************
//Esta función permite la lectura del archivo que contiene el mapa.
//************************************************************************
void MapGenerator::readData(void){
	
	mapa.clear();
	mapa.seekg(0);
	
	mapData.cells.pos = wordPosition("discretization(cells):");
	mapa.seekg(mapData.cells.pos);
	mapa>>mapData.cells.X>>mapData.cells.Y;

	cout<<"Posicion cells: "<<mapData.cells.pos<<endl;

	mapData.obsthresh.pos = wordPosition("obsthresh:");
	mapa.seekg(mapData.obsthresh.pos);
	mapa>>mapData.obsthresh.value;
	
	cout<<"Posicion obsthresh: "<<mapData.obsthresh.pos<<endl;
	
	mapData.start.pos = wordPosition("start(cells):");
	mapa.seekg(mapData.start.pos);
	mapa>>mapData.start.X>>mapData.start.Y;
	
	cout<<"Posicion START: "<<mapData.start.pos<<endl;
	
	mapData.end.pos = wordPosition("end(cells):");
	mapa.seekg(mapData.end.pos);
	mapa>>mapData.end.X>>mapData.end.Y;
	
	cout<<"Posicion END: "<<mapData.end.pos<<endl;
	
	mapData.environment.beginEnvironment = wordPosition("environment:") + 1;
	
	if (!debuggMap){
		cout<<"CELLS  Coordenada X: "<<mapData.cells.X<<" Coordenada Y: "<<mapData.cells.Y<<endl;
		cout<<"Obsthresh value: "<<mapData.obsthresh.value<<endl;
		cout<<"START  Coordenada X: "<<mapData.start.X<<" Coordenada Y: "<<mapData.start.Y<<endl;
		cout<<"END Coordenadas X: "<<mapData.end.X<<" Coordenada Y: "<<mapData.end.Y<<endl;
		cout<<"START POINT: "<<mapData.environment.beginEnvironment<<endl;
	}	
	
}


//*****************************OPEN FILE*********************************
//Se abre el archivo que contiene el mapa del entorno. Se puede abrir de dos modos.
//Se puede abrir sin ningún cambio en el archivo o eliminando todo su contenido.
//***********************************************************************
void MapGenerator::openFile(bool Mode){
	if (Mode == 0) mapa.open("Map.txt", ios::in | ios::out | ios::ate | ios::binary);
	else mapa.open("Map.txt", ios::in | ios::out | ios::trunc | ios::binary);
		
	if ( !mapa)
	{
		cerr<< "No se pudo abrir el archivo" << endl;
		exit(1);
	}
}

//*****************************CLOSE FILE*********************************
//Cierra el archivo que contiene el mapa del entorno.
//***********************************************************************

void MapGenerator::closeFile(){
	mapa.close();
}

//****************************RELATIVE TO ABSOLUTE**********************
//Esta función se encarga de ampliar el mapa cuando se obtienen coordenadas
//fuera de los límites, dando una nueva referencia.
//**********************************************************************
void MapGenerator::relativeToAbsolute(int &X, int &Y){
	int Xrel = X-1;
	int Yrel = Y-1;
	unsigned int Xlim = (position.Xboundary-1)/2;
	unsigned int Ylim = position.Yboundary;
	if (debuggMap) cout<<"position.Xboundary: "<<position.Xboundary<<endl; //DEBUGG
	if (debuggMap) cout<<"Valores al principio de relativeToAbsolute: Xlim: "<<Xlim<<" Ylim: "<<Ylim<<endl; //DEBUGG
	
	//Cuando hay un componente X menor al menor valor hasta ahora en el eje X.
	if (X<=startAbsolute.X){
		if (debuggMap) cout<<"-----------Considered lower than startAbsolute.X"<<endl; //DEBUGG
		if (debuggMap) debmap<<"-----------Considered lower than startAbsolute.X"<<endl;
		if (debuggMap) debmap<<"X = "<<X<<" startAbsolute.X = "<<startAbsolute.X<<endl;
		if (debuggMap) cout<<"Xvalue: "<<X<<endl;
		if (debuggMap) cout<<"StartAbsoluteValue: "<<startAbsolute.X<<endl;
		if (debuggMap) cout<<"X boundary before: "<<position.Xboundary<<endl;
		if (debuggMap) cout<<"Xlim before: "<<(position.Xboundary-1)/2<<endl;
    	position.Xboundary = position.Xboundary + (startAbsolute.X - Xrel)*2;
    	Xlim = (position.Xboundary-1)/2;
    	if (debuggMap) cout<<"X boundary later: "<<position.Xboundary<<endl;
    	if (debuggMap) cout<<"Xlim later: "<<Xlim<<endl;
    	
    	mapData.cells.X = Xlim;
    	
    	for (int j = 0; j < Ylim; j++){
			for (int i = 0; i < (startAbsolute.X - Xrel); i++){ //startAbsolute.X-Xrel marca la distancia entre el 
				data.insert(data.begin()+j*Xlim, 0);			//nuevo punto y el punto con menor valor en el
//				cout<<"Ahora: "<<data[j*Xlim]<<endl;			//ciclo anterior
			}
		}	
    	
		startAbsolute.X = Xrel;
	}	
	//Cuando hay un componente Y mayor al mayor valor hasta ahora en el eje Y.  
	if (Y<=startAbsolute.Y){
		if (debuggMap) cout<<"-----------Considered lower than startAbsolute.Y"<<endl; //DEBUGG
		if (debuggMap) debmap<<"-----------Considered lower than startAbsolute.Y"<<endl;
		if (debuggMap) debmap<<"Y = "<<Y<<" startAbsolute.Y = "<<startAbsolute.Y<<endl;

		position.Yboundary = position.Yboundary + (startAbsolute.Y - Yrel);

		if (debuggMap) cout<<"Before for bucle in startAbsolute.Y condition"<<endl; //DEBUGG
		if (debuggMap) cout<<"Valor de Xlim: "<<Xlim<<" position.Yboundary: "<<position.Yboundary<<" Ylim: "<<Ylim<<endl; //DEBUGG
		
		for (int j = 0; j < (position.Yboundary - Ylim); j++){
			for (int i = 0; i < Xlim; i++){
				// if (debuggMap) cout<<"Insert startAbsolute.Y condition with i: "<<i<<"  j: "<<j<<endl; //DEBUGG
				data.insert(data.begin(), 0);
			}
		}
		
		if (debuggMap) cout<<"After for bucle in startAbsolute.Y condition"<<endl; //DEBUGG

		mapData.cells.Y = position.Yboundary;
		startAbsolute.Y = Yrel;
	} 
	
	
	
	if (debuggMap) cout<<"After pass comprobation of startAbsolute"<<endl; //DEBUGG
	
	
	X = X - startAbsolute.X;
	Y = Y - startAbsolute.Y;
	
	if (debuggMap)cout<<"X relativa: "<<Xrel<<" Y relativa "<<Yrel<<endl;
	if (debuggMap)cout<<"X absoluta: "<<X<<" Y absoluta "<<Y<<endl<<endl;
	
	//Mostrar el mapa actual
	
	int saltar = 0;
/* ESCRITURA MAPA
	for (vector<int>::iterator iter=data.begin(); iter<data.end(); iter++){
		saltar++;
//		cout<<*iter<<' ';
		mapa<<*iter<<' ';
		if (saltar%(Xlim)== 0){
//			cout<<endl;
			mapa<<endl;
		} 
	}
	*/
	
	mapa.clear();
	
//	for (vector<int>::iterator iter=data.begin(); iter<data.end(); iter++){
//		cout<<*iter<<' ';
//	}
}


//****************************WORD POSITION**********************
//Encuentra la posición de la palabra indicada por parámetro en el
//archivo que contiene el mapa del entorno, esto permite saber donde
//está cada uno de los parámetros del archivo y modificarlos sin 
//muchas dificultades.
//***************************************************************
long int MapGenerator::wordPosition(const char *word){
	long int currentPosition = mapa.tellg();
	long int position;
	char aux[MAX];
	
	mapa.seekg(0);
	
	
	
	while (mapa>>aux){
	
//		cout<<"LAS PALABRAS SON: "<<endl<<word<<" y "<<endl<<aux<<" con resultado: "<<strcmp(aux, word)<<endl;
		
		
		if (strcmp(aux, word) == 0) 
		{
			position = mapa.tellg();
//			cout<<"Posicion para "<<word<<" : "<<position<<" con aux: "<<aux<<" con strcmp: "<<strcmp(aux, word)<<endl;
			mapa.seekg(currentPosition);
			return position;
		}
	}
	
	
	
}

//****************************mapData DIMENSION**********************
//Se obtienen las dimensiones del archivo del mapa en los datos del
//entorno, tanto en el eje X como en el eje Y.
//*******************************************************************
void MapGenerator::mapDimension(void){
	bool firstLine = true;
	int aux;
	int deb = 0;

	mapa.clear();
	
	
	mapa.seekg(mapData.environment.beginEnvironment + 2);
	

	cout<<"mapdimension 386"<<endl;
	if (debuggMap)	cout<<"Empezamos con una posicion en: "<<mapa.tellg()<<endl;
	while (!mapa.eof()){
		mapData.environment.endEnvironment = mapa.tellg();
		// cout<<"endEnvironment = "<<mapData.environment.endEnvironment<<endl;
		// cout<<"deb = "<<deb<<endl;
		aux = mapa.get();
		// cout<<aux<<endl;
		if (aux =='\n' && firstLine == true){
			firstLine = false;
			// cout<<"deb1 = "<<deb<<endl;
			position.Xboundary = (int)(mapa.tellg() - mapData.environment.beginEnvironment);
			// cout<<"deb2 = "<<deb<<endl;
			if (debuggMap)cout<<"La X esta en la posicion "<<position.Xboundary<<endl;
			if (debuggMap)cout<<"begin environment = "<<mapData.environment.beginEnvironment<<endl;
		}
		// mapa>>aux;	
		deb++;
	}
	cout<<"mapdimension 397 endEnvironment "<<mapData.environment.endEnvironment<<" beginEnvironment "<<mapData.environment.beginEnvironment<<" Xboundary "<<position.Xboundary<<endl;
	position.Yboundary = (mapData.environment.endEnvironment - mapData.environment.beginEnvironment) / position.Xboundary;
	cout<<"mapdimension 399"<<endl;
	mapa.clear();
	if (debuggMap) cout<<"Hemos llegado al final con una posicion: "<<mapData.environment.endEnvironment<<endl<<"position.Xboundary = "<<position.Xboundary<<endl<<"position.Yboundary = "<<position.Yboundary<<endl;
	cout<<"mapdimension 402"<<endl;
}

//****************************SET COORD**********************
//En las coordenadas [X, Y] indicadas por parámetro se introduce el valor value
//en el vector que posteriormente es escrito en el archivo del mapa del entorno.
//***********************************************************
void MapGenerator::setCoord(int X, int Y, int value){
	if (debuggMap) cout<<"Value added at the beggining: "<<(int)value<<endl;  //DEBUGG
	long int valuePosition;
	unsigned int valuePositionVector;
	long int expandPosition;
	unsigned int Xlabel; //El valor que debe tener en cuenta el proceso de expansión en el eje Y, para expandirse en el
						 //eje X en cada fila añadida. Si la coordenada pasada por parámetro es mayor que el limite del mapa,
						 //esta X sera el limite de expansion. En caso contrario vendra dado por el limite del mapa anteriormente
						 //asignado con la funcion 'mapDimension'.
	unsigned int Xlim; //Valor limite actual en el eje X del mapa
	unsigned int Ylim; //Valor limite actual en el eje Y del mapa

	if (debuggObstaclesDetectionFail) cout<<"Line 413 MSetcoord"<<endl;
	
	if (debuggMap) cout<<"Before relativeToAbsolute>>>>>>> X: "<<X<<"Y: "<<Y<<endl;
	if (debuggCoord) debmap<<"*************Set coord**********************"<<endl;
	if (debuggCoord) debmap<<"startAbsolute.X = "<<startAbsolute.X<<" startAbsolute.Y = "<<startAbsolute.Y<<endl;
	if (debuggCoord) debmap<<"setCoord values: ("<<X<<", "<<Y<<")"<<endl;
	relativeToAbsolute(X, Y); //Utilizamos esta funcion para tratar los valores de coordenadas negativos.
	if (debuggCoord) debmap<<"Coordenadas despues de relativeToAbsolute : X "<<X<<" Y "<<Y<<endl;

	Xlim = (position.Xboundary-1)/2;
	Ylim = position.Yboundary;
	Xlabel = (position.Xboundary-1)/2; //En este caso Xlabel tiene el valor limite X (columnas), indicando asi, cuando se deben realizar
						 //los saltos de linea cuando se escriba en el mapa.
	
	X++; //Aumentamos X, debido a que hay que tener en cuenta que las coordenadas tiene como origen (0,0)
	Y++; //Aumentamos Y, debido a que hay que tener en cuenta que las coordenadas tiene como origen (0,0)
	
	if (debuggObstaclesDetectionFail) cout<<"Line 430 MSetcoord"<<endl;

	mapa.seekg(mapData.environment.beginEnvironment);
	//Expandimos el mapa y agregamos el valor a la coordenada especificada

	if (debuggMap) cout<<"Before Xlim Here"<<endl; //DEBUGG
	if (debuggMap) cout<<"VVVVVVVVV"<<endl; //DEBUGG
	if (debuggMap) cout<<"The value of X is: "<<X<<" And Y: "<<Y<<endl; //DEBUGG

	if (debuggCoord) debmap<<"X "<<X<<" Y "<<Y<<" Xlim "<<Xlim<<" Ylim "<<Ylim<<endl;
	
	if (debuggObstaclesDetectionFail) cout<<"Line 441 MSetcoord"<<endl;
	if (X > Xlim)
	{


		if (debuggObstaclesDetectionFail) cout<<"Line 445 MSetcoord"<<endl;

		if (debuggMap) cout<<"beggining of X > Xlim condition"<<endl; //DEBUGG

		if (debuggXlim) debmap<<"-----X > Xlim"<<endl;

		mapData.cells.X = X+1; // X-1 //Asignamos el nuevo limite en el eje X al respectivo campo de datos del mapa
		
		if (debuggMap) cout<<"We are in X>Xlim condition"<<endl; //DEBUGG

		if (debuggMap) cout<<"Values-> position.Yboundary: "<<position.Yboundary<<" Xlim: "<<Xlim<<endl; //DEBUGG
		if (debuggMap) cout<<"*******In loop*******"<<endl; //DEBUGG

		position.Xboundary = (X+1)*2 + 1; //Cambiamos el valor del límite en el eje X

		int elementsAdded = 0;

		if (debuggXlim) debmap<<"Xlim = "<<Xlim<<"X+1 = "<<X+1<<" Yboundary = "<<position.Yboundary<<endl;
		if (debuggXlim) debmap<<"<<<<<Before loop:>>>>>>"<<endl<<endl;

		if (debuggObstaclesDetectionFail) cout<<"Line 465 MSetcoord"<<" X = "<<X<<" Y = "<<Y<<" Yboundary = "<<position.Yboundary<<endl;

		for (int j = 0; j < position.Yboundary; j++){
			// data.push_back(0);
		// for (int j = 0; j < 22; j++){
			if (debuggXlim) debmap<<"j: "<<j<<endl;
			
			for (int i = Xlim; i < X+1; i++){ //i debe llegar a X+1 para obtener un margen mas de mapa
											  //donde el planificador sepa que puede pasar.
				if (debuggXlim) debmap<<"i = "<<i<<" j = "<<j<<" X = "<<X<<" elementsAdded = "<<elementsAdded<<" (i + j*X)= "<<i+j*X<<" data.size() = "<<data.size()<<" j+1*X = "<<(j+1)*(X)<<endl;
				
				// if ((i + j*X) > data.size()){
				// 	data.resize((j+1)*(X), 4); //Redimensionamos el vector, para introducir
				// 							   //los datos del mapa
				// 	debmap<<"DATA RESIZED"<<endl;													 
				// }
				// data[i+j*X] = 3;
				if (debuggObstaclesDetectionFail) cout<<j<<" . "<<i<<" Size = "<<data.size()<<" Capacity = "<<data.capacity()<<endl;
				if (j == position.Yboundary-1) data.push_back(0);
				data.insert(data.begin() + i + j*X + j, 0);
				if (debuggObstaclesDetectionFail) cout<<"size = "<<data.size()<<" Capacity = "<<data.capacity()<<endl;

				
			}
			if (debuggXlim) {
				debmap<<"MAP : ";
				for (int k = 0; k < data.size(); k++){
					debmap<<data[k]<<' ';
				}
				debmap<<endl;
			}
		}

		if (debuggObstaclesDetectionFail) cout<<"Line 494 MSetcoord"<<endl;

		if (debuggXlim) debmap<<"<<<<<After loop:>>>>>>"<<endl<<endl;

		if (debuggMap) cout<<"After for loop"<<endl; //DEBUGG
		
//		data.erase(data.end()-1); //¡¡¡¡¡¡¡Preguntar por que!!!!!!!
		
		Xlabel = X; //Nos indica que deberemos hacer saltos de linea cuando lleguemos a la X indicada por parametro
					//en la funcion. Esto es debido a que esta X sera mayor que la X limita en este caso.
		valuePosition = mapData.environment.beginEnvironment + 2*(X-1) + 2*Xlabel*(Y-1) + (Y-1); //Indicamos la posicion en la que se cambiara el valor en el mapa.
												//En este caso, la componente X de las coordenadas pasadas por
												//parametros es mayor que el limite del eje X del mapa.
											
		if (debuggXlim) debmap<<"valuePosition = "<<valuePosition<<endl;
		// position.Xboundary = X*2 + 1; //Cambiamos el valor del límite en el eje X

		if (debuggMap) cout<<"Ending X>Xlim condition"<<endl; //DEBUGG
					
	}else{

		if (debuggObstaclesDetectionFail) cout<<"Line 515 MSetcoord"<<endl;

		if (debuggMap) cout<<"else de X>Xlim condition"<<endl; //DEBUGG

//		valuePosition = mapData.environment.beginEnvironment + (Y-1)*(position.Xboundary-1)+ (X-1)*2; //Indicamos la posicion en la que se cambiara el valor en el mapa.
													   //En este caso, la componente X de las coordenadas pasadas por
													   //parametros no es mayor que el limite del eje X del mapa.
		valuePosition = mapData.environment.beginEnvironment + 2*(X-1) + 2*Xlabel*(Y-1) + (Y-1);

		if (debuggMap) cout<<"ending el else"<<endl; //DEBUGG
	}
	
	if (debuggMap) cout<<"Before Ylim"<<endl;
	
	if (Y >= position.Yboundary){

		if (debuggObstaclesDetectionFail) cout<<"Line 531 MSetcoord"<<endl;
//		cout<<endl<<"********Nos hemos pasado de los limites de Y***********"<<endl;
		if (debuggMap) cout<<"We are out of boundaries of Y"<<endl;

		if (debuggYlim) debmap<<"Y = "<<Y<<" Yboundary = "<<position.Yboundary<<endl;

		if (debuggYlim) debmap<<"-----Y > Ylim"<<endl;
		if (debuggYlim) debmap<<"Xlabel = "<<Xlabel<<endl;
		
		mapData.cells.Y = Y+1; //Asignamos el nuevo limite en el eje Y al respectivo campo de datos del mapa

		if (debuggMap) cout<<"Before doing loop"<<endl;

		if (debuggObstaclesDetectionFail) cout<<"Line 544 MSetcoord"<<endl;

		for (int j = position.Yboundary; j < Y+1; j++){ //j debe llegar hasta Y+1 debido a que asi siempre tendremos
			for (int i = 0; i < Xlabel; i++){			//una casilla mas de mapa, dando la posibilidad de que el robot
				data.push_back(0);						//tenga un camino mas extenso en el mapa.
				//if (debuggMap) cout<<"i :"<<i<<"j :"<<j<<data[i*j+i];
			}
		}

		if (debuggObstaclesDetectionFail) cout<<"Line 553 MSetcoord"<<endl;

		if (debuggMap) cout<<"After doing loop"<<endl;
		
		position.Yboundary = Y+1;
	}
	
	if (debuggMap) cout<<"Before variables to write in the map file"<<endl;
	//***********************************
	unsigned int saltar = 0;
	unsigned int limSaltar;
	
//	if (X>((position.Xboundary-1)/2)) limSaltar = X;
//	else limSaltar = (position.Xboundary-1)/2;

	limSaltar = Xlabel;
	
	//**********************************
	
	//Cambiamos el valor de la coordenada especificada
	// debmap<<"Valor en mapa: X = "<<X<<" Y = "<<Y<<endl;
	valuePositionVector = (X-1) + Xlabel*(Y-1);
	// data.erase(data.begin() + valuePositionVector);
	// data.insert(data.begin() + valuePositionVector, value);

	//NEW*******************************
	// if (value && data[valuePositionVector]<255){
	// 	data[valuePositionVector]++;
	// }else{
		if (debuggObstaclesDetectionFail) cout<<"Line 598 MSetcoord"<<endl;
		data.erase(data.begin() + valuePositionVector);
		data.insert(data.begin() + valuePositionVector, value);
	// }

	/*
	debmap<<"MAP : ";
		for (int k = 0; k < data.size(); k++){
			debmap<<data[k]<<' ';
		}
		debmap<<endl;
	*/

	//*****************************************

	if (debuggMap) cout<<"Value added: "<<value<<endl;
	if (debuggMap) cout<<"With X: "<<X<<" Y: "<<Y<<" Added in position: "<<valuePositionVector<<endl;
	//mapa.clear();
//	cout<<"Valor de X: "<<X<<endl<<"Valor de Y: "<<Y<<endl;
	//mapa.seekg(valuePosition);
	//mapa<<value;
	if (debuggMap) cout<<"Finishing set coord"<<endl;

	if (debuggObstaclesDetectionFail) cout<<"Line 621 MSetcoord"<<endl;
	
}

//****************************PRUEBA VECTOR**********************
void MapGenerator::pruebaVector(vector<int> myvector){
	vector<int>::iterator it;
	
	for (int i = 0; i < 10; i++){
		myvector.insert(myvector.begin()+ i, i);
	}
	
	for (it = myvector.begin(); it<myvector.end(); it++){
		if (debuggMap)cout<<"Numero: "<<*it<<endl;
	}
}

//****************************READ mapData**********************
//Lee los valores actuales de los datos del entorno en el archivo
//del mapa del entorno.
//**************************************************************
void MapGenerator::readMap(void){
	int it = 0;
	int value;
	vector<int>::iterator iter;
	
	mapa.seekg(mapData.environment.beginEnvironment);
	
	while (!mapa.eof()){
		mapa>>value;
		data.insert(data.begin()+it, value);
		it++;
	}
	
	data.erase(data.end()-1); //Borrar el último dato del vector de datos
	
	if (debuggMap){
	for (iter = data.begin(); iter<data.end()-1; iter++){
		cout<<*iter<<' ';
	}
	}
	
	mapa.clear();
}

//****************************CLEAR ENVIRONMENT**********************
//Elimina los datos del entorno en el archivo del mapa.
//*******************************************************************
void MapGenerator::clearEnvironment(void){
	mapa.clear();
	mapa.seekg(mapData.environment.beginEnvironment);
	for (long int i = mapData.environment.beginEnvironment; i<mapData.environment.endEnvironment; i++) mapa<<' ';
	mapa<<ends;
	mapa.seekg(mapa.tellg()-16);
	mapa<<ends;
}

//****************************SHOW DATA*******************************
//Muestra por pantalla los parámetros que estructuran el archivo del mapa.
//********************************************************************
void MapGenerator::showData(void){
	cout<<"CELLS  Coordenada X: "<<mapData.cells.X<<" Coordenada Y: "<<mapData.cells.Y<<endl;
	cout<<"Obsthresh value: "<<mapData.obsthresh.value<<endl;
	cout<<"START  Coordenada X: "<<mapData.start.X<<" Coordenada Y: "<<mapData.start.Y<<endl;
	cout<<"END Coordenadas X: "<<mapData.end.X<<" Coordenada Y: "<<mapData.end.Y<<endl;
	cout<<"START POINT: "<<mapData.environment.beginEnvironment<<endl;
}

//*********************************************************************
//********************TRANSFORMACIÓN DE COORDENADAS********************
//*********************************************************************
//*********************************************************************
//Se obtienen las coordenadas en formato del mapa.
//*********************************************************************
void MapGenerator::coordRobotToRelMap(int &X, int &Y){
	if (debuggMap) cout<<"Inicio coordRobotToRelMap"<<endl;
	if (debuggMap) cout<<"MapAccuracy "<<mapAccuracy<<endl;
	X = X / mapAccuracy; // mapAccuracy --> 200
	Y = Y / mapAccuracy; // mapAccuracy --> 200
	if (debuggMap) cout<<"Final coordRobotToRelMap"<<endl;
}

void MapGenerator::coordRelMapToRobot(int &X, int &Y){
	X = X * mapAccuracy;
	Y = Y * mapAccuracy;
}

void MapGenerator::coordAbsMapToRelMap(int &X, int &Y){
	X = X + startAbsolute.X;
	Y = Y + startAbsolute.Y;
}

void MapGenerator::printRobotCoords(int X, int Y){
	int m_X = X;
	int m_Y = Y;
	coordAbsMapToRelMap(m_X, m_Y);
	coordRelMapToRobot(m_X, m_Y);
	printf("X:%f Y:%f", (double)(m_X/1000.0), (double)(m_Y/1000.0));
}

void MapGenerator::printMapCoords(double X, double Y){
	int m_X = (int)(X*1000.0);
	int m_Y = (int)(Y*1000.0);
	coordRobotToRelMap(m_X, m_Y);
	coordRelMapToAbsMap(m_X, m_Y);
	cout<<"Xmap = "<<m_X<<" Ymap = "<<m_Y<<endl;

}

void MapGenerator::coordRelMapToAbsMap(int &X, int &Y){
	if (debuggMap) cout<<"***********coordRelMapToAbsMap***********"<<endl;
	if (debuggMap) cout<<"Valores coordenadas: ("<<X<<", "<<Y<<")"<<endl;
	if (debuggMap) cout<<"Valores startAbsolute: ("<<startAbsolute.X<<", "<<startAbsolute.Y<<")"<<endl;

	X = X - startAbsolute.X;
	Y = Y - startAbsolute.Y;
}

//*************OBSTACLE DETECTION************************
//Esta función se encarga de la creación del mapa del entorno mediante 
//la información recibida por parte del sensor láser del robot.
//*******************************************************
void MapGenerator::obstaclesDetection(double robotPosX, double robotPosY, const std::vector<float> &scan, float maxRange){
	if (debuggObstacles) debmap<<"OBSTACLESDETECTION_TWO CYCLE:"<<mapCycle<<endl;
	mapCycle++;

	cout<<"*************OBSTACLEDETECTION_TWO***************"<<endl; //DEBUGG

	if (debuggObstaclesDetectionFail) cout<<"Line 702 M"<<endl;

	int robotX;
	int robotY;
	int scanX;
	int scanY;
	bool Xnegative = false;
	bool Ynegative = false;
	
	if (debuggMap)cout<<"fxAbs: "<<robotPosX<<"fyAbs: "<<robotPosY<<endl;

	//Debemos tener en cuenta cuando el valor de las coordenadas del robot son negativas.
	//Si no lo tenemos en cuenta, pueda haber un conflicto entre tipo de variables.
	if (robotPosX<0){
		Xnegative = true;
	if (debuggMap)	cout<<"Negative X"<<endl;
	}
	if (robotPosY>0){  		//Ponemos la condicion mayor que 0 para mantener el mismo sistema de coordenadas
		Ynegative = true;   //que el mapa del robot.
	if (debuggMap)	cout<<"Negative Y"<<endl;
	}

	if (debuggObstaclesDetectionFail) cout<<"Line 724 M"<<endl;

	cout<<"robotPosX = "<<robotPosX<<" robotPosY = "<<robotPosY<<endl;
	
	robotX = ((int)(abs(robotPosX)*1000));
	robotY = ((int)(abs(robotPosY)*1000));
	
	if (debuggMap) cout<<"Robot coord before RelMap: X "<<robotX<<" Y "<<robotY<<endl;
	if (debuggObstacles) debmap<<"Robot coord before RelMap: X "<<robotX<<" Y "<<robotY<<endl;
	
	if (debuggObstaclesDetectionFail) cout<<"Line 732 M"<<endl;

	coordRobotToRelMap(robotX, robotY);

	cout<<"robotMapaX = "<<robotX<<" robotMapaY = "<<robotY<<endl;

	if (debuggObstaclesDetectionFail) cout<<"Line 736 M"<<endl;

	if (debuggObstacles) debmap<<"After relMAp: X "<<robotX<<" Y "<<robotY<<endl;
	
	robotX *= Xnegative ? -1 : 1 ;
	robotY *= Ynegative ? -1 : 1 ;

	if (debuggObstacles) debmap<<"Negativo?: X "<<robotX<<" Y "<<robotY<<endl;

	//Guardamos las coordenadas actuales del robot en el mapa de planificacion.
	robotAbsoluteValue.X = robotX;
	robotAbsoluteValue.Y = robotY;
	
	if (debuggMap) cout<<"*******Robot X: "<<robotX<<"   Robot Y: "<<robotY<<endl;

	if (debuggObstaclesDetectionFail) cout<<"Line 754 M"<<endl;
	
	//COMPROBAR SI ES MEJOR PONER A 0 LA POSICION ACTUAL DEL ROBOT!!
	//if (debuggMap) cout<<"Value to 0"<<endl;
	setCoord(robotX, robotY, 0);

	if (debuggObstaclesDetectionFail) cout<<"Line 760 M"<<endl;

	if (debuggObstacles) debmap<<"Robot coord for setCoor: X "<<robotX<<" Y "<<robotY<<endl;
	

	int valueTest = 1;  //DEBUGG
	float distanceLimit = 0.996;
	//********DEBUGG************
	bool getData = true;
	//**************************
	if (debuggObstaclesDetectionFail) cout<<"Line 768 M"<<endl;

	for (int i = 0; i < scan.size(); i++){
		if (scan[i] < distanceLimit){
			double angle_degrees = i*180.0/scan.size();
			double angle = M_PI *( -1 + 2*angle_degrees/((float)scan.size()) );
			double XrelativePosition = scan[i]*maxRange*cos(angle);
			double YrelativePosition = scan[i]*maxRange*sin(angle); 
			double Xposition = XrelativePosition + robotPosX;
			double Yposition = YrelativePosition + robotPosY;

			if (debuggObstacles) debmap<<"______________________________________________________________"<<endl;

			if (debuggObstacles) debmap<<i<<"-> robotX = "<<robotPosX<<" robotY = "<<robotPosY<<" Xposition = "<<Xposition<<" Yposition = "<<Yposition;

			scanX = (int)(Xposition*1000);
			scanY = (int)(Yposition*1000);

			if (debuggObstaclesDetectionFail) cout<<"Line 786 M "<<i<<endl;

			coordRobotToRelMap(scanX, scanY);

			if (debuggObstacles) debmap<<" scanX = "<<scanX<<" scanY = "<<scanY<<" dist = "<<scan[i]<<endl;

			if (debuggObstaclesDetectionFail) cout<<"Line 792 M "<<i<<" scanX = "<<scanX<<" scanY = "<<scanY<<endl;

			setCoord(scanX, -scanY, 1);

			if (debuggObstaclesDetectionFail) cout<<"Line 796 M "<<i<<endl;

			if (getData){
				getData = false;
				cout<<"*********POINT DATA***********"<<endl;
				cout<<"angle_degrees = "<<angle_degrees<<endl;
				cout<<"angle = "<<angle<<endl;
				cout<<"XrelativePosition = "<<XrelativePosition<<" YrelativePosition = "<<YrelativePosition<<endl;
				cout<<"Xposition = "<<Xposition<<" Yposition = "<<Yposition<<endl;
			}

		}
	}

	if (debuggObstaclesDetectionFail) cout<<"Line 794 M"<<endl;
	
	setStartAndEndPosition(); //Escribimos el valor del target en el mapa del planificador

	if (debuggObstaclesDetectionFail) cout<<"Line 798"<<endl;

}


//*****************SET START AND END POSITION************************
//Asigna valores a la posición inicial y final del camino que se debe
//planificar.
//*******************************************************************
void MapGenerator::setStartAndEndPosition(void){
	int target_X;
	int target_Y;
	int robot_X;
	int robot_Y;

	if (debuggMap) cout<<"******En setEndPosition******"<<endl; //DEBUGG
	if (debuggMap) cout<<"targetAbsoluteValue: ("<<targetAbsoluteValue.X<<", "<<targetAbsoluteValue.Y<<")"<<endl; //DEBUGG
	if (debuggMap) cout<<"robotAbsoluteValue: ("<<robotAbsoluteValue.X<<", "<<robotAbsoluteValue.Y<<")"<<endl; //DEBUGG

	target_X = targetAbsoluteValue.X;
	target_Y = targetAbsoluteValue.Y;

	robot_X = robotAbsoluteValue.X;
	robot_Y = robotAbsoluteValue.Y;

	coordRelMapToAbsMap(target_X, target_Y);
	coordRelMapToAbsMap(robot_X, robot_Y);

	mapData.end.X = target_X;
	mapData.end.Y = target_Y;

	mapData.start.X = robot_X;
	mapData.start.Y = robot_Y;

	if (debuggMap) cout<<endl<<"***+++****+++En setEndPosition. Valores del target final: ("<<mapData.end.X<<", "<<mapData.end.Y<<")"<<endl;
	if (debuggMap) cout<<endl<<"***+++****+++En setEndPosition. Valores del robot final: ("<<mapData.start.X<<", "<<mapData.start.Y<<")"<<endl;
}

void MapGenerator::getStartAbsolute(int &X, int &Y){
	cout<<"**********getStartAbsolute***********"<<endl;
	X = startAbsolute.X;
	Y = startAbsolute.Y;
}

void MapGenerator::getCellsInformation(int &X, int &Y){
	cout<<"***********getCellsInformation***********"<<endl;
	X = mapData.cells.X;
	Y = mapData.cells.Y;
}

void MapGenerator::getDataEnvironment(std::vector<int> &dataEnvironment){
	cout<<"**********getDataEnvironment*********"<<endl;
	cout<<"SIZE = "<<data.size()<<endl;
	dataEnvironment = data;
}