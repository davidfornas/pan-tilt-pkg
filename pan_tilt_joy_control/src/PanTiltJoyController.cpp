#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <pan_tilt_camera_teleop/PanTiltController.h>
#include <std_msgs/String.h>

#define UMBRAL 0.3

/** /////////////////////////////////////////////////////////////////////////////////////////
 * 	"axes" vector de 8 elementos															/
	joy->axes[0] ---->>  joystick izq, eje X ---> derecha = -1, izquierda = 1				/
	joy->axes[1] ---->>  joystick izq, eje Y ---> arriba = 1, abajo = -1					/
	joy->axes[2] ---->>  LT ---> reposo = 1, apretado = -1, mitad = 0						/
	joy->axes[3] ---->>  joystick der, eje X ---> derecha = -1, izquierda = 1				/
	joy->axes[4] ---->>  joystick der, eje Y ---> arriba = 1, abajo = -1					/
	joy->axes[5] ---->>  RT ---> reposo = 1, apretado = -1, mitad = 0						/
	joy->axes[6] ---->>  cruceta central, eje X ---> derecha = -1, izquierda = 1			/
	joy->axes[7] ---->>  cruceta central, eje Y ---> arriba = 1, abajo = -1					/
																							/
 *	"buttons" vector de 11 elementos. Todos tienen valor 0 en reposo y valor 1 pulsados		/
	joy->buttons[0] ---->> A																/
	joy->buttons[1] ---->> B	---> AUTO-FOCUS												/
	joy->buttons[2] ---->> X	---> BUSCAR OBJETO											/
	joy->buttons[3] ---->> Y	---> ON/OFF SURF											/
	joy->buttons[4] ---->> LB																/
	joy->buttons[5] ---->> RB																/
	joy->buttons[6] ---->> BACK																/
	joy->buttons[7] ---->> START															/
	joy->buttons[8] ---->> XBox																/
	joy->buttons[9] ---->> Joystick izquierdo												/
	joy->buttons[10] ---->> Joystick derecho												/
///////////////////////////////////////////////////////////////////////////////////////////*/
//Los botones LT y RT hay que controlarlos de forma distinta por que los valores que devuelven
//son diferentes de los demas

PanTiltController _ptc;

bool _dir, _bandera = false, _buttons = false;//, apagado = true;
int _indice;
boost::thread active_Surf;
/*bool _l_joy = false;


void launch_Joy(){
	_l_joy = true;
	
	system("rosrun joy joy_node");
}*/



//Función especial para los Joysticks
//Con ella controlo la dirección dependiendo de si el valor es negativo o pisitivo
inline void auxFunction(const float dato){
	if(dato < -UMBRAL){
		_dir = false;
		_bandera = true;
	}
	if(dato > UMBRAL){
		_dir = true;
		_bandera = true;
	}
}


void activeSurf()
{
  std::cout << "Lanzando el Surf" << std::endl;
  
  system("roslaunch object_localization object_localization_surf.launch");
  //system("rosrun object_localization surfv");
}

/** Busqueda del nodo /surfv */
bool findNode()
{
	system("rosnode list >> /home/usuario/log.txt");
	
	ifstream fich("/home/usuario/log.txt");
	std::string nodeList;
	while(!fich.eof()) 
	{
		getline(fich, nodeList);
		std::cout << nodeList << std::endl;
		if(!nodeList.find("/surfv"))
		{
			fich.close();			
			system("rm /home/usuario/log.txt");
		 	return true;	
		}
	}
	fich.close();
	system("rm /home/usuario/log.txt");
	return false;
}


int compruebaOrden(){
	if(_buttons)
		return _indice+8;
	return _indice;
}



void envia_OrdenParada(){
	int dato = compruebaOrden();
	switch(dato){
		case 12:
			std::cout << "STOP ZOOM-" << std::endl;
			_ptc.wideStop();
			break;
		case 13:
			std::cout << "STOP ZOOM+" << std::endl;
			_ptc.teleStop();
			break;
		default: _ptc.stopPanTilt();	//Hay una función para parar cualquier rotación de la cámara
	}
}




void envia_Orden(){
	int dato = compruebaOrden();
	switch(dato){
		case 0:
			if(!_dir){
				std::cout << "DERECHA" << std::endl;
				_ptc.right();}
			else{
				std::cout << "IZQIERDA" << std::endl;
				_ptc.left();}
			break;
		case 1:
			if(!_dir){
				std::cout << "BAJAR" << std::endl;
				_ptc.tiltdown();}
			else{
				std::cout << "SUBIR" << std::endl;
				_ptc.tiltup();}
			break;
		case 9:
				std::cout << "AUTO-FOCUS" << std::endl;
				system("rosrun pan_tilt_image_processing auto_focus_client 30");
			break;
		case 10:
				std::cout << "BUSCAR OBJETO" << std::endl;
				system("rosrun pan_tilt_object_localization find_object_client");
			break;
		case 11:
				if (findNode())
				{ 
					system("rosnode kill surfv");
					active_Surf.join();
					//apagado = true;
				}
				else
				{ 
					//apagado = false;
					boost::thread active_Surf(&activeSurf); 
				}
			break;
		case 12:
				std::cout << "ZOOM-" <<	std::endl;
				_ptc.wideStart();
			break;
		case 13:
				std::cout << "ZOOM+" << std::endl;
				_ptc.teleStart();
			break;
	}
}



template <class T>
void findInJoy(const std::vector<T> &axes, int init, int end = 0){
	int fin = (end == 0 ? axes.size() : end);
	if(!_bandera){
		float_t dato;
		for(int i = init; i < fin; i++){
			dato = axes[i];
			if(!_buttons)	//si no son los botones, controlo el LT y el RT
				if(i == 2 or i == 5)
					dato = dato -1;
			if(dato > UMBRAL or dato < -UMBRAL){
				_indice = i;
				auxFunction(dato);
			}
					
			if(_bandera){
				envia_Orden();
				break;
			}
		}
	}
}




template <class T>
bool joyStop(const std::vector<T> &axes, T const& umb = 0){
	float_t dato = axes[_indice];
	if(!_buttons){			//si es el vector de ejes, miro si son el LT o el RT
		if(_indice == 2 or _indice == 5){
			dato = dato -1;
			if(dato > -umb){
				envia_OrdenParada();
				_bandera = false;
				return true;
			}
		}
	}
	if(abs(axes[_indice]) < umb){		// and axes[_indice] > -umbral){
		envia_OrdenParada();
		_bandera = false;
		return true;
	}
		
	return false;
}





void joy_CallBack(const sensor_msgs::Joy::ConstPtr& joy){
		if(!_bandera){
			_buttons = false;
			findInJoy(joy->axes, 0, 2); //, (float)0.3);
			if(!_bandera){
				_buttons = true;
				findInJoy(joy->buttons, 1, 6); //, (int)1);
			}
		}
		else{
			if(!_buttons)
				joyStop(joy->axes, (float)0.3);
			else{
				joyStop(joy->buttons, (int)1);
			}
		}		
}


int main(int argc, char **argv){
	/*if(!_l_joy){
		boost::thread th1(launch_Joy);
		
		th1.join();
	}*/
	
	ros::init(argc, argv, "PanTiltJoyController");
	ros::NodeHandle n;
	
	ros::Subscriber sub_joy = n.subscribe("/joy", 10, joy_CallBack);
	//ros::Publisher pub_joy = n.advertise<std_msgs::String>("/stop/surf", 1);
	
	//std_msgs::StringPtr str(new std_msgs::String);
	while(ros::ok())
	{
		//if(apagado) str->data = "Off"; 
		//else str->data = "On";
   		//pub_joy.publish(str);
		ros::spinOnce();
	}
	
	return 0;
}
