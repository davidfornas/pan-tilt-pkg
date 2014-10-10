pan-tilt-pkg
============

Pan tilt controller and grabber package.
Catkin version.

pan_tilt_camera_teleop
======================
Pan tilt camera low level driver. Controls movements, iris, zoom, etc.
PanTiltController.cpp es donde esta el driver que se comunica mediante 
puerto serie con la cámara. Tiene funciones en su mayoría que empiezan 
o acaban el movimiento físico de algún componente de la cámara.


* keyboard: permite probar las funciones de la cámara por teclado.
* keyboardToTopic: Ejemplo de cliente que pone en un mensaje el pan-tilt
deseado a partir de teclado.
* listener: Ejemplo servidor para el ejemplo de cliente.
* vs: Ejemplo de visual servoing o seguimiento que se comunica directo 
con la cámara. 

pan_tilt_camera_grabber
======================
Pan tilt camera grabber. Grabs camera image from FireWire grabber.
Use launch/PanTiltGrabberRaw.launch for raw images and 
launch/PanTiltGrabberUndistort.launch for rectified images.

