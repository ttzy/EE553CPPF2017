/**
*
* @Name:	ngimu.cpp
* @Author:	Duro UAS
* @Version:	0.0.1
* @Description:	a C++ code that publishes imu data with the following topics: imu/sensors, imu/quaternion, imu/euler, imu/rotation_matrix, imu/linear_acceleration, imu/earth_acceleration, imu/magnitudes, imu/altitude, imu/temperature, and imu/humidity. The hardware used is NGIMU developed by the x-io Technologies Inc. The ROS package runs on Jetson TX1 (Ubuntu 16.04).
*
**/

//------------------------------------------------------------------------------
// Includes
/**
* Call the required libraries for the file to run.
* Osc99 and NgimuReceive are the libraries developed by the x-io technologies Inc.
**/
/*
#include "Osc99/Osc99.h"
#include "NgimuReceive.h"
#include "NgimuReceive/NgimuReceive.h"
#include "serial/serial.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ngimu/NGIMUSensors.h>
#include <ngimu/NGIMUQuaternion.h>
#include <ngimu/NGIMUEuler.h>
#include <ngimu/NGIMURotationMatrix.h>
#include <ngimu/NGIMULinearAcceleration.h>
#include <ngimu/NGIMUEarthAcceleration.h>
#include <ngimu/NGIMUMagnitudes.h>
#include <ngimu/NGIMUAltitude.h>
#include <ngimu/NGIMUTemperature.h>
#include <ngimu/NGIMUHumidity.h>
*/

#include "ngimureceive.h"
//#include "serial/serial.h"
#include <string.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <iostream>
#include <fstream>
#include "serial_port.c"
//------------------------------------------------------------------------------
// Variables
/*
* @port : the serial port for the NGIMU to use in Ubuntu 16.04
* @baudrate : the baudrate for the serial port connection
* @timeout : the timeout rate for the serial port connection
* @loopRate : the rate at which the ros::spin() spins in Hz
* @imu_<data type>_pub : the ROS publisher instances to publish different kinds of data from the device
*/

//std::string port = "/dev/ttyACM0";

//int baudrate = 115200;
//int timeout = 1000;
/*int loopRate = 20000;

ros::Publisher imu_sensor_pub;
ros::Publisher imu_quaternion_pub;
ros::Publisher imu_euler_pub;
ros::Publisher imu_rotation_matrix_pub;
ros::Publisher imu_linear_accel_pub;
ros::Publisher imu_earth_accel_pub;
ros::Publisher imu_magnitudes_pub;
ros::Publisher imu_altitude_pub;
ros::Publisher imu_temperature_pub;
ros::Publisher imu_humidity_pub;

*/
//------------------------------------------------------------------------------
// Callback functions (Declaration)
/*
* the functions below are used as callback functions; they will be taken as parameters of the functions
* whose names start with NgimuReceiveSet- (the functions declared at the library NgimuReceive.h). 
*/
void ngimuReceiveErrorCallback(const char* const errorMessage);
void ngimuSensorsCallback(const NgimuSensors ngimuSensors);
void ngimuQuaternionCallback(const NgimuQuaternion ngimuQuaternion);
void ngimuEulerCallback(const NgimuEuler ngimuEuler);
void ngimuRotationMatrixCallback(const NgimuRotationMatrix ngimuRotationMatrix);
void ngimuLinearAccelerationCallback(const NgimuLinearAcceleration ngimuLinearAcceleration);
void ngimuEarthAccelerationCallback(const NgimuEarthAcceleration ngimuEarthAcceleration);
void ngimuMagnitudesCallback(const NgimuMagnitudes ngimuMagnitudes);
void ngimuAltitudeCallback(const NgimuAltitude ngimuAltitude);
void ngimuTemperatureCallback(const NgimuTemperature ngimuTemperature);
void ngimuHumidityCallback(const NgimuHumidity ngimuHumidity);
void ngimuAuxserialCallback(const NgimuAuxserial ngimuAuxserial);
void ngimuBatteryCallback(const NgimuBattery ngimuBattery);


//------------------------------------------------------------------------------
// Main function
int main(/*int argc, char **argv*/)
{
  /*
  // initialize a ROS node named ngimu, a NodeHandle n, and the list of publishers
  ros::init(argc, argv, "ngimu");
  ros::NodeHandle n;
  imu_sensor_pub = n.advertise<ngimu::NGIMUSensors>("imu/sensors", 1000);
  imu_quaternion_pub = n.advertise<ngimu::NGIMUQuaternion>("imu/quaternion", 1000);
  imu_euler_pub = n.advertise<ngimu::NGIMUEuler>("imu/euler", 1000);
  imu_rotation_matrix_pub = n.advertise<ngimu::NGIMURotationMatrix>("imu/rotation_matrix", 1000);
  imu_linear_accel_pub = n.advertise<ngimu::NGIMULinearAcceleration>("imu/linear_acceleration", 1000);
  imu_earth_accel_pub = n.advertise<ngimu::NGIMUEarthAcceleration>("imu/earth_acceleration", 1000);
  imu_magnitudes_pub = n.advertise<ngimu::NGIMUMagnitudes>("imu/magnitudes", 1000);
  imu_altitude_pub = n.advertise<ngimu::NGIMUAltitude>("imu/altitude", 1000);
  imu_temperature_pub = n.advertise<ngimu::NGIMUTemperature>("imu/temperature", 1000);
  imu_humidity_pub = n.advertise<ngimu::NGIMUHumidity>("imu/humidity", 1000);
  imu_auxserial_pub = n.advertise<ngimu::NGIMUAuxserial>("imu/auxserial", 1000);
  ros::Rate loop_rate(loopRate);
  */

  f = fopen("IMUdata.txt", "w");

  // call the NgimuReceiveInitialise() function from the library NgimuReceive to initialize NGIMU-related programs
  NgimuReceiveInitialise();

  // Assign the declared callback functions starting with ngimu- (e.g. ngimuSensorsCallback) to the callback functions declared in the library NgimuReceive.h
  NgimuReceiveSetReceiveErrorCallback(ngimuReceiveErrorCallback);
  NgimuReceiveSetSensorsCallback(ngimuSensorsCallback);
  NgimuReceiveSetQuaternionCallback(ngimuQuaternionCallback);
  NgimuReceiveSetEulerCallback(ngimuEulerCallback);
  NgimuReceiveSetRotationMatrixCallback(ngimuRotationMatrixCallback);
  NgimuReceiveSetLinearAccelerationCallback(ngimuLinearAccelerationCallback);
  NgimuReceiveSetEarthAccelerationCallback(ngimuEarthAccelerationCallback);
  NgimuReceiveSetMagnitudesCallback(ngimuMagnitudesCallback);
  NgimuReceiveSetAltitudeCallback(ngimuAltitudeCallback);
  NgimuReceiveSetTemperatureCallback(ngimuTemperatureCallback);
  NgimuReceiveSetAuxserialCallback(ngimuAuxserialCallback);
  NgimuReceiveSetHumidityCallback(ngimuHumidityCallback);
  NgimuReceiveSetBatteryCallback(ngimuBatteryCallback);
  
  // Initialize Serial object, including port, baudrate, and timeout. 
  // If IOException is thrown, close the serial port and exit.
    
    //This is the Serial Port to the NGIMU that will be opened
/*
  struct termios ngimuOptions;

  char *portName = "/dev/ttyACM0";

  int fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY | O_SYNC);

  if (fd == -1) {
    std::cout << fd << ", " << portName;
    printf("Serial Port: %s could not be opened properly\n", portName);
    exit(1);
  }
  else {
    printf("Serial Port: %s opened successfully\n", portName);
  }

  fcntl(fd, F_SETFL, 0);

  cfsetospeed(&ngimuOptions, B115200);
  cfsetispeed(&ngimuOptions, B115200);

  ngimuOptions.c_cflag &= ~CSIZE;
  ngimuOptions.c_cflag |= CS8;

  ngimuOptions.c_iflag &= ~IGNBRK;

  ngimuOptions.c_cc[VMIN] = 0;

  char byte [2];

  //check(fd);

  while(1) {
    read(fd, byte, 1);
    NgimuReceiveProcessSerialByte(byte[0]);
  }*/
  /*
  serial::Serial serial;
  serial.setPort(port);
  serial.setBaudrate(baudrate);
  serial::Timeout to = serial::Timeout::simpleTimeout(timeout);
  serial.setTimeout(to);
  try{
    serial.open();
  }catch(serial::IOException ex){
     
     //ROS_INFO("cannot open the serial port because of IOException");
     serial.close();
     return -1;
  }
  
  // if the serial port is not opened, exit
  if (!serial.isOpen()){
 	  std::cout << "error opening serial port";
  //ROS_INFO("error opening serial port");
    return -1;
  }else{
    std::cout << "successfully opened serial port";
    //ROS_INFO("successfully opened serial port!");
  }

  // in a while loop, read the data from serial and pass the char version of the data (its type is string),
  // and pass the data of type char to the function NgimuReceiveProcessSerialByte
  while (1) {
    if (serial.available()) {
      std::string result = serial.read();
      char data_char = *result.c_str();
      NgimuReceiveProcessSerialByte(data_char);
    }
    else {
      std::cout << "Serial not available." << endl;
    }
  }
  */
  /*
  while (ros::ok())
  {

    if (serial.available()){
    	std::string result = serial.read();
    	char data_char = *result.c_str();
    	NgimuReceiveProcessSerialByte(data_char);
    }else{
    	ROS_INFO("serial not available");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  */
  //serial.close();

  char *portname = "/dev/ttyACM0";
  int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    printf ("Serial port cannot be opened");
  }
  set_interface_attribs(fd, B115200, 0);
  set_blocking(fd, 0);
  char buf [2];
  while(1) {
    int n = read(fd, buf, 1);
    //printf("%c", buf[0]);
    NgimuReceiveProcessSerialByte(buf[0]);
  }
  fclose(f);
  return 0;
}

//------------------------------------------------------------------------------
// Callback functions (Implementation)

/**
* @brief	initializes callback function to display error message
* @parameter
*		- errorMessage			the error message to be displayed
*/
void ngimuReceiveErrorCallback(const char* const errorMessage) {
    std::cout << "Error Message Callback: " << errorMessage << std::endl;
    fprintf(f, "Error Memssage Callback: %s", errorMessage);
    //ROS_INFO("Error Message Callback: %s", errorMessage);
}


// Implement the callback functions whose names start with ngimu-.  
/*
* @brief	initializes callback function to get data of type NgimuSensors, use it to initialize ngimu::NGIMUSensors data, and 
*		publish the ngimu::NGIMUSensors data using the publisher instance imu_sensor_pub. This function is called each time a 
*		"/sensors" message is received.
* @parameter			
*		- ngimuSensors			the NgimuSensors instance declared in NgimuReceive.h
*/
void ngimuSensorsCallback(const NgimuSensors ngimuSensors) {
    std::cout << "/sensors, ";
    std::cout << ngimuSensors.gyroscopeX << ", ";
    std::cout << ngimuSensors.gyroscopeY << ", ";
    std::cout << ngimuSensors.gyroscopeZ << ", ";
    std::cout << ngimuSensors.accelerometerX << ", ";
    std::cout << ngimuSensors.accelerometerY << ", ";
    std::cout << ngimuSensors.accelerometerZ << ", ";
    std::cout << ngimuSensors.magnetometerX << ", ";
    std::cout << ngimuSensors.magnetometerY << ", ";
    std::cout << ngimuSensors.magnetometerZ << ", ";
    std::cout << ngimuSensors.barometer << std::endl;

    fprintf(f, "/sensors, ");
    fprintf(f, "%E, ", ngimuSensors.gyroscopeX);
    fprintf(f, "%E, ", ngimuSensors.gyroscopeY);
    fprintf(f, "%E, ", ngimuSensors.gyroscopeZ);
    fprintf(f, "%E, ", ngimuSensors.accelerometerX);
    fprintf(f, "%E, ", ngimuSensors.accelerometerY);
    fprintf(f, "%E, ", ngimuSensors.accelerometerZ);
    fprintf(f, "%E, ", ngimuSensors.magnetometerX);
    fprintf(f, "%E, ", ngimuSensors.magnetometerY);
    fprintf(f, "%E, ", ngimuSensors.magnetometerZ);
    fprintf(f, "%E, \n", ngimuSensors.barometer);
    /* ROS Topic
    ngimu::NGIMUSensors sensors;
    sensors.gyroscopeX = ngimuSensors.gyroscopeX;
    sensors.gyroscopeY = ngimuSensors.gyroscopeY;
    sensors.gyroscopeZ = ngimuSensors.gyroscopeZ;
    sensors.accelerometerX = ngimuSensors.accelerometerX;
    sensors.accelerometerY = ngimuSensors.accelerometerY;
    sensors.accelerometerZ = ngimuSensors.accelerometerZ;
    sensors.magnetometerX = ngimuSensors.magnetometerX;
    sensors.magnetometerY = ngimuSensors.magnetometerY;
    sensors.magnetometerZ = ngimuSensors.magnetometerZ;
    sensors.barometer = ngimuSensors.barometer;
    imu_sensor_pub.publish(sensors);
    */
}

/**
* @brief	initializes callback function to get data of type NgimuQuaternion, use it to initialize ngimu::NGIMUQuaternion data, 
*		and publish the ngimu::NGIMUQuaternion data using the publisher instance imu_quaternion_pub. This function is called 
*		each time a "/quaternion" message is received.
* @parameter
*		- ngimuQuaternion		the NgimuQuaternion instance declared in NgimuReceive.h
*/
void ngimuQuaternionCallback(const NgimuQuaternion ngimuQuaternion) {
    std::cout << "/quaternion, ";
    std::cout << ngimuQuaternion.w << ", ";
    std::cout << ngimuQuaternion.x << ", ";
    std::cout << ngimuQuaternion.y << ", ";
    std::cout << ngimuQuaternion.z << std::endl;

    fprintf(f, "/quaternion, ");
    fprintf(f, "%E, ", ngimuQuaternion.w);
    fprintf(f, "%E, ", ngimuQuaternion.x);
    fprintf(f, "%E, ", ngimuQuaternion.y);
    fprintf(f, "%E, \n", ngimuQuaternion.z);
    /*
    ngimu::NGIMUQuaternion quaternion;
    quaternion.w = ngimuQuaternion.w;
    quaternion.x = ngimuQuaternion.x;
    quaternion.y = ngimuQuaternion.y;
    quaternion.z = ngimuQuaternion.z;
    imu_quaternion_pub.publish(quaternion);
    */
}

/**
* @brief	initializes callback function to get data of type NgimuEuler, use it to initialize ngimu::NGIMUEuler data, and publish 
*		the ngimu::NGIMUEuler data using the publisher instance imu_euler_pub. This function is called each time a "/euler" 
*		message is received.
* @parameter
*		- ngimuEuler			the NgimuEuler instance declared in NgimuReceive.h
*/

void ngimuEulerCallback(const NgimuEuler ngimuEuler) {
    std::cout << "/euler, ";
    std::cout << ngimuEuler.roll << ", ";
    std::cout << ngimuEuler.pitch << ", ";
    std::cout << ngimuEuler.yaw << std::endl;

    fprintf(f, "/euler, ");
    fprintf(f, "%E, ", ngimuEuler.roll);
    fprintf(f, "%E, ", ngimuEuler.pitch);
    fprintf(f, "%E, \n", ngimuEuler.yaw);
    /*
    ngimu::NGIMUEuler euler;
    euler.roll = ngimuEuler.roll;
    euler.pitch = ngimuEuler.pitch;
    euler.yaw = ngimuEuler.yaw;
    imu_euler_pub.publish(euler);
    */
}

//This function is called each time a "/auxserial" message is received.
void ngimuAuxserialCallback(const NgimuAuxserial ngimuAuxserial){
  std::cout << "/auxserial, ";
  std::cout << ngimuAuxserial.message1 << std::endl;
  //char *token;
  fprintf(f, "/auxserial, ");
  fprintf(f, "%s, \n", ngimuAuxserial.message1);

  /*
  ngimu::NGIMUAuxserial auxserial;
  auxserial.message1 = ngimuAuxserial.message1;
  imu_auxserial_pub.publish(auxserial);
  */
}

//This function is called each time a "/battery" message is received
void ngimuBatteryCallback(const NgimuBattery ngimuBattery) {
  std::cout << "/battery, ";
  std::cout << ngimuBattery.batterylevel << ", ";
  std::cout << ngimuBattery.timetoempty << ", ";
  std::cout << ngimuBattery.voltage << ", ";
  std::cout << ngimuBattery.current << ", ";
  std::cout << ngimuBattery.state << '\n';

  fprintf(f, "/battery, ");
  fprintf(f, "%E, ", ngimuBattery.batterylevel);
  fprintf(f, "%E, ", ngimuBattery.timetoempty);
  fprintf(f, "%E, ", ngimuBattery.voltage);
  fprintf(f, "%E, ", ngimuBattery.current);
  fprintf(f, "%s, \n", ngimuBattery.state);
 
  /*
  ngimu::NGIMUBattery battery;
  battery.batterylevel = ngimuBattery.batterylevel;
  battery.timetoempty = ngimuBattery.timetoempty;
  battery.voltage = ngimuBattery.voltage;
  battery.current = ngimuBattery.current;
  battery.state = ngimuBattery.state;
  imu_battery_pub.pubish(battery);
  */
}

/**
* @brief	initializes callback function to get data of type NgimuRotationMatrix, use it to initialize ngimu::NGIMURotationMatrix *		data, and publish the ngimu::NGIMURotationMatrix data using the publisher instance imu_rotation_matrix_pub. This 
*		function is called each time a "/matrix" message is received.
* @parameter	
*		- ngimuRotationMatrix		the NgimuRotationMatrix instance declared in NgimuReceive.h
*/
void ngimuRotationMatrixCallback(const NgimuRotationMatrix ngimuRotationMatrix){
    std::cout << "/rotation matrix, ";
    std::cout << ngimuRotationMatrix.xx << ", ";
    std::cout << ngimuRotationMatrix.xy << ", ";
    std::cout << ngimuRotationMatrix.xz << ", ";
    std::cout << ngimuRotationMatrix.yx << ", ";
    std::cout << ngimuRotationMatrix.yy << ", ";
    std::cout << ngimuRotationMatrix.yz << ", ";
    std::cout << ngimuRotationMatrix.zx << ", ";
    std::cout << ngimuRotationMatrix.zy << ", ";
    std::cout << ngimuRotationMatrix.zz << '\n';

    fprintf(f, "/rotation matrix, ");
    fprintf(f, "%E, ", ngimuRotationMatrix.xx);
    fprintf(f, "%E, ", ngimuRotationMatrix.xy);
    fprintf(f, "%E, ", ngimuRotationMatrix.xz);
    fprintf(f, "%E, ", ngimuRotationMatrix.yx);
    fprintf(f, "%E, ", ngimuRotationMatrix.yz);
    fprintf(f, "%E, ", ngimuRotationMatrix.zx);
    fprintf(f, "%E, ", ngimuRotationMatrix.zy);
    fprintf(f, "%E, ", ngimuRotationMatrix.zz);
    /*
    ngimu::NGIMURotationMatrix matrix;
    matrix.xx = ngimuRotationMatrix.xx;
    matrix.xy = ngimuRotationMatrix.xy;
    matrix.xz = ngimuRotationMatrix.xz;
    matrix.yx = ngimuRotationMatrix.yx;
    matrix.yy = ngimuRotationMatrix.yy;
    matrix.yz = ngimuRotationMatrix.yz;
    matrix.zx = ngimuRotationMatrix.zx;
    matrix.zy = ngimuRotationMatrix.zy;
    matrix.zz = ngimuRotationMatrix.zz;
    imu_rotation_matrix_pub.publish(matrix);
    */
}

/**
* @brief	initializes callback function to get data of type NgimuLinearAcceleration, use it to initialize 
*		ngimu::NGIMULinearAcceleration data, and publish the ngimu::NGIMULinearAcceleration data using the publisher instance *		imu_linear_accel_pub. This function is called each time a "/linear" message is received.
* @parameter
*		- ngimuLinearAcceleration	the NgimuLinearAcceleration instance declared in NgimuReceive.h
*/
void ngimuLinearAccelerationCallback(const NgimuLinearAcceleration ngimuLinearAcceleration){
    std::cout << "/linear acceleration, ";
    std::cout << ngimuLinearAcceleration.x << ", ";
    std::cout << ngimuLinearAcceleration.y << ", ";
    std::cout << ngimuLinearAcceleration.z << std::endl;

    fprintf(f, "/linear acceleration, ");
    fprintf(f, "%E, ", ngimuLinearAcceleration.x);
    fprintf(f, "%E, ", ngimuLinearAcceleration.y);
    fprintf(f, "%E, \n", ngimuLinearAcceleration.z);
    /*
    ngimu::NGIMULinearAcceleration linear_accel;
    linear_accel.x = ngimuLinearAcceleration.x;
    linear_accel.y = ngimuLinearAcceleration.y;
    linear_accel.z = ngimuLinearAcceleration.z;
    imu_linear_accel_pub.publish(linear_accel);
    */
}

/**
* @brief	initializes callback function to get data of type NgimuEarthAcceleration, use it to initialize 
*		ngimu::NGIMUEarthAcceleration data, and publish the ngimu::NGIMUEarthAcceleration data using the publisher instance 
*		imu_earth_accel_pub. This function is called each time a "/earth" message is received.
* @parameter
*		- ngimuEarthAcceleration	the NgimuEarthAcceleration instance declared in NgimuReceive.h
*/
void ngimuEarthAccelerationCallback(const NgimuEarthAcceleration ngimuEarthAcceleration){
    std::cout << "/earth acceleration, ";
    std::cout << ngimuEarthAcceleration.x << ", ";
    std::cout << ngimuEarthAcceleration.y << ", ";
    std::cout << ngimuEarthAcceleration.z << std::endl;

    fprintf(f, "/earth acceleration, ");
    fprintf(f, "%E, ", ngimuEarthAcceleration.x);
    fprintf(f, "%E, ", ngimuEarthAcceleration.y);
    fprintf(f, "%E, \n", ngimuEarthAcceleration.z);
    /*
    ngimu::NGIMUEarthAcceleration earth_accel;
    earth_accel.x = ngimuEarthAcceleration.x;
    earth_accel.y = ngimuEarthAcceleration.y;
    earth_accel.z = ngimuEarthAcceleration.z;
    imu_earth_accel_pub.publish(earth_accel);
    */
}

/**
* @brief	initializes callback function to get data of type NgimuMagnitudes, use it to initialize ngimu::NGIMUMagnitudes data, 
*		and publish the ngimu::NGIMUMagnitudes data using the publisher instance imu_magnitudes_pub. This function is called 
*		each time a "/magnitudes" message is received.
* @parameter
*		- ngimuMagnitudes		the NgimuMagnitudes instance declared in NgimuReceive.h
*/
void ngimuMagnitudesCallback(const NgimuMagnitudes ngimuMagnitudes){
   std::cout << "/magnitudes, ";
   std::cout << "Gyroscope Magnitude, " << ngimuMagnitudes.gyroscopeMagnitude << ", ";
   std::cout << "Accelerometer Magnitude, " << ngimuMagnitudes.accelerometerMagnitude << ", ";
   std::cout << "Magnetometer Magnitude, " << ngimuMagnitudes.magnetometerMagnitude << std::endl;

   fprintf(f, "/magnitudes, ");
   fprintf(f, "Gyroscope Magnitude, %E, ", ngimuMagnitudes.gyroscopeMagnitude);
   fprintf(f, "Accelerometer Magnitude, %E, ", ngimuMagnitudes.accelerometerMagnitude);
   fprintf(f, "Magnetometer Magnitude, %E, \n", ngimuMagnitudes. magnetometerMagnitude);
   /*
   ngimu::NGIMUMagnitudes magnitudes;
   magnitudes.mag_gyroscope = ngimuMagnitudes.gyroscopeMagnitude;
   magnitudes.mag_accelerometer = ngimuMagnitudes.accelerometerMagnitude;
   magnitudes.mag_magnetometer = ngimuMagnitudes.magnetometerMagnitude;
   imu_magnitudes_pub.publish(magnitudes);
   */
}

/**
* @brief	initializes callback function to get data of type NgimuAltitude, use it to initialize ngimu::NGIMUAltitude data, and 
*		publish the ngimu::NGIMUAltitude data using the publisher instance imu_altitude_pub. This function is called each time 
*		a "/altitude" message is received.
* @parameter
*		- ngimuAltitude			the NgimuAltitude instance declared in NgimuReceive.h
*/
void ngimuAltitudeCallback(const NgimuAltitude ngimuAltitude){
   std::cout << "/altitude, ";
   std::cout << ngimuAltitude.altitude << std::endl;

   fprintf(f, "/magnitudes, ");
   fprintf(f, "%E, \n", ngimuAltitude.altitude);
   /*
   ngimu::NGIMUAltitude altitude;
   altitude.altitude = ngimuAltitude.altitude;
   imu_altitude_pub.publish(altitude);
   */
}

/**
* @brief	initializes callback function to get data of type NgimuTemperature, use it to initialize ngimu::NGIMUTemperature data, 
*		and publish the ngimu::NGIMUTemperature data using the publisher instance imu_temperature_pub. This function is called 
*		each time a "/temperature" message is received.
* @parameter
*		- ngimuTemperature		the NgimuTemperature instance declared in NgimuReceive.h
*/
void ngimuTemperatureCallback(const NgimuTemperature ngimuTemperature){
   std::cout << "/temperature, ";
   std::cout << "Processor Temperature, " << ngimuTemperature.processor_Temp << ", ";
   std::cout << "Gyroscope Temperature, " << ngimuTemperature.gyro_accel_Temp << std::endl;

   fprintf(f, "/temperature, ");
   fprintf(f, "Processor Temperature, %E, ", ngimuTemperature.processor_Temp);
   fprintf(f, "GYroscope Temperature, %E, \n", ngimuTemperature.gyro_accel_Temp);


   //std::cout << "Barometer Temperature, " << ngimuTemperature.barometer_Temp << std::endl;
   /*
   ngimu::NGIMUTemperature temperature;
   temperature.temp_processor = ngimuTemperature.processor_Temp;
   temperature.temp_gyro_accel = ngimuTemperature.gyro_accel_Temp;
   temperature.temp_barometer = ngimuTemperature.barometer_Temp;
   imu_temperature_pub.publish(temperature);
   */
}


/**
* @brief	initializes callback function to get data of type NgimuHumidity, use it to initialize ngimu::NGIMUHumidity data, and 
*		publish the ngimu::NGIMUHumidity data using the publisher instance imu_humidity_pub. This function is called each time 
*		a "/humidity" message is received.
* @parameter
*		- ngimuHumidity			the NgimuHumidity instance declared in NgimuReceive.h
*/
void ngimuHumidityCallback(const NgimuHumidity ngimuHumidity){
   std::cout << "/humidity, ";
   std::cout << ngimuHumidity.humidity << std::endl;

   fprintf(f, "/humidity, ");
   fprintf(f, "%E, \n", ngimuHumidity.humidity);
   /*
   ngimu::NGIMUHumidity humidity;
   humidity.humidity = ngimuHumidity.humidity;
   imu_humidity_pub.publish(humidity);
   */
}

