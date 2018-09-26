#ifndef NGIMU_RECEIVE_H
#define NGIMU_RECEIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Osc99.h"
#include <stdio.h>

typedef struct {
	OscTimeTag timestamp;
	float gyroscopeX;
	float gyroscopeY;
	float gyroscopeZ;
	float accelerometerX;
	float accelerometerY;
	float accelerometerZ;
	float magnetometerX;
	float magnetometerY;
	float magnetometerZ;
	float barometer;
} NgimuSensors;

typedef struct {
	OscTimeTag timestamp;
	float gyroscopeMagnitude;
	float accelerometerMagnitude;
	float magnetometerMagnitude;
} NgimuMagnitudes;

typedef struct {
	OscTimeTag timestamp;
	float w;
	float x;
	float y;
	float z;
} NgimuQuaternion;

typedef struct {
	OscTimeTag timestamp;
	float roll;
	float pitch;
	float yaw;
} NgimuEuler;

typedef struct {
	OscTimeTag timestamp;
	float xx;
	float xy;
	float xz;
	float yx;
	float yy;
	float yz;
	float zx;
	float zy;
	float zz;
} NgimuRotationMatrix;

typedef struct {
	OscTimeTag timestamp;
	float x;
	float y;
	float z;
} NgimuLinearAcceleration;

typedef struct {
	OscTimeTag timestamp;
	float x;
	float y;
	float z;
} NgimuEarthAcceleration;

typedef struct {
	OscTimeTag timestamp;
	float altitude;
} NgimuAltitude;

typedef struct {
	OscTimeTag timestamp;
	float processor_Temp;
	float gyro_accel_Temp;
	//float barometer_Temp;
} NgimuTemperature;

typedef struct {
	OscTimeTag timestamp;
	float humidity;
} NgimuHumidity;

typedef struct {
	OscTimeTag timestamp;
	char message1[1024];
} NgimuAuxserial;

typedef struct {
	OscTimeTag timestamp;
	float batterylevel;
	float timetoempty;
	float voltage;
	float current;
	char state[32];
} NgimuBattery;

FILE *f;
void NgimuReceiveInitialise();
void NgimuReceiveSetReceiveErrorCallback(void (*newReceiveErrorCallback)(const char* const errorMessage));
void NgimuReceiveSetSensorsCallback(void (*newSensorsCallback)(const NgimuSensors ngimuSensors));
void NgimuReceiveSetQuaternionCallback(void (*newQuaternionCallback)(const NgimuQuaternion ngimuQuaternion));
void NgimuReceiveSetEulerCallback(void (*newEulerCallback)(const NgimuEuler ngimuEuler));
void NgimuReceiveSetRotationMatrixCallback(void (*newRotationMatrixCallback)(const NgimuRotationMatrix ngimuRotationMatrix));
void NgimuReceiveSetLinearAccelerationCallback(void (*newLinearAccelerationCallback)(const NgimuLinearAcceleration ngimuLinearAcceleration));
void NgimuReceiveSetEarthAccelerationCallback(void (*newEarthAccelerationCallback)(const NgimuEarthAcceleration ngimuEarthAcceleration));
void NgimuReceiveSetMagnitudesCallback(void (*newMagnitudesCallback)(const NgimuMagnitudes ngimuMagnitudes));
void NgimuReceiveSetAltitudeCallback(void (*newAltitudeCallback)(const NgimuAltitude ngimuAltitude));
void NgimuReceiveSetTemperatureCallback(void (*newTemperatureCallback)(const NgimuTemperature ngimuTemperature));
void NgimuReceiveSetHumidityCallback(void (*newHumidityCallback)(const NgimuHumidity ngimuHumidity));
void NgimuReceiveSetAuxserialCallback(void (*newAuxserialCallback)(const NgimuAuxserial ngimuAuxserial));
void NgimuReceiveSetBatteryCallback(void (*newBatteryCallback)(const NgimuBattery ngimuBattery));
void NgimuReceiveProcessSerialByte(const char byte);
void NgimuReceiveProcessUdpPacket(const char * const source, const size_t sourceSize);

#ifdef __cplusplus
}
#endif

#endif
