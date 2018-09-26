/**
 * @file NgimuReceive.c
 * @author Seb Madgwick
 * @brief Module for receiving messages from an NGIMU.
 */

//------------------------------------------------------------------------------
// Includes

#include "ngimureceive.h"
#include <stddef.h>
#include <stdio.h> // snprintf

//------------------------------------------------------------------------------
// Variable declarations

static OscSlipDecoder oscSlipDecoder;
static void (*receiveErrorCallback)(const char* const errorMessage);
static void (*sensorsCallback)(const NgimuSensors ngimuSensors);
static void (*quaternionCallback)(const NgimuQuaternion ngimuQuaternion);
static void (*eulerCallback)(const NgimuEuler ngimuEuler);
static void (*auxserialCallback)(const NgimuAuxserial ngimuAuxserial);
static void (*batteryCallback)(const NgimuBattery ngimuBattery);

//**************************** CHANG LEE ****************************//
static void (*rotationMatrixCallback)(const NgimuRotationMatrix ngimuRotationMatrix);
static void (*linearAccelerationCallback)(const NgimuLinearAcceleration ngimuLinearAcceleration);
static void (*earthAccelerationCallback)(const NgimuEarthAcceleration ngimuEarthAcceleration);
static void (*magnitudesCallback)(const NgimuMagnitudes ngimuMagnitudes);
static void (*altitudeCallback)(const NgimuAltitude ngimuAltitude);
static void (*temperatureCallback)(const NgimuTemperature ngimuTemperature);
static void (*humidityCallback)(const NgimuHumidity ngimuHumidity);
//**************************** CHANG LEE ****************************//


//------------------------------------------------------------------------------
// Function prototypes

static void ProcessPacket(OscPacket * const oscMessage);
static void ProcessMessage(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);
static OscError ProcessAddress(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);
static OscError ProcessSensors(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);
static OscError ProcessQuaternion(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);
static OscError ProcessEuler(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);

//**************************** CHANG LEE ****************************//
static OscError ProcessRotationMatrix(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);
static OscError ProcessLinearAcceleration(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);
static OscError ProcessEarthAcceleration(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);
static OscError ProcessMagnitudes(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);
static OscError ProcessAltitude(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);
static OscError ProcessTemperature(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);
static OscError ProcessHumidity(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);
static OscError ProcessAuxserial(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);
static OscError ProcessBattery(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);
//**************************** CHANG LEE ****************************//

//------------------------------------------------------------------------------
// Functions

/**
 * @brief Initialises module.  This function should be called once on system
 * start up.
 */
void NgimuReceiveInitialise() {
    OscSlipDecoderInitialise(&oscSlipDecoder);
    oscSlipDecoder.processPacket = ProcessPacket;
}

/**
 * @brief Sets receive error callback function.
 * @param newReceiveErrorCallback Receive error callback function.
 */
void NgimuReceiveSetReceiveErrorCallback(void (*newReceiveErrorCallback)(const char* const errorMessage)) {
    receiveErrorCallback = newReceiveErrorCallback;
}

/**
 * @brief Sets receive "/sensors" callback function.
 * @param newSensorsCallback "/sensors" callback function.
 */
void NgimuReceiveSetSensorsCallback(void (*newSensorsCallback)(const NgimuSensors ngimuSensors)) {
    sensorsCallback = newSensorsCallback;
}

/**
 * @brief Sets receive "/quaternion" callback function.
 * @param newQuaternionCallback "/quaternion" callback function.
 */
void NgimuReceiveSetQuaternionCallback(void (*newQuaternionCallback)(const NgimuQuaternion ngimuQuaternion)) {
    quaternionCallback = newQuaternionCallback;
}

/**
 * @brief Sets receive "/euler" callback function.
 * @param newEulerCallback "/euler" callback function.
 */

void NgimuReceiveSetEulerCallback(void (*newEulerCallback)(const NgimuEuler ngimuEuler)) {
    eulerCallback = newEulerCallback;
}

void NgimuReceiveSetAuxserialCallback(void (*newAuxserialCallback)(const NgimuAuxserial ngimuAuxserial)){
    auxserialCallback = newAuxserialCallback;
}

void NgimuReceiveSetBatteryCallback(void (*newBatteryCallback)(const NgimuBattery ngimuBattery)){
    batteryCallback = newBatteryCallback;
}

//**************************** CHANG LEE ****************************//
/**
 * @brief Sets receive "/matrix" callback function.
 * @param newRotationMatrixCallback "/matrix" callback function.
 */
void NgimuReceiveSetRotationMatrixCallback(void (*newRotationMatrixCallback)(const NgimuRotationMatrix ngimuRotationMatrix)) {
    rotationMatrixCallback = newRotationMatrixCallback;
}

/**
 * @brief Sets receive "/linear" callback function.
 * @param newLinearAccelerationCallback "/linear" callback function.
 */
void NgimuReceiveSetLinearAccelerationCallback(void (*newLinearAccelerationCallback)(const NgimuLinearAcceleration ngimuLinearAcceleration)) {
    linearAccelerationCallback = newLinearAccelerationCallback;
}

/**
 * @brief Sets receive "/earth" callback function.
 * @param newEarthAccelerationCallback "/earth" callback function.
 */
void NgimuReceiveSetEarthAccelerationCallback(void (*newEarthAccelerationCallback)(const NgimuEarthAcceleration ngimuEarthAcceleration)) {
    earthAccelerationCallback = newEarthAccelerationCallback;
}

/**
 * @brief Sets receive "/magnitudes" callback function.
 * @param newMagnitudeCallback "/magnitudes" callback function.
 */
void NgimuReceiveSetMagnitudesCallback(void (*newMagnitudesCallback)(const NgimuMagnitudes ngimuMagnitude)){
    magnitudesCallback = newMagnitudesCallback;
}

/**
 * @brief Sets receive "/altitude" callback function.
 * @param newAltitudeCallback "/altitude" callback function.
 */
void NgimuReceiveSetAltitudeCallback(void (*newAltitudeCallback)(const NgimuAltitude ngimuAltitude)){
    altitudeCallback = newAltitudeCallback;
}

/**
 * @brief Sets receive "/temperature" callback function.
 * @param newTemperatureCallback "/temperature" callback function.
 */
void NgimuReceiveSetTemperatureCallback(void (*newTemperatureCallback)(const NgimuTemperature ngimuTemperature)){
    temperatureCallback = newTemperatureCallback;
}

/**
 * @brief Sets receive "/humidity" callback function.
 * @param newHumidityCallback "/humidity" callback function.
 */
void NgimuReceiveSetHumidityCallback(void (*newHumidityCallback)(const NgimuHumidity ngimuHumidity)){
    humidityCallback = newHumidityCallback;
}

//**************************** CHANG LEE ****************************//






/**
 * @brief Process byte received from NGIMU via a serial communication channel.
 * This function should be called for each byte receive within a serial stream.
 * @param byte Serial byte
 */
void NgimuReceiveProcessSerialByte(const char byte) {
    OscSlipDecoderProcessByte(&oscSlipDecoder, byte);
}

/**
 * @brief Process UDP packet received from NGIMU via Wi-Fi.
 * @param source Address of source byte array.
 * @param sourceSize Source size.
 */
void NgimuReceiveProcessUdpPacket(const char * const source, const size_t sourceSize) {
    OscPacket oscPacket;
    OscPacketInitialiseFromCharArray(&oscPacket, source, sourceSize);
    oscPacket.processMessage = &ProcessMessage;
    OscPacketProcessMessages(&oscPacket);
}

/**
 * @brief Callback function executed for each OSC packet received by a SLIP
 * decoder.
 * @param oscPacket Address of the decoded OSC packet.
 */
static void ProcessPacket(OscPacket * const oscPacket) {
    oscPacket->processMessage = &ProcessMessage;
    OscError oscError = OscPacketProcessMessages(oscPacket);
    if ((oscError != OscErrorNone) && (receiveErrorCallback != NULL)) {
        receiveErrorCallback(OscErrorGetMessage(oscError));
    }
}

/**
 * @brief Callback function executed for each message found within received OSC
 * packet.
 * @param oscTimeTag OSC time tag associated with message.
 * @param oscMessage Address of OSC message.
 */ 
static void ProcessMessage(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage) {
    const OscError oscError = ProcessAddress(oscTimeTag, oscMessage);
    if ((oscError != OscErrorNone) && (receiveErrorCallback != NULL)) {
        receiveErrorCallback(OscErrorGetMessage(oscError));
    }
}

/**
 * @brief Process OSC message according to OSC address pattern.
 * @param oscTimeTag OSC time tag associated with message.
 * @param oscMessage Address of OSC message.
 * @return Error code (0 if successful).
 */
static OscError ProcessAddress(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage) {

    // Process known message types
    if (OscAddressMatch(oscMessage->oscAddressPattern, "/sensors")) {
        return ProcessSensors(oscTimeTag, oscMessage);
    }
    if (OscAddressMatch(oscMessage->oscAddressPattern, "/quaternion")) {
        return ProcessQuaternion(oscTimeTag, oscMessage);
    }
    if (OscAddressMatch(oscMessage->oscAddressPattern, "/euler")) {
        return ProcessEuler(oscTimeTag, oscMessage);
    }

    //**************************** CHANG LEE ****************************//
    if (OscAddressMatch(oscMessage->oscAddressPattern, "/matrix")) {
        return ProcessRotationMatrix(oscTimeTag, oscMessage);
    }
    if (OscAddressMatch(oscMessage->oscAddressPattern, "/linear")) {
        return ProcessLinearAcceleration(oscTimeTag, oscMessage);
    }
    if (OscAddressMatch(oscMessage->oscAddressPattern, "/earth")) {
        return ProcessEarthAcceleration(oscTimeTag, oscMessage);
    }
    if (OscAddressMatch(oscMessage->oscAddressPattern, "/magnitudes")) {
        return ProcessMagnitudes(oscTimeTag, oscMessage);
    }
    if (OscAddressMatch(oscMessage->oscAddressPattern, "/altitude")) {
        return ProcessAltitude(oscTimeTag, oscMessage);
    }
    if (OscAddressMatch(oscMessage->oscAddressPattern, "/temperature")) {
        return ProcessTemperature(oscTimeTag, oscMessage);
    }
    if (OscAddressMatch(oscMessage->oscAddressPattern, "/humidity")) {
        return ProcessHumidity(oscTimeTag, oscMessage);
    }
    if (OscAddressMatch(oscMessage->oscAddressPattern, "/auxserial")){
        return ProcessAuxserial(oscTimeTag, oscMessage);
    }
    if (OscAddressMatch(oscMessage->oscAddressPattern, "/battery")){
        return ProcessBattery(oscTimeTag, oscMessage);
    }
    //**************************** CHANG LEE ****************************//
    // OSC address not recognised
    if (receiveErrorCallback != NULL) {
        static char string[256];
        snprintf(string, sizeof (string), "OSC address pattern not recognised: %s", oscMessage->oscAddressPattern);
        receiveErrorCallback(string);
    }
    return OscErrorNone;
}


/**
 * @brief Process "/sensors" message.
 * @param oscTimeTag OSC time tag associated with message.
 * @param oscMessage Address of OSC message.
 * @return Error code (0 if successful).
 */
static OscError ProcessSensors(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage) {

    // Do nothing if no callback assigned
    if (sensorsCallback == NULL) {
        return OscErrorNone;
    }

    // Get timestamp
    NgimuSensors ngimuSensors;
    ngimuSensors.timestamp = *oscTimeTag;

    // Get gyroscope X axis
    OscError oscError;
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuSensors.gyroscopeX);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get gyroscope Y axis
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuSensors.gyroscopeY);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get gyroscope Z axis
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuSensors.gyroscopeZ);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get accelerometer X axis
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuSensors.accelerometerX);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get accelerometer Y axis
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuSensors.accelerometerY);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get accelerometer Z axis
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuSensors.accelerometerZ);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get magnetometer X axis
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuSensors.magnetometerX);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get magnetometer Y axis
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuSensors.magnetometerY);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get magnetometer Z axis
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuSensors.magnetometerZ);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get barometer
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuSensors.barometer);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Callback
    sensorsCallback(ngimuSensors);
    return OscErrorNone;
}

/**
 * @brief Process "/quaternion" message.
 * @param oscTimeTag OSC time tag associated with message.
 * @param oscMessage Address of OSC message.
 * @return Error code (0 if successful).
 */
static OscError ProcessQuaternion(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage) {

    // Do nothing if no callback assigned
    if (quaternionCallback == NULL) {
        return OscErrorNone;
    }

    // Get timestamp
    NgimuQuaternion ngimuQuaternion;
    ngimuQuaternion.timestamp = *oscTimeTag;

    // Get W element
    OscError oscError;
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuQuaternion.w);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get X element
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuQuaternion.x);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get Y element
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuQuaternion.y);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get Z element
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuQuaternion.z);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Callback
    quaternionCallback(ngimuQuaternion);
    return OscErrorNone;
}

/**
 * @brief Process "/euler" message.
 * @param oscTimeTag OSC time tag associated with message.
 * @param oscMessage Address of OSC message.
 * @return Error code (0 if successful).
 */

static OscError ProcessEuler(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage) {

    // Do nothing if no callback assigned
    if (eulerCallback == NULL) {
        return OscErrorNone;
    }

    // Get timestamp
    NgimuEuler ngimuEuler;
    ngimuEuler.timestamp = *oscTimeTag;

    // Get roll
    OscError oscError;
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuEuler.roll);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get pitch
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuEuler.pitch);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get yaw
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuEuler.yaw);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Callback
    eulerCallback(ngimuEuler);
    return OscErrorNone;
}



//**************************** CHANG LEE ****************************//
/**
 * @brief Process "/matrix" message.
 * @param oscTimeTag OSC time tag associated with message.
 * @param oscMessage Address of OSC message.
 * @return Error code (0 if successful).
 */
static OscError ProcessRotationMatrix(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage) {

    // Do nothing if no callback assigned
    if (rotationMatrixCallback == NULL) {
        return OscErrorNone;
    }

    // Get timestamp
    NgimuRotationMatrix ngimuRotationMatrix;
    ngimuRotationMatrix.timestamp = *oscTimeTag;

    // Get xx
    OscError oscError;
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuRotationMatrix.xx);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get xy
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuRotationMatrix.xy);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get xz
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuRotationMatrix.xz);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get yx
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuRotationMatrix.yx);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get yy
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuRotationMatrix.yy);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get yz
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuRotationMatrix.yz);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get zx
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuRotationMatrix.zx);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get zy
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuRotationMatrix.zy);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get zz
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuRotationMatrix.zz);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Callback
    rotationMatrixCallback(ngimuRotationMatrix);
    return OscErrorNone;
}

/**
 * @brief Process "/linear" message.
 * @param oscTimeTag OSC time tag associated with message.
 * @param oscMessage Address of OSC message.
 * @return Error code (0 if successful).
 */
static OscError ProcessLinearAcceleration(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage){
    // Do nothing if no callback assigned
    if (linearAccelerationCallback == NULL) {
        return OscErrorNone;
    }

    // Get timestamp
    NgimuLinearAcceleration ngimuLinearAcceleration;
    ngimuLinearAcceleration.timestamp = *oscTimeTag;

    // Get x
    OscError oscError;
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuLinearAcceleration.x);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get y
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuLinearAcceleration.y);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get z
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuLinearAcceleration.z);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Callback
    linearAccelerationCallback(ngimuLinearAcceleration);
    return OscErrorNone;
}

/**
 * @brief Process "/earth" message.
 * @param oscTimeTag OSC time tag associated with message.
 * @param oscMessage Address of OSC message.
 * @return Error code (0 if successful).
 */
static OscError ProcessEarthAcceleration(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage){
    // Do nothing if no callback assigned
    if (earthAccelerationCallback == NULL) {
        return OscErrorNone;
    }

    // Get timestamp
    NgimuEarthAcceleration ngimuEarthAcceleration;
    ngimuEarthAcceleration.timestamp = *oscTimeTag;

    // Get x
    OscError oscError;
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuEarthAcceleration.x);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get y
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuEarthAcceleration.y);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get z
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuEarthAcceleration.z);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Callback
    earthAccelerationCallback(ngimuEarthAcceleration);
    return OscErrorNone;
}

/**
 * @brief Process "/magnitudes" message.
 * @param oscTimeTag OSC time tag associated with message.
 * @param oscMessage Address of OSC message.
 * @return Error code (0 if successful).
 */
static OscError ProcessMagnitudes(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage){
    // Do nothing if no callback assigned
    if (magnitudesCallback == NULL) {
        return OscErrorNone;
    }

    // Get timestamp
    NgimuMagnitudes ngimuMagnitudes;
    ngimuMagnitudes.timestamp = *oscTimeTag;

    // Get the magnitude of gyroscope 
    OscError oscError;
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuMagnitudes.gyroscopeMagnitude);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get the magnitude of accelerometer 
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuMagnitudes.accelerometerMagnitude);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get the magnitude of magnetometer
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuMagnitudes.magnetometerMagnitude);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Callback
    magnitudesCallback(ngimuMagnitudes);
    return OscErrorNone;
}

/**
 * @brief Process "/altitude" message.
 * @param oscTimeTag OSC time tag associated with message.
 * @param oscMessage Address of OSC message.
 * @return Error code (0 if successful).
 */
static OscError ProcessAltitude(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage){
    // Do nothing if no callback assigned
    if (altitudeCallback == NULL) {
        return OscErrorNone;
    }

    // Get timestamp
    NgimuAltitude ngimuAltitude;
    ngimuAltitude.timestamp = *oscTimeTag;

    // Get altitude
    OscError oscError;
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuAltitude.altitude);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Callback
    altitudeCallback(ngimuAltitude);
    return OscErrorNone;
}

/**
 * @brief Process "/temperature" message.
 * @param oscTimeTag OSC time tag associated with message.
 * @param oscMessage Address of OSC message.
 * @return Error code (0 if successful).
 */
static OscError ProcessTemperature(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage){
    // Do nothing if no callback assigned
    if (temperatureCallback == NULL) {
        return OscErrorNone;
    }

    // Get timestamp
    NgimuTemperature ngimuTemperature;
    ngimuTemperature.timestamp = *oscTimeTag;

    // Get the temperature of processor 
    OscError oscError;
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuTemperature.processor_Temp);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    //Get the temperature of gyroscope/accelerometer 
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuTemperature.gyro_accel_Temp);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Get the temperature of barometer
    /*
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuTemperature.barometer_Temp);
    if (oscError != OscErrorNone) {
        return oscError;
    }
    */
    // Callback
    temperatureCallback(ngimuTemperature);
    return OscErrorNone;
}


/**
 * @brief Process "/humidity" message.
 * @param oscTimeTag OSC time tag associated with message.
 * @param oscMessage Address of OSC message.
 * @return Error code (0 if successful).
 */
static OscError ProcessHumidity(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage){
    // Do nothing if no callback assigned
    if (humidityCallback == NULL) {
        return OscErrorNone;
    }

    // Get timestamp
    NgimuHumidity ngimuHumidity;
    ngimuHumidity.timestamp = *oscTimeTag;

    // Get humidity
    OscError oscError;
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuHumidity.humidity);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    // Callback
    humidityCallback(ngimuHumidity);
    return OscErrorNone;
}

static OscError ProcessAuxserial(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage){
      
    //Get OSC Timestamp
    NgimuAuxserial ngimuAuxserial;
    ngimuAuxserial.timestamp = *oscTimeTag;
    
    //Get auxserial blobs and store them in string buffers
    OscError oscError;
    oscError = OscMessageGetArgumentAsString(oscMessage, ngimuAuxserial.message1, 1024);
    if(oscError != OscErrorNone){
        return oscError;
    }
    
    
    
    //Callback
    auxserialCallback(ngimuAuxserial);
    return OscErrorNone;
    
}

static OscError ProcessBattery(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage){

    //Get OSC Timestamp
    NgimuBattery ngimuBattery;
    ngimuBattery.timestamp = *oscTimeTag;

    //Get battery information
    OscError oscError;
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuBattery.batterylevel);
    if (oscError != OscErrorNone) {
        return oscError;
    }
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuBattery.timetoempty);
    if (oscError != OscErrorNone) {
        return oscError;
    }
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuBattery.voltage);
    if (oscError != OscErrorNone) {
        return oscError;
    }
    oscError = OscMessageGetArgumentAsFloat32(oscMessage, &ngimuBattery.current);
    if (oscError != OscErrorNone) {
        return oscError;
    }
    oscError = OscMessageGetArgumentAsString(oscMessage, ngimuBattery.state, 32);
    if (oscError != OscErrorNone) {
        return oscError;
    }

    //Callback
    batteryCallback(ngimuBattery);
    return OscErrorNone;

}


//**************************** CHANG LEE ****************************//



//------------------------------------------------------------------------------
// End of file
