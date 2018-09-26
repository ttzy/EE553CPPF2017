project("IMU")

include_directories("${PROJECT_SOURCE_DIR}/header")


add_executable(readIMU ${PROJECT_SOURCE_DIR}/src/ngimurun.cpp 
		${PROJECT_SOURCE_DIR}/src/OscAddress.c 
		${PROJECT_SOURCE_DIR}/src/ngimureceive.c 
		${PROJECT_SOURCE_DIR}/src/OscBundle.c 
		${PROJECT_SOURCE_DIR}/src/OscCommon.c 
		${PROJECT_SOURCE_DIR}/src/OscError.c 
		${PROJECT_SOURCE_DIR}/src/OscMessage.c 
		${PROJECT_SOURCE_DIR}/src/OscPacket.c 
		${PROJECT_SOURCE_DIR}/src/OscSlip.c)
