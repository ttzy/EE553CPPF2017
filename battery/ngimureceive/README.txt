1. Compile the file using cmake:

At the build directory, type the following commands:

make

sudo ./readIMU



2. Compile the code manually using the following steps:

Put all the header files and source files in the same directory

Check the tty port number on your local device

Go to ngimurun.cpp line 141 check if the portName is the same as what the IMU is currently connecting to.

Open the terminal and compile the files by running the command:

At the file directory:
Run:
gcc ngimureceive.c OscBundle.c OscError.c OscMessage.c OscPacket.c OscSlip.c OscAddress.c OscCommon.c ngimurun.cpp -lstdc++ -o readIMU
sudo./readIMU

The data file named IMUdata will be stored in the same directory.



