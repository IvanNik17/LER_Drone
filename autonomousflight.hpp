/*! @file flight_control_sample.hpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */




#ifndef DJIOSDK_FLIGHTCONTROL_HPP
#define DJIOSDK_FLIGHTCONTROL_HPP

#define DEFAULT_READ_DELAY 25
#define HOKUYO_BUFFER_SIZE 40960
#define HOKUYO_FIRST_MEASUREMENT_POINT 0
#define HOKUYO_MIDLE_MEASUREMENT_POINT 540
#define HOKUYO_LAST_MEASUREMENT_POINT 1079

// System Includes
#include <cmath>
#include <time.h>
#include <iostream>
#include <unistd.h>
#include  <algorithm>
#include  <iterator>
#include  <sstream>
#include  <string>
#include  <iomanip>


#include <stdio.h>


#include <vector>
#include <complex> 
#include <numeric>
#include <tuple>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <Eigen/Dense>

// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>



#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252

#ifdef __linux
typedef int HANDLE;
#include <unistd.h>
#include <string.h>
#define READ_DELAY() usleep(readDelayMs * 1000)
#else
#include <Windows.h>
#define READ_DELAY() Sleep(readDelayMs)
#endif


#define SIMPLE_COMMAND(x) WriteToSerialPort(buffer,x);\
	READ_DELAY();\
	int bytesRead;\
	bytesRead = ReadFromSerialPort(buffer);\
	buffer[bytesRead] = '\0';\
	memcpy(response,buffer,bytesRead+1);\
	if(bytesRead > 0)\
		return true;\
	else\
		return false;\

using namespace std;

//!@note: All the default timeout parameters are for acknowledgement packets
//! from the aircraft.

/*! Monitored Takeoff
    This implementation of takeoff  with monitoring makes sure your aircraft
    actually took off and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
bool monitoredTakeoff(DJI::OSDK::Vehicle* vehiclePtr, int timeout = 1);

// Examples of commonly used Flight Mode APIs

/*! Position Control. Allows you to set an offset from your current
    location. The aircraft will move to that position and stay there.
    Typical use would be as a building block in an outer loop that does not
    require many fast changes, perhaps a few-waypoint trajectory. For smoother
    transition and response you should convert your trajectory to attitude
    setpoints and use attitude control or convert to velocity setpoints
    and use velocity control.
!*/
bool moveByPositionOffset(DJI::OSDK::Vehicle *vehicle, float xOffsetDesired,
                          float yOffsetDesired, float zOffsetDesired,
                          float yawDesired, float posThresholdInM = 0.2,
                          float yawThresholdInDeg = 1.0);


bool moveByLocalPositionOffset(DJI::OSDK::Vehicle *vehicle, float xOffsetDesired,
                          float yOffsetDesired, float zOffsetDesired,
                          float yawDesired, float posThresholdInM = 0.2,
                          float yawThresholdInDeg = 1.0);


bool moveByAngles(DJI::OSDK::Vehicle *vehicle, float  rollAng,
                          float pitchAng, float yawAngle,
                          float heightSet);

bool initializeSubscriptions(DJI::OSDK::Vehicle *vehicle);

float* moveToPosition(DJI::OSDK::Vehicle *vehicle, float* currentPos, float* desiredPos);

//int moveToPositionLoop(DJI::OSDK::Vehicle *vehicle, float* desiredPos);

float* getCurrentPosition(DJI::OSDK::Vehicle *vehicle, float* originGPS);

float* getCurrentAngles(DJI::OSDK::Vehicle *vehicle);

float getAltitude(DJI::OSDK::Vehicle *vehicle);



/*! Monitored Landing (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.

!*/
bool monitoredLanding(DJI::OSDK::Vehicle* vehiclePtr, int timeout = 1);

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
 * coordinates.
 *
 * Accurate when distances are small.
!*/
void localOffsetFromGpsOffset(DJI::OSDK::Vehicle*             vehicle,
                              DJI::OSDK::Telemetry::Vector3f& deltaNed,
                              void* target, void* origin);

DJI::OSDK::Telemetry::Vector3f toEulerAngle(void* quaternionData);


enum class HokuyoEncoding {TWO_CHARACTER, THREE_CHARACTER};



class Hokuyo
{
public:
	HANDLE serialPortDescriptor;
	//function prototypes
	Hokuyo();
	Hokuyo(HANDLE portDescriptor);
	~Hokuyo();
	bool GetData(HokuyoEncoding encoding, unsigned short startingStep,
		unsigned short endStep, unsigned short clusterCount,unsigned short scanInterval, 
		unsigned short nScans, unsigned char* response); //A.K.A "MDMS-Command"
	bool SwitchLaserOn(unsigned char* response); //A.K.A "BM-Command"
	bool SwitchLaserOff(unsigned char* response); //A.K.A "QT-Command"
	bool Reset(unsigned char* response); //A.K.A "RS-Command"
	bool GetVersionDetails(unsigned char* response); //A.K.A "VV-Command"
	bool GetSpecs(unsigned char* response); //A.K.A "PP-Command"
	bool GetRunningState(unsigned char* response); //A.K.A "II-Command"
	


private:
	int readDelayMs;
	bool WriteToSerialPort(unsigned char* buffer, int bytesToWrite);
	int ReadFromSerialPort(unsigned char* buffer);
	void NumberToChar(unsigned short number, int nCharacters,unsigned char string[]);

};

int OpenSerialPort(const char* device);

void scanningPattern_rotateToAngle(vector<float>& x_pos, vector<float>& y_pos, float xC, float yC, float rotationAngle);
void scanningPattern_horiz(vector<float>& x_vec, vector<float>& y_vec, vector<float>& z_vec, float xC, float yC, float radius, float height, float rotAngle, float startAngle, float endAngle, float deltaPoints);
float calculatePCA_rotAngle(vector<double>& x_pos, vector<double>& y_pos);
void lidarData_thresholdByDistance(vector<int>& angles, vector<int>& distances, float minDist, float maxDist);
tuple<double, double> calculateMeans(vector<int>& angles, vector<int>& distances);
void lidar_PolarToCart(double centerX, double centerY, vector<int>& angles, vector<int>& distances, vector<double>& posX, vector<double>& posY);
tuple<double, double, double, int, int> getPathCenterFromMaxDist(vector<double>& x_pos, vector<double>& y_pos);

class LaserData
{
public:
	int firstStep;
	int lastStep;
	HokuyoEncoding encoding;
	int data[HOKUYO_LAST_MEASUREMENT_POINT];

	LaserData();
	bool SetData(unsigned char* buffer);
	int GetReadingsCount();

private:
	bool CharToNumber(unsigned char* string,int nDigits,int* result);
	


	
};

#endif // DJIOSDK_FLIGHTCONTROL_HPP
