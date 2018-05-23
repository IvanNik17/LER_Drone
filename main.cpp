/*! @file flight-control/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
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

#include "autonomousflight.hpp"
#include <zmq.hpp>



using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using namespace std;




/*! main
 *
 */
int
main(int argc, char** argv)
{
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);


  //  Prepare our context and socket
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_REQ);
    socket.connect("tcp://localhost:5555");
    
    bool returnBool = false;
    float sensorOutputs[2];
    stringstream stream;

while(true){
	 string msgToClient("M_startMove");
    	zmq::message_t reply(msgToClient.size());
    	memcpy((void *) reply.data(), (msgToClient.c_str()), msgToClient.size());
    	socket.send(reply);

    	sleep(0.1);
	

	zmq::message_t request_start;
    	socket.recv(&request_start);
        string replyMessage_start = string(static_cast<char *>(request_start.data()), request_start.size());
     // Print out received message
        cout << "Received from server: " + replyMessage_start << endl;

        if (replyMessage_start == "start")
	{
		cout << "Starting: "<< endl;
		break;

	}
	else{
		cout << "Waiting: "<< endl;
	}
   }



    // forever loop
    while (true) {
    string msgToClient("M_currDist");
    zmq::message_t reply(msgToClient.size());
    memcpy((void *) reply.data(), (msgToClient.c_str()), msgToClient.size());
    socket.send(reply);

    sleep(0.2);

     zmq::message_t request;
    socket.recv(&request);
    string replyMessage = string(static_cast<char *>(request.data()), request.size());
	
	if (replyMessage == "break")
	{
		 cout << "Breaking"<< endl;
		vehicle->control->emergencyBrake();
		monitoredLanding(vehicle);
		break;
	}
	/*else if (replyMessage == "land")
	{
		 cout << "Landing"<< endl;
		monitoredLanding(vehicle);
	}
	else if (replyMessage == "takeoff")
	{
		 cout << " Taking  off "  << endl;
		monitoredTakeoff(vehicle);
	}
        */
	else if (replyMessage == "stop")
	{
		vehicle->control->emergencyBrake();
	}
	else if (replyMessage == "forward")
	{
		vehicle->control->positionAndYawCtrl(1, 0, 2,0);
	}
	else if (replyMessage == "backward")
	{
		vehicle->control->positionAndYawCtrl(-1, 0, 2,0);
	}

	 cout << "Received from server: " + replyMessage << endl;
	
    }

  // monitoredTakeoff(vehicle);
//initializeSubscriptions(vehicle);
//returnBool = moveByLocalPositionOffset(vehicle, 3, 3, 3, 45);
//std::cout << "Answer          = " << returnBool << "\n";

// vehicle->subscribe->removePackage(0, 1);

  

  return 0;
}
