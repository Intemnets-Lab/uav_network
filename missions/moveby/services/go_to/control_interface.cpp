/**
 * Copyright (c) 2023 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of the Parrot Company nor the names
 *   of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written
 *   permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * PARROT COMPANY BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#define ULOG_TAG reco_ctrl_ifc
#include <ulog.hpp>
ULOG_DECLARE_TAG(ULOG_TAG);

#include <math.h>
#include <msghub_utils.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <sstream>
#define status_wait_time 5
#define waypoint_wait_time 5
#define WAYCHECK "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.move/waypointcheck.txt"
#define STATUS "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.move/statusfile.txt"
#define WAYPOINT "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.move/waypointfile.txt"
#define CONFIRM "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.move/confirmheader.txt"
#define STATUSCONFIRM1 "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.move/statusconfirm1.txt"
#define STATUSCONFIRM2 "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.move/statusconfirm2.txt"

#include "control_interface.hpp"

static void onConnected(bool success, void *userdata)
{
	ULOGN("ControlInterface is connected : %s",
	      success ? "succeeded" : "failed");
}

static void onDisconnected(bool success, void *userdata)
{
	ULOGN("ControlInterface is Disconnected : %s",
	      success ? "succeeded" : "failed");
}

static void onSent(airsdk::control::ControlInterface *controlInterface,
		   const arsdk_cmd *cmd,
		   bool success,
		   void *userdata)
{
	char buf[128];
	// Format the commands the mission sends to ease their printing
	arsdk_cmd_fmt(cmd, buf, sizeof(buf));
	ULOGI("ControlInterface cmd %s has been sent", buf);
}

static void onReceived(airsdk::control::ControlInterface *controlInterface,
		       const arsdk_cmd *cmd,
		       void *userdata)
{
	// Send commands to the control interface switch case, that will react
	// to the proper events and perform the appropriate moves
	auto self = static_cast<ControlInterface *>(userdata);
	self->onCmdReceived(cmd);
}

ControlInterface::ControlInterface(pomp::Loop &loop,
				   MissionConfiguration *config)
	: mControlItf(loop), mMissionConfiguration(config), mMoveIndex(0),
	  mFirstTimeHovering(false)
{
}

int ControlInterface::start()
{
	// Set up a listener to trigger commands sending and receiving
	const airsdk::control::Listener<airsdk::control::ControlInterface>
		listener_cb = {
			.connected_cb = onConnected,
			.disconnected_cb = onDisconnected,
			.sent_cb = onSent,
			.received_cb = onReceived,
			.userdata = this,
		};
	// Fill in the trajectory plan
	generateRelativeTrajectory();
	// Control connection
	return mControlItf.connect(listener_cb);
}

int ControlInterface::cmdMoveTo(absoluteMove target,
				orientation_mode orientMode,
				float heading,
				float maxHorSpeed,
				float maxVertSpeed,
				float maxYawSpeed)
{
	arsdk_cmd cmdMoveTo;
	arsdk_cmd_init(&cmdMoveTo);
	ULOGN("#SM MOVE TO altitude:%lf lat:%lf long:%lf",
	      target.altitude,
	      target.latitude,
	      target.longitude);
	ULOGN("Config velocities maxGorSpeed:%f maxVertSpeed:%f "
	      "maxYawSpeed:%f",
	      maxHorSpeed,
	      maxVertSpeed,
	      maxYawSpeed);
	arsdk_cmd_enc_Move_Extended_move_to(
		&cmdMoveTo,
		target.latitude,  // latitude of the location [degrees] to reach
		target.longitude, //  longitude of the location [degrees] to
				  //  reach
		target.altitude,  // altitude above take off point [m] to reach
		orientMode,       // orientation mode
		heading,      // heading (relative to the North [degrees]). This
			      // value is only used if the orientation mode is
			      // HEADING_START or HEADING_DURING
		maxHorSpeed,  // maximum horizontal speed [m/s]
		maxVertSpeed, // maximum vertical speed [m/s]
		maxYawSpeed); // maximum yaw rotation speed [degrees/s]
	mControlItf.send(&cmdMoveTo);
	arsdk_cmd_clear(&cmdMoveTo);
	return 0;
}

int ControlInterface::cmdMoveBy(relativeMove target,
				float headingRotation,
				float maxHorSpeed,
				float maxVertSpeed,
				float maxYawSpeed)
{
	arsdk_cmd cmdMoveBy;
	arsdk_cmd_init(&cmdMoveBy);
	ULOGN("#SM MOVE BY x:%f y:%lf z:%f", target.dx, target.dy, target.dz);
	ULOGN("Config velocities maxHorSpeed:%f maxVertSpeed:%f "
	      "maxYawSpeed:%f",
	      maxHorSpeed,
	      maxVertSpeed,
	      maxYawSpeed);
	arsdk_cmd_enc_Move_Extended_move_by(
		&cmdMoveBy,
		target.dx,       // wanted displacement along the front axis [m]
		target.dy,       // wanted displacement along the right axis [m]
		target.dz,       // wanted displacement along the down axis [m]
		headingRotation, // wanted rotation of heading [rad]
		maxHorSpeed,     // maximum horizontal speed [m/s]
		maxVertSpeed,    // maximum vertical speed [m/s]
		maxYawSpeed);    // maximum yaw rotation speed [degrees/s]
	mControlItf.send(&cmdMoveBy);
	arsdk_cmd_clear(&cmdMoveBy);
	return 0;
}

int ControlInterface::cmdRTH()
{
	arsdk_cmd cmdRTH;
	arsdk_cmd_init(&cmdRTH);
	arsdk_cmd_enc_Rth_Return_to_home(&cmdRTH);
	mControlItf.send(&cmdRTH);
	arsdk_cmd_clear(&cmdRTH);
	return 0;
}

int ControlInterface::cmdLand()
{
	arsdk_cmd cmdLand;
	arsdk_cmd_init(&cmdLand);
	arsdk_cmd_enc_Ardrone3_Piloting_Landing(&cmdLand);
	mControlItf.send(&cmdLand);
	arsdk_cmd_clear(&cmdLand);
	return 0;
}

void ControlInterface::generateRelativeTrajectory()
{
	trajectory relativeTraj;
	std::ifstream MyFile("/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.move/statusfile.txt");
	std::string myline="+";

	getline (MyFile, myline);
	MyFile.close();
	std::ofstream thisFile("/mnt/user-internal/missions-data/com.parrot.missions.samples.move/writetestafter.txt");
	thisFile << myline << std::endl;
	thisFile.close();
	// Relative move of (5,0,0) with no rotation heading
	relativeTraj.relTarget = {.dx = 0, .dy = 0, .dz = 0};
	relativeTraj.heading = 0.0;
	this->mRelativeTrajectory.push_back(relativeTraj);


}

void ControlInterface::eventInfo(uint32_t missing_inputs)
{
	// In case you need to get the inputs the drone misses to perform
	// correctly
	arsdk_cmd eventInfo;
	arsdk_cmd_init(&eventInfo);
	arsdk_cmd_enc_Move_Info(&eventInfo, missing_inputs);
	mControlItf.send(&eventInfo);
	arsdk_cmd_clear(&eventInfo);
}

int ControlInterface::onCmdReceived(const arsdk_cmd *cmd)
{
	int32_t state = 0;
	int res = 0;
	// Compare requested positions with real positions
	trajectory realTrajectory;

	switch (cmd->id) {
	case ARSDK_ID_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED:
		// In case of flying state changed, we need to distinguish the
		// case where the drone lands and the one where the drone hovers
		// for the first time
		res = arsdk_cmd_dec_Ardrone3_PilotingState_FlyingStateChanged(
			cmd, &state);
		if (res != 0) {
			ULOG_ERRNO(
				"arsdk_cmd_dec_Ardrone3_PilotingState_FlyingStateChanged",
				-res);
			return res;
		}
		// If the drone lands, we reset everything so that the mission
		// can start again if the drone takes off once again
		if (state
		    == ARSDK_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED) {
			this->mMoveIndex = 0;
			mFirstTimeHovering = false;
			// If the drone quits hovering for the first time, we
			// launch the flightplan
		} else if (
			state == ARSDK_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING
			&& !mFirstTimeHovering) {
			cmdMoveBy(
				this->mRelativeTrajectory[mMoveIndex].relTarget,
				this->mRelativeTrajectory[mMoveIndex].heading);
			this->mMoveIndex++;
			mFirstTimeHovering = true;
		}
		break;
		// /!\ BECAREFUL with MoveTo moves : as coordinates are
		// absolute, you need to carefully make sure the targets you
		// request fit with appropriate absolute GPS position, so that
		// your drone won't fly hours to get to the other side of the
		// world
		// case ARSDK_ID_ARDRONE3_PILOTINGEVENT_MOVETOEND:
		//     // In case of a MoveTo, we retrieve the actual position
		//     the drone has
		//     // moved to, then launch the next move registered into
		//     the proper vector res =
		//     arsdk_cmd_dec_Ardrone3_PilotingEvent_MoveToEnd(
		//         cmd, &realTrajectory.abs_target.latitude,
		//         &realTrajectory.abs_target.longitude,
		//         &realTrajectory.abs_target.altitude,
		//         &realTrajectory.heading, &state);
		//     if (res != 0) {
		//         ULOG_ERRNO("arsdk_cmd_dec_Ardrone3_PilotingEvent_MoveToEnd",
		//         -res); goto error;
		//     }
		//     ULOGI("Real Trajectory executed : %f latitude (degrees),
		//     %f longitude
		//     "
		//           "(degrees), %f altitude (m), %f "
		//           "heading (rad) ; state %d",
		//           realTrajectory.abs_target.latitude,
		//           realTrajectory.abs_target.longitude,
		//           realTrajectory.abs_target.altitude,
		//           realTrajectory.heading, state);
		//     if (state ==
		//     ARSDK_ARDRONE3_PILOTINGEVENT_MOVETOEND_ERROR_OK &&
		//         this->mMoveIndex < absolute_trajectory.size()) {
		//         cmdMoveTo(this->absolute_trajectory[mMoveIndex].abs_target,
		//                   this->absolute_trajectory[mMoveIndex].heading);
		//         this->mMoveIndex++;
		//     }
		//     break;
	case ARSDK_ID_ARDRONE3_PILOTINGEVENT_MOVEBYEND:
		// In case of a MoveBy, we retrieve the actual position the
		// drone has moved by, then launch the next move registered into
		// the proper vector
		res = arsdk_cmd_dec_Ardrone3_PilotingEvent_MoveByEnd(
			cmd,
			&realTrajectory.relTarget.dx,
			&realTrajectory.relTarget.dy,
			&realTrajectory.relTarget.dz,
			&realTrajectory.heading,
			&state);
		if (res != 0) {
			ULOG_ERRNO(
				"arsdk_cmd_dec_Ardrone3_PilotingEvent_MoveByEnd",
				-res);
			return res;
		}
		ULOGI("Real Trajectory executed : %f dx (m), %f dy (m), %f dz (m), %f "
		      "heading (rad) ; state %d",
		      realTrajectory.relTarget.dx,
		      realTrajectory.relTarget.dy,
		      realTrajectory.relTarget.dz,
		      realTrajectory.heading,
		      state);
		if (state == ARSDK_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR_OK) {
			//const int wait_time = 5;
			std::ifstream statusfile;
			//The go_to service tries to open the status file. Use a while loop to constantly try to open the file.
			while (1){
				statusfile.open(STATUS);
				if(!statusfile.is_open()){

					sleep(status_wait_time);
				}
				else{
					break;// breaks the while loop if there was no issue opening the file 
				}
			}
			std::ofstream MyFile(STATUSCONFIRM1);
    		MyFile << "was able to get the status value first time1\n";// File to confirm to have the status value
			std::string stringstatus="";
			getline(statusfile,stringstatus);
			MyFile << stringstatus;
			MyFile.close();
			int status = std::stoi(stringstatus);
			statusfile.close();
			//check the current state of status, if it's 0 then the service attempts to open the heading file
			if(status==0){
				std::ofstream MyFile(CONFIRM);
    			MyFile << "was able to confirm the waypoint value\n";
    			MyFile.close();
				std::ifstream headingfile;
				//The service uses a  while loop to make sure the waypoint file is properly opened. Then read and use the waypoint value.
				while(1){
					headingfile.open(WAYPOINT);
					if(!headingfile.is_open()){
						sleep(waypoint_wait_time);
					}
					else{
						break;
					}
				}
				std::string headingstring="";
				float value1,value2,value3,value4;
				//The service reads the waypoint value and breaks it down into 4 variables
				while(getline (headingfile, headingstring)){
					std::stringstream ss(headingstring);
					char comma;
					if(ss >> value1 >> comma >> value2 >> comma >> value3 >> comma >> value4){
						std::cerr << "able to get a line!";
					}else{
						std::cerr << "Error: Unable to parse line!";
					}
				}

				headingfile.close();
				// change status file
				while(1){
					std::ofstream changestatus(STATUS);
					if(!changestatus.is_open()){
						sleep(status_wait_time);
					}
					else{
						changestatus << "1";
						changestatus.close();
						break;
					}
				}
				// write the waypoint values to a check to check and confirm they are as expected.
				std::ofstream WayFile(WAYCHECK);
				WayFile << value1 << ", " << value2 << ", "<< value3 << ", " << value4 << "\n";
				WayFile.close();
				// confirm the the status was changed 
				std::ofstream openFile(STATUSCONFIRM2);
    			openFile << "was able to change status value time\n";
    			openFile.close();
				//check the value of waypoint and run the corresponding moveby command.
				if(value1>=0.0 && value1<6.28){
					cmdMoveBy({.dx = 0, .dy = 0, .dz = 0},value1);
				}
				else{
					//if the value is out of bounds, run a move command with 0
					sleep(5);
					cmdMoveBy({.dx = 0, .dy = 0, .dz = 0},0);
				}
			}
			else{
				//if status is 1 we wait until status is 0. In the mean while cmdMoveBy() will run with a zero value.
				std::ofstream MyFile(CONFIRM);
    			MyFile << "was not able to get the header value";
    			MyFile.close();
				sleep(waypoint_wait_time);
				cmdMoveBy({.dx = 0, .dy = 0, .dz = 0},0);
			}
				
		}
		break;
	case ARSDK_ID_RTH_STATE:
		int32_t state;
		int32_t reason;
		res = arsdk_cmd_dec_Rth_State(cmd, &state, &reason);
		if (res != 0) {
			ULOG_ERRNO("arsdk_cmd_dec_Rth_State", -res);
			return res;
		}
		if (reason == ARSDK_RTH_STATE_REASON_FINISHED)
			cmdLand();

		break;
	default:
		break;
	}
	return 0;
}