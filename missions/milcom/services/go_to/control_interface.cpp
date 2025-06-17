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
 * PARROT COMPANY BE LIABLE FOR ANY DIRECIf you're using the arsdkgen tool (part of AirSDK or Olympe), you can regenerate the IDs from XML or .proto definitionsADVISED OF THE POSSIBILITY OF
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
#include <cstring>
#include <arpa/inet.h>
#define status_wait_time 5
#define waypoint_wait_time 5

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
      mFirstTimeHovering(false), droneSocket(-1), connected(false)
{
}

bool ControlInterface::connectToHost(const std::string& ip, int port) {
    if (connected) return true;

    droneSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (droneSocket < 0) {
        ULOGE("Socket creation failed");
        return false;
    }
	//setup ports and address
    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0) {
        ULOGE("Invalid address");
        close(droneSocket);
        return false;
    }

    if (connect(droneSocket, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        ULOGE("Connection failed");
        close(droneSocket);
        return false;
    }

    connected = true;
    return true;
}

bool ControlInterface::sendToHost(const std::string& message, std::string& response) {

	//debug file
	std::ofstream debugLog("/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/socket_C++.txt", std::ios::app);
    debugLog << "[SEND] " << message << std::endl;
	//ceck to see if it connects
    if (!connected && !connectToHost())
        return false;
	//send the message
    std::string full_message = message + "\n";
    if (send(droneSocket, full_message.c_str(), full_message.length(), 0) < 0) {
        ULOGE("Send failed");
        connected = false;
        return false;
    }
	//receive the response
    char buffer[1024] = {0};
    int valread = recv(droneSocket, buffer, sizeof(buffer) - 1, 0);
    if (valread <= 0) {
        ULOGE("Receive failed");
        connected = false;
        return false;
    }
	//debug file
    buffer[valread] = '\0';
    response = std::string(buffer);
	debugLog << "[RECV] " << response << std::endl;
    return true;
}

void ControlInterface::closeSocket() {
	//close socket after the program is done
    if (connected) {
        close(droneSocket);
        connected = false;
    }
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
		
	 if (!connectToHost()) {
        ULOGE("Failed to connect to Python socket server.");
        return -1;
    }

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
	closeSocket();
	return 0;
}

void ControlInterface::generateRelativeTrajectory()
{
	trajectory relativeTraj;

	// // Receive data from the server
	// char buffer[1024];
	// ssize_t recv_size = recv(sock, buffer, sizeof(buffer), 0);
	// std::ofstream SocketFile("/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/sockettext.txt");
	// if (recv_size == -1) {
	// 	SocketFile << "Failed to receive data!\n";
	// 	// std::cerr << "Failed to receive data!" << std::endl;
	// 	// close(sock);
	// 	SocketFile.close();
	// }
	// else{
	// 	buffer[recv_size] = '\0';  // Null-terminate the received data
	// 	SocketFile << buffer ;
	// 	SocketFile.close();
	// }
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
			closeSocket();
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
		//         ULOG_ERRNO("arsdk_cmd_dec_Aardrone3_PilotingEvent_MoveToEnd",
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

            std::string status;
            if (sendToHost("GET_STATUS", status)) {
				//trim whitespace and newlines
                status.erase(std::remove_if(status.begin(), status.end(), ::isspace), status.end());

				//when all waypoints are met, land the drone
				if (status == "DONE"){
					cmdLand();
					return 0;
				}

                if (status == "FLYING") {
                    std::string string_waypoint;
                    if (sendToHost("GET_WAYPOINT", string_waypoint)) {
                        std::stringstream ss(string_waypoint);

						//debug waypoints + socket in a file
                        //std::ofstream waypointFile("/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/waypoint_socket.txt", std::ios::app);
                        //waypointFile << string_waypoint << "\n";
                        //waypointFile << status << "\n";
                        //waypointFile.close();
						
						//parse waypoints into float vars
                        float headingRotation, dx, dy, dz;
                        char comma;
                        if (ss >> headingRotation >> comma >> dx >> comma >> dy >> comma >> dz) {
                            std::cerr << "Parsed successfully!\n";
                        } else {
                            std::cerr << "Error: Unable to parse line!\n";
                        }
						//6ft for max
						//1.8m in each direction
						//dz is up and dow
						//dx is forward and backwards
						//dy is right and left
                        if (headingRotation >= 0.0 && headingRotation < 6.28 &&
                            dx >= -2 && dx <= 2 &&
                            dy >= -2 && dy <= 2 &&
                            dz >= -2 && dz <= 2) {
                            cmdMoveBy({.dx = dx, .dy = dy, .dz = dz}, headingRotation);
                            std::string dummy;
                            sendToHost("WAYPOINT_INCREMENTED", dummy);
                        } else {
                            sleep(5);
                            cmdMoveBy({.dx = 0, .dy = 0, .dz = 0}, 0);
                        }
                        std::string dummy;
                        sendToHost("SET_STATUS:HOVERING", dummy);
                    }
                } else {
                    sleep(status_wait_time);
                    cmdMoveBy({.dx = 0, .dy = 0, .dz = 0}, 0);
                }
            }
        }
        break;

    case ARSDK_ID_RTH_STATE:
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