/****************************************************************************
# FroboMind wheelModule.hpp
# Copyright (c) 2011-2013, author Morten S. Laursen <morten@softwarehuset.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#  	notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#  	notice, this list of conditions and the following disclaimer in the
#  	documentation and/or other materials provided with the distribution.
#	* Neither the name FroboMind nor the
#  	names of its contributors may be used to endorse or promote products
#  	derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/
#ifndef WHEELMODULE_HPP_
#define WHEELMODULE_HPP_

#include <ros/ros.h>
#include <string.h>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <msgs/can.h>
#include <msgs/steering_angle_cmd.h>
#include <msgs/encoder.h>
#include <msgs/nmea.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>

/**
 * \mainpage
 *
 * \ref Node
 *
 * \ref Params
 *
 * \ref Topics
 *
 *
 *
 */

/**
 *
 * @addtogroup Node
 * @{
 * Class for controlling the OmniRota
 *
 * This class controls the wheels of the OmniRota given a commanded linear velocity and
 * a steering wheel angle.
 */
class wheelModule
{
private:
	double 				angle_pos_sp, drive_vel_sp;
	int angle_encoder_offset, angle_encoder_pos;

	bool 				active;
	bool				deadman;

	ros::Time 			last_deadman_received;

	void setVelocity(int velocity);
	void setAngularVelocity(int velocity);

	struct
	{
		/** @addtogroup Node
		 * @{
		 * @addtogroup Params
		 * @{
		 */
		double 				wheel_diameter;			//!< diameter of the wheels. [m]
		double				ticks_pr_round;			//!< conversion factor. [ticks/round]
//		double				ms_in_between;			//!< period time of the control loop [ms]
		double				interval;				//!< period of the timer publishing motor commands [ms]
//		double 				max_velocity;			//!< maximum possible motor command
//		double				timeout;				//!< watchdog timeout [s]
		/**@} @}*/
	} parameters;

	struct
	{
		/** @addtogroup Node
		 * @{
		 * @addtogroup Params
		 * @{
		 */
		geometry_msgs::TwistStamped 	cmd_vel_drive;
		msgs::steering_angle_cmd 	cmd_pos_angle;
		msgs::nmea 					nmea_encoder_pos;
		msgs::encoder 				raw_encoder, raw_encoder_out, raw_encoder_edge, raw_encoder_edge_out;
		msgs::can					vel_can_msg, angle_vel_can_msg, pwr_on_msg;
		/** @} */
	} messages;

	struct
	{
		/** @addtogroup Topics
		 * @{
		 *
		 * Node topics
		 */
		std::string 		cmd_vel_drive;	//!< the commanded velocity input from higher layers @type \ref TwistStamped
		std::string			cmd_pos_angle;	//!< the commanded velocity input from higher layers @type \ref steering_angle_cmd
		std::string 		can_tx_cmd;	//!< the commanded can message output to lower layers @type \ref can
		std::string			encoder_angle;	//!< the encoder output to higher layers @type \ref encoder
		std::string			raw_encoder;	//!< the encoder input from lower layers @type \ref encoder
		std::string			encoder_edge;	//!< the encoder input from lower layers @type \ref encoder
		std::string			deadman;		//!< the deadman input from higher layers @type \ref StringStamped
		std::string			nmea_sub;		//!< the nmea input from lower layers @type \ref nmea
//		std::string			encoder_drive;	//!< the encoder input from lower layers @type \ref encoder
//		std::string			nmea_pub;		//!< the nmea output to lower layers @type \ref nmea
		/** @} */
	} topics;

	struct
	{
		/** @addtogroup Node
		 * @{
		 * @addtogroup Params
		 * @{
		 */
		ros::Subscriber 	cmd_vel_drive;
		ros::Subscriber		cmd_pos_angle;
		ros::Subscriber		raw_encoder;
		ros::Subscriber		encoder_edge;
		ros::Subscriber		deadman;
		ros::Subscriber		nmea;
		/** @} */
	} subscribers;



public:
	ros::NodeHandle 	global_node_handler;
	ros::NodeHandle		local_node_handler;

	struct
	{
		/** @addtogroup Node
		 * @{
		 * @addtogroup Params
		 * @{
		 */
//		ros::Publisher 		nmea;
//		ros::Publisher		encoder_drive_vel;
		ros::Publisher		encoder_angle_pos;
		ros::Publisher		can_tx_cmd;
		ros::Publisher		raw_encoder_out;
		ros::Publisher		raw_encoder_edge_out;
		/** @} */
	} publishers;



	wheelModule();

	void makeItSpin(void);
	void on_cmd_vel_drive(const geometry_msgs::TwistStamped::ConstPtr&);
	void on_cmd_pos_angle(const msgs::steering_angle_cmd::ConstPtr&);
	void on_raw_encoder(const msgs::encoder::ConstPtr&);
	void on_encoder_edge(const msgs::encoder::ConstPtr&);
	void on_deadman(const std_msgs::Bool::ConstPtr&);
	void on_nmea(const msgs::nmea::ConstPtr& msg);
	void on_timer(const ros::TimerEvent&);
};
/** @}*/
#endif /*WHEELMODULE_HPP_*/
