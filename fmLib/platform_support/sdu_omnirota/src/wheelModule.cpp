/****************************************************************************
# FroboMind wheelModule.cpp
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
#include "wheelModule.hpp"
#include "OmnirotaCMDValues.hpp"

wheelModule::wheelModule()
:local_node_handler("~"),global_node_handler()
{
//	angle_vel = 0;
//	drive_vel = 0;
//	vel_to_motor_const = 0;

	active = false;
	deadman = false;

	last_deadman_received = ros::Time::now();
};

void wheelModule::makeItSpin( void )
{
//	ROS_INFO("ROS_INFO");
//	ROS_WARN("ROS_WARN");
//	ROS_ERROR("ROS_ERROR");

	local_node_handler.param<std::string>(	"cmd_vel_drive_sub", 	topics.cmd_vel_drive,	"/fmActuators/drive_velocity"	);
	local_node_handler.param<std::string>(	"cmd_pos_angle_sub", 	topics.cmd_pos_angle,	"/fmActuators/steering_angle"	);
	local_node_handler.param<std::string>(	"enc_pos_angle_pub", 	topics.encoder_angle,	"/fmInformation/wheel_angle"	);
	local_node_handler.param<std::string>(	"can_tx_cmd_pub", 		topics.can_tx_cmd,		"/fmCSP/can0_tx"				);
	local_node_handler.param<std::string>(	"raw_encoder", 			topics.raw_encoder,		"/fmSensors/raw_encoder"		);
	local_node_handler.param<std::string>(	"raw_encoder_edge",		topics.encoder_edge,	"/fmSensors/raw_encoder_edge"	);
  	local_node_handler.param<std::string>(	"nmea_sub", 			topics.nmea_sub, 		"/fmData/nmea_from_omnirota"	);

	local_node_handler.param<double>(		"wheel_diameter", 		parameters.wheel_diameter,	0.7		);
	local_node_handler.param<double>(		"ticks_pr_round", 		parameters.ticks_pr_round,	42		);

	local_node_handler.param<double>(		"angle_regulator_interval",	parameters.interval,		0.05	);

	subscribers.cmd_vel_drive = global_node_handler.subscribe<geometry_msgs::TwistStamped>(topics.cmd_vel_drive, 2,&wheelModule::on_cmd_vel_drive,this);
	subscribers.cmd_pos_angle = global_node_handler.subscribe<msgs::steering_angle_cmd>(topics.cmd_pos_angle,2,&wheelModule::on_cmd_pos_angle,this);
	subscribers.raw_encoder	  = global_node_handler.subscribe<msgs::encoder>(topics.raw_encoder,2,&wheelModule::on_raw_encoder,this);
	subscribers.encoder_edge  = global_node_handler.subscribe<msgs::encoder>(topics.encoder_edge,2,&wheelModule::on_encoder_edge,this);
//	subscribers.deadman = 		global_node_handler.subscribe<std_msgs::Bool>(topics.deadman,2,&FrobitInterface::on_deadman,this);
	subscribers.nmea = 			global_node_handler.subscribe<msgs::nmea>(topics.nmea_sub,2,&wheelModule::on_nmea,this);

//	publishers.nmea = 			global_node_handler.advertise<msgs::nmea>(topics.nmea_pub, 1);
//	publishers.encoder_drive_vel = 	global_node_handler.advertise<geometry_msgs::TwistStamped>(topics.encoder_drive_vel, 1);
	publishers.encoder_angle_pos = 	global_node_handler.advertise<msgs::encoder>(topics.encoder_angle, 1);
	publishers.can_tx_cmd		 = 	global_node_handler.advertise<msgs::can>(topics.can_tx_cmd, 1);
	publishers.raw_encoder_out = 	global_node_handler.advertise<msgs::encoder>(topics.raw_encoder, 1);
	publishers.raw_encoder_edge_out = 	global_node_handler.advertise<msgs::encoder>(topics.encoder_edge, 1);

	ros::Timer t1= global_node_handler.createTimer(ros::Duration(parameters.interval),&wheelModule::on_timer,this);

	messages.vel_pwr_on_msg.id = OMNIROTA_DRIVING_MOTOR_SEND;
	messages.vel_pwr_on_msg.flags = 0;
	messages.vel_pwr_on_msg.data[0] = OMNIROTA_CMD_POWER_ON;
	messages.vel_pwr_on_msg.length = 1;

	messages.angle_pwr_on_msg.id = OMNIROTA_STEERING_MOTOR_SEND;
	messages.angle_pwr_on_msg.flags = 0;
	messages.angle_pwr_on_msg.data[0] = OMNIROTA_CMD_POWER_ON;
	messages.angle_pwr_on_msg.length = 1;

	ros::spin();
}

void wheelModule::on_nmea(const msgs::nmea::ConstPtr& msg)
{
  if(msg->type == "PORST") //If nmea message is for omnirota
	{
		messages.raw_encoder_out.header.stamp = msg->header.stamp;
		messages.raw_encoder_out.encoderticks = boost::lexical_cast<int>(msg->data.at(1));
		publishers.raw_encoder_out.publish(messages.raw_encoder_out);

		messages.raw_encoder_edge_out.header.stamp = msg->header.stamp;
		messages.raw_encoder_edge_out.encoderticks = boost::lexical_cast<int>(msg->data.at(2));
		publishers.raw_encoder_edge_out.publish(messages.raw_encoder_edge_out);
	}
}

void wheelModule::on_cmd_vel_drive(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	double wheel_circumference = (3.14f * parameters.wheel_diameter);
	double gearing_ratio = 54.24;
	double vel_to_motor_const = 600.0f / wheel_circumference*gearing_ratio;
	messages.cmd_vel_drive = *msg;
	drive_vel_sp = messages.cmd_vel_drive.twist.linear.x * vel_to_motor_const;
	int velocity = drive_vel_sp+0.5; //rpm
	this->setVelocity(velocity);
}

void wheelModule::on_cmd_pos_angle(const msgs::steering_angle_cmd::ConstPtr& msg)
{
	messages.cmd_pos_angle = *msg;
}

void wheelModule::on_raw_encoder(const msgs::encoder::ConstPtr& msg)
{
	messages.raw_encoder = *msg;
}

void wheelModule::on_encoder_edge(const msgs::encoder::ConstPtr& msg)
{
	const int encoder_edge_min_valid = 512; //Defines the angular range where 
											//a edge on the inductive detector
											//is valid
	const int encoder_edge_max_valid = 768;
	messages.raw_encoder_edge = *msg;
	int encoder_offset_min_valid = (1024-angle_encoder_offset + encoder_edge_min_valid)%1024;
	int encoder_offset_max_valid = (1024-angle_encoder_offset + encoder_edge_max_valid)%1024;
	int curr_encoder_edge = messages.raw_encoder_edge.encoderticks;
	if((curr_encoder_edge >= encoder_offset_min_valid) && 
		(curr_encoder_edge <= encoder_offset_max_valid))
		angle_encoder_offset = curr_encoder_edge;
}

void wheelModule::on_deadman(const std_msgs::Bool::ConstPtr& msg)
{
	deadman = msg->data;
	last_deadman_received = ros::Time::now();
}

//void wheelModule::on_encoder(const msgs::nmea::ConstPtr& msg)
//{
//	messages.encoder.header.stamp = msg->header.stamp;
//	messages.encoder.encoderticks = boost::lexical_cast<int>(msg->data.at(1));
//	publishers.encoder_left.publish(messages.encoder);

//	messages.encoder.encoderticks = boost::lexical_cast<int>(msg->data.at(2));
//	publishers.encoder_right.publish(messages.encoder);
//}

void wheelModule::on_timer(const ros::TimerEvent& e)
{
		const double angle_to_enc = 256.0/1.570796327;
		const double P = 0.5;
		
		publishers.can_tx_cmd.publish(messages.vel_pwr_on_msg);
		publishers.can_tx_cmd.publish(messages.angle_pwr_on_msg);

		angle_pos_sp = messages.cmd_pos_angle.steering_angle;
		//0-1024 ≃-pi/2 pi/2
		double offset = 290;
//		double pos = (angle_pos_sp + 1.570796327)*(256.0/1.570796327);
//		int ipos = (pos+offset)+0.5;
		//this->setAngularVelocity(ipos%1024);
		//return;
		int sp = (int)((angle_pos_sp+1.570796327) * angle_to_enc)%1024;
		angle_encoder_pos = (int)(1024-angle_encoder_offset + messages.raw_encoder.encoderticks +offset)%1024;
		double err = sp - angle_encoder_pos;
		double correction = err * P;
		if(correction < 0)
			correction *=-1;
		if(correction>255)
			correction = 255;
		double out;
		if(sp>angle_encoder_pos)
			out = (int)(1024 + correction)%1024;
		else
			out = (int)(1024 - correction)%1024;
//		std::cout << "SP:" << sp << "	" << "pos:" << angle_encoder_pos << "	" << "err:" << err << "	" << "Correction:" << correction << "	" << "out:" << out << std::endl;
	//build can message
	int out_set = out+0.5f;
	this->setAngularVelocity(out_set);
}

void wheelModule::setVelocity(int velocity)
{
	velocity = -velocity;
	messages.vel_can_msg.header.stamp = ros::Time::now();
	messages.vel_can_msg.id = OMNIROTA_DRIVING_MOTOR_SEND;
	messages.vel_can_msg.flags = 0;
	messages.vel_can_msg.data[0] = OMNIROTA_CMD_SET_SPEED;
	if(velocity>0)
	 messages.vel_can_msg.data[1] = OMNIROTA_MOTOR_DIRECTION_FORWARD;
	else
	{
	 messages.vel_can_msg.data[1] = OMNIROTA_MOTOR_DIRECTION_BACKWARD;
	 velocity *= -1;
	}	
	messages.vel_can_msg.data[2] = velocity & 0xff;
	messages.vel_can_msg.data[3] = (velocity>>8) & 0xff;
	messages.vel_can_msg.length = 4;
	//publish message
	publishers.can_tx_cmd.publish(messages.vel_can_msg);
}

void wheelModule::setAngularVelocity(int velocity)
{
	messages.angle_vel_can_msg.header.stamp = ros::Time::now();
	messages.angle_vel_can_msg.id = OMNIROTA_STEERING_MOTOR_SEND;
	messages.angle_vel_can_msg.flags = 0;
	messages.angle_vel_can_msg.data[0] = OMNIROTA_CMD_TURN;
	messages.angle_vel_can_msg.data[1] = velocity & 0xff;
    messages.angle_vel_can_msg.data[2] = (velocity>>8) & 0xff;
    messages.angle_vel_can_msg.length = 3;
	//publish message
	publishers.can_tx_cmd.publish(messages.angle_vel_can_msg);
}

