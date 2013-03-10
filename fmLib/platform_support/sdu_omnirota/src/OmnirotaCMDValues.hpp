/****************************************************************************
# FroboMind omnirotaCMDValues.hpp
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
#ifndef OMNIROTACMDVALUES_HPP
#define OMNIROTACMDVALUES_HPP
  #define OMNIROTA_CMD_POWER_ON 0x01
  #define OMNIROTA_CMD_POWER_OFF 0x02
  #define OMNIROTA_CMD_MC_ON 0x03
  #define OMNIROTA_CMD_MC_OFF 0x04
  #define OMNIROTA_CMD_GET_CURRENT 0x05
  #define OMNIROTA_CMD_GET_SPEED 0x06
  #define OMNIROTA_CMD_SET_SPEED 0x07
  #define OMNIROTA_CMD_GET_TEMP 0x08
  #define OMNIROTA_CHECK_DCLINK_VTG 0x09
  #define OMNIROTA_CHECK_INVERTER_PWR 0x0A
  #define OMNIROTA_INVERTER_UPTIME 0x0c
  #define OMNIROTA_GET_PWR_STAGE_STATUS 0x0D
  #define OMNIROTA_CMD_HEARTBEAT 0xA0
  #define OMNIROTA_CMD_HEARTBEAT_VAL 0x0F
  #define OMNIROTA_CMD_TURN 0x0F
  
  #define OMNIROTA_DRIVING_MOTOR_SEND 0x101
  #define OMNIROTA_DRIVING_MOTOR_RECV 0x102
  #define OMNIROTA_STEERING_MOTOR_SEND 0x103
  #define OMNIROTA_STEERING_MOTOR_RECV 0x104
  
  #define OMNIROTA_MOTOR_DIRECTION_FORWARD 1
  #define OMNIROTA_MOTOR_DIRECTION_BACKWARD 2
#endif //OMNIROTACMDVALUES_HPP
