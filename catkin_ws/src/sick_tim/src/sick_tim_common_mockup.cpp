/*
 * Copyright (C) 2015, Osnabrück University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Osnabrück University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: 30.01.2015
 *
 *      Author:
 *         Martin Günther <mguenthe@uos.de>
 *
 */

#include <sick_tim/sick_tim_common_mockup.h>

namespace sick_tim
{

SickTimCommonMockup::SickTimCommonMockup(AbstractParser* parser) : SickTimCommon(parser)
{
  sub_ = nh_.subscribe("datagram", 1, &SickTimCommonMockup::datagramCB, this);
}

SickTimCommonMockup::~SickTimCommonMockup()
{
}

int SickTimCommonMockup::close_device()
{
  ROS_INFO("Mockup - close_device()");
  return 0;
}

/**
 * Send a SOPAS command to the device and print out the response to the console.
 */
int SickTimCommonMockup::sendSOPASCommand(const char* request, std::vector<unsigned char> * reply)
{
  ROS_ERROR("Mockup - sendSOPASCommand(), this should never be called");
  return ExitError;
}

/*
 * provided as a separate method (not inside constructor) so we can return error codes
 */
int SickTimCommonMockup::init_device()
{
  ROS_INFO("Mockup - init_device()");
  return ExitSuccess;
}

/*
 * provided as a separate method (not inside constructor) so we can return error codes
 */
int SickTimCommonMockup::init_scanner()
{
  ROS_INFO("Mockup - init_scanner()");
  return ExitSuccess;
}

int SickTimCommonMockup::get_datagram(unsigned char* receiveBuffer, int bufferSize, int* actual_length)
{
  ROS_DEBUG("Mockup - get_datagram()");

  // wait for next datagram
  while(!datagram_msg_)
  {
    if (!ros::ok())
      return ExitError;

    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  // copy datagram to receiveBuffer
  std::vector<char> str(datagram_msg_->data.begin(), datagram_msg_->data.end());
  str.push_back('\0');
  *actual_length = datagram_msg_->data.length();
  datagram_msg_.reset();

  if (bufferSize < *actual_length + 1)
  {
    ROS_ERROR("Mockup - Buffer too small!");
    return ExitError;
  }

  strncpy(reinterpret_cast<char *>(receiveBuffer), &str[0], *actual_length + 1);

  return ExitSuccess;
}

void SickTimCommonMockup::datagramCB(const std_msgs::String::ConstPtr &msg)
{
if (datagram_msg_)
  ROS_WARN("Mockup - dropping datagram message");

datagram_msg_ = msg;
}

} /* namespace sick_tim */
