/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_people_perception
* \note
* ROS package name: cob_people_detection
*
* \author
* Author: Richard Bormann
* \author
* Supervised by:
*
* \date Date of creation: 07.08.2012
*
* \brief
* functions for recognizing a face within a color image (patch)
* current approach: eigenfaces on color image
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/
// standard includes
#include <sstream>
#include <string>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Float32MultiArray.h>
#include <cob_people_detection_msgs/EvaluationArray.h>

#include "cob_people_detection/eval_tool.h"
class EvalNode
{
public:

	/// Constructor
	/// @param nh ROS node handle
  EvalNode(ros::NodeHandle nh);
  ~EvalNode(void);

protected:


void distancesCallback(const cob_people_detection_msgs::EvaluationArray::ConstPtr& distances);
	ros::NodeHandle node_handle_;

	ros::Subscriber distances_subscriber_;		///< subscribes to the positions of detected face regions

   EvalTool e_;


};


