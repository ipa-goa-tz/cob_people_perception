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



#include "cob_people_detection/eval_node.h"

// Boost
#include <boost/shared_ptr.hpp>
#include <std_msgs/Float32MultiArray.h>

//std
#include <iostream>


EvalNode::EvalNode(ros::NodeHandle nh)
: node_handle_(nh){

  std::vector<double> threshs;
  threshs.push_back(1900.0);

  std::string plot_title = "DIFS";
  e_.config(plot_title,threshs,0);
	// subscribe to head detection topic

  distances_subscriber_=nh.subscribe("distances",1,&EvalNode::distancesCallback,this);
}

EvalNode::~EvalNode(void){};


void EvalNode::distancesCallback(const cob_people_detection_msgs::EvaluationArray::ConstPtr& distances){

  std::vector<double> dist_vec;
  for (int i = 0; i < distances->groups[0].g_dist.size(); i++) {
    dist_vec.push_back(distances->groups[0].g_dist[i]);
  }
  e_.plot(dist_vec);
}

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "eval_node");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create FaceRecognizerNode class instance
	EvalNode evn(nh);


	// Create action nodes
	//DetectObjectsAction detect_action_node(object_detection_node, nh);
	//AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
	//TrainObjectAction train_object_node(object_detection_node, nh);

	ros::spin();

	return 0;
}
