#!/usr/bin/env python
import os
import sys
import subprocess
import time


if __name__ =="__main__":
  target_dir1="/share/goa-tz/people_detection/eval/kinect3d_face"
  target_dir2="/share/goa-tz/people_detection/eval/kinect3d_features"
  base_path="/share/goa-tz/people_detection/eval/Kinect3D"
  launch_param_file="/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/ros/launch/scene_publisher_params.yaml"
  fi_str="file: \""
  os.chdir(target_dir1)
  os.chdir(target_dir2)

  for i in range(31):
    i_str=str(i)


    person_path=os.path.join(base_path,i_str)
    l_str=fi_str+person_path+"\""
    with open(launch_param_file,"w") as lp_stream:
        lp_stream.write(l_str)
    #os.system("roslaunch cob_people_detection scene_publisher.launch")
    subprocess.call(["roslaunch","cob_people_detection","scene_publisher.launch"])
    os.chdir(target_dir1)
    os.system("mv *.jpg *.xml %s"%i_str) 
    os.chdir(target_dir2)
    os.system("mv *.jpg *.xml %s"%i_str) 
