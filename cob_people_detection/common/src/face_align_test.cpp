
#include"cob_people_detection/face_normalizer.h"
#include<iostream>
#include<opencv/cv.h>
#include<opencv/highgui.h>
int main(int argc, const char *argv[])
{


  std::cout<<"[FaceNormalizer] running scene no. "<<argv[1]<<"...\n";
  FaceNormalizer::FNConfig cfg;
  cfg.eq_ill=true;
  cfg.align=true;
  cfg.resize=true;
  cfg.cvt2gray=true;
  cfg.extreme_illumination_condtions=false;

  std::string  class_path="/opt/ros/groovy/share/OpenCV/";

  FaceNormalizer fn;
  fn.init(class_path,cfg);
  cv::Mat depth,img,xyz;
  std::string i_path;
  //else      i_path="/share/goa-tz/people_detection/eval/Kinect3DSelect/";
  i_path="/share/goa-tz/people_detection/eval/KinectIPA/";

  i_path.append(argv[1]);
  std::string xml_path=i_path;
  xml_path.append(".xml");

  if(!fn.read_scene(xyz,img,xml_path))return 0;
  cv::Mat wmat1,wmat2;
  img.copyTo(wmat1);
  img.copyTo(wmat2);
  cv::Size norm_size=cv::Size(100,100);
  //cv::cvtColor(wmat1,wmat1,CV_RGB2BGR);
  //cv::imshow("ORIGINAL",wmat1);
  cv::Mat depth_res;
  std::vector<cv::Mat> synth_images;
  // call member functions of FaceNormalizer
  //fn.synthFace(wmat1,xyz,norm_size,synth_images);
  //fn.demonstrate_head_rotation(wmat1,xyz);


  //fn.isolateFace(wmat1,xyz);
  //cv::imshow("ORIGINAL",wmat1);
  //cv::waitKey(0);
  if(!fn.normalizeFace(wmat1,xyz,norm_size,depth))fn.normalizeFaceInteractive(wmat1,xyz,norm_size);

    //fn.recordFace(wmat1,xyz);


  wmat1.convertTo(wmat1,CV_8UC1);
 // cv::equalizeHist(depth,depth);
  cv::imshow("NORMALIZED",wmat1);
  cv::waitKey(0);

  //fn.recordFace(wmat1,xyz);


  //fn.normalizeFace(wmat2,xyz,norm_size);
 // depth.convertTo(depth,CV_8UC1,255);
 // cv::equalizeHist(depth,depth);
  //wmat2.convertTo(wmat2,CV_8UC1);
  //cv::imshow("NORMALIZED",wmat2);
  //cv::waitKey(0);
  return 0;
}
