#include<cob_people_detection/face_normalizer.h>
#include<pcl/common/common.h>
#include<pcl/common/eigen.h>
#if !defined(PCL_VERSION_COMPARE)
	#include<pcl/common/transform.h>
#else
	#if PCL_VERSION_COMPARE(<,1,2,0)
		#include<pcl/common/transform.h>
	#endif
#endif

#include<fstream>

#include<limits>

#include<boost/lexical_cast.hpp>

using namespace cv;

void FaceNormalizer::init(FNConfig& i_config)
{
  std::string def_classifier_directory=     "/opt/ros/groovy/share/OpenCV/";
  this->init(def_classifier_directory,"",i_config,0,false,false);
}

void FaceNormalizer::init(std::string i_classifier_directory,FNConfig& i_config)
{
  this->init(i_classifier_directory,"",i_config,0,false,false);
}

//call initialize function with set default values
// -> no debug
// -> no storage of results
// -> epoch ctr set to zero
// -> classifier directory set to standard opencv installation dir
void FaceNormalizer::init()
{

  // set default values for face normalization
  FNConfig def_config;
  def_config.eq_ill=  true;
  def_config.align=   false;
  def_config.resize=  true;
  def_config.cvt2gray=true;
  def_config.extreme_illumination_condtions=false;

  std::string def_classifier_directory=   "/opt/ros/groovy/share/OpenCV/"  ;

  this->init(def_classifier_directory,"",def_config,0,false,false);
}



void FaceNormalizer::init(std::string i_classifier_directory,std::string i_storage_directory,FNConfig& i_config,int i_epoch_ctr,bool i_debug,bool i_record_scene)
{
  classifier_directory_=i_classifier_directory;
  storage_directory_=   i_storage_directory;
  config_=              i_config;
  epoch_ctr_=            i_epoch_ctr;
  debug_=               i_debug;
  record_scene_=        i_record_scene;

  if (config_.align)
  {
    std::string eye_r_path,eye_path,eye_l_path,nose_path,mouth_path;
    eye_r_path=    classifier_directory_+ "haarcascades/haarcascade_mcs_righteye.xml";
    eye_l_path=    classifier_directory_+ "haarcascades/haarcascade_mcs_lefteye.xml";
    nose_path=     classifier_directory_+ "haarcascades/haarcascade_mcs_nose.xml";

    eye_r_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_r_path.c_str(),0,0,0);
    eye_r_storage_=cvCreateMemStorage(0);

    eye_l_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_l_path.c_str(),0,0,0);
    eye_l_storage_=cvCreateMemStorage(0);

    nose_cascade_=(CvHaarClassifierCascade*) cvLoad(nose_path.c_str(),0,0,0);
    nose_storage_=cvCreateMemStorage(0);

    //intrinsics
    cv::Vec2f focal_length;
    cv::Point2f pp;
    focal_length[0]=524.90160178307269 ;
    focal_length[1]=525.85226379335393;
    pp.x=312.13543361773458;
    pp.y=254.73474482242005;
    cam_mat_=(cv::Mat_<double>(3,3) << focal_length[0] , 0.0 , pp.x  , 0.0 , focal_length[1] , pp.y  , 0.0 , 0.0 , 1);
    dist_coeffs_=(cv::Mat_<double>(1,5) << 0.25852454045259377, -0.88621162461930914, 0.0012346117737001144, 0.00036377459304633028, 1.0422813597203011);
  }

  initialized_=true;
}


FaceNormalizer::~FaceNormalizer()
{
  if(config_.align)
  {
	cvReleaseHaarClassifierCascade(&nose_cascade_);
	cvReleaseMemStorage(&nose_storage_);
	cvReleaseHaarClassifierCascade(&eye_l_cascade_);
	cvReleaseMemStorage(&eye_l_storage_);
	cvReleaseHaarClassifierCascade(&eye_r_cascade_);
	cvReleaseMemStorage(&eye_r_storage_);
  }
};

bool FaceNormalizer::isolateFace(cv::Mat& RGB,cv::Mat& XYZ)
{
  cv::Vec3f middle_pt=XYZ.at<cv::Vec3f>(round(XYZ.rows/2),round(XYZ.cols/2));

  float background_threshold=middle_pt[2]+0.25;

  eliminate_background(RGB,XYZ,background_threshold);
  interpolate_head(RGB, XYZ);
  return true;
}

bool FaceNormalizer::recordFace(cv::Mat&RGB,cv::Mat& XYZ)
{
  
  
  std::string filepath=storage_directory_;
  filepath.append("scene");
  save_scene(RGB,XYZ,filepath);

}

bool FaceNormalizer::synthFace(cv::Mat &RGB,cv::Mat& XYZ, cv::Size& norm_size,std::vector<cv::Mat>& synth_images)
{
  //TODO
  //maybe deveop a measure for the angles that are now being represented
  norm_size_=norm_size;
  input_size_=cv::Size(RGB.cols,RGB.rows);

  bool valid = true; // Flag only returned true if all steps have been completed successfully

  epoch_ctr_++;

  //geometric normalization
  if(config_.align)
  {
    cv::Mat GRAY;
    cv::cvtColor(RGB,GRAY,CV_RGB2GRAY);
    if(!features_from_color(GRAY))
    {
      if(!features_from_color_interactive(GRAY))return false;
    }
    if(!synth_head_poses(GRAY,XYZ,synth_images))
    {
      valid=false;
      // if head synthesis fails push back color image
      synth_images.push_back(RGB);
    }

  }

  // process remaining normalization steps with all synthetic images
  for(int n=0;n<synth_images.size();n++)
  {
  if(config_.cvt2gray)
  {
    if(synth_images[n].channels()==3)cv::cvtColor(synth_images[n],synth_images[n],CV_RGB2GRAY);
  }

  if(config_.eq_ill)
  {
    // radiometric normalization
    if(!normalize_radiometry(synth_images[n])) valid=false;
    if(debug_)dump_img(synth_images[n],"radiometry");
  }

  if(debug_ && valid)dump_img(synth_images[n],"geometry");

  if(config_.resize)
  {
  //resizing
  normalize_img_type(synth_images[n],synth_images[n]);
  cv::resize(synth_images[n],synth_images[n],norm_size_,0,0);
  }

  if(debug_)dump_img(synth_images[n],"size");
  }

  epoch_ctr_++;
  return valid;

}
void mouseEvent(int evt,int x, int y,int flags,void* param)
{
  //std::cout<<"GOT: y="<<x<<" y="<<y<<std::endl;
  if(evt==CV_EVENT_LBUTTONDOWN)
  {
    //FaceNormalizer::assign_pos_from_cb(x,y);
    FaceNormalizer* fn=(FaceNormalizer*)param;
    fn->assign_pos_from_cb(x,y);
  }
}
void FaceNormalizer::assign_pos_from_cb(int x, int y)
{
  cv::Point2f pos;
  pos.x=x;
  pos.y=y;
  img_feat_vec_.push_back(pos);
}
bool FaceNormalizer::ia_loop(cv::Mat img)
{
  char key=0;
  std::cout<<"--------------------------------------------------------------------------"<<std::endl;
  std::cout<<"------------------------Interactive Mode----------------------------------"<<std::endl;
  std::cout<<" - click on nose, left eye , right eye in this order to assign positions."<<std::endl;
  while((int)key!=27 && img_feat_vec_.size()<3)
  {
  cv::imshow("iWin",img);
  key=waitKey(1);
  }

  if(img_feat_vec_.size()==3)
  {
  std::cout<<" - recieved positions- verify positions - accept with <enter> reset with <r>, abort with <Esc>"<<std::endl;
  cv::Mat timg;
  img.copyTo(timg);
  cv::circle(timg,img_feat_vec_[0],2,cv::Scalar(255,255,255),-1,8);
  cv::circle(timg,img_feat_vec_[1],2,cv::Scalar(255,255,255),-1,8);
  cv::circle(timg,img_feat_vec_[2],2,cv::Scalar(255,255,255),-1,8);

  cv::imshow("iWin",timg);
  char key2;
  key2=cv::waitKey(0);

  while(1)
  {
  if((int)key2==10)
  {
    std::cout<<" accepting"<<std::endl;
    f_det_img_.nose=img_feat_vec_[0];
    f_det_img_.lefteye=img_feat_vec_[1];
    f_det_img_.righteye=img_feat_vec_[2];
    return true;
  }
  else if ((int)key2==114)
  {
    std::cout<<" resetting positions"<<std::endl;
    img_feat_vec_.clear();
    ia_loop(img);
    break;
  }
  else if ((int)key2==27)
  {
    std::cout<<"aborting"<<std::endl;
    img_feat_vec_.clear();
    return false;
  }
  else
  {
    std::cout<<" press <Enter> to accept - <r> ro reset"<<std::endl;
  }
  }

  }
  else
  {
    std::cout<<"not all features determined - cancelling geometrical normalization"<<std::endl;
    return false;
  }
}
bool FaceNormalizer::features_from_color_interactive(cv::Mat& img)
{
  cv::namedWindow("iWin");
  cvSetMouseCallback("iWin",mouseEvent,(void*)this);
  if(!ia_loop(img))return false;
  return true;

}
bool FaceNormalizer::normalizeFaceInteractive(cv::Mat& img, cv::Mat& depth, cv::Size& norm_size)
{
  norm_size_=norm_size;
  input_size_=cv::Size(img.cols,img.rows);

  bool valid = true; // Flag only returned true if all steps have been completed successfully

  epoch_ctr_++;

  //geometric normalization
  if(config_.align)
  {
    if(! features_from_color_interactive(img)) valid=false;
    if(!normalize_geometry_depth(img,depth)) valid=false;
  }

  if(config_.cvt2gray)
  {
    if(img.channels()==3)cv::cvtColor(img,img,CV_RGB2GRAY);
  }

  if(config_.eq_ill)
  {
    // radiometric normalization
    if(!normalize_radiometry(img)) valid=false;
    cv::imwrite("/home/goa-tz/Desktop/rad.png",img);
    if(debug_)dump_img(img,"radiometry");
  }

  if(debug_ && valid)dump_img(img,"geometry");

  if(config_.resize)
  {
  //resizing
  cv::resize(img,img,norm_size_,0,0);
  cv::resize(depth,depth,norm_size_,0,0);
  }

  if(debug_)dump_img(img,"size");

  epoch_ctr_++;
  normalize_img_type(img,img);
  return valid;

}
bool FaceNormalizer::normalizeFace( cv::Mat& RGB, cv::Mat& XYZ, cv::Size& norm_size, cv::Mat& DM)
{
  bool valid=true;
  valid =normalizeFace(RGB,XYZ,norm_size);
  XYZ.copyTo(DM);
  //reducing z coordinate and performing whitening transformation
  create_DM(DM,DM);
  return valid;
}


bool FaceNormalizer::normalizeFace( cv::Mat& img,cv::Mat& depth,cv::Size& norm_size)
{

  norm_size_=norm_size;
  input_size_=cv::Size(img.cols,img.rows);

  bool valid = true; // Flag only returned true if all steps have been completed successfully

  epoch_ctr_++;

  //geometric normalization
  if(config_.align)
  {
    if(!features_from_color(img))return false;
    if(!normalize_geometry_depth(img,depth)) valid=false;
  }

  if(config_.cvt2gray)
  {
    if(img.channels()==3)cv::cvtColor(img,img,CV_RGB2GRAY);
  }

  if(config_.eq_ill)
  {
    // radiometric normalization
    if(!normalize_radiometry(img)) valid=false;
    cv::imwrite("/home/goa-tz/Desktop/rad.png",img);
    if(debug_)dump_img(img,"radiometry");
  }

  if(debug_ && valid)dump_img(img,"geometry");

  if(config_.resize)
  {
  //resizing
  cv::resize(img,img,norm_size_,0,0);
  cv::resize(depth,depth,norm_size_,0,0);
  }

  if(debug_)dump_img(img,"size");

  epoch_ctr_++;
  normalize_img_type(img,img);
  return valid;
}


bool FaceNormalizer::normalizeFace( cv::Mat& img,cv::Size& norm_size)
{

  if(!initialized_)
  {
    std::cout<<"[FaceNormalizer] not initialized - use init() first"<<std::endl;
  }
  // set members to current values
  norm_size_=norm_size;
  input_size_=cv::Size(img.cols,img.rows);

  bool valid = true; // Flag only returned true if all steps have been completed successfully

  if(config_.cvt2gray)
  {
  if(img.channels()==3)cv::cvtColor(img,img,CV_RGB2GRAY);
  }

  if(config_.align)
  {
    std::cout<<"[FaceNormalizer] geometrical normalization only with 3D data"<<std::endl;
  //geometric normalization for rgb image only disabled
  }

  if(config_.eq_ill)
  {
  // radiometric normalization
  if(!normalize_radiometry(img)) valid=false;
  if(debug_)dump_img(img,"1_radiometry");
  }

  if(config_.resize)
  {
  //resizing
  cv::resize(img,img,norm_size_,0,0);
  if(debug_)dump_img(img,"2_resized");
  }

  epoch_ctr_++;
  normalize_img_type(img,img);
  return valid;
}

bool FaceNormalizer::normalize_radiometry(cv::Mat& img)
{
  
  if(config_.extreme_illumination_condtions==true)GammaDoG(img);
  else  GammaDCT(img);
  return true;
}

void FaceNormalizer::extractVChannel( cv::Mat& img,cv::Mat& V)
{
  if(debug_)cv::cvtColor(img,img,CV_RGB2HSV);
  else cv::cvtColor(img,img,CV_BGR2HSV);

  std::vector<cv::Mat> channels;
  cv::split(img,channels);
  channels[2].copyTo(V);

  if(debug_)cv::cvtColor(img,img,CV_HSV2RGB);
  else cv::cvtColor(img,img,CV_HSV2BGR);

  return;

}

void FaceNormalizer::subVChannel(cv::Mat& img,cv::Mat& V)
{

   if(debug_)cv::cvtColor(img,img,CV_RGB2HSV);
  else cv::cvtColor(img,img,CV_BGR2HSV);

  std::vector<cv::Mat> channels;
  cv::split(img,channels);
  channels[2]=V;
  cv::merge(channels,img);

  if(debug_)cv::cvtColor(img,img,CV_HSV2RGB);
  else cv::cvtColor(img,img,CV_HSV2BGR);

  return;
}



void FaceNormalizer::GammaDoG(cv::Mat& input_img)
{
  cv::Mat img=cv::Mat(input_img.rows,input_img.cols,CV_8UC1);
  if(input_img.channels()==3)
  {
    extractVChannel(input_img,img);
    std::cout<<"extracting"<<std::endl;
  }
  else
  {
    img=input_img;
  }


  img.convertTo(img,CV_32FC1);

  // gamma correction
  cv::pow(img,0.2,img);
  //dog
  cv::Mat g2,g1;
  cv::GaussianBlur(img,g1,cv::Size(9,9),1);
  cv::GaussianBlur(img,g2,cv::Size(9,9),2);
  cv::subtract(g2,g1,img);
  //cv::normalize(img,img,0,255,cv::NORM_MINMAX);
  img.convertTo(img,CV_8UC1,255);
  cv::equalizeHist(img,img);

  input_img=img;
}

void FaceNormalizer::GammaDCT(cv::Mat& input_img)
{
  cv::Mat img=cv::Mat(input_img.rows,input_img.cols,CV_8UC1);
  if(input_img.channels()==3)
  {
    extractVChannel(input_img,img);
  }
  else
  {
    img=input_img;
  }

  // Dct conversion on logarithmic image
  if( img.rows&2!=0 )
  {
    img=img(cv::Rect(0,0,img.cols,img.rows-1));
    input_img=img(cv::Rect(0,0,img.cols,img.rows-1));
  }
  if( img.cols&2!=0 )
  {
    img=img(cv::Rect(0,0,img.cols-1,img.rows));
    input_img=img(cv::Rect(0,0,img.cols-1,img.rows));
  }

  cv::equalizeHist(img,img);
  img.convertTo(img,CV_32FC1);
  cv::Scalar mu,sigma;
  cv::meanStdDev(img,mu,sigma);

  double C_00=log(mu.val[0])*sqrt(img.cols*img.rows);

//----------------------------
  cv::pow(img,0.2,img);
  cv::Mat imgdummy;
  img.convertTo(imgdummy,CV_8UC1);
  cv::dct(img,img);

  //---------------------------------------
  img.at<float>(0,0)=C_00;
  img.at<float>(0,1)/=10;
  img.at<float>(0,2)/=10;

  img.at<float>(1,0)/=10;
  img.at<float>(1,1)/=10;

  //--------------------------------------

  cv::idct(img,img);
  cv::normalize(img,img,0,255,cv::NORM_MINMAX);

  img.convertTo(img,CV_8UC1);
  //cv::blur(img,img,cv::Size(3,3));

  if(input_img.channels()==3)
  {
    subVChannel(input_img,img);
  }
  else
  {
    input_img=img;
  }
}


bool FaceNormalizer::synth_head_poses_relative(cv::Mat& img,cv::Mat& depth,std::vector<cv::Mat>& synth_images)
{

  // detect features
  if(!features_from_color(img))return false;
  if(debug_)dump_features(img);


   if(!features_from_depth(depth)) return false;

   Eigen::Vector3f temp,x_new,y_new,z_new,lefteye,nose,righteye,eye_middle;

   nose<<f_det_xyz_.nose.x,f_det_xyz_.nose.y,f_det_xyz_.nose.z;
   lefteye<<f_det_xyz_.lefteye.x,f_det_xyz_.lefteye.y,f_det_xyz_.lefteye.z;
   righteye<<f_det_xyz_.righteye.x,f_det_xyz_.righteye.y,f_det_xyz_.righteye.z;
   eye_middle=lefteye+((righteye-lefteye)*0.5);

   x_new<<f_det_xyz_.righteye.x-f_det_xyz_.lefteye.x,f_det_xyz_.righteye.y-f_det_xyz_.lefteye.y,f_det_xyz_.righteye.z-f_det_xyz_.lefteye.z;
   temp<<f_det_xyz_.nose.x-eye_middle[0],f_det_xyz_.nose.y-eye_middle[1],(f_det_xyz_.nose.z-eye_middle[2]);
   z_new=x_new.cross(temp);
   x_new.normalize();
   z_new.normalize();
   y_new=x_new.cross(z_new);

   if(y_new[1]<0) y_new*=-1;


   Eigen::Vector3f origin;
   origin=nose;
   //origin=nose+(0.1*z_new);

   Eigen::Affine3f T_norm;

   pcl::getTransformationFromTwoUnitVectorsAndOrigin(y_new,z_new,origin,T_norm);


   // viewing offset of normalized perspective
    double view_offset=0.6;
    Eigen::Translation<float,3> translation=Eigen::Translation<float,3>(0, 0, view_offset);

    // modify T_norm by angle for nose incline compensation
    Eigen::AngleAxis<float> roll(-0.78, x_new);
    //Eigen::AngleAxis<float> roll(-0.78, Eigen::Vector3f(1,0,0));
    T_norm=roll*T_norm;


  Eigen::Affine3f T_rot;
  cv::Mat workmat=cv::Mat(depth.rows,depth.cols,CV_32FC3);
  Eigen::AngleAxis<float> alpha;

  cv::Mat dmres;  cv::Mat imgres;
  cv::Rect roi;



  //number of poses
  int N=7;
  std::cout<<"Synthetic POSES"<<std::endl;

  for(int i=0;i<N;i++)
  {
    switch(i)
    {
      case 0:
        {
          alpha=Eigen::AngleAxis<float>((float)0, x_new);
          break;
        }
      case 1:
        {
          alpha=Eigen::AngleAxis<float>((float) 0.1*M_PI, x_new);
          break;
        }
      case 2:
        {
          alpha=Eigen::AngleAxis<float>((float)-0.1*M_PI, x_new);
          break;
        }
      case 3:
        {
          alpha=Eigen::AngleAxis<float>((float) 0.1*M_PI, y_new);
          break;
        }
      case 4:
        {
          alpha=Eigen::AngleAxis<float>((float)-0.1*M_PI, y_new);
          break;
        }
      case 5:
        {
          alpha=Eigen::AngleAxis<float>((float) 0.1*M_PI, z_new);
          break;
        }
      case 6:
        {
          alpha=Eigen::AngleAxis<float>((float)-0.1*M_PI, z_new);
          break;
        }
    }
  // ----- artificial head pose rotation

  T_rot.setIdentity();
  T_rot=alpha*T_rot;
  // ----- artificial head pose rotation

  dmres=cv::Mat::zeros(480,640,CV_32FC3);
  if(img.channels()==3)imgres=cv::Mat::zeros(480,640,CV_8UC3);
  if(img.channels()==1)imgres=cv::Mat::zeros(480,640,CV_8UC1);


  depth.copyTo(workmat);
   cv::Vec3f* ptr=workmat.ptr<cv::Vec3f>(0,0);
   Eigen::Vector3f pt;
   for(int i=0;i<img.total();i++)
   {
     pt<<(*ptr)[0],(*ptr)[1],(*ptr)[2];
     pt=T_rot*pt;
     pt=translation*pt;

    (*ptr)[0]=pt[0];
    (*ptr)[1]=pt[1];
    (*ptr)[2]=pt[2];
     ptr++;
   }

   nose<<f_det_xyz_.nose.x,f_det_xyz_.nose.y,f_det_xyz_.nose.z;
   lefteye<<f_det_xyz_.lefteye.x,f_det_xyz_.lefteye.y,f_det_xyz_.lefteye.z;
   righteye<<f_det_xyz_.righteye.x,f_det_xyz_.righteye.y,f_det_xyz_.righteye.z;

   lefteye=translation*T_rot*T_norm*lefteye;
   righteye=translation*T_rot*T_norm*righteye;
   nose=translation*T_rot*T_norm*nose;

   //transform norm coordiantes separately to  determine roi
   cv::Point2f lefteye_uv,righteye_uv,nose_uv;
   cv::Point3f lefteye_xyz,righteye_xyz,nose_xyz;


   lefteye_xyz = cv::Point3f(lefteye[0],lefteye[1],lefteye[2]);
   righteye_xyz = cv::Point3f(righteye[0],righteye[1],righteye[2]);
   cv::Mat rt=(cv::Mat_<double>(3,1)<<lefteye[0],lefteye[1],lefteye[2]);
   nose_xyz = cv::Point3f(nose[0],nose[1],nose[2]);

   projectPoint(lefteye_xyz,lefteye_uv);
   std::cout<<"CV: "<<lefteye_uv<<std::endl;
   cv::Mat t=cam_mat_*rt;
   std::cout<<"ME: "<<t/t.at<double>(2)<<std::endl;
   projectPoint(righteye_xyz,righteye_uv);
   projectPoint(nose_xyz,nose_uv);

   //determine bounding box

   float s=2;
   int dim_x=(righteye_uv.x-lefteye_uv.x)*s;
   int off_x=((righteye_uv.x-lefteye_uv.x)*s -(righteye_uv.x-lefteye_uv.x))/2;
   int off_y=off_x;
   int dim_y=dim_x;

   roi=cv::Rect(round(nose_uv.x-dim_x*0.5),round(nose_uv.y-dim_y*0.5),dim_x,dim_y);
   //roi=cv::Rect(round(lefteye_uv.x-dim_x*0.25),round(lefteye_uv.y-dim_y*0.25),dim_x,dim_y);

   if(img.channels()==3)cv::cvtColor(img,img,CV_RGB2GRAY);



  projectPointCloud(img,workmat,imgres,dmres);


  if(debug_)dump_img(imgres,"uncropped");

  if(roi.height<=1 ||roi.width<=0 || roi.x<0 || roi.y<0 ||roi.x+roi.width >imgres.cols || roi.y+roi.height>imgres.rows)
  {
    std::cout<<"[FaceNormalizer]image ROI out of limits"<<std::endl;
    return false;
  }
  imgres=imgres(roi);
  imgres=imgres(cv::Rect(2,2,imgres.cols-4,imgres.rows-4));

  synth_images.push_back(imgres);
  }

  return true;
}
bool FaceNormalizer::synth_head_poses(cv::Mat& img,cv::Mat& depth,std::vector<cv::Mat>& synth_images)
{

  // detect features
  if(debug_)dump_features(img);


   if(!features_from_depth(depth)) return false;

   Eigen::Vector3f temp,x_new,y_new,z_new,lefteye,nose,righteye,eye_middle;

   nose<<f_det_xyz_.nose.x,f_det_xyz_.nose.y,f_det_xyz_.nose.z;
   lefteye<<f_det_xyz_.lefteye.x,f_det_xyz_.lefteye.y,f_det_xyz_.lefteye.z;
   righteye<<f_det_xyz_.righteye.x,f_det_xyz_.righteye.y,f_det_xyz_.righteye.z;
   eye_middle=lefteye+((righteye-lefteye)*0.5);

   x_new<<f_det_xyz_.righteye.x-f_det_xyz_.lefteye.x,f_det_xyz_.righteye.y-f_det_xyz_.lefteye.y,f_det_xyz_.righteye.z-f_det_xyz_.lefteye.z;
   temp<<f_det_xyz_.nose.x-eye_middle[0],f_det_xyz_.nose.y-eye_middle[1],(f_det_xyz_.nose.z-eye_middle[2]);
   z_new=x_new.cross(temp);
   x_new.normalize();
   z_new.normalize();
   y_new=x_new.cross(z_new);

   if(y_new[1]<0) y_new*=-1;


   Eigen::Vector3f origin;
   origin=nose;
   //origin=nose+(0.1*z_new);

   Eigen::Affine3f T_norm;

   pcl::getTransformationFromTwoUnitVectorsAndOrigin(y_new,z_new,origin,T_norm);


   // viewing offset of normalized perspective
    double view_offset=0.6;
    Eigen::Translation<float,3> translation=Eigen::Translation<float,3>(0, 0, view_offset);

    // modify T_norm by angle for nose incline compensation
    Eigen::AngleAxis<float> roll(0.4, x_new);
    T_norm=roll*T_norm;


  Eigen::Affine3f T_rot;
  Eigen::AngleAxis<float> alpha;

  cv::Mat dmres;
  cv::Mat imgres;
  cv::Rect roi;


  float  background_thresh=view_offset+0.3;
  //eliminate_background(img,depth,background_thresh);

  cv::Mat workmat=cv::Mat(depth.rows,depth.cols,CV_32FC3);
//  // eliminate background
//  cv::Vec3f* xyz_ptr=depth.ptr<cv::Vec3f>(0,0);
//  for(int r=0;r<depth.total();r++)
//  {
//    if((*xyz_ptr)[2]>background_thresh ||(*xyz_ptr)[2]<0)
//    {
//      // set this to invalid value
//      *xyz_ptr=cv::Vec3f(-1000,-1000,-1000);
//    }
//    xyz_ptr++;
//  }

  //number of poses
  int N=5;
  std::cout<<"Synthetic POSES"<<std::endl;

  for(int i=0;i<N;i++)
  {
    switch(i)
    {
      case 0:
        {
          alpha=Eigen::AngleAxis<float>((float)0, Eigen::Vector3f(1,0,0));
          break;
        }
      case 1:
        {
          alpha=Eigen::AngleAxis<float>((float) 0.1*M_PI, Eigen::Vector3f(1,0,0));
          break;
        }
      case 2:
        {
          alpha=Eigen::AngleAxis<float>((float)-0.1*M_PI, Eigen::Vector3f(1,0,0));
          break;
        }
      case 3:
        {
          alpha=Eigen::AngleAxis<float>((float) 0.1*M_PI, Eigen::Vector3f(0,1,0));
          break;
        }
      case 4:
        {
          alpha=Eigen::AngleAxis<float>((float)-0.1*M_PI, Eigen::Vector3f(0,1,0));
          break;
        }
      case 5:
        {
          alpha=Eigen::AngleAxis<float>((float) 0.1*M_PI, Eigen::Vector3f(0,0,1));
          break;
        }
      case 6:
        {
          alpha=Eigen::AngleAxis<float>((float)-0.1*M_PI, Eigen::Vector3f(0,0,1));
          break;
        }
    }
  // ----- artificial head pose rotation

  T_rot.setIdentity();
  T_rot=alpha*T_rot;
  // ----- artificial head pose rotation

  dmres=cv::Mat::zeros(480,640,CV_32FC3);
  if(img.channels()==3)imgres=cv::Mat::zeros(480,640,CV_8UC3);
  if(img.channels()==1)imgres=cv::Mat::zeros(480,640,CV_8UC1);


  depth.copyTo(workmat);
   cv::Vec3f* ptr=workmat.ptr<cv::Vec3f>(0,0);
   Eigen::Vector3f pt;
   for(int i=0;i<img.total();i++)
   {
     pt<<(*ptr)[0],(*ptr)[1],(*ptr)[2];
     pt=T_norm*pt;
     pt=T_rot*pt;
     pt=translation*pt;

    (*ptr)[0]=pt[0];
    (*ptr)[1]=pt[1];
    (*ptr)[2]=pt[2];
     ptr++;
   }

   nose<<f_det_xyz_.nose.x,f_det_xyz_.nose.y,f_det_xyz_.nose.z;
   lefteye<<f_det_xyz_.lefteye.x,f_det_xyz_.lefteye.y,f_det_xyz_.lefteye.z;
   righteye<<f_det_xyz_.righteye.x,f_det_xyz_.righteye.y,f_det_xyz_.righteye.z;

   lefteye=translation*T_rot*T_norm*lefteye;
   righteye=translation*T_rot*T_norm*righteye;
   nose=translation*T_rot*T_norm*nose;

   //transform norm coordiantes separately to  determine roi
   cv::Point2f lefteye_uv,righteye_uv,nose_uv;
   cv::Point3f lefteye_xyz,righteye_xyz,nose_xyz;


   lefteye_xyz = cv::Point3f(lefteye[0],lefteye[1],lefteye[2]);
   righteye_xyz = cv::Point3f(righteye[0],righteye[1],righteye[2]);
   nose_xyz = cv::Point3f(nose[0],nose[1],nose[2]);


   projectPoint(lefteye_xyz,lefteye_uv);
   projectPoint(righteye_xyz,righteye_uv);
   projectPoint(nose_xyz,nose_uv);

   //determine bounding box

   float s=2;
   int dim_x=(righteye_uv.x-lefteye_uv.x)*s;
   int off_x=((righteye_uv.x-lefteye_uv.x)*s -(righteye_uv.x-lefteye_uv.x))/2;
   int off_y=off_x;
   int dim_y=dim_x;

   roi=cv::Rect(round(nose_uv.x-dim_x*0.5),round(nose_uv.y-dim_y*0.5),dim_x,dim_y);
   //roi=cv::Rect(round(lefteye_uv.x-dim_x*0.25),round(lefteye_uv.y-dim_y*0.25),dim_x,dim_y);

   if(img.channels()==3)cv::cvtColor(img,img,CV_RGB2GRAY);



  projectPointCloud(img,workmat,imgres,dmres);



  if(debug_)dump_img(imgres,"uncropped");

  if(roi.height<=1 ||roi.width<=0 || roi.x<0 || roi.y<0 ||roi.x+roi.width >imgres.cols || roi.y+roi.height>imgres.rows)
  {
    std::cout<<"[FaceNormalizer]image ROI out of limits"<<std::endl;
    return false;
  }
  imgres=imgres(roi);
  imgres=imgres(cv::Rect(2,2,imgres.cols-4,imgres.rows-4));

  synth_images.push_back(imgres);
  }

  return true;
}
bool FaceNormalizer::demonstrate_head_rotation(cv::Mat& img,cv::Mat& depth)
{

  // detect features
  if(!features_from_color(img))
  {
  if(!features_from_color_interactive(img))return false;
  }
  if(debug_)dump_features(img);


   if(!features_from_depth(depth))
   {
     std::cout<<"Could not get 3D points of features"<<std::endl;
     return false;
   }

   Eigen::Vector3f temp,x_new,y_new,z_new,lefteye,nose,righteye,eye_middle;

   nose<<f_det_xyz_.nose.x,f_det_xyz_.nose.y,f_det_xyz_.nose.z;
   lefteye<<f_det_xyz_.lefteye.x,f_det_xyz_.lefteye.y,f_det_xyz_.lefteye.z;
   righteye<<f_det_xyz_.righteye.x,f_det_xyz_.righteye.y,f_det_xyz_.righteye.z;
   eye_middle=lefteye+((righteye-lefteye)*0.5);

   x_new<<f_det_xyz_.righteye.x-f_det_xyz_.lefteye.x,f_det_xyz_.righteye.y-f_det_xyz_.lefteye.y,f_det_xyz_.righteye.z-f_det_xyz_.lefteye.z;
   temp<<f_det_xyz_.nose.x-eye_middle[0],f_det_xyz_.nose.y-eye_middle[1],(f_det_xyz_.nose.z-eye_middle[2]);
   z_new=x_new.cross(temp);
   x_new.normalize();
   z_new.normalize();
   y_new=x_new.cross(z_new);

   if(y_new[1]<0) y_new*=-1;


   Eigen::Vector3f origin;
   origin=nose;
   //origin=nose+(0.1*z_new);

   Eigen::Affine3f T_norm;
   std::cout<<"X-Axis:\n "<<x_new<<std::endl;
   std::cout<<"Y-Axis:\n "<<y_new<<std::endl;
   std::cout<<"Z-Axis:\n "<<z_new<<std::endl;

   pcl::getTransformationFromTwoUnitVectorsAndOrigin(y_new,z_new,origin,T_norm);


   // viewing offset of normalized perspective
    double view_offset=0.6;
    Eigen::Translation<float,3> translation=Eigen::Translation<float,3>(0, 0, view_offset);

    // modify T_norm by angle for nose incline compensation
    //Eigen::AngleAxis<float> yaw(-0.3, z_new);
    Eigen::AngleAxis<float> roll(-0.4, x_new);
    //Eigen::AngleAxis<float> roll(0, x_new);
    T_norm=roll*T_norm;


  Eigen::Affine3f T_rot;
  cv::Mat workmat=cv::Mat(depth.rows,depth.cols,CV_32FC3);
  Eigen::AngleAxis<float> alpha;

  cv::Mat dmres;
  cv::Mat imgres;
  cv::Rect roi;
  for(int i=0;i<240;i++)
  {
  // ----- artificial head pose rotation
  if(i<80)alpha=Eigen::AngleAxis<float>((float)i*0.025*M_PI, Eigen::Vector3f(1,0,0));
  else if(i>=80&&i<160)alpha=Eigen::AngleAxis<float>(((float)i-80)*0.025*M_PI, Eigen::Vector3f(0,1,0));
  else if(i>=160&&i<=240)alpha=Eigen::AngleAxis<float> (((float)i-160)*0.025*M_PI, Eigen::Vector3f(0,0,1));

    T_rot.setIdentity();
  T_rot=alpha*T_rot;
  // ----- artificial head pose rotation

   //if(img.channels()==3)cv::cvtColor(img,img,CV_RGB2GRAY);
  dmres=cv::Mat::zeros(480,640,CV_32FC3);
  if(img.channels()==3)imgres=cv::Mat::zeros(480,640,CV_8UC3);
  if(img.channels()==1)imgres=cv::Mat::zeros(480,640,CV_8UC1);


  depth.copyTo(workmat);
   cv::Vec3f* ptr=workmat.ptr<cv::Vec3f>(0,0);
   Eigen::Vector3f pt;
   for(int i=0;i<img.total();i++)
   {

     pt<<(*ptr)[0],(*ptr)[1],(*ptr)[2];
     pt=T_norm*pt;
     pt=T_rot*pt;
     pt=translation*pt;

    (*ptr)[0]=pt[0];
    (*ptr)[1]=pt[1];
    (*ptr)[2]=pt[2];
     ptr++;
   }

   nose<<f_det_xyz_.nose.x,f_det_xyz_.nose.y,f_det_xyz_.nose.z;
   lefteye<<f_det_xyz_.lefteye.x,f_det_xyz_.lefteye.y,f_det_xyz_.lefteye.z;
   righteye<<f_det_xyz_.righteye.x,f_det_xyz_.righteye.y,f_det_xyz_.righteye.z;

   lefteye=translation*T_rot*T_norm*lefteye;
   righteye=translation*T_rot*T_norm*righteye;
   nose=translation*T_rot*T_norm*nose;

  // create coordinate system


   //transform norm coordiantes separately to  determine roi
   cv::Point2f lefteye_uv,righteye_uv,nose_uv;
   cv::Point3f lefteye_xyz,righteye_xyz,nose_xyz;


   lefteye_xyz = cv::Point3f(lefteye[0],lefteye[1],lefteye[2]);
   righteye_xyz = cv::Point3f(righteye[0],righteye[1],righteye[2]);
   nose_xyz = cv::Point3f(nose[0],nose[1],nose[2]);

   projectPoint(lefteye_xyz,lefteye_uv);
   projectPoint(righteye_xyz,righteye_uv);
   projectPoint(nose_xyz,nose_uv);

   //determine bounding box

   float s=2;
   int dim_x=(righteye_uv.x-lefteye_uv.x)*s;
   int off_x=((righteye_uv.x-lefteye_uv.x)*s -(righteye_uv.x-lefteye_uv.x))/2;
   int off_y=off_x;
   int dim_y=dim_x;

   //roi=cv::Rect(round(nose_uv.x-dim_x*0.5),round(nose_uv.y-dim_y*0.5),dim_x,dim_y);
   roi=cv::Rect(round(lefteye_uv.x-dim_x*0.25),round(lefteye_uv.y-dim_y*0.25),dim_x,dim_y);



   //filter out background

  

  projectPointCloud(img,workmat,imgres,dmres);
  //TODO temporary
  // add debug oiints
  cv::circle(imgres,nose_uv,5,cv::Scalar(255,255,255),-1,8);
  cv::circle(imgres,lefteye_uv,5,cv::Scalar(255,255,255),-1,8);
  cv::circle(imgres,righteye_uv,5,cv::Scalar(255,255,255),-1,8);

  cv::Point3f x_draw=cv::Point3f(x_new[0],x_new[1],x_new[2])*0.05;
  cv::Point2f x_draw_2d;
  projectPoint(x_draw,x_draw_2d);
  cv::Point3f y_draw=cv::Point3f(y_new[0],y_new[1],y_new[2])*0.05;
  cv::Point2f y_draw_2d;
  projectPoint(y_draw,y_draw_2d);
  cv::Point3f z_draw=cv::Point3f(z_new[0],z_new[1],z_new[2])*0.05;
  cv::Point2f z_draw_2d;
  projectPoint(z_draw,z_draw_2d);



  //cv::line(imgres,nose_uv,x_draw_2d,cv::Scalar(255,0,0),4,8);
  //cv::line(imgres,nose_uv,y_draw_2d,cv::Scalar(0,255,0),4,8);
  //cv::line(imgres,nose_uv,z_draw_2d,cv::Scalar(0,0,255),4,8);


  string frame;
  if(i<10) frame="/home/goa-tz/Desktop/frame00"+boost::lexical_cast<std::string>(i)+".png";
  else if(i>=10 && i<100)frame="/home/goa-tz/Desktop/frame0"+boost::lexical_cast<std::string>(i)+".png";
  else if (i>=100) frame="/home/goa-tz/Desktop/frame"+boost::lexical_cast<std::string>(i)+".png";



    cv::imwrite(frame,imgres);
  cv::imshow("FULL",imgres);

  int key;
  key=cv::waitKey(100);
  if(key ==1048608)
  {
    std::cout<<"PAUSED--confirm usage with ENTER - continue with other key"<<std::endl;
    int enter;
    enter=cv::waitKey(0);
    if(enter==1048586)
    {
      std::cout<<"ENTER-- using current view for normalization"<<std::endl;
      break;
    }
    else
    {
      std::cout<<"CONTINUE"<<std::endl;
    }
  }
  }



  if(debug_)dump_img(imgres,"uncropped");

  if(roi.height<=1 ||roi.width<=0 || roi.x<0 || roi.y<0 ||roi.x+roi.width >imgres.cols || roi.y+roi.height>imgres.rows)
  {
    std::cout<<"[FaceNormalizer]image ROI out of limits"<<std::endl;
    return false;
  }
  imgres(roi).copyTo(img);
  //TODO TEMPorary swithc ogg
  dmres(roi).copyTo(depth);
  //despeckle<unsigned char>(img,img);
  //only take central region
  img=img(cv::Rect(2,2,img.cols-4,img.rows-4));

  return true;
}
bool FaceNormalizer::normalize_geometry_depth(cv::Mat& img,cv::Mat& depth)
{

  // detect features
  if(debug_)dump_features(img);


   if(!features_from_depth(depth)) return false;

   Eigen::Vector3f temp,x_new,y_new,z_new,lefteye,nose,righteye,eye_middle;

   nose<<f_det_xyz_.nose.x,f_det_xyz_.nose.y,f_det_xyz_.nose.z;
   lefteye<<f_det_xyz_.lefteye.x,f_det_xyz_.lefteye.y,f_det_xyz_.lefteye.z;
   righteye<<f_det_xyz_.righteye.x,f_det_xyz_.righteye.y,f_det_xyz_.righteye.z;
   eye_middle=lefteye+((righteye-lefteye)*0.5);

   x_new<<f_det_xyz_.righteye.x-f_det_xyz_.lefteye.x,f_det_xyz_.righteye.y-f_det_xyz_.lefteye.y,f_det_xyz_.righteye.z-f_det_xyz_.lefteye.z;
   temp<<f_det_xyz_.nose.x-eye_middle[0],f_det_xyz_.nose.y-eye_middle[1],(f_det_xyz_.nose.z-eye_middle[2]);
   z_new=x_new.cross(temp);
   x_new.normalize();
   z_new.normalize();
   y_new=x_new.cross(z_new);

   if(y_new[1]<0) y_new*=-1;

   Eigen::Vector3f nose_flat=nose;
   nose_flat[2]+=0.03;


   Eigen::Vector3f origin;
   origin=nose;

   Eigen::Affine3f T_norm;

   pcl::getTransformationFromTwoUnitVectorsAndOrigin(y_new,z_new,origin,T_norm);


   // viewing offset of normalized perspective
    double view_offset=0.6;
    Eigen::Translation<float,3> translation=Eigen::Translation<float,3>(0, 0, view_offset);

    // modify T_norm by angle for nose incline compensation
    Eigen::AngleAxis<float> roll(-0.4, x_new);
    T_norm=translation*roll*T_norm;

   cv::Vec3f* ptr=depth.ptr<cv::Vec3f>(0,0);
   Eigen::Vector3f pt;
   for(int i=0;i<img.total();i++)
   {

     pt<<(*ptr)[0],(*ptr)[1],(*ptr)[2];
     pt=T_norm*pt;

    (*ptr)[0]=pt[0];
    (*ptr)[1]=pt[1];
    (*ptr)[2]=pt[2];
     ptr++;
   }

   lefteye=T_norm*lefteye;
   righteye=T_norm*righteye;
   nose=T_norm*nose;

   //transform norm coordiantes separately to  determine roi
   cv::Point2f lefteye_uv,righteye_uv,nose_uv;
   cv::Point3f lefteye_xyz,righteye_xyz,nose_xyz;


   lefteye_xyz = cv::Point3f(lefteye[0],lefteye[1],lefteye[2]);
   righteye_xyz = cv::Point3f(righteye[0],righteye[1],righteye[2]);
   nose_xyz = cv::Point3f(nose[0],nose[1],nose[2]);

   projectPoint(lefteye_xyz,lefteye_uv);
   projectPoint(righteye_xyz,righteye_uv);
   projectPoint(nose_xyz,nose_uv);

   //determine bounding box
   //TODO temp
   //float s=3.3;
   float s=2.5;
   int dim_x=(righteye_uv.x-lefteye_uv.x)*s;
   int off_x=((righteye_uv.x-lefteye_uv.x)*s -(righteye_uv.x-lefteye_uv.x))/2;
   int off_y=off_x;
   int dim_y=dim_x;

   cv::Rect roi=cv::Rect(round(nose_uv.x-dim_x*0.5),round(nose_uv.y-dim_y*0.5),dim_x,dim_y);

   if(img.channels()==3)cv::cvtColor(img,img,CV_RGB2GRAY);

  cv::Mat imgres;
  if(img.channels()==3)imgres=cv::Mat::zeros(480,640,CV_8UC3);
  if(img.channels()==1)imgres=cv::Mat::zeros(480,640,CV_8UC1);
  cv::Mat dmres=cv::Mat::zeros(480,640,CV_32FC3);

  projectPointCloud(img,depth,imgres,dmres);
  if(debug_)dump_img(imgres,"uncropped");

  if(roi.height<=1 ||roi.width<=0 || roi.x<0 || roi.y<0 ||roi.x+roi.width >imgres.cols || roi.y+roi.height>imgres.rows)
  {
    std::cout<<"[FaceNormalizer]image ROI out of limits"<<std::endl;
    return false;
  }
  imgres(roi).copyTo(img);
  dmres(roi).copyTo(depth);
  ////TODO temp
  despeckle<unsigned char>(img,img);
  //only take central region
  img=img(cv::Rect(2,2,img.cols-4,img.rows-4));

  return true;
}



bool FaceNormalizer::features_from_color(cv::Mat& img_color)
{
  if(!detect_feature(img_color,f_det_img_.nose,FACE::NOSE))
  {
    std::cout<<"[FaceNormalizer] detected no nose"<<std::endl;
    f_det_img_.nose.x=round(img_color.cols*0.5);
    f_det_img_.nose.y=round(img_color.rows*0.5);
    return false;
  }
  if(!detect_feature(img_color,f_det_img_.lefteye,FACE::LEFTEYE))
  {
    std::cout<<"[FaceNormalizer] detected no eye_l"<<std::endl;
     return false;
  }
  if(!detect_feature(img_color,f_det_img_.righteye,FACE::RIGHTEYE))
  {
    std::cout<<"[FaceNormalizer] detected no eye_r"<<std::endl;
     return false;
  }

  if(debug_)
  {
    std::cout<<"[FaceNormalizer] detected detected image features:\n";
    std::cout<<"detected lefteye "<<f_det_img_.lefteye.x<<" - " << f_det_img_.lefteye.y<<std::endl;
    std::cout<<"detected righteye "<<f_det_img_.righteye.x<<" - " << f_det_img_.righteye.y<<std::endl;
    std::cout<<"detected nose "<<f_det_img_.nose.x<<" - " << f_det_img_.nose.y<<std::endl;
    //std::cout<<"detected mouth "<<f_det_img_.mouth.x<<" - " << f_det_img_.mouth.y<<std::endl;
  }
  return true;
}

bool FaceNormalizer::features_from_depth(cv::Mat& depth)
{
  //pick 3D points from pointcloud
  f_det_xyz_.nose=      depth.at<cv::Vec3f>(f_det_img_.nose.y,f_det_img_.nose.x)               ;
  //f_det_xyz_.mouth=     depth.at<cv::Vec3f>(f_det_img_.mouth.y,f_det_img_.mouth.x)            ;
  f_det_xyz_.lefteye=   depth.at<cv::Vec3f>(f_det_img_.lefteye.y,f_det_img_.lefteye.x)      ;
  f_det_xyz_.righteye=  depth.at<cv::Vec3f>(f_det_img_.righteye.y,f_det_img_.righteye.x)   ;

  if(debug_)
  {
    std::cout<<"[FaceNormalizer] detected Coordinates of features in pointcloud:"<<std::endl;
    std::cout<<"LEFTEYE: "<<f_det_xyz_.lefteye.x<<" "<<f_det_xyz_.lefteye.y<<" "<<f_det_xyz_.lefteye.z<<std::endl;
    std::cout<<"RIGTHEYE: "<<f_det_xyz_.righteye.x<<" "<<f_det_xyz_.righteye.y<<" "<<f_det_xyz_.righteye.z<<std::endl;
    std::cout<<"NOSE: "<<f_det_xyz_.nose.x<<" "<<f_det_xyz_.nose.y<<" "<<f_det_xyz_.nose.z<<std::endl;
    //std::cout<<"MOUTH: "<<f_det_xyz_.mouth.x<<" "<<f_det_xyz_.mouth.y<<" "<<f_det_xyz_.mouth.z<<std::endl;
  }
  if(!f_det_xyz_.valid()) return false;

  return true;
}

bool FaceNormalizer::detect_feature(cv::Mat& img,cv::Point2f& coords,FACE::FEATURE_TYPE type)
{

  //  determine scale of search pattern
  double scale=img.cols/160.0;

  CvSeq* seq;
  cv::Vec2f offset;

  switch(type)
  {

    case FACE::NOSE:
  {
    offset =cv::Vec2f(0,0);
    IplImage ipl_img=(IplImage)img;
     seq=cvHaarDetectObjects(&ipl_img,nose_cascade_,nose_storage_,1.1,1,0,cv::Size(20*scale,20*scale));
     //seq=cvHaarDetectObjects(&ipl_img,nose_cascade_,nose_storage_,1.3,2,CV_HAAR_DO_CANNY_PRUNING,cv::Size(15*scale,15*scale));
     break;
  }

    case FACE::LEFTEYE:
  {
    offset[0]=0;
    offset[1]=0;
    cv::Mat sub_img=img.clone();
    sub_img=sub_img(cvRect(0,0,f_det_img_.nose.x,f_det_img_.nose.y));
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,eye_l_cascade_,eye_l_storage_,1.1,1,0,cvSize(20*scale,10*scale));
     break;
  }

    case FACE::RIGHTEYE:
  {
    offset[0]=((int)f_det_img_.nose.x);
    offset[1]=0;
    cv::Mat sub_img=img.clone();
    sub_img=sub_img(cvRect(f_det_img_.nose.x,0,img.cols-f_det_img_.nose.x-1,f_det_img_.nose.y));
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,eye_r_cascade_,eye_r_storage_,1.1,1,0,cvSize(20*scale,10*scale));
     break;
  }

    case FACE::MOUTH:
  {
  //  offset[0]=0;
  //  offset[1]=(int)f_det_img_.nose.y;
  //  cv::Mat sub_img=img.clone();
  //  sub_img=sub_img(cvRect(0,f_det_img_.nose.y,img.cols,img.rows-f_det_img_.nose.y-1));
  //  IplImage ipl_img=(IplImage)sub_img;
  //   seq=cvHaarDetectObjects(&ipl_img,mouth_cascade_,mouth_storage_,1.3,4,CV_HAAR_DO_CANNY_PRUNING,cvSize(30*scale,15*scale));
  //   break;
  }
  }

    if(seq->total ==0) return false;
    Rect* seq_det=(Rect*)cvGetSeqElem(seq,0);
    coords.x=(float)seq_det->x+seq_det->width/2+offset[0];
    coords.y=(float)seq_det->y+seq_det->height/2+offset[1];

    return true;
}


void FaceNormalizer::dump_img(cv::Mat& data,std::string name)
{
  if(!debug_)
  {

    std::cout<<"[FaceNomalizer] dump_img() only with set debug flag and path."<<std::endl;
  }
  std::string filename =storage_directory_;
  filename.append(boost::lexical_cast<std::string>(epoch_ctr_));
  filename.append("_");
  filename.append(name);
  filename.append(".jpg");

  cv::imwrite(filename,data);
  return;
}

void FaceNormalizer::dump_features(cv::Mat& img)
{

  cv::Mat img2;
  img.copyTo(img2);
  IplImage ipl_img=(IplImage)img2;
   cv::circle(img2,cv::Point(f_det_img_.nose.x, f_det_img_.nose.y),5,CV_RGB(255,0,0));
   //cv::circle(img2,cv::Point(f_det_img_.mouth.x,f_det_img_.mouth.y),5,CV_RGB(0,255,0));
   cv::circle(img2,cv::Point(f_det_img_.lefteye.x,f_det_img_.lefteye.y),5,CV_RGB(255,255,0));
   cv::circle(img2,cv::Point(f_det_img_.righteye.x,f_det_img_.righteye.y),5,CV_RGB(255,0,255));
   if(debug_)dump_img(img2,"features");
}

bool FaceNormalizer::save_scene(cv::Mat& RGB,cv::Mat& XYZ,std::string path)
{
  path.append(boost::lexical_cast<std::string>(epoch_ctr_));
  std::cout<<"[FaceNormalizer]Saving to "<<path<<std::endl;
  std::string depth_path,color_path;
  color_path=path;
  color_path.append(".jpg");
  depth_path=path;
  depth_path.append(".xml");
  cv::FileStorage fs(depth_path,FileStorage::WRITE);
  fs << "depth"<<XYZ;
  fs << "color"<<RGB;
  fs.release();
  imwrite(color_path,RGB);

  return true;
}

bool FaceNormalizer::read_scene(cv::Mat& depth, cv::Mat& color,std::string path)
{
  cv::FileStorage fs(path,FileStorage::READ);
  if(fs.isOpened())
  {
  std::cout<<"[FaceNormalizer]Reading from "<<path<<std::endl;
  fs["depth"]>> depth;
  fs["color"]>> color;
  fs.release();
  return true;
  }
  else
  {
  std::cout<<"Cannot read file - invalid filename:  "<<path<<std::endl;
  return false;
  }
}


void FaceNormalizer::create_DM(cv::Mat& XYZ,cv::Mat& DM)
{
  //reducing to depth ( z - coordinate only)
  if(XYZ.channels()==3)
  {
  std::vector<cv::Mat> cls;
  cv::split(XYZ,cls);
  DM=cls[2];
  }
  else if (XYZ.channels()==1)
  {
    DM=XYZ;
  }

  //reduce z values
  float minval=std::numeric_limits<float>::max();
  for(int r=0;r<DM.rows;r++)
  {
    for(int c=0;c<DM.cols;c++)
    {
      if(DM.at<float>(r,c) < minval &&DM.at<float>(r,c)!=0) minval =DM.at<float>(r,c);
    }
  }

  for(int r=0;r<DM.rows;r++)
  {
    for(int c=0;c<DM.cols;c++)
    {
      if(DM.at<float>(r,c)!=0)DM.at<float>(r,c)-=minval;
      //if(DM.at<float>(r,c)==0)DM.at<float>(r,c)=255;
    }
  }
  //despeckle<float>(DM,DM);
  cv::medianBlur(DM,DM,5);
  //cv::blur(DM,DM,cv::Size(3,3));
}

bool FaceNormalizer::projectPoint(cv::Point3f& xyz,cv::Point2f& uv)
{
    std::vector<cv::Point3f> m_xyz;
    std::vector<cv::Point2f> m_uv;
    m_xyz.push_back(xyz);

    cv::Vec3f  rot=cv::Vec3f(0.0,0.0,0.0);
    cv::Vec3f  trans=cv::Vec3f(0.0,0.0,0.0);
    cv::projectPoints(m_xyz,rot,trans,cam_mat_,dist_coeffs_,m_uv);
    uv=m_uv[0];
}


bool FaceNormalizer::projectPointCloudNEW(cv::Mat& img, cv::Mat& depth,cv::Mat& img_res, cv::Mat& depth_res)
{
  int channels=img.channels();

  cv::Mat pc_xyz,pc_rgb;
  depth.copyTo(pc_xyz);
  img.copyTo(pc_rgb);

  //make point_list
  if(pc_xyz.rows>1 && pc_xyz.cols >1)
  {
    pc_xyz=pc_xyz.reshape(3,1);
  }

   //project 3d points to virtual camera
   //TODO temporary triy
   //cv::Mat pc_proj(pc_xyz.rows*pc_xyz.cols,1,CV_32FC2);
   cv::Mat pc_proj(pc_xyz.cols,1,CV_32FC2);

   cv::Vec3f rot=cv::Vec3f(0.0,0.0,0.0);
   cv::Vec3f trans=cv::Vec3f(0.0,0.0,0.0);
   cv::Size sensor_size=cv::Size(640,480);
   cv::projectPoints(pc_xyz,rot,trans,cam_mat_,dist_coeffs_,pc_proj);

   cv::Vec3f* pc_ptr=pc_xyz.ptr<cv::Vec3f>(0,0);
   cv::Vec2f* pc_proj_ptr=pc_proj.ptr<cv::Vec2f>(0,0);
   int ty,tx,dy,dx;



   if(channels==1)
   {
   // assign color values to calculated image coordinates
    cv::add(img_res,0,img_res);
   unsigned char* pc_rgb_ptr=pc_rgb.ptr<unsigned char>(0,0);

   //cv::Mat occ_grid=cv::Mat::ones(sensor_size,CV_32FC1);
   cv::Mat occ_grid=cv::Mat::zeros(sensor_size,CV_32FC1);
   cv::Mat img_cum=cv::Mat::zeros(sensor_size,CV_32FC1);
   cv::Mat depth_map=cv::Mat::zeros(sensor_size,CV_32FC1);
   for(int i=0;i<pc_proj.rows;++i)
     {
       cv::Vec2f txty=*pc_proj_ptr;
       tx=(int)round(txty[0]);
       ty=(int)round(txty[1]);

       float* img_ptr=NULL;
       float* occ_ptr=NULL;

       if (ty>1 && tx>1 && ty<sensor_size.height-1 && tx<sensor_size.width-1 && !isnan(ty) && !isnan(tx) )
       {
            if((depth_map.at<cv::Vec3f>(ty,tx)[2]==0) || (depth_map.at<cv::Vec3f>(ty,tx)[2]>(*pc_ptr)[2]))
            {
              img_ptr=img_cum.ptr<float>(ty,tx);
              occ_ptr=occ_grid.ptr<float>(ty,tx);
              //apply_binomial_kernel(*pc_rgb_ptr,sensor_size,img_ptr,occ_ptr);
              *img_ptr=*pc_rgb_ptr;
              depth_map.at<float>(ty,tx)=(*pc_ptr)[2];
            }
       }
       pc_rgb_ptr++;
       pc_proj_ptr++;
       pc_ptr++;
      }
   //img_cum=img_cum/occ_grid;
   img_cum.convertTo(img_res,CV_8UC1);
   //cv::blur(img_res,img_res,cv::Size(3,3));
   }



   return true;
}
bool FaceNormalizer::projectPointCloud(cv::Mat& img, cv::Mat& depth, cv::Mat& img_res, cv::Mat& depth_res)
{
  int channels=img.channels();

  cv::Mat pc_xyz,pc_rgb;
  depth.copyTo(pc_xyz);
  img.copyTo(pc_rgb);

  //make point_list
  if(pc_xyz.rows>1 && pc_xyz.cols >1)
  {
    pc_xyz=pc_xyz.reshape(3,1);
  }

   //project 3d points to virtual camera
   cv::Mat pc_proj(pc_xyz.cols,1,CV_32FC2);

   cv::Vec3f rot=cv::Vec3f(0.0,0.0,0.0);
   cv::Vec3f trans=cv::Vec3f(0.0,0.0,0.0);
   cv::Size sensor_size=cv::Size(640,480);
   cv::projectPoints(pc_xyz,rot,trans,cam_mat_,dist_coeffs_,pc_proj);

   cv::Vec3f* pc_ptr=pc_xyz.ptr<cv::Vec3f>(0,0);
   cv::Vec2f* pc_proj_ptr=pc_proj.ptr<cv::Vec2f>(0,0);
   int ty,tx;

   if(channels==3)
   {
    cv::add(img_res,0,img_res);
    cv::add(depth_res,0,depth_res);
   // assign color values to calculated image coordinates
   cv::Vec3b* pc_rgb_ptr=pc_rgb.ptr<cv::Vec3b>(0,0);


   cv::Mat occ_grid=cv::Mat::ones(sensor_size,CV_32FC3);
   cv::Mat img_cum=cv::Mat::zeros(sensor_size,CV_32FC3);
   cv::Vec3f occ_inc=cv::Vec3f(1,1,1);
   for(int i=0;i<pc_proj.rows;++i)
     {
       cv::Vec2f txty=*pc_proj_ptr;
       tx=(int)round(txty[0]);
       ty=(int)round(txty[1]);


       if (ty>1 && tx>1 && ty<sensor_size.height-1 && tx<sensor_size.width-1 && !isnan(ty) && !isnan(tx) )
       {
            if((depth_res.at<cv::Vec3f>(ty,tx)[2]==0) || (depth_res.at<cv::Vec3f>(ty,tx)[2]>(*pc_ptr)[2]))
            {
            img_cum.at<cv::Vec3b>(ty,tx)+=(*pc_rgb_ptr);
            img_cum.at<cv::Vec3f>(ty+1,tx)+=(*pc_rgb_ptr);
            img_cum.at<cv::Vec3f>(ty-1,tx)+=(*pc_rgb_ptr);
            img_cum.at<cv::Vec3f>(ty,tx-1)+=(*pc_rgb_ptr);
            img_cum.at<cv::Vec3f>(ty,tx+1)+=(*pc_rgb_ptr);

            occ_grid.at<cv::Vec3f>(ty,tx)+=  occ_inc;
            occ_grid.at<cv::Vec3f>(ty+1,tx)+=occ_inc;
            occ_grid.at<cv::Vec3f>(ty-1,tx)+=occ_inc;
            occ_grid.at<cv::Vec3f>(ty,tx+1)+=occ_inc;
            occ_grid.at<cv::Vec3f>(ty,tx-1)+=occ_inc;

            depth_res.at<cv::Vec3f>(ty,tx)=((*pc_ptr));
       }
       }
       pc_rgb_ptr++;
       pc_proj_ptr++;
       pc_ptr++;
      }
   img_cum=img_cum / occ_grid;
   img_cum.convertTo(img_res,CV_8UC3);
   }


   if(channels==1)
   {
   // assign color values to calculated image coordinates
    cv::add(img_res,0,img_res);
    cv::add(depth_res,0,depth_res);
   unsigned char* pc_rgb_ptr=pc_rgb.ptr<unsigned char>(0,0);

   cv::Mat occ_grid=cv::Mat::ones(sensor_size,CV_32FC1);
   cv::Mat img_cum=cv::Mat::zeros(sensor_size,CV_32FC1);
   cv::Mat occ_grid2=cv::Mat::ones(sensor_size,CV_32FC1);
   for(int i=0;i<pc_proj.rows;++i)
     {
       cv::Vec2f txty=*pc_proj_ptr;
       tx=(int)round(txty[0]);
       ty=(int)round(txty[1]);


       if (ty>1 && tx>1 && ty<sensor_size.height-1 && tx<sensor_size.width-1 && !isnan(ty) && !isnan(tx) )
       {
            //if((depth_map.at<cv::Vec3f>(ty,tx)[2]==0) || (depth_map.at<cv::Vec3f>(ty,tx)[2]>(*pc_ptr)[2]))
            //{
            img_res.at<unsigned char>(ty,tx)=(*pc_rgb_ptr);
            occ_grid2.at<float>(ty,tx)=0.0;
            img_cum.at<float>(ty+1,tx)+=(*pc_rgb_ptr);
            img_cum.at<float>(ty-1,tx)+=(*pc_rgb_ptr);
            img_cum.at<float>(ty,tx-1)+=(*pc_rgb_ptr);
            img_cum.at<float>(ty,tx+1)+=(*pc_rgb_ptr);
            img_cum.at<float>(ty+1,tx+1)+=(*pc_rgb_ptr);
            img_cum.at<float>(ty-1,tx-1)+=(*pc_rgb_ptr);
            img_cum.at<float>(ty-1,tx+1)+=(*pc_rgb_ptr);
            img_cum.at<float>(ty+1,tx-1)+=(*pc_rgb_ptr);

            occ_grid.at<float>(ty,tx)+=  1;
            occ_grid.at<float>(ty+1,tx)+=1;
            occ_grid.at<float>(ty-1,tx)+=1;
            occ_grid.at<float>(ty,tx+1)+=1;
            occ_grid.at<float>(ty,tx-1)+=1;
            occ_grid.at<float>(ty+1,tx+1)+=1;
            occ_grid.at<float>(ty-1,tx-1)+=1;
            occ_grid.at<float>(ty-1,tx+1)+=1;
            occ_grid.at<float>(ty+1,tx-1)+=1;

            depth_res.at<cv::Vec3f>(ty,tx)=((*pc_ptr));
       }
       pc_rgb_ptr++;
       pc_proj_ptr++;
       pc_ptr++;
      }

   occ_grid=occ_grid;
   img_cum=img_cum / (occ_grid.mul(occ_grid2)-1);
   img_cum.convertTo(img_cum,CV_8UC1);
   cv::add(img_res,img_cum,img_res);
   }



//   // refinement
//
//   //unsigned char* img_res_ptr=img_res.ptr<unsigned char>(0,0);
//   cv::Mat img_res2=cv::Mat::zeros(img_res.rows,img_res.cols,img_res.type());
//   for(int i=1;i<img_res.total();i++)
//   {
//     if(img_res.at<unsigned char>(i)==0 && img_res.at<unsigned char>(i-1)!=0)
//     {
//       //calc position
//       cv::Vec2f pos= pc_proj.at<cv::Vec2f>(i-1);
//       std::cout<<"POSITION"<<pos<<std::endl;
//
//       unsigned char val=pc_rgb.at<unsigned char>(round(pos[0]),round(pos[1]+1));
//       img_res2.at<unsigned char>(i)=val;
//     }
//   }
//
//   cv::imshow("IMG_RES",img_res2);
//   cv::waitKey(0);
//
   return true;
}

void FaceNormalizer::apply_binomial_kernel(int val,cv::Size dim,float* img,float* occ)
{

  int filter_width=5;

  int arr4[]={1, 2, 1,  2 ,4 ,2 ,1 ,2 ,1 };
  int arr5[]={1, 4,6,4,1,4,16,24,16,4,16,24,36,24,16,4,16,24,16,4,1, 4,6,4,1  };

  // set pointers to first element
  img-=(int)(floor(filter_width/2)*dim.width-floor(filter_width/2));
  occ-=(int)(floor(filter_width/2)*dim.width-floor(filter_width/2));
  int* f=&arr4[0];
  for(int n=0;n<filter_width*filter_width;n++)
  {
  *img+=(float)(*f) *(float)val;
  *occ+=(float)*f;
  occ++;
  img++;
  f++;
  }







}

bool FaceNormalizer::eliminate_background(cv::Mat& RGB,cv::Mat& XYZ,float background_thresh)
{
  // eliminate background
  cv::Vec3f* xyz_ptr=XYZ.ptr<cv::Vec3f>(0,0);

  if(RGB.channels()==3)
  {
  cv::Vec3b* rgb_ptr=RGB.ptr<cv::Vec3b>(0,0);

  for(int r=0;r<XYZ.total();r++)
  {
    if((*xyz_ptr)[2]>background_thresh ||(*xyz_ptr)[2]<0)
    {
      // set this to invalid value
      *xyz_ptr=cv::Vec3f(-1000,-1000,-1000);
      *rgb_ptr=cv::Vec3b(0,0,0);
    }
    xyz_ptr++;
    rgb_ptr++;
  }
  }

  else if(RGB.channels()==1)
  {
  unsigned char* rgb_ptr=RGB.ptr<unsigned char>(0,0);
  for(int r=0;r<XYZ.total();r++)
  {
    if((*xyz_ptr)[2]>background_thresh ||(*xyz_ptr)[2]<0)
    {
      // set this to invalid value
      *xyz_ptr=cv::Vec3f(-1000,-1000,-1000);
      *rgb_ptr=0;
    }
    xyz_ptr++;
    rgb_ptr++;
  }
  }
}

bool FaceNormalizer::interpolate_head(cv::Mat& RGB, cv::Mat& XYZ)
{
  std::vector<cv::Mat> xyz_vec;
  cv::split(XYZ,xyz_vec);

  cv::imshow("Z",xyz_vec[2]);
  cv::imshow("RGB",RGB);

  cv::waitKey(0);

}
bool FaceNormalizer::normalize_img_type(cv::Mat& in,cv::Mat& out)
{
  in.convertTo(out,CV_64FC1);
  return true;
}
