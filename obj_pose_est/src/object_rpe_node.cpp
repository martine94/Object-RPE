#include <iostream> 
#include <cstdlib> 
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include "obj_pose_est/model_to_scene.h"
#include "obj_pose_est/ObjectRPE.h"

using namespace std;
using namespace cv;

pcl::PointCloud<pcl::PointXYZ>::Ptr     OBBs (new pcl::PointCloud<pcl::PointXYZ>);
std::vector<string>                     OBB_names;
visualization_msgs::MarkerArray         multiMarker;
std::vector<string>                     ObjectNameList;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr  scene_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  pub_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

//std::string depth_path, rgb_path, label_path, depthrgpcd;
cv::Mat rgb_img, depth_img, rgb_label_img;
double fx, fy, cx, cy, depth_factor;

std::vector<string> object_names; // names of object detected
std::vector<string> model_paths; // path to model of object detected
std::vector<string> full_items; // full list of item names in dataset 
std::vector<Eigen::Matrix4f> transforms;

int main()
{
	//Makes the function able to create all the datasets at once, and not need a restart for every separate folder.
	for(int ff = 1; ff <=12 ; ff ++){
		bool isatendoffile = false;
		std::string filefolder;
		filefolder = std::to_string(ff);
  fx=580.0; 
  fy=580.0;
  cx=319.0; 
  cy=237.0;
  depth_factor = 1000;
  //To load a batch, change the top-value of currit (inside the for-loop)
  for (int currit = 1; currit <= 9999999 ; currit++){
	  std::string numberstring;
	  if(currit < 10){
		    numberstring = "00000"+std::to_string(currit);
	  }
	  if(currit >= 10 && currit < 100){
		    numberstring = "0000"+std::to_string(currit);
	  }
	  if(currit >= 100 && currit < 1000){
		    numberstring = "000"+std::to_string(currit);
	  }
	  if(currit >= 1000 && currit < 10000){
		    numberstring = "00"+std::to_string(currit);
	  }
	  if(currit >= 10000 && currit < 100000){
		    numberstring = "0"+std::to_string(currit);
	  }
	  if(currit >= 100000 && currit < 1000000){
		    numberstring = std::to_string(currit);
	  }
	  //the paths also has to be adjusted
  std::string depth_path = "/media/martin/Innehåller/datafortrain/datafromlabelfusion/"+filefolder+"/images/"+numberstring+"-depth.png";  //<--------------- Change number
  std::string rgb_path = "/media/martin/Innehåller/datafortrain/datafromlabelfusion/"+filefolder+"/images/"+numberstring+"-color.png";   //<--------------- Change number
  std::string rgb_label_path = "/media/martin/Innehåller/datafortrain/datafromlabelfusion/"+filefolder+"/images/"+numberstring+"_color_labels.png";  //<--------------- Change number
  std::string label_path = "/media/martin/Innehåller/datafortrain/"+filefolder+"/"+filefolder+""+numberstring+"/scan.labels";    //<--------------- Change number  
  std::string depthrgpcd = "/media/martin/Innehåller/datafortrain/"+filefolder+"/"+filefolder+""+numberstring+"/scan.pcd";  //<--------------- Change number      
  string dirnamestr = "/media/martin/Innehåller/datafortrain/"+filefolder+"/"+filefolder+""+numberstring;   //<--------------- Change number
  char dirname[dirnamestr.size() + 1];
  strcpy(dirname, dirnamestr.c_str());
  mkdir(dirname, 0777);
  depth_img = cv::imread(depth_path, -1);
  rgb_img = cv::imread(rgb_path, -1);
  rgb_label_img = cv::imread(rgb_label_path, -1);
  if(!rgb_img.data || !depth_img.data || !rgb_label_img.data)
  {
      if(!rgb_img.data) {
		  std::cerr << "Cannot read image from " << rgb_path << "\n"; 
		  isatendoffile = true;
		  currit = 10000000;
	  }
	  else if(!rgb_label_img.data){
		  std::cerr << "Cannot read image from " << rgb_label_path << "\n"; 
		  isatendoffile = true;
		  currit = 10000000;
	  }
      else{
		   std::cerr << "Cannot read image from " << depth_path << "\n";
		   isatendoffile = true;
		   currit = 10000000;
      }
      if(ff==12){
      return 0;
	}
  }

if(isatendoffile==false)
{
  ofstream myfile;
  ofstream newdepthpcd;
  myfile.open (label_path);

  //cv::imshow("rgb", rgb_img);
  //cv::waitKey(30);
 
   scene_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointXYZRGB point;
   for(int row=0; row < depth_img.rows; row++)
    {
       for(int col=0; col < depth_img.cols; col++)       
        {
          if(isnan(depth_img.at<ushort>(row, col)))
          {
			  continue;
			   }
          double depth = depth_img.at<ushort>(row, col) / depth_factor;
          point.x = (col-cx) * depth / fx;
          point.y = (row-cy) * depth / fy;
          point.z = depth;
          point.b = rgb_img.at<cv::Vec3b>(row, col)[0];
          point.g = rgb_img.at<cv::Vec3b>(row, col)[1];
          point.r = rgb_img.at<cv::Vec3b>(row, col)[2];
          if(rgb_img.at<cv::Vec3b>(row, col)[0] != rgb_label_img.at<cv::Vec3b>(row, col)[0] && rgb_img.at<cv::Vec3b>(row, col)[1] != rgb_label_img.at<cv::Vec3b>(row, col)[1] && rgb_img.at<cv::Vec3b>(row, col)[2] != rgb_label_img.at<cv::Vec3b>(row, col)[2])
          {
			myfile << "1\n";
		   }
		   //		   else if(point.x == 0 || point.y == 0 || point.z == 0){

		   else{
			myfile << "0\n";
		   }
         scene_cloud->push_back(point);

        }
    }
      myfile.close();
	  pcl::io::savePCDFileASCII(depthrgpcd, *scene_cloud);
      newdepthpcd.close();
  }
}      
}
return 0;
}
