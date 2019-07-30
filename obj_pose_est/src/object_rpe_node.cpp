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

std::string depth_path, rgb_path, label_path, depthrgpcd;
cv::Mat rgb_img, depth_img;
double fx, fy, cx, cy, depth_factor;

std::vector<string> object_names; // names of object detected
std::vector<string> model_paths; // path to model of object detected
std::vector<string> full_items; // full list of item names in dataset 
std::vector<Eigen::Matrix4f> transforms;

int main()
{
  fx=580.0; 
  fy=580.0;
  cx=319.0; 
  cy=237.0;
  depth_factor = 1000;
  depth_path = "/home/martin/Skrivbord/data/000001-depth.png";
  rgb_path = "/home/martin/Skrivbord/data/000001-color.png";
  label_path = "/home/martin/Skrivbord/data/000001-label.label";      
  depthrgpcd = "/home/martin/Skrivbord/data/000001-scan.pcd";        
  depth_img = cv::imread(depth_path, -1);
  rgb_img = cv::imread(rgb_path, -1);
  if(!rgb_img.data || !depth_img.data)
  {
      if(!rgb_img.data) {
		  std::cerr << "Cannot read image from " << rgb_path << "\n"; 
	  }
      else{
		   std::cerr << "Cannot read image from " << depth_path << "\n";
      }
      return 0;
  }

  ofstream myfile;
  ofstream newdepthpcd;
  myfile.open (label_path);

  cv::imshow("rgb", rgb_img);
  cv::waitKey(300);
 
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
          if(point.x == 0 || point.y == 0 || point.z == 0)
          {
			myfile << "0\n";
		   }
		   else{
			myfile << "1\n";
		   }
         scene_cloud->push_back(point);

        }
    }
      myfile.close();
	  pcl::io::savePCDFileASCII(depthrgpcd, *scene_cloud);

      newdepthpcd.close();
      return 0;
}
