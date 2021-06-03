#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud.h>

#include <iostream>
#include <bitset>
#include <chrono>


#include "Navigation3D.h"


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


int main(int argc, char **argv)
{
	//unsigned int x = 1073741824;


  /*
	Navigation2D* niv=new Navigation2D(Point2D(10.1,10.1),Point2D(44.6,36.6));
	niv->settree(1.45, 6, 25, 18);
	niv->buildtree();
	niv->SPFA();
	niv->outputans();	
  */
  ros::init(argc,argv,"octree");
  ros::start();
  Navigation3D mNavi;
  mNavi.init(Point3D(10.1,10.1,10.1),Point3D(44.6,36.6,-15.2));


#ifdef vins_mono
  message_filters::Subscriber<sensor_msgs::PointCloud> MPs_sub(mNavi.nh,"/vins_estimator/history_cloud",100);
#endif
#ifdef vins_fusion
  message_filters::Subscriber<sensor_msgs::PointCloud> MPs_sub(mNavi.nh,"/vins_estimator/margin_cloud",100);
#endif
  MPs_sub.registerCallback(boost::bind(&Navigation3D::getMPs,&mNavi,_1));

  /*
  mNavi.testinsert(1.2,1.2,1.2);
  mNavi.testinsert(5.2,5.2,5.2);
  mNavi.testinsert(25.2,25.2,25.2);
  */
/*
  FILE *ff;
  double tx,ty,tz,tin,tr,tg,tb;
  ff=fopen( "marketplacefeldkirch_station1_intensity_rgb.txt", "r");
  int cnt=0;
  while (!feof(ff))
  {
    fscanf(ff, "%lf", &tx);
    fscanf(ff, "%lf", &ty);
    fscanf(ff, "%lf", &tz);
    fscanf(ff, "%lf", &tin);
    fscanf(ff, "%lf", &tr);
    fscanf(ff, "%lf", &tg);
    fscanf(ff, "%lf", &tb);
    mNavi.testinsert(tx, ty,tz);
    //std::cout << "Load:"<<++cnt<<":"<<tx<< " , " << ty<<" , "<<tz<< std::endl;
  }
  fclose(ff);
  std::cout << "read points ok!" << std::endl;
  */


  ros::spin();



	return 0;
}

