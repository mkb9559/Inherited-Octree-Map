#ifndef NAVIGATION3D_H_
#define NAVIGATION3D_H_
#include <stack>
#include <queue>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <mutex>
#include <pangolin/pangolin.h>

#include <sensor_msgs/PointCloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "octree.h"
#include "Point3DNiv.h"
#define WIN32



class Navigation3D
{
public:
  Navigation3D();
  ~Navigation3D();
  Navigation3D(Point3D s, Point3D e);

  Point3D aim;
  Point3D start;
  std::stack<Point3D> way;

	void SPFA();
  void init(Point3D s,Point3D e);
  void settree(double minnuit = 2, int maxdeep = 10, double refx = -100.0, double refy = 100.0);

  ros::NodeHandle nh;
  void testinsert(double x,double y,double z);
  void getMPs(const sensor_msgs::PointCloud::ConstPtr& msg);
  void printfMPs(const sensor_msgs::PointCloud::ConstPtr& msg);

private:
  octree* mytree;

  void generateway(treenode* e);
	void outputway();


  std::thread* mpview;
  std::mutex  mlock;
  float viewx,viewy,viewz,viewf;
  int   viewPointSize;
  void Drawer();



};
#endif

