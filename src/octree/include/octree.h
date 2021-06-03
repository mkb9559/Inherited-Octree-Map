
#ifndef OCTREE_H_
#define OCTREE_H_
#include <vector>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <pangolin/pangolin.h>
#include "treenode.h"

#define boundx 1000
#define boundy 20
#define boundz 1000


#define Viewer

//#define vins_mono
#define vins_fusion

class octree
{
public:
  octree();
  ~octree();

  octree(double minnuit, int maxdeep = 1, double refx = 0.0, double refy = 0.0,double refz = 0.0);

  sensor_msgs::PointCloud tpMp;
  double maxx,maxy,minx,miny,maxz,minz;

	/******map centre*****/
	double refx;
	double refy;
  double refz;


	/******tree param*****/
	int maxdeep;		// max 10
	double minunit;		// (m)

	/********root********/
  treenode* root;

	/*******tree operate**********/
  bool insert(Point3D p);		//insert unit
  void testinsert(Point3D p);
  void inserts(const sensor_msgs::PointCloud::ConstPtr& msg);
  treenode* searchpoint(Point3D p);

	/**********io*****************/
	void printpoints();
  void visprintpoints(treenode* now);
	void printmap();
  void visprintmap(treenode* now);
  void visprintedge(treenode* now);
	void readpoints();

  /**********draw*****************************/
  void visdrawmap(treenode* now);
  void myRectf(float x1,float y1,float x2,float y2);
  void emptyRectf(float cx,float cy,float cz,float r);
  void fullRectf(float cx,float cy,float cz,float r);
  void visdrawlink(treenode *now);
  void drawlink(treenode *a, treenode *b);


	/******tools*********/
	double getr(int nowdeep);
  int getdir(treenode *s, treenode *e);
  bool inViewBound(Point3D p);
  bool inBound(double x,double up,double down);
	

private:
  void rebuildtree();
  void killtree(treenode* now);

	void initroot();
  void creattree(treenode* now, Point3D p);// insert unit
  void cleartree(treenode* now);           // reflash the tree using low cost
  void maintainfull(treenode* now);

  treenode* searchpoint(treenode* now,Point3D p);

  void inheritself(treenode* now,int octant);// inherit
  void inheritfather(treenode* now, int octant);
  void maxdeepinherit(treenode* now);
  void maintainmap(treenode* s, treenode* e);


};

#endif

