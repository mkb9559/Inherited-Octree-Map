#ifndef TREENODE_H_
#define TREENODE_H_
#include <iostream>
#include <vector>
#include <set>
#include "Point3D.h"
class treenode
{
public:
  treenode();
  ~treenode();
  treenode(treenode* fa, Point3D c, bool ex);
	/******tree info*********/
	bool exist;
	bool full;
  Point3D cen;
	int nowdeep;
  treenode* child[8];
  treenode* father;
  std::set<treenode*> eg;				// only for exist==true ,eg!=empty()

	/*******nvi info*********/
	double lowestcost;
  treenode* wayfrom;
  bool innode(Point3D p,double r);
  void clear();
  void kill();


};

#endif
