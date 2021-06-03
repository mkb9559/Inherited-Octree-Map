#ifndef POINT3DNIV_H_
#define POINT3DNIV_H_
#include "cmath"

#include "treenode.h"
class Point3DNiv
{
public:
  Point3DNiv();
  ~Point3DNiv();
  Point3DNiv(treenode* t);
  Point3DNiv(treenode* t, Point3D e);
  Point3DNiv(treenode* t, Point3D e, double coadd);
  //bool operator>(const Point3DNiv &b) const;
  bool operator<(const Point3DNiv &b) const;
	double J;
	double cost;
  treenode* tree;
private:
  void getJ(Point3D e);
};
#endif

