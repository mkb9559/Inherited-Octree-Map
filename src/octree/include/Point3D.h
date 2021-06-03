#ifndef POINT3D_H_
#define POINT3D_H_
#include <cmath>
class Point3D
{
public:
  Point3D();
  ~Point3D();

  Point3D(double x1, double y1,double z1);
  Point3D operator-(const Point3D &b);
  Point3D operator+(const Point3D &b);
	double x;
	double y;
  double z;

  int getOctantId(Point3D p);//p is not the center
  double dis3D(Point3D p);
	double length();

};


#endif

