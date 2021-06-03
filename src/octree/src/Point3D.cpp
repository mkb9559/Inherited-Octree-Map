#include "Point3D.h"


Point3D::Point3D()
{
}


Point3D::~Point3D()
{
}



Point3D Point3D::operator-(const Point3D &b)
{
  return Point3D(this->x - b.x, this->y - b.y, this->z - b.z);
}
Point3D Point3D::operator+(const Point3D &b)
{
  return Point3D(this->x + b.x, this->y + b.y, this->z + b.z);
}


Point3D::Point3D(double x1, double y1,double z1)
{
	this->x = x1;
	this->y = y1;
  this->z = z1;
}
int Point3D::getOctantId(Point3D p)
{
  Point3D div;
	int octant = -1;
	div = p- (*this);
  bool xx, yy, zz;
	xx = div.x > 0;
	yy = div.y > 0;
  zz = div.z > 0;
  if (  xx  &&   yy &&   zz )octant = 0;
  if ((!xx) &&   yy &&   zz )octant = 1;
  if ((!xx) && (!yy)&&   zz )octant = 2;
  if (   xx && (!yy)&&   zz )octant = 3;
  if (  xx  &&   yy && (!zz))octant = 4;
  if ((!xx) &&   yy && (!zz))octant = 5;
  if ((!xx) && (!yy)&& (!zz))octant = 6;
  if (   xx && (!yy)&& (!zz))octant = 7;
	return octant;
}
double Point3D::dis3D(Point3D p)
{
  return std::sqrt((this->x - p.x)*(this->x - p.x)
                   + (this->y - p.y)*(this->y - p.y)
                   + (this->z - p.z)*(this->z - p.z));
}
double Point3D::length()
{
  return dis3D(Point3D(0, 0, 0));
}
