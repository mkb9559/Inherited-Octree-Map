#include "Point3DNiv.h"


Point3DNiv::Point3DNiv()
{
}


Point3DNiv::~Point3DNiv()
{
}
/*
bool Point2DNiv::operator>(const Point2DNiv &b) const
{
	return this->J > b.J;
}
*/
bool Point3DNiv::operator<(const Point3DNiv &b) const
{
	return this->J > b.J;
}
Point3DNiv::Point3DNiv(treenode* t)
{
	this->tree = t;
	this->J = 1000000000;
	this->cost = 0;
}
Point3DNiv::Point3DNiv(treenode* t ,Point3D e)
{
	this->tree = t;
	this->cost = t->lowestcost;
	getJ(e);
}
Point3DNiv::Point3DNiv(treenode* t, Point3D e, double co)
{
	this->tree = t;
	this->cost = co;
	getJ(e);
}
void Point3DNiv::getJ(Point3D e)
{
  J = this->cost + e.dis3D(this->tree->cen);
}
