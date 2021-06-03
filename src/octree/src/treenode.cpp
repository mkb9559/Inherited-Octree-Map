#include "treenode.h"


treenode::treenode()
{
}


treenode::~treenode()
{
}
treenode::treenode(treenode* fa, Point3D c, bool ex)
{
	this->father = fa;
  for (int i = 0; i < 8; i++)
	{
		this->child[i] = nullptr;
	}
  this->cen   = c;
	this->exist = ex;
  this->full  = false;
	this->eg.clear();
	if (fa!=nullptr)this->nowdeep = fa->nowdeep + 1;
	else this->nowdeep=1;

	this->lowestcost = -1;
  this->wayfrom    = nullptr;

}

bool treenode::innode(Point3D p,double r)
{
  if (p.x < this->cen.x + r && p.x > this->cen.x - r
   && p.y < this->cen.y + r && p.y > this->cen.y - r
   && p.z < this->cen.z + r && p.z > this->cen.z - r)
	{
		return true;
	}
	else
		return false;
}

void treenode::clear()
{
  this->eg.clear();
  this->full       = false;
  this->exist      = false;
  this->lowestcost = -1;
  this->wayfrom    = nullptr;
}
void treenode::kill()
{
  delete this;
}
