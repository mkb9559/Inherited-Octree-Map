#include "octree.h"


octree::octree()
{
  this->maxx = 0;
  this->maxy = 0;
  this->maxz = 0;
  this->minx = 0;
  this->miny = 0;
  this->minz = 0;
  octree(0.1);
}


octree::~octree()
{
}

octree::octree(double minnuit, int maxdeep, double refx, double refy,double refz)
{
	this->maxdeep = maxdeep;
	if (this->maxdeep > 10)this->maxdeep = 10;
  if (this->maxdeep <  3)this->maxdeep = 3;
	this->minunit = minnuit;
	this->refx = refx;
	this->refy = refy;
  this->refz = refz;
  this->root = nullptr;
  initroot();
}
void octree::initroot()
{
  ROS_INFO("init tree");
  //we can not delete this directly, the memory will be very big if we do not reuse the octree.
	if (this->root != nullptr)
	{
    ROS_INFO("old tree");
    cleartree(this->root);
    return;
    //delete this->root;
	}
  ROS_INFO("new tree");
  this->root = new treenode(nullptr, Point3D(refx,refy,refz),true);// root is empty at first
  Point3D childcen[8];
  double r = getr(root->nowdeep);
  for (int i = 0; i < 8; i++)
  {
    switch (i)
    {
    case 0:
      childcen[i] = Point3D(root->cen.x + r/2, root->cen.y + r/2, root->cen.z + r/2); break;
    case 1:
      childcen[i] = Point3D(root->cen.x - r/2, root->cen.y + r/2, root->cen.z + r/2); break;
    case 2:
      childcen[i] = Point3D(root->cen.x - r/2, root->cen.y - r/2, root->cen.z + r/2); break;
    case 3:
      childcen[i] = Point3D(root->cen.x + r/2, root->cen.y - r/2, root->cen.z + r/2); break;
    case 4:
      childcen[i] = Point3D(root->cen.x + r/2, root->cen.y + r/2, root->cen.z - r/2); break;
    case 5:
      childcen[i] = Point3D(root->cen.x - r/2, root->cen.y + r/2, root->cen.z - r/2); break;
    case 6:
      childcen[i] = Point3D(root->cen.x - r/2, root->cen.y - r/2, root->cen.z - r/2); break;
    case 7:
      childcen[i] = Point3D(root->cen.x + r/2, root->cen.y - r/2, root->cen.z - r/2); break;
    default:
      break;
    }
    root->child[i] = new treenode(root,childcen[i], false);
  }
  inheritself(root, -1);//we creat a init submap and there is no point, that oct=-1
}

bool octree::insert(Point3D p)
{
  if(  fabs(p.x-refx)>0.5*pow(2,maxdeep-1)*minunit
     ||fabs(p.y-refy)>0.5*pow(2,maxdeep-1)*minunit
     ||fabs(p.z-refz)>0.5*pow(2,maxdeep-1)*minunit)
  {//out of the range
    return false;
  }
  else
  {
    creattree(root, p);
    return true;
  }
}
void octree::testinsert(Point3D p)
{
  if(p.x > maxx)maxx = p.x;
  if(p.x < minx)minx = p.x;
  if(p.y > maxy)maxy = p.y;
  if(p.y < miny)miny = p.y;
  if(p.z > maxz)maxz = p.z;
  if(p.z < minz)minz = p.z;
  geometry_msgs::Point32 sp;
  sp.x=p.x;
  sp.y=p.y;
  sp.z=p.z;
  tpMp.points.push_back(sp);
  if(insert(p)==false)
  {
    rebuildtree();
  }
}
void octree::inserts(const sensor_msgs::PointCloud::ConstPtr &msg)
{
  for(int i=0;i<msg->points.size();i++)
  {
#ifdef vins_mono
    //if(msg->points[i].z>4.0||msg->points[i].z<-1.0)continue;

    if(msg->points[i].x > maxx)maxx = msg->points[i].x;
    if(msg->points[i].x < minx)minx = msg->points[i].x;
    if(msg->points[i].y > maxy)maxy = msg->points[i].y;
    if(msg->points[i].y < miny)miny = msg->points[i].y;
    if(msg->points[i].z > maxz)maxz = msg->points[i].z;
    if(msg->points[i].z < minz)minz = msg->points[i].z;
#endif
#ifdef vins_fusion
    //if(msg->points[i].y>4.0||msg->points[i].y<-1.0)continue;
    if(msg->points[i].x > maxx)maxx = msg->points[i].x;
    if(msg->points[i].x < minx)minx = msg->points[i].x;
    if(msg->points[i].z > maxy)maxy = msg->points[i].z;
    if(msg->points[i].z < miny)miny = msg->points[i].z;
    if(msg->points[i].y > maxz)maxz = msg->points[i].y;
    if(msg->points[i].y < minz)minz = msg->points[i].y;
#endif
    tpMp.points.push_back(msg->points[i]);
  }
  for(int i=0;i<msg->points.size();i++)
  {
#ifdef vins_mono
    //if(msg->points[i].z>4.0||msg->points[i].z<-1.0)continue;
    if(insert(Point2D(msg->points[i].x,msg->points[i].y,msg->points[i].z))==false)
    {
      rebuildtree();
      break;
    }
#endif
#ifdef vins_fusion
    //if(msg->points[i].y>4.0||msg->points[i].y<-1.0)continue;
    if(insert(Point3D(msg->points[i].x,msg->points[i].y,msg->points[i].z))==false)
    {
      rebuildtree();
      break;
    }
#endif
  }

}

void octree::creattree(treenode* now, Point3D p)
{
  //ROS_INFO("creat tree");
	if (now->nowdeep == maxdeep)
	{
    //ROS_INFO("max tree");
		now->full = true;
		maxdeepinherit(now);
		maintainfull(now->father);
		return;
	}
  int octant = now->cen.getOctantId(p);
	double r = getr(now->nowdeep);
  Point3D childcen[8];
	if (now->child[0] != nullptr)//not first time
	{
		if (now->full==true)// if all child is exist=true, this point is useless.
		{
			return;
		}
		else
		{
			now->child[octant]->exist = true;
		}
	}
	else //first time
	{
    for (int i = 0; i < 8; i++)
		{
			switch (i)
			{
      case 0:
        childcen[i] = Point3D(now->cen.x + r/2, now->cen.y + r/2, now->cen.z + r/2); break;
      case 1:
        childcen[i] = Point3D(now->cen.x - r/2, now->cen.y + r/2, now->cen.z + r/2); break;
      case 2:
        childcen[i] = Point3D(now->cen.x - r/2, now->cen.y - r/2, now->cen.z + r/2); break;
      case 3:
        childcen[i] = Point3D(now->cen.x + r/2, now->cen.y - r/2, now->cen.z + r/2); break;
      case 4:
        childcen[i] = Point3D(now->cen.x + r/2, now->cen.y + r/2, now->cen.z - r/2); break;
      case 5:
        childcen[i] = Point3D(now->cen.x - r/2, now->cen.y + r/2, now->cen.z - r/2); break;
      case 6:
        childcen[i] = Point3D(now->cen.x - r/2, now->cen.y - r/2, now->cen.z - r/2); break;
      case 7:
        childcen[i] = Point3D(now->cen.x + r/2, now->cen.y - r/2, now->cen.z - r/2); break;
			default:
				break;
			}
      if (i != octant)//creat all 8 subtreenode
			{
        now->child[i] = new treenode(now, childcen[i], false);
			}
			else
			{
        now->child[octant] = new treenode(now, childcen[octant], true);
			}
		}
		inheritself(now, octant);
	}
	inheritfather(now, octant);
  creattree(now->child[octant], p);
}
void octree::maintainfull(treenode* now)
{
	int cnt = 0;
  for (int i = 0; i < 8; i++)
	{
		if (now->child[i]->full == true)
		{
			cnt++;
		}
	}
  if (cnt >= 8)
	{
		now->full = true;
		maintainfull(now->father);
	}
}

double octree::getr(int nowdeep)
{
	return std::pow(2, (maxdeep - nowdeep))*(minunit / 2);
}
bool octree::inViewBound(Point3D p)
{
  if(  inBound(p.x,boundx,-boundx)
     &&inBound(p.y,boundy,-boundy)
     &&inBound(p.z,boundz,-boundz))
  {
    return true;
  }
  else return false;
}
bool octree::inBound(double x,double up,double down)
{
  if(x<up&&x>down)return true;
  else            return false;
}

treenode* octree::searchpoint(Point3D p)
{
	return searchpoint(root, p);
}
treenode* octree::searchpoint(treenode* now, Point3D p)
{
	//std::cout << now->cen.x << "--,--" << now->cen.y << std::endl;
	double r = getr(now->nowdeep);
	if (now->exist == false)
	{
		return now;
	}
	if (now->nowdeep == maxdeep)
	{
		if (now->innode(p, now->nowdeep))
			return now;
		else
			return nullptr;
	}
  for (size_t i = 0; i < 8; i++)
	{
		if (now->child[i]->innode(p, getr(now->child[i]->nowdeep)))
		{
			return searchpoint(now->child[i], p);
		}
	}
	return nullptr;
}

void octree::cleartree(treenode* now)
{
  if (now->nowdeep == maxdeep)
  {
    now->clear();
    return;
  }
  if (now->child[0] != nullptr)//not first time
  {
    for (int i = 0; i < 8; i++)
    {
      cleartree(now->child[i]);
    }
  }
  now->clear();
}
void octree::killtree(treenode *now)
{
  if (now->nowdeep == maxdeep)
  {
    now->kill();
    return;
  }
  if (now->child[0] != nullptr)//not first time
  {
    for (int i = 0; i < 8; i++)
    {
      killtree(now->child[i]);
    }
  }
  now->kill();
}
void octree::rebuildtree()
{
  ROS_INFO("rebuild");
  int i=3;
  killtree(root);
  double range=std::max(maxz-minz,std::max(maxy-miny,maxx-minx));
  for(i=3;i<=15;i++)
  {
    if(pow(2.0,i-1)*minunit>range)
    {
      this->maxdeep=i;
      break;
    }
  }
  if(i==15)
  {
    ROS_WARN("some point is too far!!!");
    this->maxdeep=i;
  }
  this->refx = (maxx+minx)/2.0;
  this->refy = (maxy+miny)/2.0;
  this->refz = (maxz+minz)/2.0;
  this->root = nullptr;
  initroot();
  for(int i=0;i<tpMp.points.size();i++)
  {
#ifdef vins_mono
    creattree(root,Point3D(tpMp.points[i].x,tpMp.points[i].y,tpMp.points[i].z));
#endif
#ifdef vins_fusion
    creattree(root,Point3D(tpMp.points[i].x,tpMp.points[i].y,tpMp.points[i].z));
#endif
  }
  //visprintmap(root);
  ROS_INFO("deep:%d",maxdeep);
  ROS_INFO("REF:%lf-%lf",refx,refy);
  ROS_INFO("pointinfo:%lf-%lf-%lf-%lf",maxx,maxy,minx,miny);
}

