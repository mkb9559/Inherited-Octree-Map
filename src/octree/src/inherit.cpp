#include "octree.h"

void octree::inheritself(treenode* now, int octant)
{

  for (int i = 0; i < 8; i++)
	{
    if(now->nowdeep>=maxdeep)
    {
      ROS_ERROR("error deep in inheritself");
      break;
    }
		switch (i)
		{
    case 0://1->2,4,5
      now->child[i]->eg.insert(now->child[1]);
      now->child[i]->eg.insert(now->child[3]);
      now->child[i]->eg.insert(now->child[4]);
			break;
    case 1://2->1,3,6
      now->child[i]->eg.insert(now->child[0]);
      now->child[i]->eg.insert(now->child[2]);
      now->child[i]->eg.insert(now->child[5]);
			break;
    case 2://3->2,4,7
      now->child[i]->eg.insert(now->child[1]);
      now->child[i]->eg.insert(now->child[3]);
      now->child[i]->eg.insert(now->child[6]);
			break;
    case 3://4->3,1,8
      now->child[i]->eg.insert(now->child[2]);
      now->child[i]->eg.insert(now->child[0]);
      now->child[i]->eg.insert(now->child[7]);
			break;

    case 4://5->8,6,1
      now->child[i]->eg.insert(now->child[7]);
      now->child[i]->eg.insert(now->child[5]);
      now->child[i]->eg.insert(now->child[0]);
      break;
    case 5://6->5,7,2
      now->child[i]->eg.insert(now->child[4]);
      now->child[i]->eg.insert(now->child[6]);
      now->child[i]->eg.insert(now->child[1]);
      break;
    case 6://7->6,8,3
      now->child[i]->eg.insert(now->child[5]);
      now->child[i]->eg.insert(now->child[7]);
      now->child[i]->eg.insert(now->child[2]);
      break;
    case 7://8->7,5,4
      now->child[i]->eg.insert(now->child[6]);
      now->child[i]->eg.insert(now->child[4]);
      now->child[i]->eg.insert(now->child[3]);
      break;
		default:
      ROS_ERROR("error id in inheritself");
			break;
		}

  }
}
void octree::inheritfather(treenode* now, int octant)
{
  /*
   chooce big treenode, they can be inherit full.
   for small tree node, divide them in eight octant
   */

	if (now->eg.empty())return;

  for (std::set<treenode*>::iterator it = now->eg.begin(); it != now->eg.end(); it++)
	{
		//std::cout << now->eg.size()<<std::endl;
    if((*it)->nowdeep <= now->nowdeep)
    {
      switch (getdir(*it,now))
      {
      case 1: //x+ ->  1 4 5 8
        now->child[0]->eg.insert(*it);(*it)->eg.insert(now->child[0]);
        now->child[3]->eg.insert(*it);(*it)->eg.insert(now->child[3]);
        now->child[4]->eg.insert(*it);(*it)->eg.insert(now->child[4]);
        now->child[7]->eg.insert(*it);(*it)->eg.insert(now->child[7]);
        break;
      case 2: //x- ->2 3 6 7
        now->child[1]->eg.insert(*it);(*it)->eg.insert(now->child[1]);
        now->child[2]->eg.insert(*it);(*it)->eg.insert(now->child[2]);
        now->child[5]->eg.insert(*it);(*it)->eg.insert(now->child[5]);
        now->child[6]->eg.insert(*it);(*it)->eg.insert(now->child[6]);
        break;
      case 3: //y+ ->1 2 5 6
        now->child[0]->eg.insert(*it);(*it)->eg.insert(now->child[0]);
        now->child[1]->eg.insert(*it);(*it)->eg.insert(now->child[1]);
        now->child[4]->eg.insert(*it);(*it)->eg.insert(now->child[4]);
        now->child[5]->eg.insert(*it);(*it)->eg.insert(now->child[5]);
        break;
      case 4: //y- ->3 4 7 8
        now->child[2]->eg.insert(*it);(*it)->eg.insert(now->child[2]);
        now->child[3]->eg.insert(*it);(*it)->eg.insert(now->child[3]);
        now->child[6]->eg.insert(*it);(*it)->eg.insert(now->child[6]);
        now->child[7]->eg.insert(*it);(*it)->eg.insert(now->child[7]);
        break;
      case 5: //z+ ->1 2 3 4
        now->child[0]->eg.insert(*it);(*it)->eg.insert(now->child[0]);
        now->child[1]->eg.insert(*it);(*it)->eg.insert(now->child[1]);
        now->child[2]->eg.insert(*it);(*it)->eg.insert(now->child[2]);
        now->child[3]->eg.insert(*it);(*it)->eg.insert(now->child[3]);
        break;
      case 6: //z- ->5 6 7 8
        now->child[4]->eg.insert(*it);(*it)->eg.insert(now->child[4]);
        now->child[5]->eg.insert(*it);(*it)->eg.insert(now->child[5]);
        now->child[6]->eg.insert(*it);(*it)->eg.insert(now->child[6]);
        now->child[7]->eg.insert(*it);(*it)->eg.insert(now->child[7]);
        break;
      default:
        ROS_ERROR("inheritfather error in father I");
        break;
      }
    }
    else
    {
      switch(now->cen.getOctantId((*it)->cen))
      {
      case 0:
        now->child[0]->eg.insert(*it);(*it)->eg.insert(now->child[0]);
        break;
      case 1:
        now->child[1]->eg.insert(*it);(*it)->eg.insert(now->child[1]);
        break;
      case 2:
        now->child[2]->eg.insert(*it);(*it)->eg.insert(now->child[2]);
        break;
      case 3:
        now->child[3]->eg.insert(*it);(*it)->eg.insert(now->child[3]);
        break;
      case 4:
        now->child[4]->eg.insert(*it);(*it)->eg.insert(now->child[4]);
        break;
      case 5:
        now->child[5]->eg.insert(*it);(*it)->eg.insert(now->child[5]);
        break;
      case 6:
        now->child[6]->eg.insert(*it);(*it)->eg.insert(now->child[6]);
        break;
      case 7:
        now->child[7]->eg.insert(*it);(*it)->eg.insert(now->child[7]);
        break;
      default:
        ROS_ERROR("inheritfather error in father II");
        break;
      }
    }
    maintainmap(now, *it);
	}
  now->eg.clear();
}

void octree::maintainmap(treenode* s, treenode* e)
{
  if(e->eg.erase(s)==false)// other treenode remove the edge to my treenode
  {

    ROS_ERROR("error remove edge 1");
  }

}
void octree::maxdeepinherit(treenode* now)
{
  for (std::set<treenode*>::iterator it = now->eg.begin(); it != now->eg.end(); it++)
	{
    maintainmap(now, *it);
	}
	now->eg.clear();
}
int octree::getdir(treenode *s, treenode *e)
{
  //1 2 3 4 5 6 , x+ x- y+ y- z+ z-
  double xx=s->cen.x - e->cen.x;
  double yy=s->cen.y - e->cen.y;
  double zz=s->cen.z - e->cen.z;
  if(fabs(xx)>fabs(yy)&&fabs(xx)>fabs(zz))
  {
    return xx>0?1:2;
  }
  if(fabs(yy)>fabs(xx)&&fabs(yy)>fabs(zz))
  {
    return yy>0?3:4;
  }
  if(fabs(zz)>fabs(yy)&&fabs(zz)>fabs(xx))
  {
    return zz>0?5:6;
  }
  return -1;
}

