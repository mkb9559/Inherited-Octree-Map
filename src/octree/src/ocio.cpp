#include "octree.h"

void octree::printpoints()
{
	FILE *ff;
	ff=fopen("pts.txt", "w");
  std::cout << "print points start!" << std::endl;
  visprintpoints(this->root);

	fclose(ff);
  std::cout << "print points ok!" << std::endl;

}
void octree::visprintpoints(treenode* now)
{
	if (now->exist == false) return;
	if (now->nowdeep == maxdeep)
	{
		//std::cout << now->cen.x << " , " << now->cen.y << std::endl;
		return;
	}
  for (int i = 0; i < 8; i++)
	{
    visprintpoints(now->child[i]);
	}

}
void octree::printmap()
{
	FILE *ff,*ffway;
	ff=fopen( "map4.txt", "w");
	ffway=fopen( "edge.txt", "w");
	std::cout << "print map4 start!" << std::endl;
  visprintmap(this->root);
  visprintedge(this->root);
	fclose(ff);
	fclose(ffway);
	std::cout << "print map4 ok!" << std::endl;

}
void octree::visprintmap(treenode* now)
{
  std::cout<<"vis "<<now->cen.x << " , " << now->cen.y <<":"<<now->full<<":"<<now->exist<<std::endl;
  if (now->full == true)
  {
     std::cout << "is:"<<now->cen.x << " , " << now->cen.y << "," << now->exist<<","<<now->full<< std::endl;
     return;
  }
  if (now->exist == false)
  {
    std::cout << "no:"<<now->cen.x << " , " << now->cen.y << "," << now->exist<<","<<now->full<< std::endl;
    return;
  }
  if (now->nowdeep == maxdeep)
  {
    ROS_ERROR("error node info in vismap");
    return;
  }
  for (int i = 0; i < 4; i++)
  {
    visprintmap(now->child[i]);
  }
}
void octree::visprintedge(treenode* now)
{
  if (now->exist == false)
  {
    for (std::set<treenode*>::iterator it = now->eg.begin(); it != now->eg.end(); it++)
    {
      std::cout << now->cen.x << "," << now->cen.y<<","<<now->cen.z << " -> " << (*it)->cen.x << "," << (*it)->cen.y <<","<<(*it)->cen.z<< std::endl;
    }
    return;
  }
  if (now->nowdeep == maxdeep) return;
  for (int i = 0; i < 8; i++)
  {
    visprintedge(now->child[i]);
  }
}
void octree::readpoints()
{
	FILE *ff;
  double tx, ty, tz;
	ff=fopen( "points.txt", "r");
	std::cout << "read points start!" << std::endl;
	while (!feof(ff))
	{
		fscanf(ff, "%lf", &tx);
		fscanf(ff, "%lf", &ty);
    fscanf(ff, "%lf", &tz);
    this->insert(Point3D(tx, ty,tz));
    std::cout << "Load:"<<tx<< " , " << ty<<" , "<<tz<< std::endl;
	}
	fclose(ff);
	std::cout << "read points ok!" << std::endl;
}
void octree::visdrawmap(treenode *now)
{

  if (now->exist == false)
  {
    //if(inViewBound(now->cen))emptyRectf(now->cen.x,now->cen.y,now->cen.z,getr(now->nowdeep));

    return;
  }
  if (now->nowdeep == maxdeep)
  {
    if(inViewBound(now->cen))fullRectf(now->cen.x,now->cen.y,now->cen.z,getr(now->nowdeep));
    return;
  }
  for (int i = 0; i < 8; i++)
  {
    visdrawmap(now->child[i]);
  }
}
void octree::visdrawlink(treenode *now)
{
  if(now->exist==false)
  {
    for (std::set<treenode*>::iterator it = now->eg.begin(); it != now->eg.end(); it++)
    {
      if(inViewBound(now->cen)&&inViewBound((*it)->cen))
      {
        drawlink(now,*it);
      }
    }
    return;
  }
  if (now->nowdeep == maxdeep)
  {
    return;
  }
  for (int i = 0; i < 8; i++)
  {
    visdrawlink(now->child[i]);
  }
}
void octree::drawlink(treenode *a, treenode *b)
{

  glLineWidth(2);
  glColor3f(0.0f,1.0f,0.0f);
  glBegin(GL_LINES);
  glVertex3f(a->cen.x,a->cen.y,a->cen.z);
  glVertex3f(b->cen.x,b->cen.y,b->cen.z);
  glEnd();

}
void octree::myRectf(float x1, float y1, float x2, float y2)
{
  glColor3f(1.0,0.0,0.0);
  glBegin(GL_LINE_LOOP);
  glVertex3f(x1,y1,0);
  glVertex3f(x1,y2,0);
  glVertex3f(x2,y2,0);
  glVertex3f(x2,y1,0);
  glVertex3f(x1,y1,0);
  glEnd();
}
void octree::emptyRectf(float cx,float cy,float cz,float r)
{
  float ux = cx+r;
  float dx = cx-r;
  float uy = cy+r;
  float dy = cy-r;
  float uz = cz+r;
  float dz = cz-r;
  //glColor4f(1.0,0.8,0.9,0.5);
  glColor4f(0.0,0.0,1.0,0.5);
  glBegin(GL_LINES);
  glVertex3f(ux,uy,dz);
  glVertex3f(dx,uy,dz);
  glVertex3f(ux,dy,dz);
  glVertex3f(dx,dy,dz);
  glVertex3f(ux,dy,uz);
  glVertex3f(dx,dy,uz);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glVertex3f(ux,uy,uz);
  glVertex3f(ux,uy,dz);
  glVertex3f(ux,dy,dz);
  glVertex3f(ux,dy,uz);
  glVertex3f(ux,uy,uz);
  glVertex3f(dx,uy,uz);
  glVertex3f(dx,uy,dz);
  glVertex3f(dx,dy,dz);
  glVertex3f(dx,dy,uz);
  glVertex3f(dx,uy,uz);
  glEnd();
  /*
  glBegin(GL_LINES);
  glVertex3f( 20, 20,-20);
  glVertex3f(-20, 20,-20);
  glVertex3f( 20,-20,-20);
  glVertex3f(-20,-20,-20);
  glVertex3f( 20,-20, 20);
  glVertex3f(-20,-20, 20);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glVertex3f( 20, 20, 20);
  glVertex3f( 20, 20,-20);
  glVertex3f( 20,-20,-20);
  glVertex3f( 20,-20, 20);
  glVertex3f( 20, 20, 20);
  glVertex3f(-20, 20, 20);
  glVertex3f(-20, 20,-20);
  glVertex3f(-20,-20,-20);
  glVertex3f(-20,-20, 20);
  glVertex3f(-20, 20, 20);
  glEnd();
  */

}

void octree::fullRectf(float cx,float cy,float cz,float r)
{
  float ux = cx+r;
  float dx = cx-r;
  float uy = cy+r;
  float dy = cy-r;
  float uz = cz+r;
  float dz = cz-r;
  //glColor4f(1.0,0.8,0.9,0.5);
  glColor4f(0.0,0.0,0.5,0.5);
  glBegin(GL_QUADS);//z
  glVertex3f(ux,uy,dz);
  glVertex3f(dx,uy,dz);
  glVertex3f(dx,dy,dz);
  glVertex3f(ux,dy,dz);
  glEnd();
  glBegin(GL_QUADS);
  glVertex3f(ux,uy,uz);
  glVertex3f(dx,uy,uz);
  glVertex3f(dx,dy,uz);
  glVertex3f(ux,dy,uz);
  glEnd();

  glBegin(GL_QUADS);//y
  glVertex3f(ux,dy,uz);
  glVertex3f(dx,dy,uz);
  glVertex3f(dx,dy,dz);
  glVertex3f(ux,dy,dz);
  glEnd();
  glBegin(GL_QUADS);
  glVertex3f(ux,uy,uz);
  glVertex3f(dx,uy,uz);
  glVertex3f(dx,uy,dz);
  glVertex3f(ux,uy,dz);
  glEnd();

  glBegin(GL_QUADS);//x
  glVertex3f(dx,dy,uz);
  glVertex3f(dx,dy,dz);
  glVertex3f(dx,uy,dz);
  glVertex3f(dx,uy,uz);
  glEnd();
  glBegin(GL_QUADS);
  glVertex3f(ux,dy,uz);
  glVertex3f(ux,dy,dz);
  glVertex3f(ux,uy,dz);
  glVertex3f(ux,uy,uz);
  glEnd();




}
