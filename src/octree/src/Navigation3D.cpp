#include "Navigation3D.h"


Navigation3D::Navigation3D()
{
  mytree        = nullptr;
#ifdef Viewer
  viewx         = 0;
  viewy         = 0;
  viewz         = -200;
  viewf         = 500;
  viewPointSize = 5;
  this->mpview  = new std::thread(&Navigation3D::Drawer,this);
#endif


}



Navigation3D::~Navigation3D()
{
}

Navigation3D::Navigation3D(Point3D s, Point3D e)
{
  init(s,e);
}
void Navigation3D::init(Point3D s, Point3D e)
{
  this->aim = e;
  this->start = s;
  settree();
}
void Navigation3D::settree(double minnuit, int maxdeep, double refx, double refy)
{
  mytree = new octree(minnuit);
}
void Navigation3D::testinsert(double x, double y, double z)
{
  std::unique_lock<std::mutex> lock(mlock);
  mytree->testinsert(Point3D(x,y,z));
}
void Navigation3D::getMPs(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  std::unique_lock<std::mutex> lock(mlock);
  ROS_INFO("get mps: %d",(int)msg->points.size());
  mytree->inserts(msg);
  /*
  SPFA();
  generateway(mytree->searchpoint(aim));
  outputway();
  */

  //plot
}
void Navigation3D::SPFA()
{
  std::priority_queue<Point3DNiv> q;
  treenode* s = this->mytree->searchpoint(start);
  treenode* e = this->mytree->searchpoint(aim);
  Point3DNiv tp, rp;
  double costadd = 0;
  if (s != nullptr){}
  else
  {
    std::cout << "Nevigation error: start point uncorrect!" << std::endl;
    return;
  }
  if (e != nullptr){}
  else
  {
    std::cout << "Nevigation error: aim point uncorrect!" << std::endl;
    return;
  }
  std::cout << "Navigation start!" << std::endl;

  q.push(Point3DNiv(s, aim, this->mytree->minunit*0.1));
  s->lowestcost = this->mytree->minunit*0.1;

  while (!q.empty())
  {
    tp = q.top();
    //std::cout << tp.tree->cen.x << "--,--" << tp.tree->cen.y << std::endl;
    q.pop();
    if (tp.tree == e)
    {
      std::cout << "Navigation Success!" << std::endl;
      generateway(tp.tree);
      break;
    }
    for (std::set<treenode*>::iterator it = tp.tree->eg.begin(); it != tp.tree->eg.end(); it++)
    {
      costadd = Point3D((*it)->cen-tp.tree->cen).length();
      if ((*it)->lowestcost<0			//first time
        || (*it)->lowestcost > costadd+tp.cost)		//batter way
      {
        (*it)->lowestcost = costadd + tp.cost;
        q.push(Point3DNiv(*it, aim));
        (*it)->wayfrom = tp.tree;
      }
    }
  }
}
void Navigation3D::generateway(treenode* e)
{
  treenode* tp;
  tp = e;
  while (1)
  {
    std::cout << tp->cen.x << " , " << tp->cen.y << std::endl;
    std::cout << tp->lowestcost << std::endl;
    if (tp->wayfrom != nullptr)
    {
      way.push(tp->cen);
      tp = tp->wayfrom;
    }
    else
    {
      way.push(tp->cen);
      break;
    }
  }
}
void Navigation3D::outputway()
{
  FILE *ff;
  Point3D tp;
  ff=fopen( "path.txt", "w");
  std::cout << "print path start!" << std::endl;
  while (!way.empty())
  {
    tp = way.top();
    fprintf(ff, "%.3lf,%.3lf\n", tp.x, tp.y);
    std::cout << tp.x << " , " << tp.y << std::endl;
    way.pop();
  }
  fclose(ff);
  std::cout << "print path ok!" << std::endl;
}
void Navigation3D::printfMPs(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  FILE *ff;
  ff=fopen( "MPs.txt", "w");
  std::cout << "print Mps start!" << std::endl;
  for(int i=0;i<msg->points.size();i++)
  {
    fprintf(ff, "%.3lf,%.3lf,%.3lf,\n", msg->points[i].x, msg->points[i].y,msg->points[i].z);
  }
  fclose(ff);
  std::cout << "print MPs ok!" << std::endl;
}

#ifdef Viewer
void Navigation3D::Drawer()
{
  pangolin::CreateWindowAndBind("quadtree",1024,768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(1024,768,viewf,viewf,512,398,0.1,1000),
        pangolin::ModelViewLookAt(viewx,viewy,viewz,0,0,0,0,-1.0,0)
        );
  pangolin::View& d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0,pangolin::Attach::Pix(175),1.0,-1024.0f/768.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));
  /*
  glEnable(GL_BLEND);
  glBlendFunc(GL_ONE,GL_ZERO);
*/
  while(true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    std::unique_lock<std::mutex> lock(mlock);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f,1.0f,1.0f,1.0f);

    //glColor3f(0.0f,1.0f,0.0f);
    //glRectf(-10.0f,-10.0f,10.0f,10.0f);

    //xyz
    glLineWidth(2);
    glColor3f(1.0f,0.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(20,0,0);
    glEnd();
    glColor3f(0.0f,0.0f,1.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(0,100,0);
    glEnd();
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(0,0,100);
    glEnd();

    glColor3f(1.0f,0.0f,0.0f);
    if(mytree!=nullptr)
    {
      //mytree->visprintedge(mytree->root);
      mytree->visdrawmap(mytree->root);
      mytree->visdrawlink(mytree->root);
    }

/*
    glPointSize(viewPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,1.0);
    for(int i=0;i<mytree->tpMp.points.size();i++)
    {
      glVertex3f(mytree->tpMp.points[i].x,mytree->tpMp.points[i].y,mytree->tpMp.points[i].z);
      //std::cout<<"drow point:"<<mytree->tpMp.points[i].x<<","<<mytree->tpMp.points[i].y<<","<<mytree->tpMp.points[i].z<<std::endl;
    }
    glEnd();
    */


/*
    glColor3f(1.0f,0.0f,0.0f);
    if(mytree!=nullptr)
    {
      mytree->visdrawmap(mytree->root);
    }
    */

    glPopMatrix();
    pangolin::FinishFrame();
    ROS_INFO("end draw:%d",mytree->tpMp.points.size());
  }

}
#endif
