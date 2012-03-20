/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 

   @author Karsten Bohlmann
   @date   3/20/2012
   @file   icrtest.cpp

*/ 
#include <math.h>
#include <stdio.h>
#include <eigen3/Eigen/Core>

using namespace Eigen;


/**
  calculate instantaneous center of rotation given three points of a circle
  @param p0 first point on circle, e.g. goal point or point to steer clearenv()
  @param p1 first point on robot, e.g. front (left or right) corner of robot
  @param p2 second point on robot, e.g. rear (left or right) corner of robot
  @param wheelbase distance from front to rear axle
  @param[out] m result coordinates of icr
  @param[out] r resulting radius of motion circle
  @param[out] deltaf steer_front_rad resulting steering angle to steer clear or reach p0
  */

void calcIcr(const Vector2d &p0, const Vector2d &p1,
                           const Vector2d&p2,double wheelbase,Vector2d &m, double &r, double &deltaf)
{
  // calc circle through 3 points
  // http://2000clicks.com/mathhelp/GeometryConicSectionCircleEquationGivenThreePoints.aspx
  double a,b,c,d,e,f;
  a=p0.x();
  b=p0.y();
  c=p1.x();
  d=p1.y();
  e=p2.x();
  f=p2.y();
  m.x()= (0.5)*((a*a+b*b)*(f-d) + (c*c+d*d)*(b-f) + (e*e+f)*(d-b)) / (a*(f-d)+c*(b-f)+e*(d-b));
  m.y()= (0.5)*((a*a+b*b)*(e-c) + (c*c+d*d)*(a-e) + (e*e+f*f)*(c-a)) / (b*(e-c)+d*(a-e)+f*(c-a));


  r=(p0-m).norm();
  deltaf=atan(wheelbase/r);
  if (m.y()<0) {
    // right curve
    deltaf=fabs(deltaf)*-1.0;
  } else {
    deltaf=fabs(deltaf);
  }
}

int main(int argc, char **argv)
{
  Vector2d obstacle;
  Vector2d front_corner,rear_corner;
  double wb=2;
  obstacle << 3,-0.5;
  front_corner << 1, 0.5;
  rear_corner  << -1,0.5;
  Vector2d icr;
  double radius;
  double steer_rad;
  calcIcr(obstacle,front_corner,rear_corner,wb,icr,radius,steer_rad);
  printf("steerangle=%fdeg radius=%f center=(%f,%f)\n",steer_rad*180.0/M_PI,radius,icr.x(),icr.y());
  return 0;
}
