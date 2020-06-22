/*
 * Autor: Chang, Xu
 * This file is mainly focus on quaduped robot gait.
*/

#ifndef GAIT_HH
#define GAIT_HH
#include <Eigen/Dense>
/*
 * The GaitGenerator class wants to give advise about which limb should be in swing or support state and the target position in body frame when swing .
 *
 * Naming:
 * QRparent : GaitGenrator need a parent pointer to a QuadRobot to get useful information.
 * globaltime : This variable records passing time.
 * starttime: Records when the robot start gait plan.
 * supporttime: Support phase duration.
 * timestep: As its means.
 * liftheight: Foot endpoint lift height in direction Z. (>0)
 * initialposition: Initial reference position.
 * currentposition: current reference position.
 * targettposition: target reference position.
 * support: This vector is used to indicate whether a limb in a support state or not. 1: yes, 0: no.
 *
 * How to use:
 * 1. Call function Init to determine gait, support time etc.
 * 2. Call function Step in proper context function to perform gait plan.
 * 3. Call function GiveAdvise to get plan we need.
*/

class QuadRobot;
class GaitGenerator
{
   QuadRobot* qrparent;
   ulong globaltime = 0;
   ulong starttime = 0;
   ulong supporttime = 0;
   ulong swingtime = 0;
   ulong fourstandtime = 150;
   ulong flytime = 0;
   double timestep = 0.001;
   double liftheight = 0;
   Eigen::Vector3d initialposition;
   Eigen::Vector3d targetposition[4];
   Eigen::Vector4i supportPlan;
   Eigen::Vector4i supportFact;
   Eigen::Vector4i supportAdvise;
public:
   enum GAIT
   {
      STATIC = 0,
      WALK = 1,
      TROT = 2,
       PRONK = 3,
      BOUND = 4
   }gait;

private:
   inline void reset() { globaltime = 0, supportPlan << 1,1,1,1; }
   void GaitStop();
   void GaitWalk(const Eigen::Vector3d& vc);
   void GaitTrot(const Eigen::Vector3d& vc);
   void GaitPronk(const Eigen::Vector3d& vc);
   void GaitBound(const Eigen::Vector3d& vc);

public:
   GaitGenerator(QuadRobot* parent);
   void init(GAIT gaitchoose, const Eigen::Vector3d& initposition, double lifthei, ulong supporT, ulong swingT);
   Eigen::Vector3d* Step(const Eigen::Vector3d& vc, const Eigen::Vector4i& supportcase);
   Eigen::Vector4i GiveAdvise();
   ulong GetSupportTime();
   ulong GetSwingTime();
   inline void SetFoursStandTime(ulong st){ fourstandtime = st; }
};

#endif
