#include "ARM6Kinematics.h"

//Dummy IK library for n3bot series robots
//todo: move wheel IK here?



bool show_debug=false;

double mod_angle(double q){
  if (q>PI) return q-2*PI;
  if (q<-PI) return q+2*PI;
  return q;
}

Transform ARM6_kinematics_forward_arm(const double *q){
  Transform t;
  t = t
    .rotateZ(q[0])
    .translateZ(shoulderOffsetZ)
    .translateY(shoulderOffsetY)
    .rotateY(q[1])
    .translateZ(lowerArmLength)
    .rotateY(q[2])
    .translateZ(upperArmLength)
    .rotateY(q[3])
    .rotateX(q[4])
    .translateX(wristLength)
//now Y is forward and x is up in the EE frame
    .rotateZ(PI/2)
    .rotateX(q[5]+PI/2)
//now +x is forward and +z is up in the EE frame
    .translate(handLength+toolOffsetX,0,toolOffsetZ);
  return t;
}

Transform ARM6_kinematics_forward_wrist(const double *q){
  Transform t;
  t = t
    .rotateZ(q[0])
    .translateZ(shoulderOffsetZ)
    .translateY(shoulderOffsetY)
    .rotateY(q[1])
    .translateZ(lowerArmLength)
    .rotateY(q[2])
    .translateZ(upperArmLength)
    .rotateY(q[3])
    .rotateX(q[4])
    .translateX(wristLength)
//now Y is forward and x is up in the EE frame
    .rotateZ(PI/2)
    .rotateX(q[5]+PI/2);
  return t;
}



//or rotateY(q[5]+pi/2) trans(0,handX,handZ) * rotZ(PI/2)




std::vector<double>
ARM6_kinematics_inverse_arm(Transform trArm, const double *qOrg,bool wristRoll1){
  double handOffsetX = handLength+toolOffsetX;
  //I screwed up IK somewhere.. so let's use a hack around
  //Z is forward, Y is up
  trArm=trArm.translate(0,0,-toolOffsetZ);

  Transform t1;
  double shoulderYaw, shoulderPitch, elbowPitch, wristPitch, wristRoll, wristPitch2;

  t1=t1.translateZ(-shoulderOffsetZ)*trArm;
  double xEE[3]; t1.apply0(xEE); //shoulder yaw joint to end effector position

  t1=t1.translate(-handOffsetX,0,0);
  double xWrist[3];t1.apply0(xWrist); //shoulder yaw joint to wrist  position
  // printf("xWrist:%.3f %.3f\n",xWrist[0],xWrist[1]);

  double shoulder_wrist_2D=sqrt(xWrist[0]*xWrist[0]+xWrist[1]*xWrist[1]);
  double yaw_offset = asin(shoulderOffsetY/shoulder_wrist_2D);
  shoulderYaw = atan2(xWrist[1],xWrist[0])-yaw_offset; //standard position
  //double shoulderYaw2 = shoulderYaw + 2*yaw_offset; //flipped position.. WHICH WE WON'T USE EVER

  double cwr =(-xEE[0]*sin(shoulderYaw)+xEE[1]*cos(shoulderYaw) -shoulderOffsetY) /handOffsetX;
  if(cwr>1.0) cwr=1.0;   if(cwr<-1.0) cwr=-1.0; //nan proof
  wristRoll=acos(cwr); //roll angle 1, gripper toward left
  if (wristRoll1) wristRoll=-wristRoll; //second flipped wristroll position


  if (fabs(wristRoll)<1*PI/180.0){
    //all joints parallel case. we have infinite solution
    //now we kniw shoulderYaw AND wrist Roll (q[0] and q[4])


    //let's make sure that the wrist faces down
    //shoulderPitch + elbowPitch+wristPitch = PI

    Transform t5;
    t5=t5 //this transform is shoulderpitch to the first wrist
      .translateZ(-shoulderOffsetZ)
      .translateY(-shoulderOffsetY)
      .rotateZ(-shoulderYaw)
      *trArm
      .translate(-handOffsetX,0,0)
      .rotateX(-wristPitch2-PI/2)
      .rotateZ(-PI/2)
      .translateX(-wristLength);

    double xShoulderWrist[3]; //shoulder to first wrist
    t5.apply0(xShoulderWrist);
    // printf("shoulderWrist: %.2f %.2f %.2f\n",xShoulderWrist[0],xShoulderWrist[1],xShoulderWrist[2]);

    double shoulderwristdist=sqrt(xShoulderWrist[0]*xShoulderWrist[0]+xShoulderWrist[2]*xShoulderWrist[2]);
    double c_elbow=
      (lowerArmLength*lowerArmLength+upperArmLength*upperArmLength-shoulderwristdist*shoulderwristdist)/
      (2*lowerArmLength*upperArmLength);
    elbowPitch=PI-acos(c_elbow);

    double shoulderwristoffset=atan2(xShoulderWrist[2],xShoulderWrist[0]);
    double c_shoulder=
      (lowerArmLength*lowerArmLength-upperArmLength*upperArmLength+shoulderwristdist*shoulderwristdist)/
      (2*lowerArmLength*shoulderwristdist);
    shoulderPitch=PI/2.0 -acos(c_shoulder)-shoulderwristoffset;

    wristPitch = PI/2-shoulderPitch-elbowPitch;
    wristPitch2 = 0.0;


    // printf("Singularity ");
  }else{

    //wrist pitch2 calculation: T * RotX(-wristPitch-PI/2)*RotZ(-PI/2)*RotX(-wristRoll)*[0 1 0]'  = RotZ(-shoulderYaw)*[0 1 0]'
    Transform t2;
    t2=inv(trArm);
    double tr0[3]={-sin(shoulderYaw),cos(shoulderYaw),0};
    t2.apply(tr0); //trArm-1 * rotZ(shoulderyaw) * [0 1 0]''
    double tr1[3];  t2.apply0(tr1);
    wristPitch2 = -atan2((tr0[1]-tr1[1])/sin(wristRoll), -(tr0[2]-tr1[2])/sin(wristRoll))-PI/2;

    //now we know angle 1,5,6
    Transform t3;
    t3=t3
      .translateY(-shoulderOffsetY)
      .translateZ(-shoulderOffsetZ)
      .rotateZ(-shoulderYaw)
      *trArm
      .translate(-handOffsetX,0,0)
      .rotateX(-wristPitch2-PI/2)
      .rotateZ(-PI/2)
      .translateX(-wristLength)
      .rotateX(-wristRoll);
    //t3 is now rotY(q1)trZ(lowerArmLength)rotY(q2)trZ(upperArmLength)rotY(q3)

    //this transform is for joint 2,3,4
    double xArm[3]; t3.apply0(xArm);
    double armdist = sqrt(xArm[0]*xArm[0]+xArm[1]*xArm[1]+xArm[2]*xArm[2]);
    double aa= lowerArmLength*lowerArmLength + upperArmLength*upperArmLength - armdist*armdist;
    double c_ep = aa/(2.0*lowerArmLength*upperArmLength);
    if (c_ep>1.0) c_ep=1.0;if (c_ep<-1.0) c_ep=-1.0;
    elbowPitch = PI-acos(c_ep); //one of two solutions
    double c_spo=(armdist*armdist + lowerArmLength*lowerArmLength - upperArmLength*upperArmLength)/(2*armdist*lowerArmLength);
    double shoulderPitchOffset = acos(c_spo);
    shoulderPitch = atan2(xArm[0],xArm[2])-shoulderPitchOffset;
    if ((c_spo>1.0)||(c_spo<-1.0)){
      shoulderPitch =qOrg[1]; //use older value if the target is too far away
    }
    // printf("aa:%.3f c_ep:%.3f Elbow:%.3f SPA:%.3f\n",aa,c_ep,elbowPitch*180.0/3.1415,shoulderPitchOffset*180.0/3.1415);
    double allPitch = atan2(t3(0,2),t3(0,0));
    wristPitch = allPitch-shoulderPitch-elbowPitch;
    // printf("Ningularity ");

  }


  std::vector<double> qArm(6);
  qArm[0] = shoulderYaw;
  qArm[1] = shoulderPitch;
  qArm[2] = elbowPitch;
  // qArm[3] = wristPitch;
  qArm[3] = qOrg[3]+mod_angle(wristPitch-qOrg[3]);//to prevent the wrist pitch to rotate 360 deg
  qArm[4] = wristRoll;
  qArm[5] = wristPitch2;


  // printf("SY %.1f SP %.1f EP %.1f WP %.1f WR %.1f WP2 %.1f\n",
  //   qArm[0]*180.0/PI,
  //   qArm[1]*180.0/PI,
  //   qArm[2]*180.0/PI,
  //   qArm[3]*180.0/PI,
  //   qArm[4]*180.0/PI,
  //   qArm[5]*180.0/PI
  // );
  return qArm;
}

















std::vector<double>
ARM6_kinematics_inverse_arm2(Transform trArm, const double *qOrg,bool wristRoll1){

  double handOffsetX = handLength+toolOffsetX;
  //I screwed up IK somewhere.. so let's use a hack around
  //Z is forward, Y is down
  trArm=trArm.translate(0,0,-toolOffsetZ);

  Transform t1;
  double shoulderYaw, shoulderPitch, elbowPitch, wristPitch, wristRoll, wristPitch2;

  t1=t1.translateZ(-shoulderOffsetZ)*trArm;
  double xEE[3]; t1.apply0(xEE); //shoulder yaw joint to end effector position

  t1=t1.translate(-handOffsetX,0,0);
  double xWrist[3];t1.apply0(xWrist); //shoulder yaw joint to wrist  position
  // printf("xWrist:%.3f %.3f\n",xWrist[0],xWrist[1]);

  double shoulder_wrist_2D=sqrt(xWrist[0]*xWrist[0]+xWrist[1]*xWrist[1]);
  double yaw_offset = asin(shoulderOffsetY/shoulder_wrist_2D);
  shoulderYaw = atan2(xWrist[1],xWrist[0])-yaw_offset; //standard position
  //double shoulderYaw2 = shoulderYaw + 2*yaw_offset; //flipped position.. WHICH WE WON'T USE EVER

  double cwr =(-xEE[0]*sin(shoulderYaw)+xEE[1]*cos(shoulderYaw) -shoulderOffsetY) /handOffsetX;
  if(cwr>1.0) cwr=1.0;   if(cwr<-1.0) cwr=-1.0; //nan proof
  wristRoll=acos(cwr); //roll angle 1, gripper toward left
  if (wristRoll1) wristRoll=-wristRoll; //second flipped wristroll position


  if (fabs(wristRoll)<1*PI/180.0){
    //all joints parallel case. we have infinite solution
    //now we kniw shoulderYaw AND wrist Roll (q[0] and q[4])


    //let's make sure that the wrist faces down
    //shoulderPitch + elbowPitch+wristPitch = PI

    Transform t5;
    t5=t5 //this transform is shoulderpitch to the first wrist
      .translateZ(-shoulderOffsetZ)
      .translateY(-shoulderOffsetY)
      .rotateZ(-shoulderYaw)
      *trArm
      .translate(0,-wristLength,0);

    double xShoulderWrist[3]; //shoulder to first wrist
    t5.apply0(xShoulderWrist);
    printf("shoulderWrist: %.2f %.2f %.2f\n",xShoulderWrist[0],xShoulderWrist[1],xShoulderWrist[2]);

    double shoulderwristdist=sqrt(xShoulderWrist[0]*xShoulderWrist[0]+xShoulderWrist[2]*xShoulderWrist[2]);
    double c_elbow=
      (lowerArmLength*lowerArmLength+upperArmLength*upperArmLength-shoulderwristdist*shoulderwristdist)/
      (2*lowerArmLength*upperArmLength);
    elbowPitch=PI-acos(c_elbow);

    double shoulderwristoffset=atan2(xShoulderWrist[2],xShoulderWrist[0]);
    double c_shoulder=
      (lowerArmLength*lowerArmLength-upperArmLength*upperArmLength+shoulderwristdist*shoulderwristdist)/
      (2*lowerArmLength*shoulderwristdist);
    shoulderPitch=PI/2.0 -acos(c_shoulder)-shoulderwristoffset;

    wristPitch = PI/2-shoulderPitch-elbowPitch;
    wristPitch2 = 0.0;


    printf("Singularity ");
  }else{

    //wrist pitch2 calculation: T * RotX(-wristPitch-PI/2)*RotZ(-PI/2)*RotX(-wristRoll)*[0 1 0]'  = RotZ(-shoulderYaw)*[0 1 0]'
    Transform t2;
    t2=inv(trArm);
    double tr0[3]={-sin(shoulderYaw),cos(shoulderYaw),0};
    t2.apply(tr0); //trArm-1 * rotZ(shoulderyaw) * [0 1 0]''
    double tr1[3];  t2.apply0(tr1);
    wristPitch2 = -atan2((tr0[1]-tr1[1])/sin(wristRoll), -(tr0[2]-tr1[2])/sin(wristRoll))-PI/2;

    //now we know angle 1,5,6
    Transform t3;
    t3=t3
      .translateY(-shoulderOffsetY)
      .translateZ(-shoulderOffsetZ)
      .rotateZ(-shoulderYaw)
      *trArm
      .translate(-handOffsetX,0,0)
      .rotateX(-wristPitch2-PI/2)
      .rotateZ(-PI/2)
      .translateX(-wristLength)
      .rotateX(-wristRoll);
    //t3 is now rotY(q1)trZ(lowerArmLength)rotY(q2)trZ(upperArmLength)rotY(q3)

    //this transform is for joint 2,3,4
    double xArm[3]; t3.apply0(xArm);
    double armdist = sqrt(xArm[0]*xArm[0]+xArm[1]*xArm[1]+xArm[2]*xArm[2]);
    double aa= lowerArmLength*lowerArmLength + upperArmLength*upperArmLength - armdist*armdist;
    double c_ep = aa/(2.0*lowerArmLength*upperArmLength);
    if (c_ep>1.0) c_ep=1.0;if (c_ep<-1.0) c_ep=-1.0;
    elbowPitch = PI-acos(c_ep); //one of two solutions
    double c_spo=(armdist*armdist + lowerArmLength*lowerArmLength - upperArmLength*upperArmLength)/(2*armdist*lowerArmLength);
    double shoulderPitchOffset = acos(c_spo);
    shoulderPitch = atan2(xArm[0],xArm[2])-shoulderPitchOffset;
    if ((c_spo>1.0)||(c_spo<-1.0)){
      shoulderPitch =qOrg[1]; //use older value if the target is too far away
    }
    // printf("aa:%.3f c_ep:%.3f Elbow:%.3f SPA:%.3f\n",aa,c_ep,elbowPitch*180.0/3.1415,shoulderPitchOffset*180.0/3.1415);
    double allPitch = atan2(t3(0,2),t3(0,0));
    wristPitch = allPitch-shoulderPitch-elbowPitch;
    printf("Ningularity ");

  }


  std::vector<double> qArm(6);
  qArm[0] = shoulderYaw;
  qArm[1] = shoulderPitch;
  qArm[2] = elbowPitch;
  qArm[3] = qOrg[3]+mod_angle(wristPitch-qOrg[3]);//to prevent the wrist pitch to rotate 360 deg
  qArm[4] = wristRoll;
  qArm[5] = wristPitch2;
  printf("SY %.1f SP %.1f EP %.1f WP %.1f WR %.1f WP2 %.1f\n",
    qArm[0]*180.0/PI,
    qArm[1]*180.0/PI,
    qArm[2]*180.0/PI,
    qArm[3]*180.0/PI,
    qArm[4]*180.0/PI,
    qArm[5]*180.0/PI
  );
  return qArm;
}
