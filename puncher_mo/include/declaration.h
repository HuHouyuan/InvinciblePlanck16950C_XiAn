#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include "robot-config.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include<cmath>
using namespace std;

// some declarations and Macro
#define TIMER           Brain.timer(vex::timeUnits::msec)

#define Ch1             Controller1.Axis1.position(percent)
#define Ch2             Controller1.Axis2.position(percent)
#define Ch3             Controller1.Axis3.position(percent)
#define Ch4             Controller1.Axis4.position(percent)

#define BA              Controller1.ButtonA.pressing()
#define BB              Controller1.ButtonB.pressing()
#define BX              Controller1.ButtonX.pressing()
#define BY              Controller1.ButtonY.pressing()

#define L1              Controller1.ButtonL1.pressing()
#define L2              Controller1.ButtonL2.pressing()
#define R1              Controller1.ButtonR1.pressing()
#define R2              Controller1.ButtonR2.pressing()

#define UP              Controller1.ButtonUp.pressing()
#define DOWN            Controller1.ButtonDown.pressing()
#define LEFT            Controller1.ButtonLeft.pressing()
#define RIGHT           Controller1.ButtonRight.pressing()

#define SDFile          "t.h"

#define getCataEncoder  (cat.position(deg))
#define getLimitValue   (lmt.value())
#define getLimit2Value  (lmt2.value())
#define getDis          (dis.objectDistance(mm))
#define isSDInserted    (Brain.SDcard.isInserted())
#define isFileExists    (Brain.SDcard.exists(SDFile))
#define sign(x)         (x==0?0:(x>0?1:-1))
#define sgn(x)          (x > 0 ? 1 : -1)
#define getGyro         Gyro.rotation()
#define mygyro          Gyro
#define IGR             initGyroRotation
#define delay           vexDelay
#define PI              M_PI
#define cap(x, cap)     (fabs(x) > cap ?  sign(x) * cap : x)   

static int      ch_state            = 0;
static bool     lck                 = 1;
static bool     lckReset            = 0;
static bool     autoCata            = 0;
static bool     manual              = 1;
static bool     AutoCataInterrupt   = 0;
static int      autoRoutine         = 1;
static float    rota_0              = 0.5;
static float    rota_1              = 0.3;
static float    targetDis           = 32;

bool readyToPunch = 0;
bool inAuto = 0;
bool inManualControl = 0;

Matrix2f SpinMatrix(float d){
  d=d*M_PI/180;
  Matrix2f sm;
  sm<< cos(d), -sin(d), 
  sin(d), cos(d);
  return sm;
}

float DegToAnotherVector(Vector2f myVector){
  Vector2f StandardC;
  Vector2f now;
  float dc;
  float Deg;
  float isDeg;
  StandardC<< 0, 1;
  dc = mygyro.rotation() - 90;
  now = SpinMatrix(dc) * StandardC;
  Deg = acos(myVector.dot(now)/(now.norm()*myVector.norm()));
  Deg = Deg/M_PI * 180;
  // Vector2f isResult = SpinMatrix(Deg) * now;
  // isDeg = acos(isResult.dot(myVector)/(isResult.norm()*myVector.norm()));
  // isDeg = isDeg/M_PI*180;
  // if (isDeg<-2 || isDeg>2) Deg = -Deg;
  return Deg;
} 

float rx = 0;
float ry = 0;
float corX = 0;
float corY = 0;


float realGyro;
float fakeGyro = 114514;
float xratio = 60 / 924.904; //60 / 979.4;
float yratio = 60 / 1001.671; //60 / 1009.1;
float initGyroRotation = 90;
float dis;
float puncherTarget;
float aim_ch1 = 0;
float globalRot;
float globalGyro;

Vector2f shootPoint = Vector2f(-132, 132);
Vector2f globalspeed = Vector2f(0,0);
Vector2f redShoot = Vector2f(132, -132);
Vector2f blueShoot = Vector2f(-132, 132);

#endif