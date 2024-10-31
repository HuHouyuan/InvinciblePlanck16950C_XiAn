#ifndef _TASK_
#define _TASK_
#include "declaration.h"
#include "robot-config.h"
#include <iostream>
#undef __ARM_NEON__
#undef __ARM_NEON
#include <eigen-3.4.0/Eigen/Dense>
using namespace Eigen;

float vch3 = false;
float vch4 = false;
float vch1 = false;

bool lL1 = false;
bool lR1 = false;
bool lR2 = false;
bool lL2 = false;
bool lBX = false;
bool lBY = false;
bool lBB = false;
bool lUP = false;
bool lBA = false;
bool lRight = false;
bool lDown = false;
bool lLEFT = false;
bool isGod = false;
bool isAim = false;
bool isOpen = false;
bool currentBool = false;
bool special = false;
bool criticalAdd = false;
bool ManualShoot = false;
bool chooseRed = false;
bool chooseBlue = false;
bool fifteen = false;
bool notBad = false;
bool afraid = false;
int manualPuncherSet = false;

void punch()
{
  currentBool = inAuto;
  // open the cap of the puncher
  inAuto = 1;
  intake_setspeed(-100);
  delay(300);
  // activate the pneumatic & shoot the disks
  puncher_air.set(1);
  puncher_setspeed(-5);
  // cout<<puncherTarget<<endl;
  delay(200);
  readyToPunch = 0;
  puncher_air.set(0);
  delay(400);
  inAuto = currentBool;
  // reset
  puncher_setspeed(5);
  delay(550);
  leftPuncher.resetPosition();

  // if it's in the autonomous period, we always want to keep the intake on
  if (inAuto)
  {
    intake_setspeed(100);
  }
  else
  {
    intake_setspeed(100);
    delay(100);
    intake_setspeed(30);
    delay(200);
    intake_setspeed(0);
  }
}

int PuncherJudger()
{
  delay(100);
  // initialize puncher encoder
  leftPuncher.resetPosition();
  readyToPunch = 0;
  float myTime = Brain.timer(msec);

  while (1)
  {
    // check if the puncher can punch and the user pressed the buttom of punching
    if (readyToPunch)
      punch();
    /* if the user isn't going to punch, adjust the position of the puncher automatically according to the
    current position. The process involves using the P and D term for regulation */
    else
    {
      puncher_setspeed(3 * (puncherTarget - leftPuncher.position(deg)) + 10);
      // cout<< "setspeed " <<3*(puncherTarget - leftPuncher.position(deg))+5<<endl;
    }
    delay(10);
  }
  return 0;
}

// convert how many encoder units is pulled down by the puncher to expected shooting range.
double distance2encoder(double x)
{
  double y = -0.0000186435 * pow(x, 3) + 0.0129697 * pow(x, 2) - 0.669693 * x + 630.031; // 578
  // 420.388 + 2.9685*x - 0.00747167 * pow(x,2) + 0.000015325 * pow(x,3); //370.388 // 400

  // 1050 being the maximum range for the puncher to shoot
  if (y > 1050)
  {
    return 1050;
  }
  else
  {
    return y;
  }
}

// judge which goal, the red goal or the blue goal, to shoot.
int ShootJudger()
{
  delay(100);
  float toRed = 0;
  float toBlue = 0;
  while (1)
  {
    // target vector, from the bot, to the goal
    toRed = Vector2f(redShoot[0] - corX, redShoot[1] - corY).norm();
    toBlue = Vector2f(blueShoot[0] - corX, blueShoot[1] - corY).norm();
    // during the 15 sec autonomous period, there's only one target to aim at
    if (fifteen)
    {
      shootPoint = Vector2f(-132, 132);
    }
    // when users want to manually control the puncher
    else if (ManualShoot)
    {
      if (chooseBlue)
        shootPoint = blueShoot;
      else
        shootPoint = redShoot;
    }
    // during the autonomous period of the skill competition, auto aim at the nearest target
    else
    {
      if (toRed < toBlue)
        shootPoint = redShoot;
      else
        shootPoint = blueShoot;
    }

    delay(10);
  }
  return 0;
}

// updating the strength of the puncher (puncher target) automatically or manually
int PuncherEncControl()
{
  delay(100);
  dis = 0;
  puncherTarget = 600;
  while (1)
  {
    dis = Vector2f(shootPoint[0] - corX, shootPoint[1] - corY).norm();
    // automatically control how much encoder units the puncher should pull down
    if (manualPuncherSet == 0)
      puncherTarget = distance2encoder(dis);
    // control the puncherTarget manually
    else if (manualPuncherSet == 1)
      puncherTarget = 660;
    else
      puncherTarget = 830;
    // if(BA&&!lBA) puncherTarget+=10;
    // if(BY&&!lBY) puncherTarget-=10;
    // if(DOWN&&!lDown) puncherTarget+=5;

    // cout<<"puncherTarget: "<<puncherTarget<<endl;
    // cout<<"nowpuncher: "<<leftPuncher.position(deg)<<endl;
    // lBA = BA;
    // lBY = BY;
    // lDown  = DOWN;
    delay(10);
  }
  return 0;
}

// updating the globalRot, which represents the orientation of the bot on the coordinate plane.
int updateGlobalRot()
{
  delay(100);
  globalRot = 180 - IGR;
  float initGyro = mygyro.rotation();
  float finalGyro = initGyro;
  float dG = 0;
  while (1)
  {
    // cout<<"deltaGyro: "<<dG<<" ||globalRot: "<<globalRot<<endl;
    finalGyro = mygyro.rotation();
    dG = finalGyro - initGyro;
    globalRot -= dG;
    initGyro = finalGyro;
    delay(10);
  }
  return 0;
}

// updating the position of the bot through using wheel odometry
Vector2f result;
int updateCOR()
{
  delay(100);

  // initialization
  xpos.resetRotation();
  ypos.resetRotation();
  mygyro.setRotation(IGR, deg);
  float cor;
  Vector2f dis;
  dis << 0, 16;
  float d;
  Vector2f initPos;
  Vector2f finalPos;
  initPos << 0, 0;
  finalPos << 0, 0;
  float initGyro = 90;
  float finalGyro;
  corX = 0;
  corY = 0;
  // rx = 135.25;
  // ry = 34.3;
  rx = 0; //-75.25;
  ry = 0; //-145.7;
  while (1)
  {
    // obtain and calculate the current orientation in the coordinate plane
    finalGyro = IGR - mygyro.rotation();
    d = (initGyro + finalGyro) / 2;
    cor = IGR - mygyro.rotation();
    // obtain the change of displacement in the 10 msec loop & convert them into global displacement vectors
    finalPos << -xpos.rotation(deg), -ypos.rotation(deg);
    result = SpinMatrix(d) * (finalPos - initPos);
    result = SpinMatrix(45) * result;
    // updating position
    rx += result[0] * xratio;
    ry += result[1] * yratio;
    corX = rx + (SpinMatrix(cor) * dis)[0];
    corY = ry + (SpinMatrix(cor) * dis)[1];
    // obtaining the bot's speed too
    globalspeed = Vector2f(result[0] * xratio / 100, result[1] * yratio / 100);
    initPos = finalPos;
    initGyro = finalGyro;
    delay(10);
  }
  return 0;
}

/* this function literally enables the bot to look at a specific point. The direc parameter is a paramter not used anymore
   after a version of update. But because previous routes in the autopath already used the old version, the parameter is kept,
   but it no longer serves a function. The target paramter represents the target point to look at*/

float LookAt(float direc, Vector2f target)
{
  Vector2f toTarget;
  // get the target direction which the robot needs to rotate to
  toTarget << target[0] - corX, target[1] - corY;
  Vector2f StandardC;
  Vector2f now;
  float dc;
  float Deg;
  float isDeg;
  StandardC << 0, 1;
  dc = IGR - mygyro.rotation();
  now = SpinMatrix(dc) * StandardC;
  // calculate the angle between the target facing vector and current facing vactor
  Deg = acos(toTarget.dot(now) / (now.norm() * toTarget.norm()));
  Deg = Deg / M_PI * 180;
  Vector2f isResult = SpinMatrix(-Deg) * now;
  // check if the direction of the turn is correct or not, if not, set it to the opposite direction
  isDeg = acos(isResult.dot(toTarget) / (isResult.norm() * toTarget.norm()));
  isDeg = isDeg / M_PI * 180;
  if (isDeg > 3)
    Deg = -Deg;
  // Controller1.Screen.clearScreen();
  // Controller1.Screen.setCursor(1, 1);
  // Controller1.Screen.print(Deg);
  return Deg;
}

float getSpeed()
{
  return sqrt(result[0] * result[0] + result[1] * result[1]);
}

/* this function enables the car to automatically turn and aim at the goal even when users are 
moving the car without affecting the locomotion of the car */
int giveSpeedInAim()
{
  delay(100);
  // initialization
  float target = mygyro.rotation() + LookAt(0, shootPoint);
  Vector2f fakeShootPoint = shootPoint;
  float tolerance = 0.5;
  float kp;
  float ki;
  float kd;
  float imin = 10;  
  float istart = 60; 
  float dtol = 0.2;
  float errortolerance = tolerance; 
  float lim = 100;
  float error = target - Gyro.rotation(deg);
  float lasterror = error;
  float v = 0;
  float i = 0;
  bool arrived = false;
  bool firstOver = false;
  arrived = error == 0;
  // the distance from the bot to the goal
  float predis = sqrt((shootPoint[0] - corX) * (shootPoint[0] - corX) + (shootPoint[1] - corY) * (shootPoint[1] - corY));
  // Simply put, if we wish to shoot the disks into the goal while moving, we need to aim at a point near the goal taking the bot's velocity into consideration
  fakeShootPoint = shootPoint - globalspeed * (16.3 + predis * 0.1);
  while (1)
  {
    predis = sqrt((shootPoint[0] - corX) * (shootPoint[0] - corX) + (shootPoint[1] - corY) * (shootPoint[1] - corY));
    // the angle needed to turn to the target direction
    target = mygyro.rotation() + LookAt(0, fakeShootPoint);
    if (arrived)
      aim_ch1 = 0;
    // if the car is in stationary, it's simply another lookat where the target point is the goal
    if (getSpeed() < 1.2 && fabs(error) < 0.5)
    {
      fakeShootPoint = shootPoint;
      kp = 5.5;
      ki = 0.2;
      kd = 55;
    }
    else
    // if the bot is in motion, we set a different target for the bot to aim at by considering the bot's velocity
    {
      fakeShootPoint = shootPoint - globalspeed * (16.3 + predis * 0.2);
      kp = 5.5;
      ki = 0;
      kd = 47;
    }
    // PID regulation
    error = target - Gyro.rotation();
    // v = (error - lasterror) / dt;
    v = Gyro.gyroRate(zaxis, dps) / 100;
    if ((fabs(error) < errortolerance && fabs(v) <= dtol)
    )
    {
      arrived = true;
    }
    if (fabs(error) < istart)
      i += sgn(error);
    if (error * lasterror <= 0)
    {
      if (firstOver)
      {
        i = sgn(error) * imin;
        firstOver = false;
      }
      else
      {
        i = 0;
      }
    }

    aim_ch1 = kp * error - kd * v + ki * i;
    aim_ch1 = fabs(aim_ch1) > lim ? sgn(aim_ch1) * lim : aim_ch1;
    // pow = fabs(pow) > slow ? sgn(pow) * slow : pow;
    // printScreen(10,100,"Iner %f",Iner.rotation());
    lasterror = error;
    delay(10);
  }
  return 0;
}

/* 
1. If button B is pressed, the bot switch to the god mode, in which users can control the bot using a thrid person
perspective. For instance, normally, if the car is facing east and we push the stick forward, the car goes forward in
its direction, which ends up going towards east. However, in god mode, when users push the stick forward, the car
moves towards north, which is defined by us as the positive y direction on the coordinate. Therefore, in god mode, users control
the bot through using absolute direction instead of relative direction
2. If button X is pressed, the bot goes into auto aim mode, in which the bot auto aims at the goal
3. The last mode is the normal controlling mode
*/
int vac()
{
  delay(100);
  while (1)
  {
    if (BB && !lBB)
    {
      isGod = !isGod;
    }
    if (BX && lBX)
    {
      isAim = 1;
    }
    else
    {
      isAim = 0;
    }
    lR2 = R2;
    lBB = BB;
    delay(10);
  }
}

void normalMove()
{
  vch3 = Ch3;
  vch4 = Ch4;
}

// calculate the velocity of each wheel during god perspective
void godPerspective()
{
  Vector2f GodV;
  // get the input from the controller
  GodV << Ch4, Ch3;
  // spin the vector to the third person persepctive
  GodV = SpinMatrix(mygyro.rotation() - 90) * GodV;
  // set the speed
  vch4 = GodV[0];
  vch3 = GodV[1];
}

Vector2f preaim;
void Aim()
{
  vch1 = aim_ch1;
}
void notAim()
{
  vch1 = Ch1;
}

// the function used for regulating the movement of the bot during the user control phase
void ch_move()
{

  if (!isGod)
    normalMove();
  else
    godPerspective();
  if (!isAim)
    notAim();
  else
    Aim();
  // if(!inAuto){
  LF.spin(fwd, (vch3 + vch4 + vch1) * 120, vex::voltageUnits::mV);
  LB.spin(fwd, (vch3 - vch4 + vch1) * 120, vex::voltageUnits::mV);
  RF.spin(fwd, (-vch3 + vch4 + vch1) * 120, vex::voltageUnits::mV);
  RB.spin(fwd, (-vch3 - vch4 + vch1) * 120, vex::voltageUnits::mV);
  //}
  delay(10);
}

void intakeAndRoll()
{
  if ((L1 && lL1) || (L2 && lL2))
  {
    if (L1 && lL1)
    {
      // cout<<"inL1"<<endl;
      intk.spin(fwd, 120 * 100, vex::voltageUnits::mV);
      roller.spin(fwd, 100 * 120, vex::voltageUnits::mV);
    }
    else
    {
      //  cout<<"inL2"<<endl;
      roller.spin(fwd, -100 * 120, vex::voltageUnits::mV);
      intk.spin(fwd, -100 * 120, vex::voltageUnits::mV);
    }
  }
  else if (!inAuto)
  {
    // cout<<"bad:("<<endl;
    roller.spin(fwd, 0, vex::voltageUnits::mV);
    intk.spin(fwd, 0, vex::voltageUnits::mV);
  }
  lL1 = L1;
  lL2 = L2;
}

// print some status of the bot and other essential info on the screen of the controller
int printStuff()
{
  delay(100);
  int a;
  int b;
  while (1)
  {
    a = isGod ? 1 : 0;
    b = isAim ? 1 : 0;
    Brain.Screen.printAt(10, 40, "COR:%.2f %.2f",
                         corX,
                         corY);
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("P: %d||A: %d||G: %.2f", a, b, mygyro.rotation());
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("X: %.2f||Y: %.2f", corX, corY);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Routine: %d||Afraid: %d", autoRoutine, afraid);

    // Controller1.Screen.setCursor(2, 1);
    // Controller1.Screen.print("LF: %.2f||LB: %.2f", LF.position(deg), LB.position(deg));
    // Controller1.Screen.setCursor(3,1);
    // Controller1.Screen.print("RF: %.2f||RB: %.2f", RF.position(deg), RB.position(deg));

    // Controller1.Screen.setCursor(2, 1);
    // Controller1.Screen.print("Gyro: %.2f",mygyro.rotation());
    // Controller1.Screen.setCursor(3,1);
    // Controller1.Screen.print(autoRoutine);

    // Controller1.Screen.setCursor(1, 1);
    // Controller1.Screen.print("leftEncoder: %.2f", leftPuncher.position(deg));
    // Controller1.Screen.setCursor(2, 1);
    // Controller1.Screen.print("dis: %.2f", Vector2f(shootPoint[0]-corX,shootPoint[1]-corY).norm());
    // Controller1.Screen.setCursor(3,1);
    // Controller1.Screen.print("puncherTarget: %.2f", puncherTarget);
    delay(10);
  }
}
#endif
