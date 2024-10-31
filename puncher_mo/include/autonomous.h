#ifndef _AUTO_
#define _AUTO_
#include "declaration.h"
using namespace vex;

void driveForward(float v){
  v*=120;
  LB.spin(fwd, v, vex::voltageUnits::mV);
  LF.spin(fwd, v, vex::voltageUnits::mV);
  RB.spin(fwd, -v, vex::voltageUnits::mV);
  RF.spin(fwd, -v, vex::voltageUnits::mV);
}

void driveHorizontal(float v){
  v*=120;
  LB.spin(fwd, -v, vex::voltageUnits::mV);
  LF.spin(fwd, -v, vex::voltageUnits::mV);
  RB.spin(fwd, -v, vex::voltageUnits::mV);
  RF.spin(fwd, -v, vex::voltageUnits::mV);
}

void driveRotate(float v){
  v = v * 120;
  LB.spin(fwd, v, vex::voltageUnits::mV);
  LF.spin(fwd, v, vex::voltageUnits::mV);
  RB.spin(fwd, v, vex::voltageUnits::mV);
  RF.spin(fwd, v, vex::voltageUnits::mV);
}

void intake_setspeed(float v){
  intk.spin(fwd, v*120, vex::voltageUnits::mV);
  roller.spin(fwd, v * 120, vex::voltageUnits::mV);
}

void roller_setspeed(float v){
  roller.spin(fwd, v * 120, vex::voltageUnits::mV);
}

void roller_clearspeed(){
  roller.spin(fwd, 0, vex::voltageUnits::mV);
}

void puncher_setspeed(float v){
  leftPuncher.spin(fwd, v*120, vex::voltageUnits::mV);
  rightPuncher.spin(fwd, v*120, vex::voltageUnits::mV);
}

void timerForward(float v, float t){
  float initT = Brain.timer(msec);
  while(Brain.timer(msec) - initT < t){
    driveForward(v);
  }
  driveForward(0);
}

void timerRoll(float v, float t){
  float initT = Brain.timer(msec);
  while(Brain.timer(msec) - initT < t){
    roller.spin(fwd, v * 120, vex::voltageUnits::mV);
  }
  roller.spin(fwd, 0, vex::voltageUnits::mV);
}

void timerHorizontal(float v, float t){
  float initT = Brain.timer(msec);
  while(Brain.timer(msec) - initT < t){
    driveHorizontal(v);
  }
  driveForward(0);
}

void timerRotate(float v, float t){
  float initT = Brain.timer(msec);
  while(Brain.timer(msec) - initT < t){
    driveRotate(v);
  }
  driveRotate(0);
}

void EncoderRoll(float v, float target){
  float enc = roller.position(deg);
  float iniT = Brain.timer(msec);
  while(fabs(roller.position(deg) - enc) < fabs(target)){
    roller_setspeed(v);
    if(Brain.timer(msec)-iniT > 1000) break;
  }
  roller_setspeed(0);
}

void EncoderForward(float v, float target){
  float enc = (LF.position(deg)+LB.position(deg))/2;
  float iniT = Brain.timer(msec);
  while(fabs((LF.position(deg)+LB.position(deg))/2 - enc) < fabs(target)){
    driveForward(v);
    if(Brain.timer(msec)-iniT > 3000) break;
  }
  driveForward(0);
}

// assign the velocity of each wheel seperately
void VectorForward(Vector4f ch_speed){
  //cout<<"inmove"<<endl;
  LF.spin(fwd, ch_speed[0]* 120, vex::voltageUnits::mV);
  LB.spin(fwd, ch_speed[1]* 120, vex::voltageUnits::mV);
  RF.spin(fwd, ch_speed[2]* 120, vex::voltageUnits::mV);
  RB.spin(fwd, ch_speed[3]* 120, vex::voltageUnits::mV);
}

float getForwardEncoder() {
  return (fabs(RB.position(deg)) + fabs(LB.position(deg))) / 2.0;
}

void resetLeftEncoder() {
  LF.resetPosition();
  // ChLM.resetRotation();
  LB.resetPosition();
}
void resetRightEncoder() {
  RF.resetPosition();
  // ChRM.resetRotation();
  RB.resetPosition();
}
void resetChassisEncoder() {
  resetLeftEncoder();
  resetRightEncoder();
}

void PIDGyroTurn(float target, float tolerance = 0.5, float slow_increment = 5) {
  float timeroffset = TIMER;
  float timeused = 0;
  float kp = 5.5;        
  float ki = 0.5;       
  float kd = 40;       
  float istart = 10; 
  float dtol = 0.2;
  float errortolerance = tolerance; 
  float lim = 100;
  float error = target - Gyro.rotation(deg);
  float lasterror = error;
  float v = 0;
  float i = 0;
  bool arrived = false;
  float pow = 0;
  while (!arrived) {
    timeused = TIMER - timeroffset;
    error = target - Gyro.rotation();
    // v = (error - lasterror) / dt;
    v = -Gyro.gyroRate(zaxis, dps)/100;
    if ((fabs(error) < errortolerance && fabs(v) <= dtol)
        //||timeused > timetol
    ) {
      arrived = true;
    }
    if (fabs(error) < istart)
      i += error;
    else 
      i = 0;
    if (error * lasterror <= 0) {
        i = 0;
    }
    pow = kp * error + kd * v + ki * i;
    pow = fabs(pow) > lim ? sgn(pow) * lim : pow;
    driveRotate(pow);
    //cout << pow << "   " << error << "    " << v << "   " << i<< endl;
    // printScreen(10,100,"Iner %f",Iner.rotation());
    lasterror = error;
    delay(10);
  }
  //cout << timeused << endl;
  driveRotate(0);
}

// a function used in the move function
float rotateCalculate(float targetAng, float speedRatio, float rotateKp, bool &finished)
{
  // cout<<"error " << fabs(targetAng - globalRot)<<endl;
  // cout<<"target: "<<targetAng<<" ||globalRot: "<<globalRot<<endl;
  if(fabs(targetAng - globalRot) < 0.5) finished = 1;
  return -cap((targetAng - globalRot) * rotateKp, speedRatio);
}

// a function used in the move function
Vector4f getGoSpeed(float xtarget, float ytarget, bool &arrived){
  Vector2f toTarget = Vector2f(xtarget - corX , ytarget - corY);
  //cout<<"toTarget:" << toTarget[0] << " " << toTarget[1]<<endl;
  Vector2f localTarget = SpinMatrix(mygyro.rotation() - IGR) * toTarget;
  //cout<<"localTarget:" << localTarget[0] << " " << localTarget[1]<<endl;
  arrived = (localTarget[1] < 0 && localTarget.norm() < 8 && globalspeed.norm()<0.4) || (localTarget.norm() < 5 && globalspeed.norm()<0.4);
  return Vector4f(localTarget[1] + localTarget[0],
      localTarget[1] - localTarget[0],
      -localTarget[1] + localTarget[0],
      -localTarget[1] - localTarget[0]);
}

float R2D(float Rad){
  return Rad*180/PI;
}

float D2R(float Deg){
  return Deg*PI/180;
}

float toSameRange(float Deg){
  return Deg - 2 * PI * floor((Deg + PI) / (2 * PI));;
}

float getMax(Vector4f input)
{
  float absmax = 0;
  for(int i = 0; i < 4; i++)
  {
    if(fabs(input[i]) > fabs(absmax)) absmax = input[i];
  }
  return absmax;
}

Vector4f maxScale(Vector4f input, float largest)
{
  return input * fabs(largest / getMax(input));
}

// a function used in the move function
Vector4f mixSpeed(Vector4f go, float turn){
  float max = turn>=0?100-turn:-100-turn;
  //float min = rotatespeed<=0?100-rotatespeed:-100-rotatespeed;
 // cout << max <<endl;
  Vector4f resultMixSpeed;
  for (int i=0;i<=3;i++)
  { 
    if (fabs(go[i])>100) {
      go = maxScale(go, 100);}
  }
  for (int i=0;i<=3;i++)
  { 
    if (fabs(go[i]+turn)>100) {go =  (max/go[i])*go;}
  }
  resultMixSpeed = go+Vector4f(turn,turn,turn,turn);
  return resultMixSpeed; 
}

// this function allows the bot to automatically calculate the speed and move from its current point to another point.
void Odom_move(float xtarget, float ytarget, float rTarget){ 
  // initialization
  float goKp = 7;
  float goKd = 13;
  float rotate_Kp = 2.7;
  float rotate_Kd = 0;
  bool arrived = 0;
  float rotateerror;
  float lastError = 0;
  float rotateSpeed;
  Vector4f goVec;
  Vector4f rVec;
  float errorRatio;
  float rError;
  float goError;
  Vector4f pow = Vector4f(0, 0, 0, 0);
  bool rotateFinished = 0;
  while(!arrived){
    //cout<<"--------------newWhile-----------------"<<endl;
    //cout<<rotateFinished<<endl;
    //get the direction vector of moving forward
    goVec = getGoSpeed(xtarget, ytarget, arrived);
    //cout<< "goVec: " << goVec[0]<<" "<<goVec[1]<<" "<<goVec[2]<<" "<<goVec[3]<< endl;
    //get the distance between the current position and target position (both moving and rotating)
    goError = (Vector2f(xtarget - corX, ytarget - corY)).norm();
    rError = fabs(rTarget - globalRot);
    errorRatio = fabs((goError/45)/(rError/90+goError/45)); 
    //cout<< goError<<endl;
    //calculate the moving speed and rotating speed
    goVec.normalize();
    rotateSpeed = rotateFinished? 0 :rotateCalculate(rTarget, 100*(1-errorRatio),
             rotate_Kp, rotateFinished) - rotate_Kd * Gyro.gyroRate(zaxis, dps)/50;
    goVec = (goError * goKp - (goError-lastError)*goKd) * goVec;
    
    //rVec << rotateSpeed, rotateSpeed, rotateSpeed, rotateSpeed;
    //cout<< "goVec with p&d: "<< goVec[0] << " " << goVec[1] <<endl;
    //combine the moving speed and rotating speed
    pow = mixSpeed(goVec, rotateSpeed);
    //cout<<"pow:"<<pow[0]<<endl;
    VectorForward(pow);
    //cout<<pow[0]<<endl;
    lastError = goError;
    delay(20);
  }
  //cout<<"FINIESHEDFINIESHEDFINIESHED"<<endl;
  driveForward(0);


}





#endif