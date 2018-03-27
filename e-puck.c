// By Pamir Ghimire (pamirghimire <> gmail)
// For Autonomous Robotics coursework
// Graduate Student, M1
// MSCV (Computer Vision and Robotics CVR)
// Universite De Bourgogne, France 

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <stdio.h>

#define TIME_STEP 32
#define Dwth 160

//SOME BEHAVIORS: Declarations here, definitions after main()
//----------------------------------------------------------------------
// Move forward with speed = sForward, over ntimeSteps
bool moveForward(double sForward);
// Stop robot!
bool stop();
// Turn robot 90 degrees ccw in place
bool turn90ccw();
// Follow the obstacle boundary on right side of robot
bool followWall(double sForward, double eprev);
// Return true if open space is available in front of robot
bool openSpaceAhead();
// Return true if obstacle present on right hand side of the robot
bool wallToRight();
// Read the indicated distance sensor and return value
double readDsensor(int nsensor);
//-----------------------------------------------------------------------
//---------------  MAIN : -----------------------------------------------
int main(int argc, char *argv[]) {

  // initialize robot
  wb_robot_init();
  
  // Forward speed
  double sForward = 250;
  // variables for error minimized in wall following behavior
  double eprev = 0;
  double ecurr = 0;
  // integral error
  double w_d2 = 0.7;
  
    /* main loop */
  for (;;) {
    
    eprev = ecurr;
  
    if (openSpaceAhead()){
      printf("\n -- Not following wall, just moving forward");
      printf("\n Right distance sensor = %f", readDsensor(2));
      moveForward(sForward);
    }
    else{
      // Enter wall following mode:
      turn90ccw(); 
      while (openSpaceAhead()){
        printf("\n\n Wall following mode:");
        eprev = ecurr;
        followWall(sForward, eprev);
        
        // If wall no longer to the right, stop following wall
        if (!wallToRight()){
          followWall(sForward, ecurr);
          if(!wallToRight()){
            break;
          }
        }
        
        ecurr = (w_d2*readDsensor(2) + (1-w_d2)*readDsensor(1))- Dwth;
        wb_robot_step(TIME_STEP);

      }
    }
    
    ecurr = (w_d2*readDsensor(2) + (1-w_d2)*readDsensor(1))- Dwth;
    
    wb_robot_step(TIME_STEP);
  }
  
  // main ends
  return 0;
}


//---------------------------------------------------------------------
// Move robot forward by setting left and right wheels to same speeds
bool moveForward(double sForward){
    
    wb_differential_wheels_set_speed(sForward, sForward);
    
    return true;
}
//----------------------------------------------------------------------
// Stop robot by setting both wheel speeds to zero
bool stop(){
    wb_differential_wheels_set_speed(0, 0);
    return true;
}

//-----------------------------------------------------------------------
// Turn robot in place
bool turn90ccw(){
  stop();
  int i; int nsteps90 = 102;
  for (i = 0; i < nsteps90; i++){
    wb_differential_wheels_set_speed(-100, 100);
    wb_robot_step(TIME_STEP);
  }
  return true;  
}


//------------------------------------------------------------------------
// Return true if open space in front of the robot
bool openSpaceAhead(){
  double D0;
  double D7;
  D0 = readDsensor(0);
  D7 = readDsensor(7);
  
  if (D0 >= Dwth && D7 >= Dwth){
  return 0;
  }
  else{
  return true;
  }
  
  return false;
}

//------------------------------------------------------------------------
// Return true if obstacle is present on right hand side of the robot
bool wallToRight(){
  if (readDsensor(2) > 0.20*Dwth){
  return true;
  }
  else{
  return false;
  }
}
//------------------------------------------------------------------------
// Make robot follow an obstacle
bool followWall(double sForward, double eprev){
  
  double w_d2 = 0.7;
  double ecurr = (w_d2*readDsensor(2) + (1-w_d2)*readDsensor(1))- Dwth;
  // define proportional, differential and integral error terms
  double ep = ecurr;
  double ed = ecurr - eprev;
  double ei = ecurr + eprev ;
  
  // define pid gains
  double Kp = 0.2;
  double Kd = 0.6;
  double Ki = 0.05;
  
  double sLeft = sForward;
  double sRight = sForward + (Kp*ep + Kd*ed + Ki*(ei));
  
  printf("\n Current error = %f", ecurr);
  printf("\n speed_LeftW = %f, speed_RightW = %f", sLeft, sRight);

  wb_differential_wheels_set_speed(sLeft, sRight);

  
  return true;
}


//------------------------------------------------------------------------
// Function for reading distance sensors: to be called by other functions
double readDsensor(int nsensor){
  // Declare some device tags
  // Distance sensor tags: 
  WbDeviceTag distsensor0;
  WbDeviceTag distsensor1;
  WbDeviceTag distsensor2;
  WbDeviceTag distsensor3;
  WbDeviceTag distsensor4;
  WbDeviceTag distsensor5;
  WbDeviceTag distsensor6;
  WbDeviceTag distsensor7;
  
  /* initialize Webots before calling any other webots functions:*/
  wb_robot_init();

  // Initialize the declared device tags:
  // Initialize distance sensors
  distsensor0 = wb_robot_get_device("ps0");
  distsensor1 = wb_robot_get_device("ps1");
  distsensor2 = wb_robot_get_device("ps2");
  distsensor3 = wb_robot_get_device("ps3");
  distsensor4 = wb_robot_get_device("ps4");
  distsensor5 = wb_robot_get_device("ps5");
  distsensor6 = wb_robot_get_device("ps6");
  distsensor7 = wb_robot_get_device("ps7");
  
  
  // Enable the distance sensors
  wb_distance_sensor_enable(distsensor0, TIME_STEP);
  wb_distance_sensor_enable(distsensor1, TIME_STEP);
  wb_distance_sensor_enable(distsensor2, TIME_STEP);
  wb_distance_sensor_enable(distsensor3, TIME_STEP);
  wb_distance_sensor_enable(distsensor4, TIME_STEP);
  wb_distance_sensor_enable(distsensor5, TIME_STEP);
  wb_distance_sensor_enable(distsensor6, TIME_STEP);
  wb_distance_sensor_enable(distsensor7, TIME_STEP);
  //------------------------------------------------------
  switch (nsensor){
    case 0:
      return wb_distance_sensor_get_value(distsensor0);
      break;
    case 1:
      return wb_distance_sensor_get_value(distsensor1);
      break;
    case 2:
      return wb_distance_sensor_get_value(distsensor2);
      break;
    case 3:
      return wb_distance_sensor_get_value(distsensor3);
      break;
    case 4:
      return wb_distance_sensor_get_value(distsensor4);
      break;
    case 5:
      return wb_distance_sensor_get_value(distsensor5);
      break;
    case 6:
      return wb_distance_sensor_get_value(distsensor6);
      break;
    case 7:
      return wb_distance_sensor_get_value(distsensor7);
      break;
    default:
      return 0;
  }
 
}

