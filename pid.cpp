#include "vex.h"
#include <cmath>
 
using namespace vex;
 
// A global instance of competition
competition Competition;
 
void preauton() {
  vexcodeInit();
}
 
//settiings for PD loop. Need to be tuned for acurate movement
double kP =0.00000001;
double kD =0.0;
double turnkP=0.09;
double turnkD= 0.007;
 
//Foward/Backward Constants
double error =0.0;
double prevError = 0.0;
double derivative = 0.0;
 
//Turning Constants
double TurnError=0.0;
double prevTurnError=0.0;
double TurnDerivative=0.0;
int maxPower=5;
double motorPower= (
		    FR.voltage() +
		    BR.voltage() +
		    BL.voltage() +
		    FL.voltage() +
		    ML.voltage() +
		    FL.voltage()
		    );
 
bool Drunning=true;
 
void PID(double target){
  Drunning=true;

  //double timesWithinRange=0.0;
  while(Drunning){
    if (motorPower > maxPower){
      motorPower=maxPower;
    }
    
    //Average Position for calculations
    double LeftPosition = Left.position(deg);
    double RightPosition = Right.position(deg);
    double averagePosition = (LeftPosition+RightPosition)/2;
    error = target - averagePosition;
    derivative=error - prevError;
    double lateralPower= kP * error + kD * derivative;
    
    FR.spin(fwd,lateralPower,voltageUnits::volt);
    BR.spin(fwd,lateralPower,voltageUnits::volt);
    FL.spin(fwd,lateralPower,voltageUnits::volt);
    BL.spin(fwd,lateralPower,voltageUnits::volt);
    ML.spin(fwd,lateralPower,voltageUnits::volt);
    MR.spin(fwd,lateralPower,voltageUnits::volt);
    
    Brain.Screen.print(error);
    Brain.Screen.newLine();

    if (std::abs(error)<30){
      timesWithinRange++;
    } else  {
      timesWithinRange=0;
    }

    Drunning=timesWithinRange < 10;
    error=prevError;
    vex::task::sleep(20);
  }
  return;
}

 
//PD On/Off swicth
bool running= true;
 
void drivePID(double turnTarget) {
  double timesWithinRange=0.0;
  running=true;
  while(running){
    double LeftPosition=Left.position(deg);
    double RightPosition=Right.position(deg);
    
    double TurnDifference = RightPosition-LeftPosition;
    
    TurnError= turnTarget - TurnDifference;
    TurnDerivative= TurnError-prevTurnError;
    //Calculates power the motors recieve   
    double TurnMotorPower = (TurnError*turnkP + TurnDerivative*turnkD);
    //Turn Power Output
    
    FR.spin(fwd,TurnMotorPower,voltageUnits::volt);
    BR.spin(fwd,TurnMotorPower,voltageUnits::volt);
    FL.spin(fwd,TurnMotorPower,voltageUnits::volt);
    BL.spin(fwd,TurnMotorPower,voltageUnits::volt);
    ML.spin(fwd,TurnMotorPower,voltageUnits::volt);
    MR.spin(fwd,TurnMotorPower,voltageUnits::volt);
      
    //Allows us to gauge how close the Error & TurnError are to 0 
    Brain.Screen.print("error:");
    Brain.Screen.print(error);
    Brain.Screen.newLine();
    Brain.Screen.print("TurnError");
    Brain.Screen.print(TurnError);
    //Breaks "while loop" so the program can continue onto next commands
    if (std::abs(TurnError)<30){
      timesWithinRange++;
    }
    else  {
      timesWithinRange=0;
    }
    running=timesWithinRange<10;
    vex::task::sleep(10);}
  return;
}

void autonomous(void) {
  //On/Off switch for the PD loop, stops it from running in UserControl
  running=true;
  Drunning=true;
  drivePID(1000);
  
  //Resets Encoders so a new value can be put in
  //drivePID(2000,-0);
  
  vex::task::sleep(1000);
 
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}
