/*
* continuous controller parameters:

 * float act  [ 0 -100] actual value
 * float set[ 0 –100] setpoint
 * float diff  [-100, 100] control difference
 * float out [ 0 – 100]  PID output value
 * float track [ 0 – 100] tracking value , only for cascade of two controllers 
 * bool auto_man switching auto/manual
 * bool out_up push button output up
 * bool out_down push button output down
 * bool cascade if cascade=true, master controller

 * float gain   controller gain
 * float Ti[s]   controller integration time
 * float Td[s]  controller derivative time

 * Function PT
 * 
 * parameters:
 * float in input signal
 * float out  filtered output
 * float T[s] time constant 
  
 *  Function Dead

 * parameters:
 * float in input
 * float out output
 * int deadtime [s] deadtime  0- 30 multply sample time
 * 
 * By means of function we delay analog values on output in relation to input
 * at defined dead time.
 * 
 * Funcion SetTimeTable
 * 
 * parameters:
 * float set setpoint
 * int TimeMin[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
 * int TimeSec[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
 * float SetArray[20]={50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};

 *  bool start starting the sequence

* This function produces setpoint set due to 20 values of setpoint
* stored in SetArray and time stored in arrays TimeSec and TimeMin.
* After bit start is set, the program begins from the first value of setpoint from SetArray[0]
* und put it to the output set as long as the sum of time stored in both arrays TimeMin
* and TimeSec in the same position is not over. Then switches to the next setpoint
* from SetArray.

* Function SetJpRamp
* 
* parameters:
* float act actual value
* float set setpoint
* float out output
* float speed_higher [%]
* float speed_lower [%]

* bool start  enable ramp


* This is another function for smoothing jumps of setpoint input.
* After new setpoint jump on input set, SetJpRamp generates on output out
* ramps with different speed, depending on the direction of setpoint change.
* Either ramp up, if new setpoint is bigger the last one setpoint, with 
* speed set_higher %/s , or ramp down with speed set_lower %/s.
* The ramping begins from actual value on input act and stops,
* when the target setpoint is reached.
* When the setpoint was changed once again during the ramping, 
* the reaction depends on the progress in ramping.
* If the new setpoint is bigger as the the ramp value on out and the ramping
* was direction higher, the ramp will continue moving.
* If the new setpoint is lower out, new ramp, with a speed set_lower
* direction down, starts.
* By ramping down und new setpoint the action is similar, but into the
* opposite direction.

 *see:
 *https://github.com/KorneliusThomas/PIDcontrollersModularProfessional

 */

#include <CPID.h>
#include<ConFunct.h>
Takt100 takt100;
Takt1000 takt1000;
CPID con1;   
PT   pt1;
Dead dead1;
SetTimeTable table1;
SetJpRamp sramp1;
//
//variables for time sampling
//

int takt1;
int takt2;


void setup() {
 
Serial.begin(9600); //start serial commmunication



con1.auto_man=1; //switch controller to auto

// continous controller parameter

con1.gain=6;    // 
con1.Ti=20;     //according to Ziegler-Nichols Ti = 2*deadttime
con1.Td=5;     //according to Ziegler-Nichols Td=deadtime/2

//PT control plant simulation

pt1.T=100;            // time constant PT element = 100s

//Dead control plant simulation
dead1.start=1;       //initialize deadtime
dead1.deadtime=10;  // Takt1000 1 s sample time, deadtime= 10s


//SetTimeTable time table with setpoints for controller

table1.start=1; //initialize SetTimeTable

table1.TimeMin[0]=5;table1.TimeMin[1]= 8;table1.TimeMin[2]=10; table1.TimeMin[3]=12;table1.TimeMin[4]=13;table1.TimeMin[5]=15;table1.TimeMin[6]=18; table1.TimeMin[7]=20;
table1.SetArray[0]=30;table1.SetArray[1]=20;table1.SetArray[2]=55;table1.SetArray[3]=70;table1.SetArray[4]=30;table1.SetArray[5]=60;table1.SetArray[6]=20;table1.SetArray[7]=70;


//SetJpRamp replace setpoints jumps from time through ramping

sramp1.start=1;
sramp1.speed_higher=1;  //ramping speed up
sramp1.speed_lower=1;   //ramping speed down

}

void loop() {

takt1=takt100.fupdate();   //calling 100ms takt
takt2=takt1000.fupdate();  //calling 1s takt

 

///===========time sampling 100ms
if (takt1==1){

con1.cupdate( );   //call controller CPID
pt1.fupdate( );    //call function PT
sramp1.fupdate( ); //call fuction SetJpRamp

//close the control loop < pt1< dead1 < con1
con1.act=pt1.out;
pt1.in=dead1.out;
dead1.in=con1.out;

//connect setpoint time table to setpoint ramping and to controller input
sramp1.act=con1.act;    
sramp1.set=table1.set;
con1.set=sramp1.out;
}

///===========time sampling 1s
if (takt2==0){

dead1.fupdate( );
table1.fupdate( );


Serial.println(con1.set+String(" ")+con1.act+String(" ")+con1.out+String("  "));  //look for simulation results in plotter

            }
}
