
/*
 *step PID controller 
 *
*parameters:
* float act  [0 -100] actual value
* float set [0 -100] setpoint
* float  diff [-100, 100] control difference

* bool auto_man switching auto/manual
* bool butt_plu push button direction plus
* bool butt_min push button direction minus
* bool motor_plu digital output direction plus
* bool motor_min digital output direction minus
* bool limit_plu limit switch direction plus– stopping pulses and DPID calculation
* bool limit_min limit switch direction minus – stopping pulses and DPIDcalculation


* float gain controller gain
* float Ti[s] controller integral time
* float Td[s] controller derivate  time
* float onvalue [ 0 – 100]
* float offvalue [ 0 – 100]
* float mimp[s] minimal motor impulse length 
* float motime[s] motor running time from minimal to maximal position

* Function PT

 * parameters:
 * float in input signal
 * float out  filtered output
 * float T[s] time constant 
 * 
 *  Function Dead
 *  
 * parameters:
  * float in input
 * float out output
 * int deadtime [s] deadtime  0- 30 multply sample time
 
 *  By means of function we delay analog values on output in relation to input
 * at defined dead time.

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




#include <SPID.h>
#include<ConFunct.h>
Takt100 takt100;
Takt1000 takt1000;
SPID con1;   
Int  int1;
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

//step controller parameter


con1.gain=6;
con1.Ti=20;//according to Ziegler-Nichols Ti=deadtime*2
con1.Td=5;//according to Ziegler-Nichols Td=deadtime/2
con1.onvalue=0.5;
con1.offvalue=0;
con1.mimp=0.05;
con1.motime=10;


//Integrator - servo motor simulation

int1.Ti=10;


//PT control plant simulation

pt1.T=100;            // time constant PT element = 100s


//Dead control plant simulation
dead1.start=1;       //initialize deadtime
dead1.deadtime=10;  // Takt1000 1 s sample time, deadtime= 10s

//SetTimeTable - put values into table and initialize sequence

table1.start=1; //initialize sequence

table1.TimeMin[0]=5;table1.TimeMin[1]= 8;table1.TimeMin[2]=10; table1.TimeMin[3]=12;table1.TimeMin[4]=13;table1.TimeMin[5]=15;table1.TimeMin[6]=18; table1.TimeMin[7]=20;
table1.SetArray[0]=30;table1.SetArray[1]=20;table1.SetArray[2]=55;table1.SetArray[3]=70;table1.SetArray[4]=30;table1.SetArray[5]=60;table1.SetArray[6]=20;table1.SetArray[7]=70;



//SetJpRamp - replacing setpoint jumps from table1 through ramps

sramp1.start=1; //initialize ramping
sramp1.speed_higher=1;
sramp1.speed_lower=1;

}

void loop() {

takt1=takt100.fupdate();   //calling 100ms takt
takt2=takt1000.fupdate();  //calling 1s takt

 

///===========time sampling 100ms
if (takt1==1){


con1.cupdate( );   //call controller CPID
pt1.fupdate( );    //call function PT
sramp1.fupdate( ); //call fuction SetJpRamp
int1.fupdate( );   //call integrator

//servo motor simulation

if (con1.motor_plu)
{ int1.in=100;}
if (con1.motor_min)
{int1.in=-100;}
if((con1.motor_plu==false)&&(con1.motor_min==false))
{int1.in=0;}

//close the control loop < con1< pt1 < dead1 < int1

dead1.in=int1.out;
pt1.in=dead1.out;
con1.act=pt1.out;

//connect setpoint time table to setpoint ramping to controller input
sramp1.act=con1.act;    
sramp1.set=table1.set;
con1.set=sramp1.out;

}

///===========time sampling 1s
if (takt2==0){

dead1.fupdate( );
table1.fupdate( );


Serial.println(con1.set+String(" ")+con1.act+String(" ")+String("  ")+int1.out+String("  ")+10*con1.motor_plu+String("  ")+10*con1.motor_min);//see the simulation results in plotter
//Serial.println(con1.diffhyst_last+String("  ")+con1.PxD+String("  ")+con1.IxD+String("  ")+String("  ")+con1.DxD);
            }
            
}
