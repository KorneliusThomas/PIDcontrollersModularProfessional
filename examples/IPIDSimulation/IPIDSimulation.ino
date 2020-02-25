/*
impulse PID controller
*
*parameters:
* float act [0 -100] actual value
* float set [0 -100] setpoint
* float diff [-100, 100] control difference
* float out [0 -100] continous output 
* float track [ 0 -100] tracking value, only for cascade

* bool auto_man switching auto/manual
* bool butt_pl push button direction plus
* bool butt_min push button direction minus
* bool heating output heating
* bool cooling output cooling

* bool three_two controller type =false , only heating. =true heating and cooling
* bool cascade if cascade true=true, master controller


* float gain controller gain
* float Ti[s] controller integration time
* float Td[s] controller derivate time
* float period[s] impulse/pause period by PWD modulation
* float min_imp[s] minimum impulse length
* float cool_to_heat  ratio cooling/heating  in range < 0, 1  >
* float zone_high [0-100] control zone high
* float zone_low [0 -100] control zone low 

* Function PT

 * parameters:
 * float in input signal
 * float out  filtered output
 * float T[s] time constant 
 * 

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
* see:
* https://github.com/KorneliusThomas/PIDcontrollersModularProfessional
*/

#include <IPID.h>
#include<ConFunct.h>
Takt100 takt100;
Takt1000 takt1000;
IPID con1;   
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

//switch impuls controller to auto
con1.auto_man=true;

//controller parameter


con1.zone_high=20;
con1.zone_low=20;
con1.gain=6 ;    // optimal threepoint =3, twopoint =3 
con1.Ti=20;//according to Ziegler-Nichols Ti = 2*deadttime
con1.Td=5;//according to Ziegler-Nichols Td=deadtime/2
con1.period=10;
con1.min_imp=1;
con1.cool_to_heat=1.0;
con1.three_two=1;

//PT - control plant simulation

pt1.T=100;            // time constant PT element = 50s


//Dead - control plant simulation

dead1.start=1;       //initialize Dead
dead1.deadtime=10;  // Takt1000 1 s sample time, deadtime= 10s

//SetTimeTable

//put values into time table setpoints for controller

table1.start=1; //initialize SetTimeTable

table1.TimeMin[0]=5;table1.TimeMin[1]= 8;table1.TimeMin[2]=10; table1.TimeMin[3]=12;table1.TimeMin[4]=13;table1.TimeMin[5]=15;table1.TimeMin[6]=18; table1.TimeMin[7]=20;
table1.SetArray[0]=30;table1.SetArray[1]=20;table1.SetArray[2]=55;table1.SetArray[3]=70;table1.SetArray[4]=30;table1.SetArray[5]=60;table1.SetArray[6]=20;table1.SetArray[7]=70;


//SetJpRamp - replacing setpoints values from table through ramping

sramp1.start=1;
sramp1.speed_higher=0.5; //ramping speed high
sramp1.speed_lower=0.5;  //ramping speed lowe

}

void loop() {

takt1=takt100.fupdate();   //calling 100ms takt
takt2=takt1000.fupdate();  //calling 1s takt


///===========time sampling 100ms
if (takt1==1){

con1.cupdate( );   //call controller IPID
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


Serial.println(con1.set+String(" ")+con1.act+String(" ")+con1.out+String("  ")+10*con1.heating+String("  ")+10*con1.cooling);  //look for simulation results in plotter

            }

}
