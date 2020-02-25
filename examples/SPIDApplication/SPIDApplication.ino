
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

 * parameters
 * float in input signal
 * float out  filtered output
 * float T[s] time constant 
 * 
  *see:
 *https://github.com/KorneliusThomas/PIDcontrollersModularProfessional

 */





#include <SPID.h>
#include<ConFunct.h>
Takt100 takt100;
Takt1000 takt1000;
SPID con1;   
PT pT1;
//
//variables for time sampling
//

int takt1;
int takt2;

void setup() {
//define voltage for analog input
analogReference(DEFAULT);
//define digital pin 5 and 6 as digital output
pinMode(5, OUTPUT);
pinMode(6, OUTPUT);
 
Serial.begin(9600); //start serial commmunication

//filter time constant

pT1.T=2;


// step controller parameter

con1.gain=6;  
con1.Ti=20;  
con1.Td=5;
con1.onvalue=0.5;
con1.offvalue=0;
con1.mimp=0.1;
con1.motime=10;

}

void loop() {

takt1=takt100.fupdate();   //calling 100ms takt
takt2=takt1000.fupdate();  //calling 1s takt

//switch controller auto/man

con1.auto_man=digitalRead(2);
//push buttons to manipulate controller output
con1.butt_plu=digitalRead(3);
con1.butt_min=digitalRead(4);
//limit switches
con1.limit_plu=digitalRead(6);
con1.limit_min=digitalRead(7);


//provide actual value and setpoint

pT1.in=analogRead(0)/10.23; //convert from range 0 - 1023 to 0 - 100;
con1.act=pT1.out;   //provide actual value
con1.set=50; //provide setpoint


//controller digital outputs
digitalWrite(5, con1.motor_plu);
digitalWrite(6, con1.motor_min);


///===========time sampling 100ms
if (takt1==1){

con1.cupdate( );   //call controller CPID
pT1.fupdate( );    //call filter
}

///===========time sampling 1s
if (takt2==0){

Serial.println(con1.set+String(" ")+con1.act+String(" ")+con1.motor_min*10+String("  ")+con1.motor_plu*10);  //look for simulation results in plotter

            }
}
