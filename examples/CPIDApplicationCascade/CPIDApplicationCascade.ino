/*
* continuous PID controller.
* 
* parameters:
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

 * parameters
 * float in input signal
 * float out  filtered output
 * float T[s] time constant 
 
 *see:
 *https://github.com/KorneliusThomas/PIDcontrollersModularProfessional

 */




#include <CPID.h>
#include<ConFunct.h>
Takt100 takt100;
Takt1000 takt1000;
CPID con1;   
CPID con2;
PT  pT1;
PT pT2;

//
//variables for time sampling
//

int takt1;
int takt2;

int analogin_master;
int  analogin_follow;
int analogout_follow;


void setup() {
//define voltage for all analog inputs
analogReference(DEFAULT);
//define digital pin 5 as analog output
pinMode(5, OUTPUT);

 
Serial.begin(9600); //start serial commmunication

//PT low pass filter
pT1.T=2;
pT2.T=2;


con1.cascade=1;  //master controller cascade
con1.gain=6;  
con1.Ti=100;  
con1.Td=25;

//follow up controller parameter

con2.gain=6;  
con2.Ti=10;  
con2.Td=0;

}

void loop() {

takt1=takt100.fupdate();   //calling 100ms takt
takt2=takt1000.fupdate();  //calling 1s takt

//provide actual value and setpoint for master controller

analogin_master=analogRead(0);
pT1.in=analogin_master/10.23; //convert from range 0 - 1023 to 0 - 100;
con1.act=pT1.out;

con1.set=50;  // master controller setpoint

//follow up controller actual value and output
analogin_follow=analogRead(1);
pT2.in=analogin_follow/10.23;//convert from range 0 - 1023 to 0 - 100;
con2.act=pT2.out;

//follow up controller output to digital PDM modulated output 5

analogout_follow=con2.out*2.55; //convert from range 0 - 100 to 0 - 255;
analogWrite(5,analogout_follow);

//switching auto/man
con1.auto_man=digitalRead(2);
con2.auto_man=con1.auto_man;


//push buttons to manipulate follow up controller output
con2.out_up=digitalRead(3);
con2.out_down=digitalRead(4);

//connect controllers


con1.track=con2.act; // connect input tracking to follow up actual value
con2.set=con1.out;  //connect controllers, master output to follow up setpoint



///===========time sample 100ms
if (takt1==1){



con1.cupdate( );   //call master controller CPID
con2.cupdate( );   //call follow up controller CPID

pT1.fupdate( );
pT2.fupdate( );

}

///===========time sample 1s
if (takt2==0){

Serial.println(con1.set+String(" ")+con1.act+String(" ")+con1.out+String("  ")+con2.act+String("  ")+con2.out);  //look for simulation results in plotter

            }
}
