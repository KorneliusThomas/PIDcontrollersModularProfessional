/*
* continuous PID controller
 *
 *parameters:
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

 * parameters:
 * float in input signal
 * float out  filtered output
 * float T[s] time constant 
 * 
  *see:
 *https://github.com/KorneliusThomas/PIDcontrollersModularProfessional

 */


#include <CPID.h>
#include<ConFunct.h>
Takt100 takt100;
Takt1000 takt1000;
CPID con1;   
PT   pT1;
//
//variables for time sampling
//

int takt1;
int takt2;

int analogin;
int analogout;


void setup() {
//define voltage for analog input
analogReference(DEFAULT);

//define digital pin 5 as analog output
pinMode(5,OUTPUT);

Serial.begin(9600); //start serial commmunication

//filter time

pT1.T=2;


// continous controller parameter

con1.gain=6;  
con1.Ti=20;  
con1.Td=5;


}

void loop() {

takt1=takt100.fupdate();   //calling 100ms takt
takt2=takt1000.fupdate();  //calling 1s takt
//switch controller auto/man

con1.auto_man=digitalRead(2);
//pushbuttons to manipulate controller output
con1.out_up=digitalRead(3);
con1.out_down=digitalRead(4);


//provide actual value and setpoint

analogin=analogRead(0);
pT1.in=analogin/10.23; //convert from range 0 - 1023 to 0 - 100 and connect to filter input
con1.act=pT1.out;


con1.set=50;

//controller output to digital PDM modulated output 5

analogout=con1.out*2.55; //convert from range 0 - 100 to 0 - 255;
analogWrite(5,analogout);

///===========time sampling 100ms
if (takt1==1){

con1.cupdate( );   //call controller CPID
pT1.fupdate( );
}

///===========time sampling 1s
if (takt2==0){


Serial.println(con1.set+String("  ")+con1.act+String(" ")+con1.out+String("  "));  //look for simulation results in plotter

            }
}
