/*
impulse PID controller
*
* parameters:
* float act [0 -100] actual value
* float set [0 -100] setpoint
* float diff [-100, 100] control difference
* float out [0 -100] continous output 
* float track [ 0 -100] tracking value, only for cascade

* bool auto_man switching auto/manual
* bool butt_pl push button direction plus
* bool butt_min push button direction minus
* bool heating output heating
*bool cooling output cooling

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

 * parameters
 * float in input signal
 * float out  filtered output
 * float T[s] time constant 
 * 
  *see:
 *https://github.com/KorneliusThomas/PIDcontrollersModularProfessional

 */

#include <IPID.h>
#include<ConFunct.h>
Takt100 takt100;
Takt1000 takt1000;
IPID con1;   
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

//filter time

pT1.T=2;



// controller parameter

con1.zone_high=20;
con1.zone_low=20;

con1.gain=6 ;    
con1.Ti=20;
con1.Td=5;
con1.period=10;
con1.min_imp=0.1;
con1.cool_to_heat=1.0;
con1.three_two=1;

}

void loop() {

takt1=takt100.fupdate();   //calling 100ms takt
takt2=takt1000.fupdate();  //calling 1s takt

//switch controller auto/man

con1.auto_man=digitalRead(2);
//pushbuttons to manipulate controller output
con1.butt_plu=digitalRead(3);
con1.butt_min=digitalRead(4);


//provide actual value and setpoint

pT1.in=analogRead(0)/10.23; //convert from range 0 - 1023 to 0 - 100;
con1.act=pT1.out;
con1.set=50;


//controller digital outputs
digitalWrite(5, con1.heating);
digitalWrite(6, con1.cooling);




///===========time sampling 100ms
if (takt1==1){

con1.cupdate( );   //call controller CPID
pT1.fupdate( );   //call PT1 filter
}

///===========time sampling 1s
if (takt2==0){

Serial.println(con1.set+String(" ")+con1.act+String(" ")+con1.out+String(" ")+con1.heating*10+String("  ")+con1.cooling*10);  //look for simulation results in plotter

            }
}
