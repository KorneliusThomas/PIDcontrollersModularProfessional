#ifndef ConFunct_h
#define ConFunct_h
#include "Arduino.h"

//====================================
//takt generator 1s - after every 200 ms function equals 0,1,2,3,4,otherwise = 5
//values 0,1,2,3,4 are appear every 1s.
//Call only once Takt1000 instance in the loop and ask return value calling controllers like
//this:
//#include <ConFunct_h>
//Takt1000 takt1000;  //takt1000 is instance class Takt1000
//int value;
//
//loop (
//
//value=takt1000.update();
//if (value ==0 )
//{ con1.cupdate(); }     //instance 1 of controller is processed every second
//if (value ==1)
//{ con2.cupdate(); }     //instance 2 of controller is processed 100ms later every second
//      )
//This way secures good sharring of processing power in case of many controllers.
//
//



class Takt1000
{
public:

int fupdate();
int takt200;
//internal
unsigned int long millisec_old;

};

//==================================
//takt generator 100ms - after every 20ms function equals 0,1,2,3,4,otherwise = 5
//values 0,1,2,3,4 appear every 100ms.
//Call only once Takt100 instance in the loop and ask return value calling controllers like
//this:
//#include <ConFunct_h>
//Takt100 takt100;  //takt100 is instance class Takt100
//int value;
//
//loop (
//
//value=takt100.update();
//if (value ==0 )
//{ con1.cupdate(); }     //instance 1 of controller is processed every 100ms
//if (value ==1)
//{ con2.cupdate(); }     //instance 2 of controller is processed 20 ms later every 100ms
//     )
//This way secures good sharring of processing power in case of many controllers.



class Takt100
{
public:

int fupdate();


int takt20;

//internal
unsigned int long millisec_old;

};


//===================================
//mean value for filtering controller actual value
// out=(in + 7*out last)/8


class Mean
{
public:

void fupdate();

float in;
float out;
bool start;

};
//================================
// low pass filter PT1 element
// Laplace function 1/(1 + pT)

class PT

{
public:
  
void fupdate ();


//PT data

float in;
float out;

float T;

};

//==================================
// Diff element
// Laplace function pT/(1 + pT)

class Diff
{
public:

void fupdate ();

//Diff data

float in;
float out;
float T;

//internal
float DF;
};

//==================================
//dead time element
// Laplace function (e)-p*dead

class Dead
{
public:

void fupdate();

float in;
float out;
int deadtime;

float inArray[31] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

bool start;
//internal
bool start_last;

};


//=================================
//Integration of input signal
//used for simulation of servo motor

class Int
{
public:

void fupdate ();


float in;
float out;

float Ti;

};


//==================================================
//Linerization of input signal in
//

class Linear

{

public:

void fupdate ();

//Linear data

float LinArr[21]={0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100};
float in;
float out;

};

//hysteresis for setpoint - actual value

class Hyst
{
public:

void fupdate();

float act;
float set;
float out;
float onvalue;
float offvalue;

//internal
bool hystbigger; 

};


//==================================================
//setpoint generator
//

class SetGen

{

public:

void fupdate ();


//SetGen data


float set;

bool butt_up;
bool butt_down;


float set_slow;
float set_fast;
float switch_time;

//internal

float time_counter;
float set_speed;
};



//=====================================================
//Setpoint jump replaced through a ramp
//

class SetJpRamp

{

public:

void fupdate ();
//data

float act; 
float set; 
float out;
float speed_higher;
float speed_lower;

bool start;
//internal
bool setup;
bool runlower;
bool runhigher;



float set_last;

};
//====================================================
//setpoint is generated on set output according to a schedule
//stored in TimeMin, TimeSec, SetArray
//


class SetTimeTable

{

public:

void fupdate ();

//

float set;
int TimeMin[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int TimeSec[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
float SetArray[20]={50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};
bool start;
//internal

bool setup;

int timestep;
int iset;
int TimeArray_last;


};



#endif







