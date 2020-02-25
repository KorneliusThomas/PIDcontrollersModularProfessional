#ifndef CPID_h
#define CPID_h
#include "Arduino.h"

//continous controller

class CPID

{
public:
  
void cupdate ();


//controller data

float act;
float set;
float diff;
float out;

float track;


bool auto_man; 
bool out_up;
bool out_down;
bool cascade;
bool tracking;



float gain;
float Ti ;
float Td ;

//internal
float P ;
float I ;
float D ;
float DF; 
float IK;

};

#endif







