#ifndef SPID_h
#define SPID_h
#include "Arduino.h"

//step controller

class SPID

{
public:
  
void cupdate ();
float DiffHyst();

//controller data


float act;
float set;
float diff;

bool auto_man; 
bool butt_plu;
bool butt_min;
bool motor_plu;
bool motor_min;
bool limit_plu;
bool limit_min;


//internal variable
bool hystbigger;
bool switch_to_auto;


float gain;
float Ti;
float Td;
float onvalue;
float offvalue;
float mimp;
float motime;


//internal variable

float PxD;
float IxD;
float DxD;
float diffhyst_last;
float DxDfilt;
float DTI;


};

#endif







