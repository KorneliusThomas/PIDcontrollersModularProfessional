#ifndef IPID_h
#define IPID_h
#include "Arduino.h"


//I controller

class IPID

{

public:
  
void cupdate ();
void impu();
float act;
float set;
float diff;
float out;

float track;

bool auto_man;
bool butt_plu;
bool butt_min;
bool heating;
bool cooling;

bool three_two;
bool cascade;

//internal
bool tracking;



float gain;
float Ti;
float Td;
float period;
float min_imp;
float cool_to_heat;
float zone_high;
float zone_low;

//internal
float Imcount;
float P;
float I;
float D;
float DF;
float IK;

};

#endif
