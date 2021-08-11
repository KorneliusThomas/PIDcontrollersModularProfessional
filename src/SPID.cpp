/* Copyright 2019 Kornelius Thomas, Germany
 * 
 * Kornelius_Thomas@yahoo.com
 *
 *see:
 *https://github.com/KorneliusThomas/PIDcontrollersModular
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */



#include "Arduino.h"
#include <SPID.h>


void SPID:: cupdate ()

{




//internal variabels

float diffhyst;
float var1;

diff=set-act;
  

// check bit manual/auto




if (auto_man==false) {

//==============controller in manual mode

switch_to_auto =false;   //prepare switching to auto and setup


//check button +
if (butt_plu==true&&butt_min==false)
  {motor_plu=true; // digital output motor +
   motor_min=false; //digital output motor - 
   return;}
  

//check button -
if (butt_min==true&&butt_plu==false)
  {motor_plu=false; // digital output motor +
   motor_min=true; //digital output motor - 
   return;} 
//remaining button combinations, both pressed or no button
//pressed - motor outputs zero
motor_plu=false;
motor_min=false; 
return; 
            }


//====================================================
                         
         
else {

 if (switch_to_auto==false)  //first setup ?
{
  hystbigger=false;
  DTI=0;
  diffhyst_last=0;
  DxDfilt=0;
  switch_to_auto=true; return;}  
 
//============controller in automatic mode

// check control difference 
// diffhyst as DxPID input, equals zero or ( setpoint - actual value )
 

diffhyst=DiffHyst();



//filter for diffhyst - mean value 15 last
   
diffhyst=(diffhyst+14*diffhyst_last)/15;




//calculate P x D


   PxD= gain*(diffhyst- diffhyst_last);


//calculate I x D


  
if(Ti<=0) //check Ti plausibility,replace if 0 through 0.1
  {Ti=0.1;}




  IxD=gain*diffhyst/(10*Ti); //Ti*10 for 0.1 s cycle
  
 

// calculate DxD


  if(Td<=0) //check Td plausibility, erase DxD if Td <=0
  { DxD=0;
   DxDfilt=0;
   diffhyst_last=diffhyst;
   goto CheckLimit;}

   var1=((diffhyst- diffhyst_last) - DxDfilt)/(10*Td) +DxDfilt;
   DxD=gain*(var1- DxDfilt)*Td*10;//Td*10 is used because of 0.1s cycle
   
   DxDfilt=var1;
   diffhyst_last=diffhyst;

CheckLimit:


//check limit switches - if on, stop impulse output if needed

if ( limit_plu==true ) // limit switch + ?
  {
   
    if ( DTI > 0) 
        {IxD=0;         // limit switch + and DTI last counter goto + , erase IxD new
         DTI=0;         // stop integration

        goto PID;}
   
       else{goto PID;}
  
 }
    


if ( limit_min==true )// limit switch - ?
  {     
 //check old value DTI    

  if (DTI < 0 ) 
        {IxD=0;         // limit switch - and DTI last counter - , erase IxD new
         DTI=0;}         // stop integration
   
  }      


//// calculate Dx(PID) algoritm and convert into motor running time DTI

PID:

   DTI= DTI + motime*(PxD+IxD+DxD)/100;  //  

/// limiting running time of motor from - motime to + motime

    if ( DTI > motime)
       {DTI=motime;}
    if ( DTI < -motime)
       {DTI= - motime;}



///
// maybe new commands + -  ? - how big is DTI ?
run:

if ( DTI >=mimp){  //go direction +
  
      motor_plu=true;
      motor_min=false;
      DTI=DTI-0.1 ;   // counter steps -0.1 s
      return;
                }

if ( -DTI >=mimp){
 
       motor_min=true;//go direction -
       motor_plu=false;
       DTI=DTI+0.1 ;   // counter steps +0.1 
       return;
                 }


// no new commands because DT1 is too small, -mimp < DTI < mimp
motor_min=false;
motor_plu=false;

}
}

float SPID::DiffHyst()

{

//=================Hysteresis for DxPID controller input======


float diffhyst;



if (act- set>=0){             
 ////////////actual value is bigger setpoint
  
       if (act- set>=onvalue )
       { hystbigger=true; diffhyst=set-act; return diffhyst; }  
  
      if (act-set>offvalue && act-set<onvalue && hystbigger==true)
        { diffhyst=set-act;return diffhyst;  }

      if ( act-set<=offvalue  )
      { hystbigger=false; diffhyst=0;return diffhyst;}

      if ( act-set >offvalue && act-set <onvalue && hystbigger==false)
        { diffhyst=0;return diffhyst;}
}

if (set- act>0){             
 ////////////actual value is smaller setpoint
  
       if (set-act>=onvalue )
       { hystbigger=true; diffhyst=set-act;return diffhyst;}  
  
      if (set-act>offvalue && set-act<onvalue && hystbigger==true)
        { diffhyst=set-act; return diffhyst;}

      if ( set-act<=offvalue  )
      { hystbigger=false; diffhyst=0;return diffhyst;}

      if ( set-act>offvalue && set-act <onvalue && hystbigger==false)
        { diffhyst =0;return diffhyst;}
}

}






