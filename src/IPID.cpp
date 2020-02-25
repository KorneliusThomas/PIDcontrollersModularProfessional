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
#include <IPID.h>


void IPID:: cupdate ()
{


//internal
float variab1;
float I_last;

//for integration stopping
I_last=I;

//calculate control difference

diff=set-act;

//manual/auto

if( auto_man==false ){

//==============controller in manual mode


 tracking=0;  // prepare next switching into auto 

// check button output higher

if ( butt_plu==true )
{
 plus: if ( out >=100.0 )
        { out=100.0;}
        
// increase output 0.1 in manual mode                                      
       else { out = out +0.1;}
  }

// check button output lower

if ( butt_min==true )

{ 
minu: if ( out <=0.0 )
         {out =0.0 ;}
       
// decrease output 0.1 in manual mode     
        else{ out = out - 0.1 ;}
           
}

                    }
          
//============controller in automatic mode
 
else {

// cotroller on automatic mode

if( tracking==false)// after switching to auto first tracking
{ // prepare auto mode

if ( cascade==true )// controller cascade ?
{ I=track-P; 
  out=track-P;} 
  
else{ I=out;} //     

IK=0;
DF=diff;// no D jump after switching to auto

//Imcount=0; // erase counter impulses  

tracking=true; 
return;
}


   
//=======check if zone limits exceeded.

if((-diff)>=zone_high)
{ out=0;      // heating off , cooling max for 3 point controller
 goto istop; } //stop integration and output of impulses

if( diff >=zone_low) 
{ out=100; //heating max, cooling off for 3 point controller
goto istop;} //stop integration and output of impulses


//calculate P

P=diff*gain;

//calcualte I

//check Ti

if ( Ti<=0 )//check Ti plausibility
{Ti=0.1;}//replace 


IK=IK+P*0.1/Ti;  //use Ik for calculating I

if ( I+IK !=I )
 { I=I+IK; IK=0.0;}//Ik is big enough, reset IK
   

if (Td<=0) // if controller parameter Td time zero - no filtr for D part.Td negative replace through Td=0
     {D=0; variab1=0;
      goto ou;}

//calculate D according to formula pTd(1+pTd)

variab1 = ( diff - DF )*0.1/Td +DF ;
D = ( variab1 - DF )* Td * gain * 10;

ou: DF=variab1 ;

//calculate PID output
//
out=P+I+D;

// limitation of controller output from 0 to 100.0 and integration stopping

istop:

if ( out >=100.0 ){
   out = 100.0 ;
//stop integration 
IK=0; 
I=I_last;
                  }

 if ( out <=0 ){
   out = 0.0 ;
//stop integration 
  IK=0;
  I=I_last;
               }

  }

impu();

};


void IPID:: impu ()
{

///===============================================================
///===============================================================
/// conversion of continous controller output into heating and cooling pulses

Imcount=Imcount+0.1;               //100 ms sample time

if( Imcount > period)             //impuls counter maximal value period is exceeded
{Imcount = 0.1; }


if (three_two==false) {

//===============================
// two points controller

if( out<min_imp)
 { heating=false; //heating permanent off - eliminate to short heating pulses
 return;}

 if (out>100-min_imp)
 { heating=true; //heating permanent on - eliminate short pauses between heating pulses 
   return; }
 
 // calculate heating pulses

if ( Imcount < out*period/100)
   {heating=true;}

   else {heating=false;}
  
  return;
                      }



//three points controller - cooling and heating
 
 
// cooling
 if (out <=50 )
{
      if (out>50-min_imp )
     { cooling=false; //cooling permament off - eliminate too short cooling pulses
       return; } 
      
      if (out<min_imp)
     {cooling=true; //cooling permament on - eliminate too short pauses between cooling pulses
      return; }

//calculate cooling pulses

//check ratio cooling to heating - must be smaller or equals 1
     
      if (cool_to_heat > 1) //check plausibility, replace 
      { cool_to_heat=1; }
   
      if ( Imcount< cool_to_heat*(50-out)*period/50 )
      {cooling=true;}
       
      else { cooling=false;}
       
      return;
}     
      
else  {

//heating

      if (out<50+min_imp )
      {heating=false; //heating permanent off - eliminate too short heating pulses
       return;}
      if (out>100-min_imp)
      {heating=true; //heating permanent on - eliminate too short pauses between heating pulses
       return;}
//calculate heating pulses

      if ( Imcount< (out-50)*period/50 )
      {heating=true;}
      else
      { heating=false;}
 
     return;
   
 }

};




