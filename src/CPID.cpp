 /* Copyright 2019 Kornelius Thomas, Germany
 *
 * Kornelius_Thomas@yahoo.com
 
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
#include <CPID.h>


void CPID:: cupdate ()

{


//internal variable


float variab1;
float I_last;

//store I for integration stop

I_last=I;




//calculate control difference

diff=set-act;

// bit manual/auto

if (auto_man==false){

//==============controller in manual mode


tracking=false;  // prepare next switching into auto 

// check button output higher

if (out_up==true )

{
 plus: if ( out >=100.0 )
        { out=100.0; return ;}
 
// increase output 0.1 in manual mode                                      
       else{ out = out +0.1; return ;}
        
}


// check button output lower

if ( out_down==true )

{ 
minu: if ( out <=0.0 )
         {out =0.0 ; return ;}

// decrease output 0.1 in manual mode     
       else{ out = out - 0.1 ; return;}
     

}
                       }   
          
//============controller in automatic mode
 
else{


// controller in automatic mode

if(tracking==false)// after switching to auto first tracking
{
 

if ( cascade==true )// controller cascade ?

{ I=track-P;
  out=track-P; } 

  else{ I=out;}


IK=0;
DF=diff ;  // no D jump after switching to auto
tracking=true;
return;
   
} 
//calculate P

P=diff*gain;

//calcualte I

//check Ti


if ( Ti<=0 ) //check plausibility.Replace through 0.1 
{Ti=0.1;}


IK=IK+P*0.1/Ti ;  //use Ik for calculating I
if ( I+IK !=I ){
 //Ik is big enough, reset IK
   I=I+IK;
   IK=0.0;
               }

if (Td==0) // if controller parameter Td time zero - no filtr for D part
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

if ( out >=100.0 )
{out = 100.0 ;
//stop integration 
 IK=0; 
 I=I_last;}
         
 if ( out <=0 )
  { out = 0.0 ;
//stop integration 
  IK=0;
  I =I_last;}
              

 }


}



