 /* Copyright 2019 Kornelius Thomas, Germany
 *
 *   Kornelius_Thomas@yahoo.com
 *see:
 *https://github.com/KorneliusThomas/PID controllers Modular

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
#include <ConFunct.h>



//====================================
//takt generator 1s - after every 200 ms function equals 0,1,2,3,4,in the time between takt = 5
// it means, value 0,1,2,3,4 are comming every 1s
//Call only once Takt1000 instance in the loop and later ask return value calling controllers like
//this:

//value=takt1000.fupdate;
//if (value ==0 )
//{ con1.cupdate; }     //instance 1 of controller is processed every second
//if (value ==1)
//{ con2.cupdate; }     //instance 2 of controller is processed 100ms later every second
//Only on this way processor time is optimaly used.

int Takt1000::fupdate()
{


// create 200 ms cycle 


if ( millis() - millisec_old >= 200 ) 
{

 millisec_old=millis();

takt200++;
 
  if (takt200==5)
     {takt200=0;}
return takt200;
}                               

else {
return 5;
}

}


//==================================
//takt generator 100ms - after every 20ms function equals 0,1,2,3,4,in the time between takts = 5
//it means value 0,2,3,4 are comming every 100ms
//Call only once Takt100 instance in the loop and later ask return value calling controllers like
//this:

//value=takt100.update;
//if (value ==0 )
//{ con1.update; }     //instance 1 of controller is processed every 100ms
//if (value ==1)
//{ con2.update; }     //instance 2 of controller is processed 20 ms later every 100ms
//Only on this way processor time is optimaly used.


int Takt100::fupdate()
{

// create 20 ms cycle 


if ( millis() - millisec_old >= 20 ) 
{

 millisec_old=millis();

takt20++;
 
  if (takt20==5)
     {takt20=0;}
return takt20;
}                               

else {
return 5;
}

}


//===================================
//mean value for filtering controller actual value
// out=(in + 7*out)/8


void Mean::fupdate()
{
//
//new start ?

if (start==false)
{start=true;
out=in;}


out=(in+7*out)/8; //calculate new output

}
//==================================================
//low pass filter PT1 element
//Laplace function 1/(1+pT)

void PT:: fupdate ()

{

//PT data


out = 0.1*(in - out)/T + out;

  
 }



//======================================================


// Diff element
// Laplace function pT/(1 + pT)


  
void Diff::fupdate ()
{

//Diff data


//intern

float Dd0;
//diff data

if (T==0) // if parameter T time zero - no filtr for D part
     {
      out=0;
      Dd0=0;
      DF=Dd0;
      return;}


//calculate D according to formula pTd(1+pTd)

Dd0 = 0.1*( in - DF )/T +DF ;
out= ( Dd0 - DF )* T*10;

DF=Dd0 ;


}
//dead time element
// e-p*dead


void Dead::fupdate()
{

//intern

int i;

//plausibility control of parameter deadtime

if (deadtime<0) // if parameter deadtime smaller zero - put zero
     { deadtime=0;}
if (deadtime>30) // if parameter deadtime bigger 30 - put 30
     { deadtime=30;}

//check initialization

if ( start==true )

{ i=0;

start=false;
  
 while( i<=deadtime )
 {inArray[i]=in; i++;}  //write into inArray deadtime values of input in
}



//one second gone - shift all stored input values

inArray[30]=inArray[29];
inArray[29]=inArray[28];
inArray[28]=inArray[27];
inArray[27]=inArray[26];
inArray[26]=inArray[25];
inArray[25]=inArray[24];
inArray[24]=inArray[23];
inArray[23]=inArray[22];
inArray[22]=inArray[21];
inArray[21]=inArray[20];
inArray[20]=inArray[19];
inArray[19]=inArray[18];
inArray[18]=inArray[17];
inArray[17]=inArray[16];
inArray[16]=inArray[15];
inArray[15]=inArray[14];
inArray[14]=inArray[13];
inArray[13]=inArray[12];
inArray[12]=inArray[11];
inArray[11]=inArray[10];
inArray[10]=inArray[9];
inArray[9]=inArray[8];
inArray[8]=inArray[7];
inArray[7]=inArray[6];
inArray[6]=inArray[5];
inArray[5]=inArray[4];
inArray[4]=inArray[3];
inArray[3]=inArray[2];
inArray[2]=inArray[1];
inArray[1]=inArray[0];


inArray[0]=in;

out=inArray[deadtime]; //output of stored value shifted by dead time seconds

}


//===================================
//integration of input
// used for simulation of servo motor


void Int::fupdate ()

{

if (Ti<=0 )
{Ti=0.1;}  //replace Ti <=0

out = out + in*0.1/Ti;

//limitting

if ( out >=100 )
{out=100;}

if (out <=0)
{out=0;}
}


//==================================================
//Linearization of input signal in
//
void Linear::fupdate ()

{

//Linear data data


//internal variable

float limup;
float limdown;
int pos;

limup=LinArr[20];
limdown=LinArr[0];

if( in >limup || in <limdown)
//outside range

{ return;}   

//inside range - find the position 0 - 20

else {
pos=21*(in-limdown)/(limup-limdown);  //posistion on x axis 0 - 20

// calculate value out=f(in) of nonlinear function defined through values in LinArray by aproximation 
out=LinArr[pos-1]+(LinArr[pos]-LinArr[pos-1])*(in - 5*(pos-1 ))/5; // calculate out=f(in) //for given in value
}

}

//=================Hysteresis 

void Hyst:: fupdate ()

{

//=================hysteresis between set and act======




if (act- set>=0){             
 ////////////actual value is bigger setpoint
  
       if (act- set>=onvalue )
       { hystbigger=true; out=set-act; return; }  
  
      if (act-set>offvalue && act-set<onvalue && hystbigger==true)
        { out=set-act;return;  }

      if ( act-set<=offvalue  )
      { hystbigger=false; out=0;return;}

      if ( act-set >offvalue && act-set <onvalue && hystbigger==false)
        { out=0;return;}
}

if (set- act>0){             
 ////////////actual value is smaller setpoint
  
       if (set-act>=onvalue )
       { hystbigger=true; out=set-act;return;}  
  
      if (set-act>offvalue && set-act<onvalue && hystbigger==true)
        { out=set-act; return;}

      if ( set-act<=offvalue  )
      { hystbigger=false; out=0;return;}

      if ( set-act>offvalue && set-act <onvalue && hystbigger==false)
        { out=0;return;}
}

}



//==================================================
//setpoint generator
//

void SetGen::fupdate ()

{

//SetGen data



//increase time counter if one button pressed
if ( butt_up==true||butt_down==true)
{time_counter=time_counter + 0.1;}
else
{time_counter=0;}  // no button pressed

//chose setpoint fast speed after switch time
if (time_counter>=switch_time)
   {set_speed=set_fast/10;}
else
   {set_speed=set_slow/10;}

// check button setpoint higher

if ( butt_up==true )

{
 plus: if ( set >=100.0 )
        { set=100.0;
         return;}
 
// increase output 0.1 in manual mode                                      
       else{ 
            set= set +set_speed;
            return;
            }
}

// check button setpoint lower

if ( butt_down ==true )

{ 
minu: if ( set <=0.0 )
         {set =0.0 ;
          return;}

// decrease output 0.1 in manual mode     
        else{ 
            set = set - set_speed ;
            return;
            }
}

}                            




//=====================================================
//Setpoint jump replaced through a ramp
//
void SetJpRamp::fupdate ()
{


// This function replaces setpoint jump through a ramp.After a setpoint jump
// the value on output out goes from the actual value to the setpoint with a given speed speed_higher or speed_lower.
// On this way bumping of setpoint is eliminated and replaced through the smoothie ramp.


//is function SetJpRamp started ?


if(start==false||setup==false) //no start ? setup isnt done ?
{
//no start or setup is done

runlower=false;
runhigher=false;
setup=true;

out=set;      //bypass setpoint
          
set_last=set;
return;

}




if ( set==set_last)
{ goto some_running; } //no change of setpoint, maybe a ramping is running ?

//There is a new setpoint - is any ramping already runnig ?

if ( runlower==false && runhigher==false)// no ramping is running - start ramp
   {  
                       
// there is a new setpoint, no ramping running - look at the position of setpoint.
               
                 if (set > act ) //
                 //initialize ramping higher
                { runhigher=true;// set bit ramping goes higher;
                  runlower=false;
                  out = act;
                  goto some_running; }
               
                 if (set < act ) //
                 //initialize ramping lower               
                 {runlower=true;// set bit ramping goes lower;
                  runhigher=false;
                  out = act;
                  goto some_running;}
   }             
             else {
//===================change of setpoint and one ramping is running - what ramping, where is the setpoint now ?

           if (runhigher==true )// ramping was direction higher ?
             {
                  if ( out >set ) {
                  // initialize new ramping direction -
                runlower=true; //set bit ramping lower;
                runhigher=false;//reset bit
                out = act;
                goto some_running;
                    }
             }                           
              if (runlower==true )// ramping was direction lower ? 
                 {
                     if ( set >out ) {
                  // initialize new ramping direction +
                 runhigher=true; // set bit ramping goes higher;
                 runlower=false; //reset bit lower
                 out = act;
                 goto some_running;                
                    }
           
                }
             }

//=========================some ramping is running 
                 some_running:
 //What ramping is running ?
                   if ( runhigher==true)
                    {
                    out = out + 0.1*speed_higher;
                 //ramping reached the setpoint ?
                    if ( out >= set )
                       {  out =set;   // equals the to actual value, stop ramping;
                        runhigher=false;//stop ramping
                        }
                    set_last=set;
                    return;
                    }
   
                    if(runlower==true)
                     {
                   out= out - 0.1*speed_lower;
                
                //ramping reached the setpoint ?
                  
                      if ( out <= set )
                      {  out =set;   // equals the to actual value, stop ramping;
                      runlower=false; //stop ramping
                       }
                    set_last=set;
                    return;
                     }
set_last=set;
}

//==================================================
//setpoint according to schedule stored in Array TimeSec, TimeMin, SetArray
//


void SetTimeTable::fupdate ()

{

//internal variable

int TimeArray_new;

//start or setup ?

if(start==false|| setup==false) // no start ? setup isnt done ?

{TimeArray_last=0;
iset=0;
timestep=0;
set=SetArray[0];
setup=true;
return;}



TimeArray_new=TimeSec[iset]+TimeMin[iset]*60;



if ((TimeArray_new> 0)==true && (iset <=19)==true&&(TimeArray_last<=TimeArray_new)==true)

{
  
 
  TimeArray_last=TimeArray_new;


             
  if ( TimeArray_new==timestep)
     { 
 
     TimeArray_new=TimeSec[iset+1]+TimeMin[iset+1]*60;    


     if (TimeArray_new>0 )

     {set=SetArray[iset+1];}  //setpoint output +1 only if next time >0
     iset=iset+1;
     }

timestep=timestep+1;

return;

}

start=false;     //plausibility negativ
return;

}






















