/** @file encoder.cpp
 *  @brief This file contains data relevant to interfacing with a rotary encoder
 *         content was used taken from Dejan Nedelkovski at  www.HowToMechatronics.com
 *  @author Original Dejan Nedelkovski
 *  @date 2022-11-01 Pulled code into .cpp and .h for easier use 
 *  
 */
#include <Arduino.h>
#include <encoder.h>

/*     Arduino Rotary Encoder Tutorial
 *      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */



encoder::encoder(int8_t out_a, int8_t out_b)
{
  pin1 = out_a;
  pin2 = out_b;
  counter = 0;
  

   pinMode (pin1,INPUT_PULLUP);
   pinMode (pin2,INPUT_PULLUP);
   
   // Reads the initial state of the outputA
   aLastState = digitalRead(pin1); 
}

int32_t encoder::measure(void)
{ 
  aState = digitalRead(pin1); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(pin2) != aState) 
     { 
       counter ++;
     } 
     
     else {
       counter --;
     }

   aLastState = aState; // Updates the previous state of the outputA with the current state
   return aState;
}



void encoder::reset(void)
{ 
  counter = 0;
}