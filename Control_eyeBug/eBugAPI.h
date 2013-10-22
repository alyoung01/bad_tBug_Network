// $Id: eBugAPI.h,v 1.3 2013/02/01 05:43:13 ahmet Exp $

/************************************************************************/
/* eBugAPI.h                                                            */
/*                                                                      */
/* Nick D'Ademo <nickdademo@gmail.com>                                  */
/*                                                                      */
/* Copyright (c) 2012 Nick D'Ademo                                      */
/*                                                                      */
/* Permission is hereby granted, free of charge, to any person          */
/* obtaining a copy of this software and associated documentation       */
/* files (the "Software"), to deal in the Software without restriction, */
/* including without limitation the rights to use, copy, modify, merge, */
/* publish, distribute, sublicense, and/or sell copies of the Software, */
/* and to permit persons to whom the Software is furnished to do so,    */
/* subject to the following conditions:                                 */
/*                                                                      */
/* The above copyright notice and this permission notice shall be       */
/* included in all copies or substantial portions of the Software.      */
/*                                                                      */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF   */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                */
/* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS  */
/* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   */
/* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN    */
/* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     */
/* SOFTWARE.                                                            */
/*                                                                      */
/************************************************************************/

#ifndef EBUGAPI_H
#define EBUGAPI_H

#include <stdint.h>
#include <vector>

char calculateChecksum(std::vector<char> packetIn);

///////////////////
// STEPPER MOTOR //
///////////////////
std::vector<char> StepperMotorLeftStep( uint16_t freqHz, uint16_t nSteps, bool directionForward, uint8_t stepMode, bool rgbON,
                                                bool xBee, uint8_t options);
std::vector<char> StepperMotorRightStep(uint16_t freqHz, uint16_t nSteps, bool directionForward, uint8_t stepMode, bool rgbON,
                                                bool xBee, uint8_t options);
std::vector<char> StepperMotorLeftRightStep(uint16_t freqHz1, uint16_t nSteps1, bool directionForward1, uint8_t stepMode1, bool rgbON1,
                                                uint16_t freqHz2, uint16_t nSteps2, bool directionForward2, uint8_t stepMode2, bool rgbON2,
                                                bool xBee, uint8_t options);
std::vector<char> StepperMotorStopBoth(bool disable1, bool disable2, bool xBee, uint8_t options);
std::vector<char> StepperMotorStopLeft(bool disable, bool xBee, uint8_t options);
std::vector<char> StepperMotorStopRight(bool disable, bool xBee, uint8_t options);
std::vector<char> StepperMotorCWStep(   uint16_t freqHz, uint16_t nSteps, bool directionForward, uint8_t stepMode, bool rgbON,
                                                bool xBee, uint8_t options);
std::vector<char> StepperMotorCCWStep(  uint16_t freqHz, uint16_t nSteps, bool directionForward, uint8_t stepMode, bool rgbON,
                                                bool xBee, uint8_t options);

//////////////
// RGB LEDs //
//////////////
std::vector<char> TLC5947_SetAllRed(uint16_t value, bool xBee, uint8_t options);
std::vector<char> TLC5947_SetAllGreen(uint16_t value, bool xBee, uint8_t options);
std::vector<char> TLC5947_SetAllBlue(uint16_t value, bool xBee, uint8_t options);
std::vector<char> TLC5947_SetAll(uint16_t value, bool xBee, uint8_t options);
std::vector<char> TLC5947_SetAllOff(bool xBee, uint8_t options);
std::vector<char> TLC5947_SetRed(uint8_t led, uint16_t value, bool xBee, uint8_t options);
std::vector<char> TLC5947_SetGreen(uint8_t led, uint16_t value, bool xBee, uint8_t options);
std::vector<char> TLC5947_SetBlue(uint8_t led, uint16_t value, bool xBee, uint8_t options);
std::vector<char> TLC5947_SetMultiple(uint16_t mask1, uint16_t mask2, uint16_t mask3, uint16_t value, bool xBee, uint8_t options);
std::vector<char> TLC5947_SetMultipleHold(uint16_t mask1, uint16_t mask2, uint16_t mask3, uint16_t value, uint16_t mask, bool xBee, uint8_t options);
std::vector<char> TLC5947_SetAllHold(uint16_t value, uint16_t mask, bool xBee, uint8_t options);

/////////
// ADC //
/////////
std::vector<char> LM35_GetTemperature(bool xBee, uint8_t options);
               
#endif // EBUGAPI_H

