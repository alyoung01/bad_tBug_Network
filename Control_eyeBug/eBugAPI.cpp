/************************************************************************/
/* eBugAPI.cpp                                                          */
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

#include "eBugAPI.h"

///////////////////
// Packet Format //
///////////////////
// Packet[0]:       Frame Start [XBee=0x00, Expansion Header=0x7E]
// Packet[1]:       Packet Length (number of bytes between Packet Length and Checksum)
// Packet[2]:       Options {Checksum Enable (1-bit), Class (4-bits), Reserved (3-bits)}
// Packet[3]:       Type
// Packet[4]:       Sub-Type
// Packet[5+n]:     Payload (n bytes)
// Packet[5+n+1]:   Checksum [optional: set to 0x00 if unused]

// NOTE: Class => 0x0 [eBug Command], 0x1 [eBug Response]
// The appropriate value should be set in Options when sending the packet.

char calculateChecksum(std::vector<char> packetIn)
{
    // Local variables
    int packetLength;
    int checksumTotal=0;
    // Save packet length
    packetLength=packetIn.at(1);
    // Calculate checksum (covers from Options to end of Payload)
    for(int i=0;i<packetLength;i++)
        checksumTotal+=packetIn.at(2+i);
    // Return checksum value
    return (0xFF-checksumTotal);
} // calculateChecksum()

///////////////////
// STEPPER MOTOR //
///////////////////
enum stepModes {
    FULL = 0,
    HALF = 1,
    QUARTER = 2,
    EIGHTH = 3,
    SIXTEENTH = 8
};

std::vector<char> StepperMotorLeftStep( uint16_t freqHz, uint16_t nSteps, bool directionForward, uint8_t stepMode, bool rgbON,
                                        bool xBee, uint8_t options)
{
    // RESPONSE: None
   
    // Packet has a fixed size of 11 bytes
    std::vector<char> packet(11);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x08;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x01;                 // Type: Stepper Motor
    packet[4]=0x00;                 // Sub-Type: stepperMotor1_step
   
    packet[5]=(freqHz>>8)&0xFF;     // Frequency (upper-byte)
    packet[6]=(freqHz)&0xFF;        // Frequency (lower-byte)
    packet[7]=(nSteps>>8)&0xFF;     // Number of steps (upper-byte)
    packet[8]=(nSteps)&0xFF;        // Number of steps (lower-byte)
    packet[9]=(stepMode&0xF)|(directionForward<<4)|(rgbON<<5);
   
    // Checksum
    if((options&0x80)==0x80)
        packet[10]=calculateChecksum(packet);  
    else
        packet[10]=0x00;
    // Return packet to caller
    return packet;
} // StepperMotorLeftStep()

std::vector<char> StepperMotorRightStep(uint16_t freqHz, uint16_t nSteps, bool directionForward, uint8_t stepMode, bool rgbON,
                                        bool xBee, uint8_t options)
{
    // RESPONSE: None
   
    // Packet has a fixed size of 11 bytes
    std::vector<char> packet(11);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x08;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x01;                 // Type: Stepper Motor
    packet[4]=0x01;                 // Sub-Type: stepperMotor2_step
   
    packet[5]=(freqHz>>8)&0xFF;     // Frequency (upper-byte)
    packet[6]=(freqHz)&0xFF;        // Frequency (lower-byte)
    packet[7]=(nSteps>>8)&0xFF;     // Number of steps (upper-byte)
    packet[8]=(nSteps)&0xFF;        // Number of steps (lower-byte)
    packet[9]=(stepMode&0xF)|(directionForward<<4)|(rgbON<<5);
   
    // Checksum
    if((options&0x80)==0x80)
        packet[10]=calculateChecksum(packet);
    else
        packet[10]=0x00;
    // Return packet to caller
    return packet;
} // StepperMotorRightStep()

std::vector<char> StepperMotorLeftRightStep(uint16_t freqHz1, uint16_t nSteps1, bool directionForward1, uint8_t stepMode1, bool rgbON1,
                                            uint16_t freqHz2, uint16_t nSteps2, bool directionForward2, uint8_t stepMode2, bool rgbON2,
                                            bool xBee, uint8_t options)
{
    // RESPONSE: None
   
    // Packet has a fixed size of 16 bytes
    std::vector<char> packet(16);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x0D;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x01;                 // Type: Stepper Motor
    packet[4]=0x05;                 // Sub-Type: stepperMotor12_step
   
    packet[5]=(freqHz1>>8)&0xFF;    // Frequency (upper-byte)
    packet[6]=(freqHz1)&0xFF;       // Frequency (lower-byte)
    packet[7]=(nSteps1>>8)&0xFF;    // Number of steps (upper-byte)
    packet[8]=(nSteps1)&0xFF;       // Number of steps (lower-byte)
    packet[9]=(stepMode1&0xF)|(directionForward1<<4)|(rgbON1<<5);
   
    packet[10]=(freqHz2>>8)&0xFF;   // Frequency (upper-byte)
    packet[11]=(freqHz2)&0xFF;      // Frequency (lower-byte)
    packet[12]=(nSteps2>>8)&0xFF;   // Number of steps (upper-byte)
    packet[13]=(nSteps2)&0xFF;      // Number of steps (lower-byte)
    packet[14]=(stepMode2&0xF)|(directionForward2<<4)|(rgbON2<<5);
   
    // Checksum
    if((options&0x80)==0x80)
        packet[15]=calculateChecksum(packet);  
    else
        packet[15]=0x00;
    // Return packet to caller
    return packet;
} // StepperMotorLeftRightStep()

std::vector<char> StepperMotorStopBoth(bool disable1, bool disable2, bool xBee, uint8_t options)
{
    // RESPONSE: None
   
    // Packet has a fixed size of 8 bytes
    std::vector<char> packet(8);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x05;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x01;                 // Type: Stepper Motor
    packet[4]=0x04;                 // Sub-Type: stepperMotor12_stopDisable
   
    packet[5]= disable1 ? 0x01 : 0x00;
    packet[6]= disable2 ? 0x01 : 0x00;
   
    // Checksum
    if((options&0x80)==0x80)
        packet[7]=calculateChecksum(packet);    
    else
        packet[7]=0x00;
    // Return packet to caller
    return packet;
} // StepperMotorStopBoth()

std::vector<char> StepperMotorStopLeft(bool disable, bool xBee, uint8_t options)
{
    // RESPONSE: None
   
    // Packet has a fixed size of 7 bytes
    std::vector<char> packet(7);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x04;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x01;                 // Type: Stepper Motor
    packet[4]=0x02;                 // Sub-Type: stepperMotor1_stopDisable
   
    packet[5]= disable ? 0x01 : 0x00;
   
    // Checksum
    if((options&0x80)==0x80)
        packet[6]=calculateChecksum(packet);    
    else
        packet[6]=0x00;
    // Return packet to caller
    return packet;
} // StepperMotorStopLeft()

std::vector<char> StepperMotorStopRight(bool disable, bool xBee, uint8_t options)
{
    // RESPONSE: None
   
    // Packet has a fixed size of 7 bytes
    std::vector<char> packet(7);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x04;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x01;                 // Type: Stepper Motor
    packet[4]=0x03;                 // Sub-Type: stepperMotor2_stopDisable
   
    packet[5]= disable ? 0x01 : 0x00;
   
    // Checksum
    if((options&0x80)==0x80)
        packet[6]=calculateChecksum(packet);    
    else
        packet[6]=0x00;
    // Return packet to caller
    return packet;
} // StepperMotorStopRight()

std::vector<char> StepperMotorCWStep(   uint16_t freqHz, uint16_t nSteps, bool directionForward, uint8_t stepMode, bool rgbON,
                                        bool xBee, uint8_t options)
{

    return StepperMotorLeftRightStep(freqHz,nSteps,true,stepMode,rgbON,freqHz,nSteps,false,stepMode,rgbON,xBee,options);
} // StepperMotorCWStep()

std::vector<char> StepperMotorCCWStep(  uint16_t freqHz, uint16_t nSteps, bool directionForward, uint8_t stepMode, bool rgbON,
                                        bool xBee, uint8_t options)
{

    return StepperMotorLeftRightStep(freqHz,nSteps,false,stepMode,rgbON,freqHz,nSteps,true,stepMode,rgbON,xBee,options);
} // StepperMotorCCWStep()

//////////////
// RGB LEDs //
//////////////
std::vector<char> TLC5947_SetAllRed(uint16_t value, bool xBee, uint8_t options)
{
    // RESPONSE: None
   
    // Packet has a fixed size of 8 bytes
    std::vector<char> packet(8);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x05;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x02;                 // Type: RGB LEDs
    packet[4]=0x00;                 // Sub-Type: TLC5947_SetAllRed
   
    packet[5]=(value>>8)&0xFF;
    packet[6]=(value)&0xFF;
   
    // Checksum
    if((options&0x80)==0x80)
        packet[7]=calculateChecksum(packet);    
    else
        packet[7]=0x00;
    // Return packet to caller
    return packet;
} // TLC5947_SetAllRed()

std::vector<char> TLC5947_SetAllGreen(uint16_t value, bool xBee, uint8_t options)
{
    // RESPONSE: None
   
    // Packet has a fixed size of 8 bytes
    std::vector<char> packet(8);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x05;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x02;                 // Type: RGB LEDs
    packet[4]=0x01;                 // Sub-Type: TLC5947_SetAllGreen
   
    packet[5]=(value>>8)&0xFF;
    packet[6]=(value)&0xFF;
   
    // Checksum
    if((options&0x80)==0x80)
        packet[7]=calculateChecksum(packet);    
    else
        packet[7]=0x00;
    // Return packet to caller
    return packet;
} // TLC5947_SetAllGreen()

std::vector<char> TLC5947_SetAllBlue(uint16_t value, bool xBee, uint8_t options)
{
    // RESPONSE: None
   
    // Packet has a fixed size of 8 bytes
    std::vector<char> packet(8);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x05;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x02;                 // Type: RGB LEDs
    packet[4]=0x02;                 // Sub-Type: TLC5947_SetAllBlue
   
    packet[5]=(value>>8)&0xFF;
    packet[6]=(value)&0xFF;
   
    // Checksum
    if((options&0x80)==0x80)
        packet[7]=calculateChecksum(packet);    
    else
        packet[7]=0x00;
    // Return packet to caller
    return packet;
} // TLC5947_SetAllBlue()

std::vector<char> TLC5947_SetAll(uint16_t value, bool xBee, uint8_t options)
{
    // RESPONSE: None

    // Packet has a fixed size of 8 bytes
    std::vector<char> packet(8);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x05;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x02;                 // Type: RGB LEDs
    packet[4]=0x03;                 // Sub-Type: TLC5947_SetAll
   
    packet[5]=(value>>8)&0xFF;
    packet[6]=(value)&0xFF;
   
    // Checksum
    if((options&0x80)==0x80)
        packet[7]=calculateChecksum(packet);    
    else
        packet[7]=0x00;
    // Return packet to caller
    return packet;
} // TLC5947_SetAll()

std::vector<char> TLC5947_SetAllOff(bool xBee, uint8_t options)
{
    // RESPONSE: None
   
    // Packet has a fixed size of 6 bytes
    std::vector<char> packet(6);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x03;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x02;                 // Type: RGB LEDs
    packet[4]=0x04;                 // Sub-Type: TLC5947_SetAllOff
   
    // Checksum
    if((options&0x80)==0x80)
        packet[5]=calculateChecksum(packet);    
    else
        packet[5]=0x00;
    // Return packet to caller
    return packet;
} // TLC5947_SetAllOff()

std::vector<char> TLC5947_SetRed(uint8_t led, uint16_t value, bool xBee, uint8_t options)
{
    // RESPONSE: None
   
    // Packet has a fixed size of 9 bytes
    std::vector<char> packet(9);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x06;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x02;                 // Type: RGB LEDs
    packet[4]=0x05;                 // Sub-Type: TLC5947_SetRed
   
    packet[5]=led;
    packet[6]=(value>>8)&0xFF;
    packet[7]=(value)&0xFF;
   
    // Checksum
    if((options&0x80)==0x80)
        packet[8]=calculateChecksum(packet);    
    else
        packet[8]=0x00;
    // Return packet to caller
    return packet;
} // TLC5947_SetRed()

std::vector<char> TLC5947_SetGreen(uint8_t led, uint16_t value, bool xBee, uint8_t options)
{
    // RESPONSE: None
   
    // Packet has a fixed size of 9 bytes
    std::vector<char> packet(9);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x06;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x02;                 // Type: RGB LEDs
    packet[4]=0x06;                 // Sub-Type: TLC5947_SetGreen
   
    packet[5]=led;
    packet[6]=(value>>8)&0xFF;
    packet[7]=(value)&0xFF;
   
    // Checksum
    if((options&0x80)==0x80)
        packet[8]=calculateChecksum(packet);    
    else
        packet[8]=0x00;
    // Return packet to caller
    return packet;
} // TLC5947_SetGreen()

std::vector<char> TLC5947_SetBlue(uint8_t led, uint16_t value, bool xBee, uint8_t options)
{
    // RESPONSE: None
   
    // Packet has a fixed size of 9 bytes
    std::vector<char> packet(9);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x06;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x02;                 // Type: RGB LEDs
    packet[4]=0x07;                 // Sub-Type: TLC5947_SetBlue
   
    packet[5]=led;
    packet[6]=(value>>8)&0xFF;
    packet[7]=(value)&0xFF;
   
    // Checksum
    if((options&0x80)==0x80)
        packet[8]=calculateChecksum(packet);    
    else
        packet[8]=0x00;
    // Return packet to caller
    return packet;
} // TLC5947_SetBlue()

std::vector<char> TLC5947_SetMultiple(uint16_t mask1, uint16_t mask2, uint16_t mask3, uint16_t value, bool xBee, uint8_t options)
{
    // RESPONSE: None
   
    // Packet has a fixed size of 14 bytes
    std::vector<char> packet(14);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x0B;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x02;                 // Type: RGB LEDs
    packet[4]=0x08;                 // Sub-Type: TLC5947_SetMultiple
   
    packet[5]=(mask1>>8)&0xFF;
    packet[6]=(mask1)&0xFF;
    packet[7]=(mask2>>8)&0xFF;
    packet[8]=(mask2)&0xFF;
    packet[9]=(mask3>>8)&0xFF;
    packet[10]=(mask3)&0xFF;
    packet[11]=(value>>8)&0xFF;
    packet[12]=(value)&0xFF;
   
    // Checksum
    if((options&0x80)==0x80)
        packet[13]=calculateChecksum(packet);  
    else
        packet[13]=0x00;
    // Return packet to caller
    return packet;
} // TLC5947_SetMultiple()

std::vector<char> TLC5947_SetMultipleHold(uint16_t mask1, uint16_t mask2, uint16_t mask3, uint16_t value, uint16_t mask, bool xBee, uint8_t options)
{
    // RESPONSE: None

    // Packet has a fixed size of 16 bytes
    std::vector<char> packet(16);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x0D;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x02;                 // Type: RGB LEDs
    packet[4]=0x09;                 // Sub-Type: TLC5947_SetMultipleHold
   
    packet[5]=(mask1>>8)&0xFF;
    packet[6]=(mask1)&0xFF;
    packet[7]=(mask2>>8)&0xFF;
    packet[8]=(mask2)&0xFF;
    packet[9]=(mask3>>8)&0xFF;
    packet[10]=(mask3)&0xFF;
    packet[11]=(value>>8)&0xFF;
    packet[12]=(value)&0xFF;
    packet[13]=(mask>>8)&0xFF;
    packet[14]=(mask)&0xFF;
   
    // Checksum
    if((options&0x80)==0x80)
        packet[15]=calculateChecksum(packet);  
    else
        packet[15]=0x00;
    // Return packet to caller
    return packet;
} // TLC5947_SetMultipleHold()

std::vector<char> TLC5947_SetAllHold(uint16_t value, uint16_t mask, bool xBee, uint8_t options)
{
    // RESPONSE: None
   
    // Packet has a fixed size of 10 bytes
    std::vector<char> packet(10);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x07;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x02;                 // Type: RGB LEDs
    packet[4]=0x0A;                 // Sub-Type: TLC5947_SetAllHold
   
    packet[5]=(value>>8)&0xFF;
    packet[6]=(value)&0xFF;
    packet[7]=(mask>>8)&0xFF;
    packet[8]=(mask)&0xFF;
   
    // Checksum
    if((options&0x80)==0x80)
        packet[9]=calculateChecksum(packet);    
    else
        packet[9]=0x00;
    // Return packet to caller
    return packet;
} // TLC5947_SetAllHold()

/////////
// ADC //
/////////
std::vector<char> LM35_GetTemperature(bool xBee, uint8_t options)
{
    // RESPONSE: 16-bit value read from ADC (divide this value by 20 to get temperature in degrees Celcius).

    // Packet has a fixed size of 6 bytes
    std::vector<char> packet(6);
    // Create packet    
    packet[0]= xBee ? 0x00 : 0x7e;  // Frame Start
    packet[1]=0x03;                 // Packet Length
    packet[2]=options;              // Options
    packet[3]=0x03;                 // Type: ADC
    packet[4]=0x00;                 // Sub-Type: LM35_GetTemperature
    // Checksum
    if((options&0x80)==0x80)
        packet[5]=calculateChecksum(packet);    
    else
        packet[5]=0x00;
    // Return packet to caller
    return packet;
} // LM35_GetTemperature()

