/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2023 Damien Maguire
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "2ML70.h"
#include "hwinit.h"
#include "my_math.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
/*
*E31 840CI Tacho:
*1000RPM = 70Hz
*2000RPM = 140Hz
*5000RPM = 345Hz
*6000RPM = 413Hz
*/
///Automatic Transmission Gears
static uint8_t NotSupported     = 0;
static uint8_t First_Gear       = 1;
static uint8_t Second_Gear      = 2;
static uint8_t Third_Gear       = 3;
static uint8_t Fourth_Gear      = 4;
static uint8_t Fifth_Gear       = 5;
static uint8_t Sixth_Gear       = 6;
static uint8_t Seventh_Gear     = 7;
static uint8_t Eigth_Gear       = 8;
static uint8_t CVT_Fwd_Gear     = 12;
static uint8_t Neutral_Gear     = 13;
static uint8_t Reverse_Gear     = 14;
static uint8_t Park_Gear        = 15;

// Automatic transmission shifter selections
static uint8_t Between_Range       = 0;
static uint8_t Park_Range          = 1;
static uint8_t Revernse_Range      = 2;
static uint8_t Neutral_Range       = 3;
static uint8_t ForwardA_Range      = 4;
static uint8_t ForwardB_Range      = 5;
static uint8_t ForwardC_Range      = 6;
static uint8_t ForwardD_Range      = 7;
static uint8_t Unknown_Range       = 0xF;

//Transmission Engage States
static uint8_t tNotEngaged          = 0;
static uint8_t tEngagedForward      = 1;
static uint8_t tEngagedReverse      = 2;
static uint8_t tOpStateNotReached   = 3;

//1F5
uint8_t GearCmdPos             = 0;
uint8_t GearEstPos             = 0;
uint8_t ShiftLeverPos          = 1; //Assume Park
uint8_t TranEngageState        = 0; // default to not engage

//4C9
int16_t TMOilTemp              = 0; //-40 to 215 Deg C N-40
bool    TM_Fault               = false;


//0C7
uint8_t RotDirection            = 0 ;
uint8_t RotStatus_LastTranType  = 0;
uint8_t RotStatus_ResetOccur    = 0 ;
uint8_t RotStatus_Validity      = 0;   
uint8_t RotStatus_PulseCounter  = 0; 
uint8_t RotStatus_TimeStamp     = 0; 



//We use this as an init function
void 2ML70::SetCanInterface(CanHardware* c)
{
    can = c;
    utils::SpeedoStart();
    can->RegisterUserMessage(0x153);//ASC message. Will confirm.
}


void 2ML70::DecodeCAN(int id, uint32_t* data)
{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. 


    //1F5 71 D 0 0 0 0 3 
    if (id == 0x1F5)// PPEI Transmission General Status 2 per GMW8762
    {
        GearCmdPos     = byte[1] & 0x0F; 
        GearEstPos     = byte[0] & 0x0F;
        ShiftLeverPos  = byte[3] & 0x0F;
        TMEngageState  = byte[6] & 0x07;
        // Is there a general Gear Param::SetFloat(Param::Veh_Speed, road_speed);

    }

    if (id == 0x4C9)// PPEI Transmission General Status 3 per GMW8762
    {
        TMOilTemp     = byte[1]-40; 
        TM_Fault      = byte[0] & 0x10;

    }

    //C7 63 FE 0 0 0 0 
    //77F 71 18 5 62 0 7F 3 

    if (id == 0x0C7)// PPEI Transmission Output Rotational Status per GMW8762
    {
        TMOilTemp     = byte[1]-40; 
        TM_Fault      = byte[0] & 0x10;

    }

    if (id == 0x0F9)// PPEI Transmission General Status 1 per GMW8762
    {
        TMOilTemp     = byte[1]-40; 
        TM_Fault      = byte[0] & 0x10;

    }
    if (id == 0x77F)// Diagnostic Trouble Code Information Extended per GMW8762
    {
        TMOilTemp     = byte[1]-40; 
        TM_Fault      = byte[0] & 0x10;

    }
    
}


void BMW_E31::SetTemperatureGauge(float temp)
{
    float dc = temp * 10; //TODO find right factor for value like 0..0.5 or so
    //Would like to use digi pots here
    dc = dc;
}

void BMW_E31::DecodeCAN(int id, uint32_t* data)
{
    uint8_t* bytes = (uint8_t*)data;//E31 CAN to be added here

    if (id == 0x153)// ASC1 contains road speed signal. Unsure if applies to E31 as yet ....
    {
        //Vehicle speed signal in Km/h
        //Calculation = ( (HEX[MSB] * 256) + HEX[LSB]) * 0.0625
        //Min: 0x160 (0 Km/h)

        float road_speed = 0.0625f * (((bytes[2] << 8) | (bytes[1])) - 0x160);

        Param::SetFloat(Param::Veh_Speed, road_speed);
    }
}

void BMW_E31::EGSMsg43B()  //EGS1
{

    uint8_t bytes[3];

    bytes[0]=0x46;
    bytes[1]=0x00;
    bytes[2]=0x00;

    can->Send(0x43B, (uint32_t*)bytes,3);
}

void BMW_E31::EGSMsg43F(int8_t gear)
{
    //Can bus data packet values to be sent
    uint8_t bytes[8];
    // Source: https://www.bimmerforums.com/forum/showthread.php?1887229-E46-Can-bus-project&p=30055342#post30055342
    // byte 0 = 0x81 //doesn't do anything to the ike
    bytes[0] = 0x81;
    // byte 1 = 0x01 where;
    // 01 = first gear
    // 02= second gear
    // 03 = third gear
    // 04 = fourth gear
    // 05 = D
    // 06 = N
    // 07 = R
    // 08 = P
    // 09 = 5
    // 0A = 6
    switch (gear)
    {
    case -1 /* Reverse */:
        bytes[1] = 0x07;
        break;
    case 0 /* Neutral */:
        bytes[1] = 0x06;
        break;
    case 1 /* Drive */:
        bytes[1] = 0x05;
        break;
    default:
        bytes[1] = 0x08;
        break;
    }

    // byte 2 = 0xFF where;
    // FF = no display
    // 00 = E
    // 39 = M
    // 40 = S
    bytes[2] = 0xFF;

    // byte 3 = 0xFF //doesn't do anything to the ike
    bytes[3] = 0xFF;

    // byte 4 = 0x00 //doesn't do anything to the ike
    bytes[4] = 0x00;

    // byte 5 = 0x80 where;
    // 80 = clears the gear warning picture - all other values bring it on
    bytes[5] = 0x80;

    // byte 6 = 0xFF //doesn't do anything to the ike
    bytes[6] = 0xFF;

    // byte 7 = 0x00 //doesn't do anything to the ike
    bytes[7] = 0xFF;

    can->Send(0x43F, bytes, 8);
}

void BMW_E31::Task1Ms()
{


}


void BMW_E31::Task10Ms()
{
    if(DigIo::t15_digi.Get() == 1)
    {
        EGSMsg43B();//EGS1
        if (Param::GetBool(Param::Transmission))
            EGSMsg43F(Param::GetInt(Param::dir));
    }

}


void BMW_E31::Task100Ms()
{
    if (!Param::GetInt(Param::T15Stat))
    {
        utils::SpeedoSet(0);//set speedo off
    }
}


bool BMW_E31::Ready()
{
    return DigIo::t15_digi.Get();
}

bool BMW_E31::Start()
{
    return Param::GetBool(Param::din_start);
}
