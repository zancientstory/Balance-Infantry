

#include "RefereeCan.h"

#include <string.h>


RefereeInformation_t RefereeInformation;

void RefereePowerHeatNode0InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Realtime.ChassisVoltage, data, 8);
}
void RefereePowerHeatNode1InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Realtime.ChassisBufferEnergy, data, 8);
}
void RefereeAmmoSpeedNode0InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Ammo0Speed, data, 4);
}
void RefereeAmmoSpeedNode1InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Ammo1Speed, data, 4);
}
void RefereeAmmoSpeedNode2InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Ammo2Speed, data, 4);
}
void RefereeAmmoLimitNode0InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Ammo0Limit.Cooling, data, 6);
}
void RefereeAmmoLimitNode1InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Ammo1Limit.Cooling, data, 6);
}
void RefereeAmmoLimitNode2InformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.Ammo2Limit.Cooling, data, 6);
}
void RefereeSelfStateNodeInformationUpdate(uint8_t *data)
{
    memcpy(&RefereeInformation.RobotState.RobotID, data, 6);
}

void GetRefereeInformation(RefereeInformation_t *Inf)
{
    memcpy(Inf, &RefereeInformation, sizeof(RefereeInformation_t));
}




// ÈÈÁ¿±Õ»·

void AmmoHeatSettlementInterpolation(void)
{
    int16_t temp;
    // 10Hz Loop
    temp = RefereeInformation.Realtime.Ammo0Heat - (RefereeInformation.Ammo0Limit.Cooling / 10);
    if (temp >= 0) {
        RefereeInformation.Realtime.Ammo0Heat = temp;
    }
    else {
        RefereeInformation.Realtime.Ammo0Heat = 0;
    }

    temp = RefereeInformation.Realtime.Ammo1Heat - (RefereeInformation.Ammo1Limit.Cooling / 10);
    if (temp >= 0) {
        RefereeInformation.Realtime.Ammo1Heat = temp;
    }
    else {
        RefereeInformation.Realtime.Ammo1Heat = 0;
    }
    
    temp = RefereeInformation.Realtime.Ammo2Heat - (RefereeInformation.Ammo2Limit.Cooling / 10);
    if (temp >= 0) {
        RefereeInformation.Realtime.Ammo2Heat = temp;
    }
    else {
        RefereeInformation.Realtime.Ammo2Heat = 0;
    }
}

void Ammo0HeatUpdateInterpolation(void)
{
    RefereeInformation.Realtime.Ammo0Heat += 10;
}

void Ammo1HeatUpdateInterpolation(void)
{
    RefereeInformation.Realtime.Ammo1Heat += 10;
}

void Ammo2HeatUpdateInterpolation(void)
{
    RefereeInformation.Realtime.Ammo2Heat += 100;
}


