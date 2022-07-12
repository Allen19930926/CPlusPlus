#ifndef C45E082C_1A9F_49A4_924B_96AB88405874
#define C45E082C_1A9F_49A4_924B_96AB88405874

#include "data_base.h"

class CanFusionAlgo
{
public:
    static void ProcessRecieveData(uint8_t* data, uint16_t len) { ProcessCanHostVehicleInfo(data, len); }
private:
    static void ProcessCanHostVehicleInfo(uint8_t* buf, uint16_t len);
};

#endif /* C45E082C_1A9F_49A4_924B_96AB88405874 */
