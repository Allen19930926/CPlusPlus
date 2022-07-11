#ifndef ACC10646_E5D7_4615_BD39_6B2AABB877E5
#define ACC10646_E5D7_4615_BD39_6B2AABB877E5

#include <stdint.h>

class EventDispatcher
{
public:
    static void ProcessMessage();

private:
    static void ProcessGSentrySatatus(uint8_t* buf, uint16_t len);
    static void ProcessHostVehiExtraMapInfo(uint8_t* buf, uint16_t len);
    static void ProcessSpatInfo(uint8_t* buf, uint16_t len);
    static void ProcessObjVehiInfo(uint8_t* buf, uint16_t len);
    static void ProcessHostVehiMapInfo(uint8_t* buf, uint16_t len);
    static void ProcessObjVehiMapInfo(uint8_t* buf, uint16_t len);
    static void ProcessGSentryWarningInfo(uint8_t* buf, uint16_t len);
    static void ProcessJ3CameraData(uint8_t* buf, uint16_t len);
    static void ProcessCanHostVehicleInfo(uint8_t* buf, uint16_t len);
};

#endif /* ACC10646_E5D7_4615_BD39_6B2AABB877E5 */
