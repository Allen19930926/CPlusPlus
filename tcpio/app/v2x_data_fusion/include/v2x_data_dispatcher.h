#ifndef ACC10646_E5D7_4615_BD39_6B2AABB877E5
#define ACC10646_E5D7_4615_BD39_6B2AABB877E5

#include <stdint.h>

class V2xDataDispatcher
{
public:
    static void ProcessV2xMessage();

private:
    static void ProcessGSentrySatatus(uint8_t* buf, uint16_t len);
    static void ProcessCalcMapResult(uint8_t* buf, uint16_t len);
    static void ProcessSpatInfo(uint8_t* buf, uint16_t len);
    static void ProcessObjVehiInfo(uint8_t* buf, uint16_t len);
};

#endif /* ACC10646_E5D7_4615_BD39_6B2AABB877E5 */
