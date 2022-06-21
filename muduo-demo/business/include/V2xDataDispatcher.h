#ifndef ACC10646_E5D7_4615_BD39_6B2AABB877E5
#define ACC10646_E5D7_4615_BD39_6B2AABB877E5

class V2xDataDispatcher
{
public:
    static void ProcessGSentrySatatus(const char* buf, uint16_t len);
    static void ProcessCalcMapResult(const char* buf, uint16_t len);
    static void ProcessSpatInfo(const char* buf, uint16_t len);
    static void ProcessObjVehiInfo(const char* buf, uint16_t len);
}

#endif /* ACC10646_E5D7_4615_BD39_6B2AABB877E5 */
