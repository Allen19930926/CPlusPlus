#include "V2xDataDispatcher.h"
#include <cstdio>

void V2xDataDispatcher::ProcessGSentrySatatus(const char* buf, uint16_t len)
{
    printf("recieve gSentry Status msg!\n");
}

void V2xDataDispatcher::ProcessCalcMapResult(const char* buf, uint16_t len)
{
    printf("recieve gSentry Calc Map msg!\n");
}

void V2xDataDispatcher::ProcessSpatInfo(const char* buf, uint16_t len)
{
    printf("recieve gSentry Spat msg!\n");
}

void V2xDataDispatcher::ProcessObjVehiInfo(const char* buf, uint16_t len)
{
    printf("recieve gSentry ObjVehiInfo msg!\n");
}
