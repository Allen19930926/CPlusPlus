#ifndef CE7E1519_3340_44E9_AB21_FCB786359DCE
#define CE7E1519_3340_44E9_AB21_FCB786359DCE

#include <cstdio>
#include <cstdint>

class CDebugFun
{
public:
    static void PrintBuf(uint8_t* buf, uint16_t len)
    {
        uint8_t* pbuf = buf;
        printf("\n line: %d \t", 1);
        for (uint16_t i=0; i<len; i++)
        {
            printf("%02x", *(pbuf++));
            if ((i+1) % 4 == 0)
            {
                printf("\t");
            }
            if ((i+1) % 16 == 0)
            {
                printf("\n line: %d \t", (i+1)/16 + 1);
            }
        }
        printf("\n");
    }
};

// void CDebugFun::PrintBuf(uint8_t* buf, uint16_t len)

#endif /* CE7E1519_3340_44E9_AB21_FCB786359DCE */
