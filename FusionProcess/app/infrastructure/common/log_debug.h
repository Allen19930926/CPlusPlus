#ifndef D0F71194_1528_4D5C_8DA0_0F1E7A47BE85
#define D0F71194_1528_4D5C_8DA0_0F1E7A47BE85

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
            if ((i+1) % 16 == 0 && (i+1) != len)
            {
                printf("\n line: %d \t", (i+1)/16 + 1);
            }
        }
        printf("\n");
    }
};


#endif /* D0F71194_1528_4D5C_8DA0_0F1E7A47BE85 */
