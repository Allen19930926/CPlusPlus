#ifndef B7C663B3_1E76_4947_8AE7_C977E5A08220
#define B7C663B3_1E76_4947_8AE7_C977E5A08220

#include "adas_asio_tcp_server.h"
#include "adas_asio_tcp_client.h"
#include <iostream>
#include "event_msg.h"
#include <string>

class IProxy
{
public:
    virtual void Start() {}
    virtual void SetPeriodWriteTask(const uint32_t interval) {}
private:
};

#endif /* B7C663B3_1E76_4947_8AE7_C977E5A08220 */
