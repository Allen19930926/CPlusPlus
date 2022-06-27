#ifndef E6B3AD34_AEA5_4E04_9047_493401736438
#define E6B3AD34_AEA5_4E04_9047_493401736438

#include <vector>
#include <unordered_map>
#include <functional>
#include "cdd_fusion.h"

class DataProcessor
{
public:
    DataProcessor();
    void ProcessV2xData(uint8_t* buf, uint16_t len);
    void ProcessCameraData(uint8_t* buf, uint16_t len);
public: //for test
    bool IsGsentryWork() {return gSentryStatus == 3;}
private:
    void ProcessGSentryStatus(uint8_t* buf, uint16_t len);
private:
    uint8_t gSentryStatus;
    std::unordered_map<int, std::function<void(uint8_t*,uint16_t)>> dataDispatcher;
    std::vector<CDDFusionGSentryObj> gSentryObjs;
    std::vector<CDDFusionCameraObj> cameraObjs;
};

#endif /* E6B3AD34_AEA5_4E04_9047_493401736438 */
