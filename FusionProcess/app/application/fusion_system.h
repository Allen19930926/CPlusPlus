#ifndef C1E75B9A_A0C0_4323_B95D_5C0A6FD95686
#define C1E75B9A_A0C0_4323_B95D_5C0A6FD95686
#include "fusion_track_manager.h"

class FusionSystem
{
public:
    void Fuse(uint8_t* data, uint16_t len);
private:
    void AddSensorFrame(uint8_t* data, uint16_t len);
    void FuseFrame(const SensorFrame& frame);
    
};

#endif /* C1E75B9A_A0C0_4323_B95D_5C0A6FD95686 */
