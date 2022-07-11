#ifndef A77E8904_57C1_47A7_9989_2D1B85029189
#define A77E8904_57C1_47A7_9989_2D1B85029189

#include "data_base.h"
#include <vector>

using std::vector;

class V2xFusionAlgo
{
public:
    static void TransV2xVehi2CddVehi(const V2X::AdasObjVehInfo& v2x, const CAN::HostVehiclePos& host, CDDFusion::CDDFusionGSentryObj& cdd);
    static void TransV2xSpat2CddSpat(const V2X::AdasSpatInfo& v2x, CDDFusion::CDDCurntLaneTrafficLightInfo& cdd);
    static void TransV2xWarn2CddWarn(const V2X::WarningInfo& v2x, CDDFusion::CDDgSentryWarningInfo& cdd);
private:
    static vector<float> TransWgs84ToVcsCoordinate(const CAN::HostVehiclePos& host, const V2X::Position& remote);
    static vector<float> TransObjVcsToHostVcs(const uint16_t objXVector, const uint16_t objYVector, const float hosthead, const float remotehead);
};

#endif /* A77E8904_57C1_47A7_9989_2D1B85029189 */
