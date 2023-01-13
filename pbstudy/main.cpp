#include "fusion_track.pb.h"
#include <iostream>

using namespace std;

int main()
{
    Fusion::FusionData data;
    Fusion::FusionTrack track1;

    Fusion::FusionTrack* tracks = data.add_track();
    tracks->set_time_stamp(0);
    cout << data.SerializeAsString().size() << endl;
    return 0;
}