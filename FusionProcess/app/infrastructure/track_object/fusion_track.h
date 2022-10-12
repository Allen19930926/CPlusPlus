#ifndef A1368429_4ADE_4930_BDEB_451B238EB8EC
#define A1368429_4ADE_4930_BDEB_451B238EB8EC

#include "Eigen/Dense"

using KfVector = Eigen::Matrix<float, 6, 1>;
using KfMatrix = Eigen::Matrix<float, 6, 6>;

struct alignas(16) FusionTrackKfData
{
	bool     inited;
    KfVector x_prior;
    KfMatrix p_prior;
    KfVector x_poster;
    KfMatrix p_poster;
};


struct FusionTrack
{
    uint32_t trackId;
    
};

#endif /* A1368429_4ADE_4930_BDEB_451B238EB8EC */
