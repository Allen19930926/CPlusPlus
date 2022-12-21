#ifndef _ADASMATH_H
#define _ADASMATH_H

#include <iostream>
#include <sstream>
#include <cinttypes>
#include <cmath>
#include <math.h>
#include <limits.h>
#include <string>

const double ADAS_TIMEGAD_UNIT = 0.01;
const double ADAS_DISTANCE_UNIT = 0.01;
const double ADAS_ACC_UNIT = 0.01;


const double ADAS_LATLON_UNIT = 0.0000001;
const double ADAS_HEADING_UNIT = 0.0125;

const double ADAS_SEMI_DEG = 180.0;
const double PI = 3.1415926535897932384626;
const double ADAS_EARTH_POLAR_RADIUS = 6356725.0;
const double ADAS_EARTH_EQUAT_RADIUS = 6378137.0;

///////////////////////////

const double ADAS_SPEED_UNIT = 3.6;
const double ADAS_EPS = 1e-14;
const double ADAS_SPEEDRATE_UNIT = 0.02;

class PositionModel
{
public:
    PositionModel() = default;
    PositionModel(std::initializer_list<double> ini);
    std::string to_string()
    {
        std::stringstream ss;
        ss.setf(std::ios::fixed);
        ss.precision(15);
        ss << "longitude:" << m_Longitude;
        ss << " latitude:" << m_Latitude;
        return ss.str();
    }
    std::string to_string() const
    {
        std::stringstream ss;
        ss.setf(std::ios::fixed);
        ss.precision(15);
        ss << "longitude:" << m_Longitude;
        ss << " latitude:" << m_Latitude;
        return ss.str();
    }

    /**
      * @brief  获取经度
      * @return double  GPS经度,范围[-180,180],单位:度
      */
    double getLongitude() const
    {
        return m_Longitude;
    }

    /**
      * @brief  设置GPS经度
      * @param  longitude  GPS经度,范围[-180,180],单位:度
      * @return void
      */
    void setLongitude(double longitude)
    {
        m_Longitude = longitude;
    }

    /**
      * @brief  获取纬度
      * @return double GPS纬度,范围[-90,90],单位:度
      */
    double getLatitude() const
    {
        return m_Latitude;
    }

    /**
      * @brief  设置纬度
      * @param  latitude GPS纬度,范围[-90,90],单位:度
      * @return void
      */
    void setLatitude(double latitude)
    {
        m_Latitude = latitude;
    }
private:
    double m_Longitude = INT_MAX;   ///< GPS经度,范围[-180,180],单位:度
    double m_Latitude = INT_MAX;    ///< GPS纬度,范围[-90,90],单位:度

};


class ADASMath
{
public:

static double convertDegLatLonI2F(int32_t latlon){return latlon * ADAS_LATLON_UNIT;}
static int32_t convertDegLatLonF2I(double latlon){return latlon / ADAS_LATLON_UNIT;}
	/**
	 *
	 * @param Deg
	 * @return
	 */
static double convertDeg2Rad(double Deg){ return (Deg/ADAS_SEMI_DEG*PI);}
    /**
	 * 计算当前经纬度下的地球半径
	 * @param deglat
	 * @return
	 */
static double getEarthRadius(double deglat)
{
	return ADAS_EARTH_POLAR_RADIUS + (ADAS_EARTH_EQUAT_RADIUS - ADAS_EARTH_POLAR_RADIUS)*(90 - deglat)/90;
}
	/**
	 * 计算两个点的距离，单位：米
	 * @param node1
	 * @param node2
	 * @return
	 */
static double getDistance(const PositionModel& node1, const PositionModel& node2)
{
    double lat1 = convertDeg2Rad(node1.getLatitude());
    double lon1 = convertDeg2Rad(node1.getLongitude());
    double lat2 = convertDeg2Rad(node2.getLatitude());
    double lon2 = convertDeg2Rad(node2.getLongitude());

    double ret = (sin(lat2)*sin(lat1)) + (cos(lat2)*cos(lat1)*cos(lon2-lon1));

    if(fabs(ret - 1.0) < ADAS_EPS)
    {
        return 0.0;
    }

    double radian = acos(ret);

    if(isnan(radian))
    {
        if(ret >= 1.0)
        {
            radian = 0.0;
        }
        else if(ret <= -1.0)
        {
            radian = PI;
        }
    }

    double radius = getEarthRadius(node1.getLatitude());

    return (radian * radius);
}



// 时间精度转换
static double convertTimeI2F(double time)
{
    return time * ADAS_TIMEGAD_UNIT;
}
static double convertTimeF2I(double time)
{
    return time / ADAS_TIMEGAD_UNIT;
}

// 距离精度转换
static double convertDistanceI2F(double distance)
{
    return distance * ADAS_TIMEGAD_UNIT;
}
static double convertDistanceF2I(double distance)
{
    return distance / ADAS_TIMEGAD_UNIT;
}


/* 速度单位转换 m/s -> km/h */
static double convertSpeedMs2Kmh(double speed)
{
    return speed * ADAS_SPEED_UNIT;
}
/* 速度单位转换 km/h -> m/s */
static double convertSpeedKmh2Ms(double speed)
{
    return speed / ADAS_SPEED_UNIT;
}

//加速度单位转换
static double convertAccI2F(double speed)
{
    return speed * ADAS_ACC_UNIT;
}
static double convertAccF2I(double speed)
{
    return speed / ADAS_ACC_UNIT;
}

// 航向角精度转换
static double convertHeadingI2F(double heading)
{
    return heading * ADAS_HEADING_UNIT;
}
static double convertHeadingF2I(double heading)
{
    return heading / ADAS_HEADING_UNIT;
}

/* 速度分辨率转换 乘分辨率0.02 */
static double convertSpeedI2F(double speed)
{
    return speed * ADAS_SPEEDRATE_UNIT;
}

/* 速度分辨率转换 除分辨率0.02 */
static double convertSpeedF2I(double speed)
{
    return speed / ADAS_SPEEDRATE_UNIT;
}

};
#endif