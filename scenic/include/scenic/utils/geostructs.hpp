
/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About store geographical information in structs 
*/

#pragma once

namespace Scenic
{
struct UTMPoint
{
    double easting;
    double northing;
};

struct LatLonPoint
{
    double latitude;
    double longitude;
};
}
