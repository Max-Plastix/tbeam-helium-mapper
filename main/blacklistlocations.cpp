#include "blacklistlocations.h"



//Blacklisted Locations are defined as latitude, longitude and a radius in metres
BlacklistLocation blacklistedLocations[]={
                                            { 38.897666, -77.036514,10} //A 10 metre radius centred on The Whitehouse
                                            ,{ 48.858357, 2.294476,50} //A 50 metre radius centred on the Eiffel Tower
                                        };


bool isLocationBlacklisted(float latitude, float longitude) {
    for (BlacklistLocation loc: blacklistedLocations) {
        if (gps_distanceBetween(latitude,longitude,loc.latitude,loc.longitude) <= loc.radius)
            return true;
    }  
    return false;
}
