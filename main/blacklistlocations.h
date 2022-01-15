
#pragma once

#include <Arduino.h>
#include "gps.h"

struct BlacklistLocation
{
   float latitude;
   float longitude;
   unsigned int radius; //Metres
};

bool isLocationBlacklisted(float lat, float lon) ;

//Uncomment this if you want to use the location blacklists in blacklistlocations.cpp
#define USE_LOCATION_BLACKLIST