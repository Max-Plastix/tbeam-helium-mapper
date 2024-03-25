// Decoder for MaxPlastix mappers
//
// 11 Byte payload: 
// 3 Lat, 3 Long, 2 Altitude (m), 1 Speed (km/hr), 1 Battery, 1 Sats.
// Accuracy is a dummy value required by some Integrations.
// Battery is 1/100 of a volt, offset by 2v for a range of 2.00 to 4.56 volts.
//

function Decoder(bytes, port) {
  var decoded = {};

  // All formats carry a lat & lon reading:
  var latitude = ((bytes[0] << 16) >>> 0) + ((bytes[1] << 8) >>> 0) + bytes[2];
  latitude = (latitude / 16777215.0 * 180) - 90;

  var longitude = ((bytes[3] << 16) >>> 0) + ((bytes[4] << 8) >>> 0) + bytes[5];
  longitude = (longitude / 16777215.0 * 360) - 180;

  switch (port) {
    case 2: // Mapper! (Cargo and Heatmap too)
      decoded.latitude = latitude;
      decoded.longitude = longitude;

      var altValue = ((bytes[6] << 8) >>> 0) + bytes[7];
      var sign = bytes[6] & (1 << 7);
      if (sign)
        decoded.altitude = 0xFFFF0000 | altValue;
      else
        decoded.altitude = altValue;

      decoded.speed = parseFloat((((bytes[8])) / 1.609).toFixed(2));
      decoded.battery = parseFloat((bytes[9] / 100 + 2).toFixed(2));
      decoded.sats = bytes[10];
      decoded.accuracy = 2.5; // Bogus Accuracy required by Cargo/Mapper integration
      break;
    case 3: // Mapper short! (Cargo and Heatmap too)
      decoded.latitude = latitude;
      decoded.longitude = longitude;
      decoded.altitude = (bytes[6] >> 3)*150;
      decoded.sats = (bytes[6] & 0x7)+3;
      decoded.accuracy = 2.5; // Bogus Accuracy required by Cargo/Mapper integration
      break;
    case 5: // System status
      decoded.last_latitude = latitude;
      decoded.last_longitude = longitude;
      decoded.battery = parseFloat((bytes[6] / 100 + 2).toFixed(2));
      decoded.value = bytes[8];
      decoded.status = bytes[7];
      switch (bytes[7]) {
        case 1:
          decoded.status = "BOOT";
          break;
        case 2:
          decoded.status = "USB ON";
          break
        case 3:
          decoded.status = "USB OFF";
          break;
      }
      break;
    case 6: // Lost GPS
      decoded.last_latitude = latitude;
      decoded.last_longitude = longitude;
      decoded.battery = parseFloat((bytes[6] / 100 + 2).toFixed(2));
      decoded.sats = bytes[7];
      decoded.minutes = ((bytes[8] << 8) >>> 0) + bytes[9];
      break;
  }

  return decoded;
}

// Wrapper for ChirpStack V4:
function decodeUplink(input) {
  return { 
      data: Decoder(input.bytes, input.fPort)
  };   
}
