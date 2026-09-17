#ifndef THR_ARRAY_AVAILABILITY_MESSAGE_F32_H
#define THR_ARRAY_AVAILABILITY_MESSAGE_F32_H

#include "definitions.h"
#include "utilities/fsw/deviceAvailability.h"

/*! @brief FSW message definition carrying the availability of each thruster */
typedef struct {
    //!< [-] state of each thruster; entries past numThrusters are ignored
    DeviceAvailability_c thrusterAvailability[kMaxThrusterCount];
} THRArrayAvailabilityMsgF32Payload;

#endif
