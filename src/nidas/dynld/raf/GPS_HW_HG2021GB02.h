// -*- mode: C++; indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4; -*-
// vim: set shiftwidth=4 softtabstop=4 expandtab:
/*
 ******************************************************************
 Copyright 2005 UCAR, NCAR, All Rights Reserved

 $LastChangedDate$

 $LastChangedRevision$

 $LastChangedBy$

 $HeadURL$

 ******************************************************************
 */
#ifndef NIDAS_DYNLD_RAF_GPS_HW_HG2021GB02_H
#define NIDAS_DYNLD_RAF_GPS_HW_HG2021GB02_H

#include <nidas/dynld/raf/DSMArincSensor.h>

namespace nidas { namespace dynld { namespace raf {

using namespace nidas::core;

/**
 * ARINC IRS label processor.
 *
 * Taken from the Honeywell Installation Manual PN HG2021GB02/GD02
 * Table 207 "GNSSU ARINC 429 Output Data" (GPS)  (pages 217-219).
 */
class GPS_HW_HG2021GB02 : public DSMArincSensor {

public:

    /**
     * No arg constructor.  Typically the device name and other
     * attributes must be set before the sensor device is opened.
     */
    GPS_HW_HG2021GB02() : 
        Pseudo_Range_sign(floatNAN),
        SV_Position_X_sign(floatNAN),
        SV_Position_Y_sign(floatNAN),
        SV_Position_Z_sign(floatNAN),
        GPS_Latitude_sign(floatNAN),
        GPS_Longitude_sign(floatNAN),
        _lat110(doubleNAN),
        _lon111(doubleNAN){

#ifdef DEBUG
            err("");
#endif
        }

    /** Process all labels from this instrument. */
    double processLabel(const int data, sampleType*);

private:

    /** Mutli-label values' sign is based on the first label. */
    float Pseudo_Range_sign;
    float SV_Position_X_sign;
    float SV_Position_Y_sign;
    float SV_Position_Z_sign;
    float GPS_Latitude_sign;
    float GPS_Longitude_sign;

    double _lat110;
    double _lon111;
};

}}}	// namespace nidas namespace dynld namespace raf

#endif
