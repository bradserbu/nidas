/*
 ******************************************************************
    Copyright 2005 UCAR, NCAR, All Rights Reserved

    $LastChangedDate$

    $LastChangedRevision$

    $LastChangedBy$

    $HeadURL$

 ******************************************************************
*/
#ifndef NIDAS_DYNLD_RAF_A2DBOARDTEMPSENSOR_H
#define NIDAS_DYNLD_RAF_A2DBOARDTEMPSENSOR_H

#include <nidas/core/DSMSensor.h>
#include <nidas/rtlinux/irigclock.h>

namespace nidas { namespace dynld { namespace raf {

using namespace nidas::core;

/**
 * The I2C temperature sensor on the ADS3 A2D board.
 */
class A2DBoardTempSensor : public DSMSensor {

public:

    A2DBoardTempSensor();
    ~A2DBoardTempSensor();

    IODevice* buildIODevice() throw(nidas::util::IOException);

    SampleScanner* buildSampleScanner();

    /**
     * Open the device connected to the sensor.
     */
    void open(int flags)
    	throw(nidas::util::IOException,nidas::util::InvalidParameterException);

    void init() throw(nidas::util::InvalidParameterException);
                                                                                
    /*
     * Close the device connected to the sensor.
     */
    void close() throw(nidas::util::IOException);

    void printStatus(std::ostream& ostr) throw();

    /**
     * Process a raw sample, which in this case means convert
     * the 16 bit signed value to degC.
     */
    bool process(const Sample*,std::list<const Sample*>& result)
        throw();

    /**
     * Get the current temperature. Sends a ioctl to the driver module.
     */
    float getTemp() throw(nidas::util::IOException);

private:

    dsm_sample_id_t sampleId;

    /**
     * Sample rate.
     */
    enum irigClockRates rate;

    /**
     * Conversion factor from 16 bit raw sensor value to degC
     */
    const float DEGC_PER_CNT;

};

}}}	// namespace nidas namespace dynld namespace raf

#endif
