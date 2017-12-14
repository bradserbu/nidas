// -*- mode: C++; indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4; -*-
// vim: set shiftwidth=4 softtabstop=4 expandtab:
/*
 ********************************************************************
 ** NIDAS: NCAR In-situ Data Acquistion Software
 **
 ** 2007, Copyright University Corporation for Atmospheric Research
 **
 ** This program is free software; you can redistribute it and/or modify
 ** it under the terms of the GNU General Public License as published by
 ** the Free Software Foundation; either version 2 of the License, or
 ** (at your option) any later version.
 **
 ** This program is distributed in the hope that it will be useful,
 ** but WITHOUT ANY WARRANTY; without even the implied warranty of
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 ** GNU General Public License for more details.
 **
 ** The LICENSE.txt file accompanying this software contains
 ** a copy of the GNU General Public License. If it is not found,
 ** write to the Free Software Foundation, Inc.,
 ** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 **
 ********************************************************************
*/

#ifndef _nidas_dynld_raf_pip_hdlc_h_
#define _nidas_dynld_raf_pip_hdlc_h_

#include <nidas/core/DSMSensor.h>

#include <nidas/util/EndianConverter.h>
#include <nidas/util/InvalidParameterException.h>


namespace nidas { namespace dynld { namespace raf {

using namespace nidas::core;

/**
 * Base class for PMS 2D particle probes on a USB interface.  Covers
 * both the Fast2DC and the white converter box for older 2D probes.
 */
class PIP_HDLC : public DSMSensor
{

public:
    PIP_HDLC();
    ~PIP_HDLC();

    IODevice *buildIODevice() throw(nidas::util::IOException);

    SampleScanner *buildSampleScanner()
        throw(nidas::util::InvalidParameterException);

    /**
     * open the sensor and perform any intialization to the driver.
     */
    void open(int flags)
        throw(nidas::util::IOException,nidas::util::InvalidParameterException);

    void close() throw(nidas::util::IOException);

    bool process(const Sample* samp,std::list<const Sample*>& results)
        throw();

    /**
     * The probe resolution in meters.  Probe resolution is also the diameter
     * of the each diode.  Typical values are 25 for the 2DC and 200
     * micrometers for the 2DP.
     * @returns The probe resolution in meters.
     */
   // float getResolution() const { return _resolutionMeters; }
     
    /**
     * The probe resolution in micrometers.  Probe resolution is also the diameter
     * of the each diode.  Typical values are 25 for the 2DC and 200
     * micrometers for the 2DP.
     * @returns The probe resolution in micrometers.
     */
   // unsigned int getResolutionMicron() const { return _resolutionMicron; }
     
    /**
     * Number of diodes in the probe array.  This is also the bits-per-slice
     * value.  Traditional 2D probes have 32 diodes, the HVPS has 128 and
     * the Fast2DC has 64. PIP has 64?. 
     * @returns the number of bits per data slice.
     */
    virtual int NumberOfDiodes() const { return 64; }

    /**
     * Called by post-processing code 
     */
//    void init() throw(nidas::util::InvalidParameterException);
   // void printStatus(std::ostream& ostr) throw();


protected:

    // Probe produces Big Endian.
    static const nidas::util::EndianConverter * bigEndian;

    // Tap2D value sent back from driver has little endian ntap value
    static const nidas::util::EndianConverter * littleEndian;

    /**
     * Initialize parameters for real-time and post-processing.
     */
    virtual void init_parameters()
        throw(nidas::util::InvalidParameterException);


    /**
     * Clear size_dist arrays.
     */
    virtual void clearData();
//@}

private:

    /** No copying. */
    PIP_HDLC(const PIP_HDLC&);

    /** No copying. */
    PIP_HDLC& operator=(const PIP_HDLC&);
    unsigned int _hdlc_disc;
    unsigned int _size;
    unsigned char *_buf;

};


}}}                     // namespace nidas namespace dynld namespace raf
#endif
