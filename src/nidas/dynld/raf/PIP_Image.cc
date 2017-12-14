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

#include "PIP_HDLC.h"
#include <nidas/core/UnixIODevice.h>
#include <nidas/core/Parameter.h>
#include <nidas/core/SampleTag.h>
#include <nidas/core/Variable.h>

#include <nidas/util/Logger.h>
#include <nidas/util/UTime.h>

#include <iostream>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace nidas::dynld::raf;

namespace n_u = nidas::util;

NIDAS_CREATOR_FUNCTION_NS(raf,PIP_HDLC)


const n_u::EndianConverter * PIP_HDLC::bigEndian =
    n_u::EndianConverter::getConverter(n_u::EndianConverter::
                                       EC_BIG_ENDIAN);

const n_u::EndianConverter * PIP_HDLC::littleEndian =
    n_u::EndianConverter::getConverter(n_u::EndianConverter::
                                       EC_LITTLE_ENDIAN);

PIP_HDLC::PIP_HDLC() : _hdlc_disc(13), _size(4096), _buf(0)
{
    setDefaultMode(O_RDWR);
}

PIP_HDLC::~PIP_HDLC()
{
}

IODevice *PIP_HDLC::buildIODevice() throw(n_u::IOException)
{
    return new UnixIODevice();
}

SampleScanner *PIP_HDLC::buildSampleScanner()
    throw(n_u::InvalidParameterException)
{   
    return new DriverSampleScanner((4112 + 8) * 4);
}


/*---------------------------------------------------------------------------*/
void PIP_HDLC::open(int flags)
    throw(n_u::IOException,n_u::InvalidParameterException)
{
cerr<<"in open"<<endl;
    DSMSensor::open(flags);
    
            cerr<<"before hdlc ioctl "<<endl;
        // set N_HDLC line discipline (used for SDLC mode)
        int rc = ::ioctl(getReadFd(), TIOCSETD, &_hdlc_disc);
            cerr<<"after hdlc ioctl "<<endl;
        if(rc < 0) {
            cerr<<"RC<0 "<<endl;
        }
    init_parameters();
/*
         rc = read(fd, buf, size);
        if (rc < 0) {
            printf("read() error=%d %s\n", errno, strerror(errno));
        }
        if (rc == 0) {
            printf("read() returned with no data\n");
        }
*/
}

void PIP_HDLC::close() throw(n_u::IOException)
{
    DSMSensor::close();

}

/*---------------------------------------------------------------------------*/
/* Initialization of things that are needed in real-time
 * and when post-processing.  Don't put stuff here that
 * is *only* needed during post-processing (the idea is to
 * save memory on DSMs).
 */
void PIP_HDLC::init_parameters() throw(n_u::InvalidParameterException)
{
  /*      struct seamac_params params;  
        // modify device parameters 
        params.mode = SEAMAC_MODE_SDLC;
        params.rate = 0;    // this is a slave, getting clk from master
        params.encoding = SEAMAC_ENCODE_NRZ;
        params.txclk = SEAMAC_CLK_RXCLK;
        params.rxclk = SEAMAC_CLK_RXCLK;
        params.rxclktype = SEAMAC_RXCLK_TTL;
        params.telement = SEAMAC_TELEMENT_DPLL;
        params.parity = SEAMAC_PARITY_NONE;
        params.txbits = SEAMAC_BITS_8;
        params.rxbits = SEAMAC_BITS_8;
        params.addrfilter = 0xFF;
        params.addrrange = 0;
        params.crcpreset = SEAMAC_CRC_PRESET0;
        params.idlemode = SEAMAC_IDLE_FLAG;
        params.underrun = SEAMAC_UNDERRUN_FLAG;

        // If the device has software selectable interface, this will set it
        params.interface = SEAMAC_IF_232;

        // set current device parameters
        rc = ioctl(fd, SEAMAC_IOCTL_SPARAMS, &params); 
*/
/* const Parameter *p;

    // Acquire probe diode/pixel resolution (in micrometers) for tas encoding.
    p = getParameter("RESOLUTION");
    if (!p)
        throw n_u::InvalidParameterException(getName(), "RESOLUTION","not found");
    _resolutionMicron = (int)p->getNumericValue(0);
    _resolutionMeters = (float)_resolutionMicron * 1.0e-6;
   
    p = getParameter("TAS_RATE");
    if (!p)
        throw n_u::InvalidParameterException(getName(), "TAS_RATE","not found");
    setTASRate((int)(rint(p->getNumericValue(0)))); //tas_rate is the same rate used as sor_rate
*/
}

/*---------------------------------------------------------------------------*/
/* Stuff that is necessary when post-processing.
 
void PIP_HDLC::init() throw(n_u::InvalidParameterException)
{
    DSMSensor::init();
    init_parameters();

    // Find SampleID for 1D & 2D arrays.
    list<SampleTag *>& tags = getSampleTags();
    list<SampleTag *>::const_iterator si = tags.begin();
    for ( ; si != tags.end(); ++si) {
        const SampleTag * tag = *si;
        Variable & var = ((SampleTag *)tag)->getVariable(0);

        if (var.getName().compare(0, 3, "A1D") == 0) {
            _1dcID = tag->getId();
            _nextraValues = tag->getVariables().size() - 1;
        }

        if (var.getName().compare(0, 3, "A2D") == 0)
            _2dcID = tag->getId();
    }

    _prevTime = 0;

    clearData();
}
*/

bool PIP_HDLC::process(const Sample* samp,list<const Sample*>& results)
        throw()
{
cerr<<"PIP_HDLC process\n";

return true;
}

/*---------------------------------------------------------------------------*/
/*void PIP_HDLC::printStatus(std::ostream& ostr) throw()
{
    DSMSensor::printStatus(ostr);
    if (getReadFd() < 0) {
	ostr << "<td align=left><font color=red><b>not active</b></font></td>" << endl;
	return;
    }
    struct usb_twod_stats status;

    try {
	ioctl(USB2D_GET_STATUS,&status,sizeof(status));
	long long tnow = n_u::getSystemTime();
	float imagePerSec = float(status.numImages - _numImages) /
		float(tnow - _lastStatusTime) * USECS_PER_SEC;
	_numImages = status.numImages;
	_lastStatusTime = tnow;

	ostr << "<td align=left>" << "imgBlks/sec=" <<
		fixed << setprecision(1) << imagePerSec <<
		",lost=" << status.lostImages << ",lostSOR=" << status.lostSORs <<
		",lostTAS=" << status.lostTASs << ", urbErrs=" << status.urbErrors <<
                ",TAS=" << setprecision(0) << _trueAirSpeed << "m/s" <<
		"</td>" << endl;
    }
    catch(const n_u::IOException& ioe) {
        ostr << "<td>" << ioe.what() << "</td>" << endl;
	n_u::Logger::getInstance()->log(LOG_ERR,
            "%s: printStatus: %s",getName().c_str(),
            ioe.what());
    }
}
*/

/*---------------------------------------------------------------------------*/
void PIP_HDLC::clearData()
{
//    _recordsPerSecond = 0;
}
