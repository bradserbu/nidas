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

#include "PIP_Serial.h"
#include <nidas/core/PhysConstants.h>
#include <nidas/core/Parameter.h>
#include <nidas/core/Variable.h>
#include <nidas/util/Logger.h>

using namespace nidas::core;
using namespace nidas::dynld::raf;
using namespace std;

namespace n_u = nidas::util;

NIDAS_CREATOR_FUNCTION_NS(raf,PIP_Serial)

const size_t PIP_Serial::PIPEDV0 = 0;
const size_t PIP_Serial::PIPEDV64= 1;
const size_t PIP_Serial::PIPEDV32= 2;
const size_t PIP_Serial::PIPQC= 3;
const size_t PIP_Serial::PIPPS= 4;
const size_t PIP_Serial::PIPLWC = 5;
const size_t PIP_Serial::PIPLWCSLV= 6;
const size_t PIP_Serial::PIPCBTMP = 7;
const size_t PIP_Serial::PIPRH = 0;
const size_t PIP_Serial::PIPRT = 0;
const size_t PIP_Serial::PIPLSRC = 0;
const size_t PIP_Serial::PIPLSRP = 0;
const size_t PIP_Serial::REJOFLOW = 0;
const size_t PIP_Serial::REJDOF = 0;
const size_t PIP_Serial::REJEND = 0;


PIP_Serial::PIP_Serial(): SppSerial("PIP"),
    _dofReject(1), _airspeedSource(1)
{
    //
    // Make sure we got compiled with the packet structs packed appropriately.
    // If any of these assertions fails, then we can't just memcpy() between
    // the actual DMT packets and these structs, and we count on being able to
    // do that...
    //
    char* headPtr;
    char* chksumPtr;
    
    InitPIP_blk init;
    headPtr = (char*)&init;
    chksumPtr = (char*)&(init.chksum);
    assert((chksumPtr - headPtr) == (_InitPacketSize - 2));
    
    _nChannels = N_PIP_CHANNELS;
    PIP_blk data;
    headPtr = (char*)&data;
    chksumPtr = (char*)&(data.chksum);
    assert((chksumPtr - headPtr) == (packetLen() - 2));
    
    //
    // This number should match the housekeeping added in ::process, so that
    // an output sample of the correct size is created.
    //
    _nHskp = 15;

}

void PIP_Serial::validate()
    throw(n_u::InvalidParameterException)
{
    //pretty sure this isn't going to be needed
    //as both pas and dofreject will be static for PIP
    const Parameter *p;

    p = getParameter("DOF_REJ");
    if (!p) throw n_u::InvalidParameterException(getName(),
          "DOF_REJ","not found");
    _dofReject = (unsigned short)p->getNumericValue(0);

    // Initialize the message parameters to something that passes
    // SppSerial::validate(). The packet length
    // is not actually yet known, because it depends on _nChannels
    // which is set in SppSerial::validate(). This prevents an
    // InvalidParameterException in SppSerial::validate(),
    // until we can set it later.
    try {
        setMessageParameters(packetLen(),"",true);
    }
    catch(const n_u::IOException& e) {
        throw n_u::InvalidParameterException(getName(),"message parameters",e.what());
    }

    SppSerial::validate();
}

void PIP_Serial::sendInitString() throw(n_u::IOException)
{
    // zero initialize
    InitPIP_blk setup_pkt = InitPIP_blk();

    //init packet setup
    setup_pkt.esc = 0x1b;
    setup_pkt.id = 0x01;
    PackDMT_UShort(setup_pkt.airspeedSource, _airspeedSource); //0x0001: host = computer which sends packet
    PackDMT_UShort(setup_pkt.dofRej, _dofReject);
    setup_pkt.pSizeDim = 0x01;
    setup_pkt.rc = 0xFF;

    //send time to probe.
    SetAbsoluteTime setTime_pkt = SetAbsoluteTime();
    setTime_pkt.esc = 0x1b;
    setTime_pkt.id = 0x05;
    //figure out how to get time, probably should do that right before sending the packet out.
    // gettimeofday() ?

/* 
    //send data setup
    //currently just going to set this to a constant, not actually calculate airspeed 
    sendData_pkt.esc = 0x1b;
    sendData_pkt.id = 0x02;
    //DMT_UShort  hostSyncCounter;//0x01
    //DMT_ULong PASCoefficient; //??? 0x02df2e0c for 25 um, 35 m/s pas.
    //DMT_UShort  relayControl; //0x00
*/



    // exclude chksum from the computation
    PackDMT_UShort(setup_pkt.chksum,
		   computeCheckSum((unsigned char*)&setup_pkt,
				   _InitPacketSize - 2));

    // Expect 4 byte ack from PIP, instead of normal 2 for other probes.
    sendInitPacketAndCheckAck(&setup_pkt, _InitPacketSize, 4);

    //set time
    
    //send setTime packet
    
    //may need lock on xml or something to prevent half formed pas
    //can I just modify the 4 relevant bytes?

    try {
        setMessageParameters(packetLen(),"",true);
    }
    catch(const n_u::InvalidParameterException& e) {
        throw n_u::IOException(getName(),"init",e.what());
    }
}

bool PIP_Serial::process(const Sample* samp,list<const Sample*>& results)
	throw()
{
    if (! appendDataAndFindGood(samp))
        return false;

    // * Copy the good record into our PIP_blk struct.
    PIP_blk inRec;

    ::memcpy(&inRec, _waitingData, packetLen() - 2);
    ::memcpy(&inRec.chksum, _waitingData + packetLen() - 2, 2);

    // * Shift the remaining data in _waitingData to the head of the line
    _nWaitingData -= packetLen();
    ::memmove(_waitingData, _waitingData + packetLen(), _nWaitingData);

   //  * Create the output stuff
    SampleT<float>* outs = getSample<float>(_noutValues);

    dsm_time_t ttag = samp->getTimeTag();
    outs->setTimeTag(ttag);
    outs->setId(getId() + 1);

    float * dout = outs->getDataPtr();
    float value;
    const float * dend = dout + _noutValues;
    unsigned int ivar = 0;


    // these values must correspond to the sequence of
    // <variable> tags in the <sample(xml)> for this sensor.
    //*dout++ = convert(ttag,UnpackDMT_UShort(inRec.header1),ivar++);
    //*dout++ = convert(ttag,UnpackDMT_UShort(inRec.header2),ivar++);
   /* *dout++ = convert(ttag,UnpackDMT_UShort(inRec.packetByteCount),ivar++);
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.oversizeReject),ivar++);
    //laser power is a char
    *dout++ = convert(ttag,UnpackDMT_ULong(inRec.laserPower),ivar++);
    *dout++ = convert(ttag,UnpackDMT_ULong(inRec.DOFRejectCount),ivar++);
    *dout++ = convert(ttag,UnpackDMT_ULong(inRec.EndRejectCount),ivar++);
    *dout++ = convert(ttag,UnpackDMT_ULong(inRec.ParticleCounter),ivar++);
    *dout++ = convert(ttag,UnpackDMT_ULong(inRec.SecMili),ivar++);
    *dout++ = convert(ttag,UnpackDMT_ULong(inRec.HourMin),ivar++);
    *dout++ = convert(ttag,UnpackDMT_ULong(inRec.hostSyncCounter),ivar++);
    *dout++ = convert(ttag,UnpackDMT_ULong(inRec.resetFlag),ivar++);
    // if reset Flag==1, send init packet
    */
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.housekeeping[PIPEDV0]), ivar++);
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.housekeeping[PIPEDV64]), ivar++);
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.housekeeping[PIPEDV32]), ivar++);
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.housekeeping[PIPQC]), ivar++);
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.housekeeping[PIPPS]), ivar++);
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.housekeeping[PIPLWC]), ivar++);
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.housekeeping[PIPLWCSLV]), ivar++);
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.housekeeping[PIPCBTMP]), ivar++);
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.housekeeping[PIPRH]), ivar++);
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.housekeeping[PIPRT]), ivar++);
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.housekeeping[PIPLSRC]), ivar++);
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.housekeeping[PIPLSRP]), ivar++);
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.housekeeping[REJOFLOW]), ivar++);
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.housekeeping[REJDOF]), ivar++);
    *dout++ = convert(ttag,UnpackDMT_UShort(inRec.housekeeping[REJEND]), ivar++);
    


    #ifdef ZERO_BIN_HACK
    // add a bogus zeroth bin for historical reasons
    *dout++ = 0.0;
    #endif
    for (int iout = 0; iout < _nChannels; ++iout)
	*dout++ = UnpackDMT_ULong(inRec.binCount[iout]);

    // Compute DELTAT.
    if (_outputDeltaT) {
        if (_prevTime != 0)
            *dout++ = (ttag - _prevTime) / USECS_PER_SEC;
        else *dout++ = 0.0;
        _prevTime = ttag;
    }

    // If this fails then the correct pre-checks weren't done in validate().
    assert(dout == dend);

    results.push_back(outs);
 


	return true;
}
