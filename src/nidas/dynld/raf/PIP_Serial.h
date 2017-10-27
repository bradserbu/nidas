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

#ifndef NIDAS_DYNLD_RAF_PIP_SERIAL_H
#define NIDAS_DYNLD_RAF_PIP_SERIAL_H

#include "SppSerial.h"

#include <iostream>

namespace nidas { namespace dynld { namespace raf {

/**
 * A class for reading PMS1D probes with the DMT interface conversion.
 * RS-422 @ 38400 baud.
 */
class PIP_Serial : public SppSerial
{
public:

  PIP_Serial();

  void validate()
        throw(nidas::util::InvalidParameterException);

  void sendInitString() throw(nidas::util::IOException);

  bool process(const Sample* samp,std::list<const Sample*>& results)
        throw();

  static const size_t N_PIP_CHANNELS = 62;
  static const size_t N_PIP_HSKP = 16;

  // Packet to initialize probe with.
  struct InitPIP_blk
  {
    char    esc;                                // ESC 0x1b
    char    id;                                 // cmd id
    DMT_UShort  airspeedSource;                 // PAS
    DMT_UShort  dofRej;
    char    pSizeDim;                           // ParticleSizingDimension
    char    rc;                                 // recovery coefficient
    DMT_UShort  chksum;                         // cksum
  };
 
  static const int _InitPacketSize = 10;

 /**
  * Packet sent to probe to begin sending data. 
  * Usually this is put in the xml, but because
  * PASCoefficient has to be calculated by the
  * server, and thus is not static, is here.
  * Don't actually know that it'll be used though.
  */
  struct SendPIP_BLK
  {
    char    esc;                                // ESC 0x1b
    char    id;                                 // cmd id (2 for this one)
    DMT_UShort  hostSyncCounter;
    DMT_ULong PASCoefficient;
    DMT_UShort  relayControl;
    DMT_UShort  chksum;                         // cksum
  };

  static const int _SendDataPacketSize = 12;
  struct SetAbsoluteTime
  {
    char esc;
    char id; //5
    DMT_UShort secMili; //seconds and miliseconds
    DMT_UShort hourMin; //set hour and min
    DMT_UShort chksum;
  };

  /**
   * Data packet back from probe.
   */
  struct PIP_blk
  {
      char header1;
      char header2;
      DMT_UShort packetByteCount;
      DMT_UShort oversizeReject;
      DMT_UShort binCount[N_PIP_CHANNELS];
      //DMT_ULong rejDOF;
      DMT_UShort DOFRejectCount;
      DMT_UShort EndRejectCount; 
      DMT_UShort housekeeping[N_PIP_HSKP];
      char laserPower;
      DMT_UShort ParticleCounter;   
      DMT_UShort SecMili; //Seconds and Milliseconds
      DMT_UShort HourMin; //Hour and minute
      DMT_UShort hostSyncCounter; 
      DMT_UShort resetFlag;
      DMT_UShort chksum;
      char trailer1;
      char trailer2;
  };

protected:

  int packetLen() const {
    return (179);    //use _nChannels if binCount ends up being variable
  }

  // These are instantiated in in .cc
  static const size_t FLSR_CUR_INDX, FLSR_PWR_INDX, FWB_TMP_INDX, FLSR_TMP_INDX,
    SIZER_BLINE_INDX, QUAL_BLINE_INDX, VDC5_MON_INDX, FCB_TMP_INDX;

  unsigned short _dofReject;
  unsigned short _airspeedSource;
};

}}}	// namespace nidas namespace dynld raf

#endif
