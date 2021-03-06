// -*- mode: C++; indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4; -*-
// vim: set shiftwidth=4 softtabstop=4 expandtab:
/*
 ********************************************************************
 ** NIDAS: NCAR In-situ Data Acquistion Software
 **
 ** 2006, Copyright University Corporation for Atmospheric Research
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

#ifndef NIDAS_DYNLD_ISFF_GOESXMTR_H
#define NIDAS_DYNLD_ISFF_GOESXMTR_H

#include <nidas/core/IOChannel.h>
#include <nidas/core/SampleTag.h>
#include <nidas/util/SerialPort.h>
#include <nidas/util/UTime.h>

#include <string>
#include <iostream>
#include <vector>

namespace nidas { namespace dynld { namespace isff {

using namespace nidas::core;

/**
 * Support for a GOES transmitter, implemented as an IOChannel.
 */

class GOESXmtr: public IOChannel {

public:

    /**
     * Constructor.
     */
    GOESXmtr();

    /**
     * Copy constructor.
     */
    GOESXmtr(const GOESXmtr&);

    /**
     * Destructor.
     */
    virtual ~GOESXmtr();

    const std::string& getName() const
    {
	return _port.getName();
    }

    void setName(const std::string& val)
    {
	_port.setName(val);
    }

    void setId(unsigned long val)
    {
        _id = val;
    }

    unsigned long getId() const
    {
        return _id;
    }
    
    void setChannel(int val)
    {
        _channel = val;
    }

    int getChannel() const
    {
        return _channel;
    }
    
    /**
     * Set the transmission interval.
     * @param val Interval, in seconds.
     */
    void setXmitInterval(long val)
    {
        _xmitInterval = val;
    }

    int getXmitInterval() const
    {
        return _xmitInterval;
    }
    
    /**
     * Set the transmission offset.
     * @param val Offset, in seconds.
     */
    void setXmitOffset(long val)
    {
        _xmitOffset = val;
    }

    int getXmitOffset() const
    {
        return _xmitOffset;
    }
    
    /**
     * Set the RF baud rate
     * @param val RF baud, in bits/sec.
     */
    virtual void setRFBaud(long val) throw(nidas::util::InvalidParameterException) = 0;

    virtual int getRFBaud() const = 0;
    
    /**
     * Request a connection.
     */
    void requestConnection(IOChannelRequester* rqstr)
    	throw(nidas::util::IOException);

    /**
     * 
     */
    IOChannel* connect() throw(nidas::util::IOException);

    void setNonBlocking(bool val __attribute__ ((unused)) ) throw(nidas::util::IOException)
    {
        // ignore for now.
    }

    bool isNonBlocking() const throw(nidas::util::IOException)
    {
        return false;
    }

    virtual void open() throw(nidas::util::IOException);

    /**
     * Initialize tranmitter.
     */
    virtual void init() throw(nidas::util::IOException) = 0;

    /**
     * Queue a sample for writing to a GOES transmitter.
    */
    virtual void transmitData(const nidas::util::UTime& at,
    	int configid,const Sample*) throw (nidas::util::IOException) = 0;

    virtual unsigned long checkId() throw(nidas::util::IOException) = 0;

    /**
     * Check transmitter clock, and correct it if necessary.
     * @return transmitter clock minus system clock, in milliseconds.
     */
    virtual int checkClock() throw(nidas::util::IOException) = 0;

    virtual void reset() throw(nidas::util::IOException) = 0;

    /**
     * Request that transmitter status be printed to an output stream.
     */
    virtual void printStatus() throw() = 0;

    void flush() throw (nidas::util::IOException) 
    {
        _port.flushBoth();
    }

    void close() throw (nidas::util::IOException) {
        _port.close();
    }

    int getFd() const
    {
        return _port.getFd();
    }

    void setStatusFile(const std::string& val)
    {
        _statusFile = val;
    }

    const std::string& getStatusFile() const {
        return _statusFile;
    }

    void fromDOMElement(const xercesc::DOMElement* node)
	throw(nidas::util::InvalidParameterException);

protected:

    nidas::util::SerialPort _port;

private:

    unsigned long _id;
    
    int _channel;

    int _xmitInterval;

    int _xmitOffset;

    std::string _statusFile;

};


}}}	// namespace nidas namespace dynld namespace isff

#endif
