/*
 ********************************************************************
    Copyright 2005 UCAR, NCAR, All Rights Reserved

    $LastChangedDate$

    $LastChangedRevision$

    $LastChangedBy$

    $HeadURL$
 ********************************************************************

*/

#include <nidas/dynld/RawSampleOutputStream.h>
#include <nidas/core/Datagrams.h>

#include <nidas/util/Logger.h>

#include <iostream>

using namespace nidas::core;
using namespace nidas::dynld;
using namespace std;

namespace n_u = nidas::util;

NIDAS_CREATOR_FUNCTION(RawSampleOutputStream)

RawSampleOutputStream::RawSampleOutputStream(): SampleOutputStream() 
{
}

RawSampleOutputStream::RawSampleOutputStream(IOChannel* i):SampleOutputStream(i)
{
}

RawSampleOutputStream::RawSampleOutputStream(RawSampleOutputStream& x,
	IOChannel* iochannel):
	SampleOutputStream(x,iochannel)
{
}

RawSampleOutputStream::~RawSampleOutputStream()
{
}

RawSampleOutputStream* RawSampleOutputStream::clone(IOChannel* iochannel)
{
    return new RawSampleOutputStream(*this,iochannel);
}

void RawSampleOutputStream::fromDOMElement(const xercesc::DOMElement* node)
        throw(n_u::InvalidParameterException)
{
    SampleOutputStream::fromDOMElement(node);
    if (getIOChannel()->getRequestType() < 0)
    	getIOChannel()->setRequestType(RAW_SAMPLE);
}
