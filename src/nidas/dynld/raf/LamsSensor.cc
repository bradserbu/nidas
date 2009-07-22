/* 
  LamsSensor
  Copyright 2007 UCAR, NCAR, All Rights Reserved
 
   Revisions:
     $LastChangedRevision:  $
     $LastChangedDate:  $
     $LastChangedBy:  $
     $HeadURL: http://svn/svn/nidas/trunk/src/nidas/dynid/LamsSensor.cc $
*/


#include <nidas/rtlinux/lams.h>
#include <nidas/dynld/raf/LamsSensor.h>
#include <nidas/core/DSMConfig.h>
#include <nidas/core/DSMEngine.h>
#include <nidas/core/RTL_IODevice.h>
#include <nidas/core/UnixIODevice.h>
#include <nidas/core/Site.h>
#include <nidas/core/Project.h>
#include <nidas/util/Logger.h>
#include <nidas/util/UTime.h>

using namespace std;
using namespace nidas::dynld::raf;

namespace n_u = nidas::util;

NIDAS_CREATOR_FUNCTION_NS(raf,LamsSensor)

LamsSensor::LamsSensor() :
    DSMSensor(), calm(0), nAVG(20), nPEAK(1000), nSKIP(0) {}

void LamsSensor::fromDOMElement(const xercesc::DOMElement* node)
    throw(n_u::InvalidParameterException)
{
    DSMSensor::fromDOMElement(node);

    const Parameter *p;

    // Get manditory parameter(s)
    p = getParameter("calm");
    if (!p)
        throw n_u::InvalidParameterException(getName(), "calm","not found");
    calm = (int)p->getNumericValue(0);

    // Get optional parameter(s)
    p = getParameter("nAVG");
    if (p) nAVG  = (unsigned int)p->getNumericValue(0);
    p = getParameter("nPEAK");
    if (p) nPEAK = (unsigned int)p->getNumericValue(0);
    p = getParameter("nSKIP");
    if (p) nSKIP = (unsigned int)p->getNumericValue(0);
}

bool LamsSensor::process(const Sample* samp,list<const Sample*>& results) throw()
{
    const unsigned char * input = (unsigned char *) samp->getConstVoidDataPtr();

    // there are two spectral arrays generated by LAMS: average and peak
    const unsigned int   * iAvrg = (const unsigned int*)input;
    const unsigned short * iPeak =
      (const unsigned short*)&input[MAX_BUFFER * sizeof(int)];

    // allocate samples
    SampleT<float> * outs = getSample<float>(MAX_BUFFER * 2);
    outs->setTimeTag(samp->getTimeTag());
    outs->setId(getId() + 1);  

    // extract data from a lamsPort structure
    float * dout = outs->getDataPtr();
    size_t iout;

    for (iout = 0; iout < MAX_BUFFER; iout++)
      *dout++ = (float)*iAvrg++;

    for (iout = 0; iout < MAX_BUFFER; iout++)
      *dout++ = (float)*iPeak++;
  
    results.push_back(outs);
    return true;
} 

void LamsSensor::open(int flags) throw(n_u::IOException,
    n_u::InvalidParameterException)
{
    DSMSensor::open(flags);

    // Request that fifo be opened at driver end.
    if (DSMEngine::getInstance()) {
      struct lams_set lams_info;
      lams_info.channel = 1;//TODO GET FROM MXL CONFIG?
      ioctl(LAMS_SET_CHN, &lams_info, sizeof(lams_info));
//    ioctl(AIR_SPEED, 0,0);
      ioctl(CALM,      &calm,      sizeof(calm));
      ioctl(N_AVG,     &nAVG,      sizeof(nAVG));
      ioctl(N_SKIP,    &nSKIP,     sizeof(nSKIP));
      ioctl(N_PEAKS,   &nPEAK,     sizeof(nPEAK));
    }
    n_u::Logger::getInstance()->log(LOG_NOTICE,"LamsSensor::open(%x)", getReadFd());
}
