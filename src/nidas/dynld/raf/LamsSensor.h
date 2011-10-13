/* LamsSensor.h

   Copyright 2007 UCAR, NCAR, All Rights Reserved
   Revisions:
    $LastChangedRevision: $
    $LastChangedDate:  $
    $LastChangedBy:  $
    $HeadURL: http://svn/svn/nidas/trunk/src/nidas/dynid/LamsSensor.h $
*/


#ifndef NIDAS_DYNLD_RAF_LAMSSENSOR_H
#define NIDAS_DYNLD_RAF_LAMSSENSOR_H

#include <iostream>
#include <iomanip>

#include <nidas/linux/lams/lamsx.h>

#include <nidas/core/DSMSensor.h>
#include <nidas/core/DerivedDataClient.h>

#include <nidas/util/InvalidParameterException.h>

namespace nidas { namespace dynld { namespace raf {

using namespace nidas::core;
namespace n_u = nidas::util;
 
/**
 * Sensor class supporting the NCAR/EOL Laser Air Motion Sensor (LAMS)
 * via a DSM.  This is the original LAMS implementation.
 */
class LamsSensor : public DSMSensor, public DerivedDataClient
{
public:
  LamsSensor();

  void fromDOMElement(const xercesc::DOMElement* node)
      throw(nidas::util::InvalidParameterException);

  void printStatus(std::ostream& ostr) throw();

  virtual void
  derivedDataNotify(const nidas::core:: DerivedDataReader * s)
        throw();

  bool process(const Sample* samp,std::list<const Sample*>& results)
        throw();
	
  IODevice* buildIODevice() throw(n_u::IOException);
  	
  SampleScanner* buildSampleScanner() throw(n_u::InvalidParameterException);
  
  /**
   * Open the device connected to the sensor.
   */
  void open(int flags) throw(nidas::util::IOException,
        nidas::util::InvalidParameterException);

  void close() throw(nidas::util::IOException);

private:

  int nAVG;
  
  int nPEAK;

  float TAS_level;
  enum {BELOW, ABOVE} TASlvl;

  float tas;     // True Airspeed.  Meters per second
  int tas_step;  // generate a True Airspeed (set to 0 to disable)

  /**
   * Number of initial spectral values to skip when reading from the card.
   */
  int nSKIP;
};

}}}

#endif
