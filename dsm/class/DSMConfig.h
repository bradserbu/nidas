/*
 ********************************************************************
    Copyright by the National Center for Atmospheric Research

    $LastChangedDate: 2004-10-15 17:53:32 -0600 (Fri, 15 Oct 2004) $

    $LastChangedRevision$

    $LastChangedBy$

    $HeadURL: http://orion/svn/hiaper/ads3/dsm/class/RTL_DSMSensor.h $
 ********************************************************************

*/

#ifndef DSM_DSMCONFIG_H
#define DSM_DSMCONFIG_H

#include <DOMable.h>
#include <DSMSensor.h>
#include <SampleOutputStream.h>

#include <list>

namespace dsm {

/**
 * Class that should include all that is configurable about a
 * DSM.  It should be able to initialize itself from a
 * <dsm> XML element, and provide get methods to access
 * its essential pieces, like sensors.
 */
class DSMConfig : public DOMable {
public:
    DSMConfig();
    virtual ~DSMConfig();

    const std::string& getName() const { return name; }
    void setName(const std::string& val) { name = val; }

    const std::string& getLocation() const { return location; }
    void setLocation(const std::string& val) { location = val; }

    void addSensor(DSMSensor* sensor) { sensors.push_back(sensor); }
    const std::list<DSMSensor*>& getSensors() const { return sensors; }

    void addOutput(SampleOutputStream* output) { outputs.push_back(output); }
    const std::list<SampleOutputStream*>& getOutputs() const { return outputs; }

    void fromDOMElement(const xercesc::DOMElement*)
	throw(atdUtil::InvalidParameterException);

    xercesc::DOMElement*
    	toDOMParent(xercesc::DOMElement* parent)
    		throw(xercesc::DOMException);

    xercesc::DOMElement*
    	toDOMElement(xercesc::DOMElement* node)
    		throw(xercesc::DOMException);

protected:

    std::string name;
    
    std::string location;

    /**
     * The list of sensors on this DSM.
     */
    std::list<DSMSensor*> sensors;

    std::list<SampleOutputStream*> outputs;

};

}

#endif
