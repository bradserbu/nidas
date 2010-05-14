/*
 ********************************************************************
    Copyright 2005 UCAR, NCAR, All Rights Reserved

    $LastChangedDate$

    $LastChangedRevision$

    $LastChangedBy$

    $HeadURL$
 ********************************************************************

*/

#ifndef NIDAS_CORE_XDOM_H
#define NIDAS_CORE_XDOM_H

#include <nidas/core/XMLStringConverter.h>

#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMAttr.hpp>
#include <xercesc/dom/DOMTypeInfo.hpp>

#include <string>
#include <map>

namespace nidas { namespace core {

/**
 * Wrapper class providing convienence methods to access the
 * string attributes of a DOMElement.
 */
class XDOMElement {
public:
    XDOMElement(const xercesc::DOMElement*e) :
    	elemConst(e),elemNonConst(0),
	nodename(XMLStringConverter(e->getNodeName())),
        nodetype( e->getTypeInfo() ? XMLStringConverter(e->getTypeInfo()->getName()) : "")
    {
    }

    XDOMElement(xercesc::DOMElement*e) :
    	elemConst(e),elemNonConst(e),
	nodename(XMLStringConverter(e->getNodeName()))
    {
    }

    /**
     * @return an empty string if attribute does not have a
     *  specified or default value.
     */
    const std::string& getAttributeValue(const std::string& aname) {
	std::map<std::string,std::string>::const_iterator ai = attrs.find(aname);
	if (ai == attrs.end()) {
	    XMLStringConverter cname(aname.c_str());

	    // returns empty string if attribute does not have a
	    // specified or default value.
	    XMLStringConverter aval(elemConst->getAttribute((const XMLCh*)cname));

	    std::pair<const std::string,const std::string>
		    p(aname,aval);
	    attrs.insert(p);
	    ai = attrs.find(aname);
	}
	return ai->second;
    }
    void setAttributeValue(const std::string& name,const std::string& val)
    {
        XMLStringConverter aname(name);
        XMLStringConverter aval(val);
        if (elemNonConst)
            elemNonConst->setAttribute((const XMLCh*)aname,(const XMLCh*)aval);
        attrs[name] = val;
    }
    const std::string& getNodeName() const { return nodename; }

    const std::string& getNodeType() const {return nodetype; }

    const xercesc::DOMElement* getElement() const { return elemConst; }

private:
    const xercesc::DOMElement* elemConst;
    xercesc::DOMElement* elemNonConst;
    std::map<std::string,std::string> attrs;
    std::string nodename;
    std::string nodetype;
};

/**
 * Class providing convienence methods to access the string
 * attributes of a DOMAttr.
 */
class XDOMAttr {
public:
    XDOMAttr(const xercesc::DOMAttr*a) :
    	attr(a),
	name(XMLStringConverter(a->getName())),
	value(XMLStringConverter(a->getValue()))
    {
    }
    const std::string& getName() const { return name; }
    const std::string& getValue() const { return value; }
protected:
    const xercesc::DOMAttr* attr;
    std::string name;
    std::string value;
};

}}	// namespace nidas namespace core

#endif
