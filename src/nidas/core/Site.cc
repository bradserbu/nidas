/*
 ********************************************************************
    Copyright 2005 UCAR, NCAR, All Rights Reserved

    $LastChangedDate$

    $LastChangedRevision$

    $LastChangedBy$

    $HeadURL$
 ********************************************************************

*/

#include <nidas/core/Site.h>
#include <nidas/core/DSMServer.h>
#include <nidas/core/DSMConfig.h>
#include <nidas/core/DSMSensor.h>
#include <nidas/core/SampleTag.h>
#include <nidas/core/Variable.h>

#include <iostream>
#include <set>

using namespace nidas::core;
using namespace std;

namespace n_u = nidas::util;

Site::Site(): _project(0),_number(0)
{
}

Site::~Site()
{

    map<string,Parameter*>::iterator pi;

    for (pi = _parameterMap.begin(); pi != _parameterMap.end(); ++pi)
    	delete pi->second;

    // cerr << "deleting DSMServers" << endl;
    for (list<DSMServer*>::iterator is = _servers.begin();
    	is != _servers.end(); ++is) delete *is;

    // cerr << "deleting DSMConfigs" << endl;
    for (list<DSMConfig*>::iterator it = _ncDsms.begin();
    	it != _ncDsms.end(); ++it) delete *it;

}

DSMServerIterator Site::getDSMServerIterator() const
{
    return DSMServerIterator(this);
}

DSMServiceIterator Site::getDSMServiceIterator() const
{
    return DSMServiceIterator(this);
}

ProcessorIterator Site::getProcessorIterator() const
{
    return ProcessorIterator(this);
}

DSMConfigIterator Site::getDSMConfigIterator() const
{
    return DSMConfigIterator(this);
}

SensorIterator Site::getSensorIterator() const
{
    return SensorIterator(this);
}

SampleTagIterator Site::getSampleTagIterator() const
{
    return SampleTagIterator(this);
}

VariableIterator Site::getVariableIterator() const
{
    return VariableIterator(this);
}

/**
 * Initialize all sensors for a Site.
 */
void Site::initSensors() throw(n_u::IOException)
{
    list<const DSMConfig*>::const_iterator di = getDSMConfigs().begin();
    for ( ; di != getDSMConfigs().end(); ++di) {
	DSMConfig* ncdsm = const_cast<DSMConfig*>(*di);
    	ncdsm->initSensors();
    }
}

/**
 * Initialize all sensors for a given dsm.
 */
void Site::initSensors(const DSMConfig* dsm) throw(n_u::IOException)
{
    list<const DSMConfig*>::const_iterator di = getDSMConfigs().begin();
    for ( ; di != getDSMConfigs().end(); ++di) {
	DSMConfig* ncdsm = const_cast<DSMConfig*>(*di);
    	if (ncdsm == dsm) ncdsm->initSensors();
    }
}

const list<string> Site::getAllowedParameterNames() const
{
    return _allowedParameterNames;
}

/**
 * Add a parameter to this Site. Site
 * will then own the pointer and will delete it
 * in its destructor.
 */
void Site::addParameter(Parameter* val)
{
    _parameterMap.insert(make_pair<string,Parameter*>(
	    val->getName(),val));
    _constParameters.push_back(val);
}

const Parameter* Site::getParameter(const string& name) const
{
    map<string,Parameter*>::const_iterator pi = _parameterMap.find(name);
    if (pi == _parameterMap.end()) return 0;
    return pi->second;
}

const list<const Parameter*>& Site::getParameters() const
{
    return _constParameters;
}

void Site::fromDOMElement(const xercesc::DOMElement* node)
	throw(n_u::InvalidParameterException)
{
    XDOMElement xnode(node);
    if (xnode.getNodeName() != "site" &&
    	xnode.getNodeName() != "aircraft")
	    throw n_u::InvalidParameterException(
		    "Site::fromDOMElement","xml node name",
		    	xnode.getNodeName());
		    
    if(node->hasAttributes()) {
	// get all the attributes of the node
	xercesc::DOMNamedNodeMap *pAttributes = node->getAttributes();
	int nSize = pAttributes->getLength();
	for(int i=0;i<nSize;++i) {
	    XDOMAttr attr((xercesc::DOMAttr*) pAttributes->item(i));
	    string aname = attr.getName();
	    string aval = attr.getValue();
	    if (aname == "name") setName(aval);
	    else if (aname == "suffix") setSuffix(aval);
	    else if (aname == "number") {
	        istringstream ist(aval);
		int num;
		ist >> num;
		if (ist.fail()) 
		    throw n_u::InvalidParameterException(
		    	((getName().length() == 0) ? "site" : getName()),
				aname,aval);
		setNumber(num);
	    }
	}
    }

    // keep a set of DSM ids to make sure they are unique
    set<int> dsm_ids;
    // likewise with dsm names
    set<string> dsm_names;

    xercesc::DOMNode* child;
    for (child = node->getFirstChild(); child != 0;
	    child=child->getNextSibling())
    {
	if (child->getNodeType() != xercesc::DOMNode::ELEMENT_NODE) continue;
	XDOMElement xchild((xercesc::DOMElement*) child);
	const string& elname = xchild.getNodeName();
	// cerr << "element name=" << elname << endl;

	if (elname == "dsm") {
	    DSMConfig* dsm = new DSMConfig();
	    dsm->setSite(this);
	    try {
		dsm->fromDOMElement((xercesc::DOMElement*)child);
	    }
	    catch(const n_u::InvalidParameterException& e) {
	        delete dsm;
		throw;
	    }
	    if (!dsm_ids.insert(dsm->getId()).second) {
		ostringstream ost;
		ost << dsm->getId();
		delete dsm;
		throw n_u::InvalidParameterException("dsm id",
			ost.str(),"is not unique");
	    }
	    if (!dsm_names.insert(dsm->getName()).second) {
		const string& dsmname = dsm->getName();
		delete dsm;
		throw n_u::InvalidParameterException("dsm name",
			dsmname,"is not unique");
	    }
	    addDSMConfig(dsm);
	}
	else if (elname == "server") {
	    DSMServer* server = new DSMServer();
	    server->setProject(const_cast<Project*>(getProject()));
	    server->setSite(this);
	    try {
		server->fromDOMElement((xercesc::DOMElement*)child);
	    }
	    catch(const n_u::InvalidParameterException& e) {
	        delete server;
		throw;
	    }
	    addServer(server);
	}
	else if (elname == "parameter")  {
	    Parameter* parameter =
	    	Parameter::createParameter((xercesc::DOMElement*)child);
	    addParameter(parameter);
	}
    }

    validateVariables();

}

void Site::validateVariables() const
{
    // Check that variables are unique. Loop over dsms and
    // sensors so that you can report the dsm and sensor name
    // of duplicate variable.
    set<Variable> varset;
    set<Variable> dupvarset;
    pair<set<Variable>::const_iterator,bool> ins;
    set<Variable>::const_iterator it;

    for (DSMConfigIterator di = getDSMConfigIterator(); di.hasNext(); ) {
        const DSMConfig* dsm = di.next();
        for (SensorIterator si = dsm->getSensorIterator(); si.hasNext(); ) {
            const DSMSensor* sensor = si.next();
	    for (VariableIterator vi = sensor->getVariableIterator();
		vi.hasNext(); ) {
		const Variable* var = vi.next();
		if (sensor->getDuplicateIdOK()) {
		    set<Variable>::const_iterator vi = varset.find(*var);
		    if (vi != varset.end()) {
			ostringstream ost;
			ost << var->getName() << " from sensor=" <<
			    sensor->getName() << '(' <<
			    sensor->getDSMId() << ',' <<
			    sensor->getSensorId() << ')';
			throw n_u::InvalidParameterException("variable",
			    ost.str(),"is not unique");
		    }
		    dupvarset.insert(*var);
		}
		else {
		    ins = varset.insert(*var);
		    it = dupvarset.find(*var);
		    if (!ins.second || it != dupvarset.end()) {
			ostringstream ost;
			ost << var->getName() << " from sensor=" <<
			    sensor->getName() << '(' <<
			    sensor->getDSMId() << ',' <<
			    sensor->getSensorId() << ')';
			throw n_u::InvalidParameterException("variable",
			    ost.str(),"is not unique");
		    }
		}
	    }
        }
    }
}

xercesc::DOMElement* Site::toDOMParent(xercesc::DOMElement* parent,bool complete) const
    throw(xercesc::DOMException)
{
    xercesc::DOMElement* elem =
        parent->getOwnerDocument()->createElementNS(
            DOMable::getNamespaceURI(),
            (const XMLCh*)XMLStringConverter("site"));
    parent->appendChild(elem);
    return toDOMElement(elem,complete);
}

xercesc::DOMElement* Site::toDOMElement(xercesc::DOMElement* elem,bool complete) const
    throw(xercesc::DOMException)
{
    if (complete) return 0; // not supported yet

    XDOMElement xelem(elem);
    xelem.setAttributeValue("name",getName());

    ostringstream ost;
    ost << getNumber();
    ost << ends;
    xelem.setAttributeValue("number",ost.str());

    for (DSMConfigIterator di = getDSMConfigIterator(); di.hasNext(); ) {
        const DSMConfig* dsm = di.next();
        dsm->toDOMParent(elem,complete);
    }
    return elem;
}

/**
 * Look for a server on this aircraft that either has no name or whose
 * name matches hostname.  If none found, remove any domain names
 * and try again.
 */
DSMServer* Site::findServer(const string& hostname) const
{
    DSMServer* server = 0;
    for (list<DSMServer*>::const_iterator si=_servers.begin();
	si != _servers.end(); ++si) {
	DSMServer* srvr = *si;
	if (srvr->getName().length() == 0 ||
	    srvr->getName() == hostname) {
	    server = srvr;
	    break;
	}
    }
    if (server) return server;

    // Not found, remove domain name, try again
    int dot = hostname.find('.');
    for (list<DSMServer*>::const_iterator si=_servers.begin();
	si != _servers.end(); ++si) {
	DSMServer* srvr = *si;
	const string& sname = srvr->getName();
	int sdot = sname.find('.');
	if (!sname.compare(0,sdot,hostname,0,dot)) {
	    server = srvr;
	    break;
	}
    }
    return server;
}

const DSMConfig* Site::findDSM(const n_u::Inet4Address& addr) const
{
    for (list<const DSMConfig*>::const_iterator di=_dsms.begin();
	di != _dsms.end(); ++di) {
	const DSMConfig* dsm = *di;
#ifdef DEBUG
	cerr << "Checking dsm " << dsm->getName() << endl;
#endif
        try {
	    list<n_u::Inet4Address> addrs =
		n_u::Inet4Address::getAllByName(dsm->getName());
	    for (list<n_u::Inet4Address>::const_iterator ai=addrs.begin();
		ai != addrs.end(); ++ai) {
		if (*ai == addr) return dsm;
	    }
        }
	catch(n_u::UnknownHostException &e) {}
    }
    return 0;
}

const DSMConfig* Site::findDSM(unsigned int id) const
{
    for (list<const DSMConfig*>::const_iterator di=_dsms.begin();
	di != _dsms.end(); ++di) {
	const DSMConfig* dsm = *di;
#ifdef DEBUG
	cerr << "Checking dsm " << dsm->getName() << " for id=" << id << endl;
#endif
	if (dsm->getId() == id) return dsm;
    }
    return 0;
}

const DSMConfig* Site::findDSM(const string& name) const
{
    for (list<const DSMConfig*>::const_iterator di=_dsms.begin();
	di != _dsms.end(); ++di) {
	const DSMConfig* dsm = *di;
#ifdef DEBUG
	cerr << "Checking dsm " << dsm->getName()
	     << " for name=" << name << endl;
#endif
	if (dsm->getName() == name) return dsm;
    }
    return 0;
}

DSMSensor* Site::findSensor(unsigned int id) const
{
    SensorIterator si = getSensorIterator();
    for ( ; si.hasNext(); ) {
	DSMSensor* sensor = si.next();

#ifdef DEBUG
	cerr << "Site::findSensor, " << getName() << ", getId=" <<
	    sensor->getDSMId() << ',' << sensor->getSensorId() <<
	    " against id=" <<
	    GET_DSM_ID(id) << ',' << GET_SPS_ID(id) << endl;
#endif
	if (sensor->getId() == id) return sensor;
        SampleTagIterator sti = sensor->getSampleTagIterator();
        for ( ; sti.hasNext(); ) {
            const SampleTag* stag = sti.next();
            if (stag->getId() == id) return sensor;
        }
    }
    return 0;
}
