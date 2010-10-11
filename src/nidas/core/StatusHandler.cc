/*
 ********************************************************************
    Copyright 2005 UCAR, NCAR, All Rights Reserved

    $LastChangedDate$

    $LastChangedRevision$

    $LastChangedBy$

    $HeadURL$
 ********************************************************************
*/
#include <xercesc/sax2/Attributes.hpp>

#include <nidas/core/DSMSensor.h>
#include <nidas/core/StatusHandler.h>
#include <nidas/core/StatusListener.h>
#include <nidas/core/XMLStringConverter.h>

#include <iostream>             // cerr
#include <fstream>              // ofstream
#include <map>

using namespace std;
using namespace nidas::core;

// ---------------------------------------------------------------------------
//  StatusHandler: Overrides of the SAX ErrorHandler interface
// ---------------------------------------------------------------------------
void StatusHandler::warning(const xercesc::SAXParseException & e)
{
    cerr << "\nWarning at file " << (string) XMLStringConverter(e.
                                                                getSystemId
                                                                ())
        << ", line " << e.getLineNumber()
        << ", char " << e.getColumnNumber()
        << "\n  Message: " << (string) XMLStringConverter(e.
                                                          getMessage()) <<
        endl;
}

void StatusHandler::error(const xercesc::SAXParseException & e)
{
    cerr << "\nError at file " << XMLStringConverter(e.getSystemId())
        << ", line " << e.getLineNumber()
        << ", char " << e.getColumnNumber()
        << "\n  Message: " << XMLStringConverter(e.getMessage()) << endl;
}

void StatusHandler::fatalError(const xercesc::SAXParseException & e)
{
    cerr << "\nFatal Error at file " << XMLStringConverter(e.getSystemId())
        << ", line " << e.getLineNumber()
        << ", char " << e.getColumnNumber()
        << "\n  Message: " << XMLStringConverter(e.getMessage()) << endl;
}


// ---------------------------------------------------------------------------
//  StatusHandler: Overrides of the SAX DocumentHandler interface
// ---------------------------------------------------------------------------
void StatusHandler::startElement(const XMLCh * const uri,
                                 const XMLCh * const localname,
                                 const XMLCh * const qname,
                                 const xercesc::Attributes & attributes)
{
//   cerr << "qname: " << XMLStringConverter(qname) << endl;
//   unsigned int len = attributes.getLength();
//   for (unsigned int index = 0; index < len; index++) {
//     cerr << "attributes.getQName(" << index << "): ";
//     cerr << XMLStringConverter(attributes.getQName(index)) << endl;
//     cerr << "attributes.getValue(" << index << "): ";
//     cerr << XMLStringConverter(attributes.getValue(index)) << endl;
//   }
    if ((string) XMLStringConverter(qname) == "name")
        _element = SOURCE;
    else if ((string) XMLStringConverter(qname) == "clock")
        _element = TIME;
    else if ((string) XMLStringConverter(qname) == "status")
        _element = STATUS;
    else if ((string) XMLStringConverter(qname) == "samplepool")
        _element = SAMPLEPOOL;
}

void StatusHandler::endElement(const XMLCh * const uri,
                               const XMLCh * const localname,
                               const XMLCh * const qname)
{
    _element = NONE;
}

void StatusHandler::characters(const XMLCh * const chars,
                               const unsigned int length)
{
    switch (_element) {
    case SOURCE:
        _src = XMLStringConverter(chars);
        break;

    case TIME:
        _listener->_clocks[_src] = XMLStringConverter(chars);
        break;

    case STATUS:
        _listener->_status[_src] = XMLStringConverter(chars);
        break;
    case SAMPLEPOOL:
        _listener->_samplePool[_src] = XMLStringConverter(chars);
        break;
    case NONE:
        break;
    }
}
