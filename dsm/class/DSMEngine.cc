/*
 ********************************************************************
    Copyright 2005 UCAR, NCAR, All Rights Reserved

    $LastChangedDate$

    $LastChangedRevision$

    $LastChangedBy$

    $HeadURL$
 ********************************************************************
*/

#include <DSMEngine.h>

#include <XMLStringConverter.h>
#include <XMLParser.h>

#include <XMLConfigInput.h>
#include <XMLFdInputSource.h>

#include <iostream>

using namespace dsm;
using namespace std;
using namespace xercesc;

DSMRunstring::DSMRunstring(int argc, char** argv):
    _debug(false),_wait(false)
{
    try {
	_mcastSockAddr = atdUtil::Inet4SocketAddress(
	    atdUtil::Inet4Address::getByName(DSM_MULTICAST_ADDR),
	    DSM_MULTICAST_PORT);
    }
    catch(const atdUtil::UnknownHostException& e) {}	// won't happen

    // extern char *optarg;  /* set by getopt()  */
    extern int optind;       /*  "  "     "      */
    int opt_char;            /* option character */

    while ((opt_char = getopt(argc, argv, "dw")) != -1) {
	switch (opt_char) {
	case 'd':
	    _debug = true;
	    break;
	case 'w':
	    _wait = true;
	    break;
	case '?':
	    usage(argv[0]);
	}
    }
    if (optind == argc - 1) {
        string url = string(argv[optind++]);
	if(url.length() > 7 && !url.compare(0,7,"mcsock:")) {
	    url = url.substr(7);
	    size_t ic = url.find(':');
	    if (ic == string::npos) {
		cerr << "Invalid host:port parameter: " << url << endl;
		usage(argv[0]);
	    }
	    string mcastAddr = url.substr(0,ic);
	    istringstream ist(url.substr(ic+1));
	    int port;
	    ist >> port;
	    if (ist.fail()) {
		cerr << "Invalid port number: " << url.substr(ic+1) << endl;
		usage(argv[0]);
	    }
	    try {
		_mcastSockAddr = atdUtil::Inet4SocketAddress(
		    atdUtil::Inet4Address::getByName(mcastAddr),port);
	    }
	    catch(const atdUtil::UnknownHostException& e) {
	        cerr << e.what() << endl;
		usage(argv[0]);
	    }	
	}
	else _configFile = url;
    }
    cerr << "optind=" << optind << " argc=" << argc << endl;

    if (optind != argc) usage(argv[0]);
}

/* static */
void DSMRunstring::usage(const char* argv0) 
{
    cerr << "\
Usage: " << argv0 << " [-dw] [ config ]\n\n\
  -d:     debug - Send error messages to stderr, otherwise to syslog\n\
  -w:     wait  - wait for the XmlRpc 'start' cammand\n\
  config: either the name of a local DSM configuration XML file to be read,\n\
      or a multicast socket address in the form \"mcsock:addr:port\".\n\
The default config is \"mcsock:" <<
	DSM_MULTICAST_ADDR << ":" << DSM_MULTICAST_PORT << "\"" << endl;
    exit(1);
}

/* static */
DSMEngine* DSMEngine::_instance = 0;

/* static */
int DSMEngine::main(int argc, char** argv) throw()
{
    cerr << "compiled on " << __DATE__ << " at " << __TIME__ << endl;

    DSMRunstring rstr(argc,argv);

    auto_ptr<DSMEngine> engine(createInstance(&rstr));

    engine->_logger->log(LOG_ERR,"calling engine->run()");
    try {
      engine->run();
    }
    catch (const atdUtil::Exception& e) {
      engine->_logger->log(LOG_ERR,e.what()); }
    catch (...) {
      engine->_logger->log(LOG_ERR,"Unknown error thrown..."); }

    return 0;
}

void DSMEngine::run() throw()
{
    DOMDocument* projectDoc = 0;

    // start the xmlrpc control thread
    _xmlrpcThread = new XmlRpcThread("DSMEngineXmlRpc");
    _xmlrpcThread->start();

    // initialize the status thread
    _statusThread = new StatusThread("DSMEngineStatus",10);

    // default the loop to run only once
    _quit = true;
    do {
      _logger->log(LOG_ERR,
         "---------------------> top of DSMEngine loop <-------------------------");

      // purge members before re-starting the loop
      if (_selector) {
        _selector->interrupt();
        _selector->join();
        delete _selector;	// this closes any still-open sensors
        _selector = 0;
      }
      if (_project) {
        delete _project;
        _project = 0;
        _dsmConfig = 0;
      }
      // stop the status Thread
      _statusThread->interrupt();

      if (_rstr->_wait) {
	_quit = false;
        _logger->log(LOG_ERR,"wait on the _runCond condition variable...");
        // wait on the _runCond condition variable
        _runCond.lock();
        while (!_run)
          _runCond.wait();
        _run = false;
        _runCond.unlock();
        if (_quit) continue;
      }
      else
        _logger->log(LOG_ERR,"don't wait on the _runCond condition variable...");

      if (projectDoc) {
        projectDoc->release();
        projectDoc = 0;
      }
      cerr << "DSMEngine: first fetch the configuration" << endl;
      // first fetch the configuration
      try {
	if (_rstr->_configFile.length() == 0)
          projectDoc = requestXMLConfig(_rstr->_mcastSockAddr);
	else
          projectDoc = parseXMLConfigFile(_rstr->_configFile);
      }
      catch (const atdUtil::Exception& e) {
	// DSMEngine::interrupt() does an _xmlRequestSocket->close(),
	// which will throw an IOException in requestXMLConfig 
	// if we were still waiting for the XML config.
	_logger->log(LOG_ERR,e.what());
	continue;
      }
      catch (const SAXException& e) {
	_logger->log(LOG_ERR,
                    XMLStringConverter(e.getMessage()));
	continue;
      }
      catch (const DOMException& e) {
	_logger->log(LOG_ERR,
                    XMLStringConverter(e.getMessage()));
	continue;
      }
      catch (const XMLException& e) {
	_logger->log(LOG_ERR,
                    XMLStringConverter(e.getMessage()));
	continue;
      }
      if (_interrupt) continue;
      cerr << "DSMEngine: then initialize the DSMEngine" << endl;
      // then initialize the DSMEngine
      try {
        if (projectDoc)
          initialize(projectDoc);
      }
      catch (const atdUtil::InvalidParameterException& e) {
	_logger->log(LOG_ERR,e.what());
	continue;
      }
      projectDoc->release();
      projectDoc = 0;

      cerr << "DSMEngine: start your sensors" << endl;
      // start your sensors
      try {
	openSensors();
	connectOutputs();
      }
      catch (const atdUtil::IOException& e) {
	_logger->log(LOG_ERR,e.what());
	continue;
      }
      if (_interrupt) continue;

      // start the status Thread
      _statusThread->start();

      cerr << "DSMEngine: wait()" << endl;
      try {
	wait();
      }
      catch (const atdUtil::Exception& e) {
	_logger->log(LOG_ERR,e.what());
	continue;
      }
    } while (!_quit);

    if (projectDoc)
      projectDoc->release();

    cerr << "DSMEngine::main() exiting..." << endl;
}

void DSMEngine::mainStart()
{
  _runCond.lock();
  _run = true;
  _runCond.signal();
  _runCond.unlock();
}

void DSMEngine::mainStop()
{
  _runCond.lock();
  _run = false;
  _runCond.signal();
  _runCond.unlock();
  interrupt();
}

void DSMEngine::mainRestart()
{
  _runCond.lock();
  _run = true;
  _runCond.signal();
  _runCond.unlock();
  interrupt();
}

void DSMEngine::mainQuit()
{
  _runCond.lock();
  _run  = true;
  _runCond.signal();
  _runCond.unlock();
  interrupt();
  _quit = true;
}

/* static */
void DSMEngine::setupSignals()
{
    sigset_t sigset;
    sigemptyset(&sigset);
    sigaddset(&sigset,SIGHUP);
    sigaddset(&sigset,SIGTERM);
    sigaddset(&sigset,SIGINT);
    sigprocmask(SIG_UNBLOCK,&sigset,(sigset_t*)0);
                                                                                
    struct sigaction act;
    sigemptyset(&sigset);
    act.sa_mask = sigset;
    act.sa_flags = SA_SIGINFO;
    act.sa_sigaction = DSMEngine::sigAction;
    sigaction(SIGHUP,&act,(struct sigaction *)0);
    sigaction(SIGINT,&act,(struct sigaction *)0);
    sigaction(SIGTERM,&act,(struct sigaction *)0);
}

/* static */
void DSMEngine::sigAction(int sig, siginfo_t* siginfo, void* vptr) {
    atdUtil::Logger::getInstance()->log(LOG_INFO,
    	"received signal %s(%d), si_signo=%d, si_errno=%d, si_code=%d",
	strsignal(sig),sig,
	(siginfo ? siginfo->si_signo : -1),
	(siginfo ? siginfo->si_errno : -1),
	(siginfo ? siginfo->si_code : -1));
                                                                                
    switch(sig) {
    case SIGHUP:
    case SIGTERM:
    case SIGINT:
      DSMEngine::getInstance()->mainQuit();
      break;
    }
}

DSMEngine::DSMEngine(const DSMRunstring* rstr):
    _interrupt(false),_runCond("_runCond"),_project(0),_dsmConfig(0),_selector(0),
    _statusThread(0),_xmlrpcThread(0),_xmlRequestSocket(0),_rstr(rstr)
{
  setupSignals();

  if (_rstr->_debug)
    _logger = atdUtil::Logger::createInstance(stderr);

  else {
    char hostname[128];
    gethostname(hostname,sizeof(hostname));
    _logger = atdUtil::Logger::createInstance(hostname,LOG_CONS,LOG_LOCAL5);
  }
}

DSMEngine::~DSMEngine()
{
    delete _statusThread;
    delete _xmlrpcThread;
    delete _xmlRequestSocket;
    delete _selector;	// this closes any still-open sensors

    _outputMutex.lock();
    list<SampleOutput*>::const_iterator oi;
    for (oi = _connectedOutputs.begin(); oi != _connectedOutputs.end(); ++oi) {
	SampleOutput* output = *oi;
	try {
	    output->flush();
	    output->close();
	}
	catch(const atdUtil::IOException& e) {
	    atdUtil::Logger::getInstance()->log(LOG_INFO,
		"~DSMEngine %s: %s",output->getName().c_str(),e.what());
	}
    }
    _outputMutex.unlock();

    if (_dsmConfig) {
	const list<SampleOutput*>& outputs = _dsmConfig->getOutputs();
	for (oi = outputs.begin(); oi != outputs.end(); ++oi) {
	    SampleOutput* output = *oi;
	    try {
		output->close();
	    }
	    catch(const atdUtil::IOException& e) {
		atdUtil::Logger::getInstance()->log(LOG_INFO,
		    "~DSMEngine %s: %s",output->getName().c_str(),e.what());
	    }
	}
    }
    cerr << "DSMEngine deleted" << endl;
    delete _project;
}

DSMEngine* DSMEngine::createInstance(const DSMRunstring* rstr)
{
  if (!_instance) {
    _instance = new DSMEngine(rstr);
  }
  return _instance;
}

DSMEngine* DSMEngine::getInstance() 
{
    return _instance;
}

DOMDocument* DSMEngine::requestXMLConfig(
	const atdUtil::Inet4SocketAddress &mcastAddr)
	throw(atdUtil::Exception,
	    DOMException,SAXException,XMLException)
{
    auto_ptr<XMLParser> parser(new XMLParser());
    // throws Exception, DOMException

    // If parsing xml received from a server over a socket,
    // turn off validation - assume the server has validated the XML.
    parser->setDOMValidation(false);
    parser->setDOMValidateIfSchema(false);
    parser->setDOMNamespaces(true);
    parser->setXercesSchema(false);
    parser->setXercesSchemaFullChecking(false);
    parser->setDOMDatatypeNormalization(false);
    parser->setXercesUserAdoptsDOMDocument(true);

    delete _xmlRequestSocket;
    _xmlRequestSocket = 0;
    _xmlRequestSocket = new XMLConfigInput();
    _xmlRequestSocket->setInet4McastSocketAddress(mcastAddr);

    auto_ptr<atdUtil::Socket> configSock(_xmlRequestSocket->connect());
    	// throws IOException

    _xmlRequestSocket->close();
    delete _xmlRequestSocket;
    _xmlRequestSocket = 0;

    std::string sockName = configSock->getInet4SocketAddress().toString();
    XMLFdInputSource sockSource(sockName,configSock->getFd());

    cerr << "parsing socket input" << endl;
    DOMDocument* doc = 0;
    try {
	// throws SAXException, XMLException, DOMException
	// according to xerces API doc
	// (xercesc source code doesn't have throw lists)
	doc = parser->parse(sockSource);
    }
    catch(...) {
        cerr << "XML Config Service did not find this DSM" << endl;
	configSock->close();
	throw;
    }
    cerr << "DSMEngine::requestXMLConfig: configSock closing" << endl;
    configSock->close();
    cerr << "DSMEngine::requestXMLConfig: configSock closed" << endl;
    return doc;
}

/* static */
DOMDocument* DSMEngine::parseXMLConfigFile(const string& xmlFileName)
	throw(atdUtil::Exception,
	DOMException,SAXException,XMLException)
{
    auto_ptr<XMLParser> parser(new XMLParser());
    // throws Exception, DOMException

    // If parsing a local file, turn on validation
    parser->setDOMValidation(true);
    parser->setDOMValidateIfSchema(true);
    parser->setDOMNamespaces(true);
    parser->setXercesSchema(true);
    parser->setXercesSchemaFullChecking(true);
    parser->setDOMDatatypeNormalization(false);
    parser->setXercesUserAdoptsDOMDocument(true);

    DOMDocument* doc = parser->parse(xmlFileName);

    return doc;
}

void DSMEngine::initialize(DOMDocument* projectDoc)
	throw(atdUtil::InvalidParameterException)
{
    _project = Project::getInstance();

    _project->fromDOMElement(projectDoc->getDocumentElement());
    // throws atdUtil::InvalidParameterException;

    const list<Site*>& sitelist = _project->getSites();
    if (sitelist.size() == 0)
    	throw atdUtil::InvalidParameterException("project","site",
		"no site tag in XML config");

    if (sitelist.size() > 1)
    	throw atdUtil::InvalidParameterException("project","site",
		"multiple site tags in XML config");
    const Site* site = sitelist.front();

    const list<DSMConfig*>& dsms = site->getDSMConfigs();
    if (dsms.size() == 0)
    	throw atdUtil::InvalidParameterException("site","dsm",
		"no dsm tag in XML config");

    if (dsms.size() > 1)
    	throw atdUtil::InvalidParameterException("site","dsm",
		"multiple dsm tags in XML config");
    _dsmConfig = dsms.front();
}

void DSMEngine::openSensors() throw(atdUtil::IOException)
{
    _selector = new PortSelector;
    _selector->start();
    _dsmConfig->openSensors(_selector);
}

void DSMEngine::connectOutputs() throw(atdUtil::IOException)
{
    // request connection for outputs
    bool processedOutput = false;
    const list<SampleOutput*>& outputs = _dsmConfig->getOutputs();
    list<SampleOutput*>::const_iterator oi;

    for (oi = outputs.begin(); oi != outputs.end(); ++oi) {
	SampleOutput* output = *oi;
	if (!output->isRaw()) processedOutput = true;
	output->requestConnection(this);
    }
    
    const list<DSMSensor*>& sensors = _dsmConfig->getSensors();
    list<DSMSensor*>::const_iterator si;
    for (si = sensors.begin(); si != sensors.end(); ++si) {
	DSMSensor* sensor = *si;
	// If we're outputting processed samples add
	// sensors as a RawSampleClient of themselves.
	// If it's a clock sensor, also have the clock
	// sensor process its raw samples.
	if (processedOutput) sensor->addRawSampleClient(sensor);
    }
}


/* A remote system has connnected to one of our outputs.
 * We don't clone the output here.
 */
void DSMEngine::connected(SampleOutput* output) throw()
{
    try {
	output->init();
    }
    catch (const atdUtil::IOException& ioe) {
	atdUtil::Logger::getInstance()->log(LOG_ERR,
	    "DSMEngine: error in init of %s: %s",
	    	output->getName().c_str(),ioe.what());
	disconnected(output);
    }

    const list<DSMSensor*>& sensors = _dsmConfig->getSensors();
    list<DSMSensor*>::const_iterator si;

    for (si = sensors.begin(); si != sensors.end(); ++si) {
	DSMSensor* sensor = *si;
	cerr << "adding " << (output->isRaw() ? "raw" : "nonraw") <<
		" output to sensor " << sensor->getName() << 
		" output=" << output->getName() << endl;
	if (output->isRaw()) sensor->addRawSampleClient(output);
	else sensor->addSampleClient(output);
    }
    _outputMutex.lock();
    _connectedOutputs.push_back(output);
    _outputMutex.unlock();
}

/* An output wants to disconnect (probably the remote server went down) */
void DSMEngine::disconnected(SampleOutput* output) throw()
{
    cerr << "SampleOutput " << output->getName() << " disconnected" << endl;
    const list<DSMSensor*>& sensors = _dsmConfig->getSensors();
    list<DSMSensor*>::const_iterator si;
    for (si = sensors.begin(); si != sensors.end(); ++si) {
	DSMSensor* sensor = *si;
	cerr << "removing output from sensor " << sensor->getName() << endl;
	if (output->isRaw()) sensor->removeRawSampleClient(output);
	else sensor->removeSampleClient(output);
    }
    try {
	output->close();
    }
    catch (const atdUtil::IOException& ioe) {
	atdUtil::Logger::getInstance()->log(LOG_ERR,
	    "DSMEngine: error closing %s: %s",
	    	output->getName().c_str(),ioe.what());
    }

    _outputMutex.lock();
    list<SampleOutput*>::iterator oi;
    for (oi = _connectedOutputs.begin(); oi != _connectedOutputs.end();) {
	if (*oi == output) oi = _connectedOutputs.erase(oi);
	else ++oi;
    }
    _outputMutex.unlock();

    // try to reconnect (should probably restart and request XML again)
    output->requestConnection(this);
    return;
}

void DSMEngine::interrupt() throw(atdUtil::Exception)
{
    _interrupt = true;
    atdUtil::Logger::getInstance()->log(LOG_INFO,
	"DSMEngine::interrupt() called");
    if (_selector) {
	atdUtil::Logger::getInstance()->log(LOG_INFO,
	    "DSMEngine::interrupt, interrupting PortSelector");
        _selector->interrupt();
    }
    if (_statusThread) {
      _statusThread->interrupt();
      _statusThread->join();
    }
    // If DSMEngine is waiting for an XML connection, closing the
    // _xmlRequestSocket here will cause an IOException in
    // DSMEngine::requestXMLConfig().
    if (_xmlRequestSocket) _xmlRequestSocket->close();

    atdUtil::Logger::getInstance()->log(LOG_INFO,
	"DSMEngine::interrupt() done");

    // do not quit when controlled by XMLRPC
    // TODO override this feature for Ctrl-C
    if (!_rstr->_wait) // || signal_seen(SIGINT))
      _quit = true;
    else
      _logger->log(LOG_ERR,"DSMEngine::interrupt wait on the _runCond condition variable...");
}

void DSMEngine::wait() throw(atdUtil::Exception)
{
  _selector->join();
  cerr << "DSMEngine::wait() _selector joined" << endl;
}
