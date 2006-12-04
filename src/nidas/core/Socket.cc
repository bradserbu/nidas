/*
 ********************************************************************
    Copyright 2005 UCAR, NCAR, All Rights Reserved

    $LastChangedDate$

    $LastChangedRevision$

    $LastChangedBy$

    $HeadURL$
 ********************************************************************

*/

#include <nidas/core/Socket.h>
#include <nidas/core/McSocket.h>
#include <nidas/util/Logger.h>

using namespace nidas::core;
using namespace std;
using namespace xercesc;

namespace n_u = nidas::util;

Socket::Socket():
	remoteSockAddr(
		auto_ptr<n_u::SocketAddress>(new n_u::Inet4SocketAddress())),
	socket(0),firstRead(true),newFile(true),keepAliveIdleSecs(7200),
        minWriteInterval(USECS_PER_SEC/100),lastWrite(0),
        nonBlocking(true)
{
    setName("Socket " + remoteSockAddr->toString());
}

/*
 * Copy constructor.  Should only be called before connection.
 */
Socket::Socket(const Socket& x):
	remoteSockAddr(
		auto_ptr<n_u::SocketAddress>(x.remoteSockAddr->clone())),
	socket(0),name(x.name),
	firstRead(true),newFile(true),keepAliveIdleSecs(x.keepAliveIdleSecs),
        minWriteInterval(x.minWriteInterval),lastWrite(0),
        nonBlocking(x.nonBlocking)
{
}

/*
 * Constructor with a connected n_u::Socket.
 */
Socket::Socket(n_u::Socket* sock):
	remoteSockAddr(
	    auto_ptr<n_u::SocketAddress>(sock->getRemoteSocketAddress().clone())),
	socket(sock),firstRead(true),newFile(true),
	keepAliveIdleSecs(7200),
        minWriteInterval(USECS_PER_SEC/100),lastWrite(0)
{
    setName(remoteSockAddr->toString());
    try {
	keepAliveIdleSecs = socket->getKeepAliveIdleSecs();
        nonBlocking = sock->isNonBlocking();
    }
    catch (const n_u::IOException& e) {
    }
}

Socket::~Socket()
{
    close();
    delete socket;
}

Socket* Socket::clone() const 
{
    return new Socket(*this);
}

size_t Socket::getBufferSize() const throw()
{
    size_t blen = 16384;
    try {
	if (socket) blen = socket->getReceiveBufferSize();
    }
    catch (const n_u::IOException& e) {}

    // linux sockets (x86 laptop, FC5) return a receive buffer
    // sizeof 87632.  We don't need that much.
    // 
    if (blen > 16384) blen = 16384;
    return blen;
}

n_u::Inet4Address Socket::getRemoteInet4Address() const throw()
{
    if (socket) {
	const n_u::SocketAddress& addr = socket->getRemoteSocketAddress();
	const n_u::Inet4SocketAddress* i4addr =
		dynamic_cast<const n_u::Inet4SocketAddress*>(&addr);
	if (i4addr) return i4addr->getInet4Address();
    }
    return n_u::Inet4Address();
}


IOChannel* Socket::connect() throw(n_u::IOException)
{
    n_u::Socket* waitsock = new n_u::Socket();
    waitsock->connect(*remoteSockAddr.get());
    waitsock->setKeepAliveIdleSecs(keepAliveIdleSecs);
    waitsock->setNonBlocking(nonBlocking);
    return new nidas::core::Socket(waitsock);
}

void Socket::requestConnection(ConnectionRequester* requester)
	throw(n_u::IOException)
{
    n_u::Socket* waitsock = new n_u::Socket();
    waitsock->connect(*remoteSockAddr.get());
    waitsock->setKeepAliveIdleSecs(keepAliveIdleSecs);
    waitsock->setNonBlocking(nonBlocking);
    // cerr << "Socket::connected " << getName();
    requester->connected(new nidas::core::Socket(waitsock));
}

/*
 * Do the actual hardware read.
 */
size_t Socket::read(void* buf, size_t len) throw (n_u::IOException)
{
    if (firstRead) firstRead = false;
    else newFile = false;
    return socket->recv(buf,len);
}

ServerSocket::ServerSocket():
	localSockAddr(new n_u::Inet4SocketAddress(0)),
        servSock(0),connectionRequester(0),
        thread(0),keepAliveIdleSecs(7200),
        minWriteInterval(USECS_PER_SEC/100),
        nonBlocking(true)
{
    setName("ServerSocket " + localSockAddr->toString());
}

ServerSocket::ServerSocket(const n_u::SocketAddress& addr):
	localSockAddr(addr.clone()),
        servSock(0),connectionRequester(0),
        thread(0),keepAliveIdleSecs(7200),
        minWriteInterval(USECS_PER_SEC/100),
        nonBlocking(true)
{
    setName("ServerSocket " + localSockAddr->toString());
}

ServerSocket::ServerSocket(const ServerSocket& x):
	localSockAddr(x.localSockAddr->clone()),name(x.name),
	servSock(0),connectionRequester(0),thread(0),
	keepAliveIdleSecs(x.keepAliveIdleSecs),
        minWriteInterval(x.minWriteInterval),
        nonBlocking(x.nonBlocking)
{
}

ServerSocket::~ServerSocket()
{
    if (thread) {
	try {
	    if (thread->isRunning()) thread->cancel();
#ifdef DEBUG
            cerr << "~ServerSocket joining thread" << endl;
#endif
	    thread->join();
#ifdef DEBUG
            cerr << "~ServerSocket joined thread" << endl;
#endif
	}
	catch(const n_u::Exception& e) {
	}
	delete thread;
    }
    close();
    delete servSock;
}

ServerSocket* ServerSocket::clone() const 
{
    return new ServerSocket(*this);
}

void ServerSocket::close() throw (nidas::util::IOException)
{
    if (servSock) servSock->close();
}


IOChannel* ServerSocket::connect() throw(n_u::IOException)
{
    if (!servSock) servSock= new n_u::ServerSocket(*localSockAddr.get());
    n_u::Socket* newsock = servSock->accept();
    newsock->setKeepAliveIdleSecs(keepAliveIdleSecs);

    nidas::core::Socket* newCSocket = new nidas::core::Socket(newsock);
    newCSocket->setMinWriteInterval(getMinWriteInterval());
    newCSocket->setNonBlocking(nonBlocking);

    return newCSocket;
}

void ServerSocket::requestConnection(ConnectionRequester* requester)
	throw(n_u::IOException)
{
    connectionRequester = requester;
    if (!servSock) servSock= new n_u::ServerSocket(*localSockAddr.get());
    if (!thread) thread = new ServerSocketConnectionThread(*this);
    try {
	if (!thread->isRunning()) thread->start();
    }
    catch(const n_u::Exception& e) {
        throw n_u::IOException(getName(),"requestConnection",e.what());
    }
}

int ServerSocketConnectionThread::run() throw(n_u::IOException)
{
    for (;;) {
	// create nidas::core::Socket from n_u::Socket
	n_u::Socket* lowsock = socket.servSock->accept();
	lowsock->setKeepAliveIdleSecs(socket.getKeepAliveIdleSecs());
        lowsock->setNonBlocking(socket.isNonBlocking());

	nidas::core::Socket* newsock = new nidas::core::Socket(lowsock);
        newsock->setMinWriteInterval(socket.getMinWriteInterval());

	n_u::Logger::getInstance()->log(LOG_DEBUG,
		"Accepted connection: remote=%s",
		newsock->getRemoteInet4Address().getHostAddress().c_str());
	socket.connectionRequester->connected(newsock);
    }
    return RUN_OK;
}

/* static */
IOChannel* Socket::createSocket(const DOMElement* node)
            throw(n_u::InvalidParameterException)
{
    IOChannel* channel = 0;
    XDOMElement xnode(node);
    const string& type = xnode.getAttributeValue("type");

    if (type == "mcaccept" || type == "mcrequest" ||
	type == "dgaccept" || type == "dgrequest")
    	channel = new McSocket();
    else if (type == "server")
    	channel = new ServerSocket();
    else if (type == "client" || type.length() == 0)
    	channel = new Socket();
    else throw n_u::InvalidParameterException(
	    "Socket::createSocket","unknown socket type",type);
    return channel;
}

void Socket::fromDOMElement(const DOMElement* node)
	throw(n_u::InvalidParameterException)
{
    n_u::Inet4Address addr;
    bool inet4Addr = false;
    int port = -1;
    string path;

    XDOMElement xnode(node);
    if(node->hasAttributes()) {
    // get all the attributes of the node
        DOMNamedNodeMap *pAttributes = node->getAttributes();
        int nSize = pAttributes->getLength();
        for(int i=0;i<nSize;++i) {
            XDOMAttr attr((DOMAttr*) pAttributes->item(i));
            // get attribute name
            const std::string& aname = attr.getName();
            const std::string& aval = attr.getValue();
            // Inet4 address
	    if (aname == "address") {
		try {
		    addr = n_u::Inet4Address::getByName(aval);
		}
		catch(const n_u::UnknownHostException& e) {
		    throw n_u::InvalidParameterException(
			"socket","unknown host",aval);
		}
                inet4Addr = true;
	    }
            // Unix socket address
	    else if (aname == "path") {
                path = aval;
	    }
	    else if (aname == "port") {
		istringstream ist(aval);
		ist >> port;
		if (ist.fail())
			throw n_u::InvalidParameterException(
			    "socket","invalid port number",aval);
                inet4Addr = true;
	    }
	    else if (aname == "type") {
		if (aval != "client")
			throw n_u::InvalidParameterException(
			    "socket","invalid socket type",aval);
	    }
	    else if (aname == "maxIdle") {
		istringstream ist(aval);
		int secs;
		ist >> secs;
		if (ist.fail())
		    throw n_u::InvalidParameterException(getName(),"maxIdle",aval);
		try {
		    setKeepAliveIdleSecs(secs);
		}
		catch (const n_u::IOException& e) {		// won't happen
		}
	    }
	    else if (aname == "minWrite") {
		istringstream ist(aval);
		int usecs;
		ist >> usecs;
		if (ist.fail())
		    throw n_u::InvalidParameterException(getName(),"minWrite",aval);
                setMinWriteInterval(usecs);
	    }
	    else throw n_u::InvalidParameterException(
	    	string("unrecognized socket attribute: ") + aname);
	}
    }
    if (inet4Addr && path.length() > 0)
        throw n_u::InvalidParameterException("socket","address",
            "cannot specify both an IP socket address and a unix socket path");
    if (port >= 0 && path.length() > 0)
        throw n_u::InvalidParameterException("socket","address",
            "cannot specify both an IP socket port and a unix socket path");

    if (path.length() > 0) 
        remoteSockAddr.reset(new n_u::UnixSocketAddress(path));
    else
        remoteSockAddr.reset(new n_u::Inet4SocketAddress(addr,port));
}

DOMElement* Socket::toDOMParent(
    DOMElement* parent)
    throw(DOMException)
{
    DOMElement* elem =
        parent->getOwnerDocument()->createElementNS(
                (const XMLCh*)XMLStringConverter("dsmconfig"),
			DOMable::getNamespaceURI());
    parent->appendChild(elem);
    return toDOMElement(elem);
}

DOMElement* Socket::toDOMElement(DOMElement* node)
    throw(DOMException)
{
    return node;
}

void ServerSocket::fromDOMElement(const DOMElement* node)
	throw(n_u::InvalidParameterException)
{
    int port = -1;
    string path;

    XDOMElement xnode(node);
    if(node->hasAttributes()) {
    // get all the attributes of the node
        DOMNamedNodeMap *pAttributes = node->getAttributes();
        int nSize = pAttributes->getLength();
        for(int i=0;i<nSize;++i) {
            XDOMAttr attr((DOMAttr*) pAttributes->item(i));
            // get attribute name
            const std::string& aname = attr.getName();
            const std::string& aval = attr.getValue();
	    if (aname == "port") {
		istringstream ist(aval);
		ist >> port;
		if (ist.fail())
			throw n_u::InvalidParameterException(
			    "socket","invalid port number",aval);
	    }
            // Unix socket address
	    else if (aname == "path") {
                path = aval;
	    }
	    else if (aname == "type") {
		if (aval != "server")
			throw n_u::InvalidParameterException(
			    "socket","invalid socket type",aval);
	    }
	    else if (aname == "maxIdle") {
		istringstream ist(aval);
		int secs;
		ist >> secs;
		if (ist.fail())
		    throw n_u::InvalidParameterException(getName(),"maxIdle",aval);
		try {
		    setKeepAliveIdleSecs(secs);
		}
		catch (const n_u::IOException& e) {		// won't happen
		}
	    }
	    else if (aname == "minWrite") {
		istringstream ist(aval);
		int usecs;
		ist >> usecs;
		if (ist.fail())
		    throw n_u::InvalidParameterException(getName(),"minWrite",aval);
                setMinWriteInterval(usecs);
	    }
	    else throw n_u::InvalidParameterException(
	    	string("unrecognized socket attribute: ") + aname);
	}
    }
    if (port >= 0 && path.length() > 0)
        throw n_u::InvalidParameterException("socket","address",
            "cannot specify both an IP socket port and a unix socket path");

    if (path.length() > 0) 
        localSockAddr.reset(new n_u::UnixSocketAddress(path));
    else
        localSockAddr.reset(new n_u::Inet4SocketAddress(port));
}

DOMElement* ServerSocket::toDOMParent(
    DOMElement* parent)
    throw(DOMException)
{
    DOMElement* elem =
        parent->getOwnerDocument()->createElementNS(
                (const XMLCh*)XMLStringConverter("dsmconfig"),
			DOMable::getNamespaceURI());
    parent->appendChild(elem);
    return toDOMElement(elem);
}

DOMElement* ServerSocket::toDOMElement(DOMElement* node)
    throw(DOMException)
{
    return node;
}


