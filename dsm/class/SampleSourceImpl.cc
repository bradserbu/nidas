/*
 ********************************************************************
    Copyright by the National Center for Atmospheric Research

    $LastChangedDate: 2004-10-15 17:53:32 -0600 (Fri, 15 Oct 2004) $

    $LastChangedRevision$

    $LastChangedBy$

    $HeadURL: http://orion/svn/hiaper/ads3/dsm/class/RTL_DSMSensor.h $
 ********************************************************************

*/


#include <SampleSourceImpl.h>

using namespace dsm;

void SampleSourceImpl::addSampleClientImpl(SampleClient* client) {
    clients.add(client);
}

void SampleSourceImpl::removeSampleClientImpl(SampleClient* client) {
    clients.remove(client);
}
  
void SampleSourceImpl::removeAllSampleClientsImpl() {
    clients.removeAll();
}
  
void SampleSourceImpl::distributeImpl(const Sample* sample)
	throw(SampleParseException,atdUtil::IOException) {

    // copy constructor does a lock
    SampleClientList tmp(clients);

    /* There is a multithreading issue at this point.
     * If a SampleClient removes themselves from the list
     * AND immediately delete's themselves, when the SampleSource
     * is executing this method, right here between the above
     * copy and the clients receive(), then things will crash.
     *
     * (I should keep this comment short to avoid this situation :)
     *
     * We don't want to keep the lock however, because
     * perhaps the SampleClient may want to remove themselves
     * within the receive() (if they get an IOException for example).
     * That situation would cause a deadlock.
     * To avoid this issue, SampleClients should either make
     * sure the SampleSource isn't executing at the time of
     * removeSampleClient or that there is a "long" time between
     * removeSampleClient and their destruction.  Hmmm, needs 
     * more thought.
     */

    std::list<SampleClient*>::iterator li;
    for (li = tmp.begin(); li != tmp.end(); ++li)
	(*li)->receive(sample);
    sample->freeReference();
    numSamplesSent++;
}

