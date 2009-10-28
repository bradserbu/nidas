/*
 ********************************************************************
    Copyright 2005 UCAR, NCAR, All Rights Reserved

    $LastChangedDate: 2009-06-25 11:42:06 -0600 (Thu, 25 Jun 2009) $

    $LastChangedRevision: 4698 $

    $LastChangedBy: maclean $

    $HeadURL: http://svn.eol.ucar.edu/svn/nidas/trunk/src/nidas/core/SampleInput.cc $
 ********************************************************************
*/

#include <nidas/core/SamplePipeline.h>
#include <nidas/core/SampleBuffer.h>
#include <nidas/core/SampleSorter.h>
#include <nidas/core/DSMSensor.h>
#include <nidas/core/Project.h>

#include <nidas/util/Logger.h>

using namespace nidas::core;
using namespace std;

namespace n_u = nidas::util;

SamplePipeline::SamplePipeline() :
	_name("SamplePipeline"),
	_rawSorter(0),
	_procSorter(0),
        _realTime(false),
        _rawSorterLength(0.0),
        _procSorterLength(0.0),
#ifdef NIDAS_EMBEDDED
        _rawHeapMax (5000000),
        _procHeapMax(5000000),
#else
        _rawHeapMax (100000000),
        _procHeapMax(100000000),
#endif
        _heapBlock(false),
        _keepStats(false)
{
}

SamplePipeline::~SamplePipeline()
{
    _rawMutex.lock();
    if (_rawSorter) {
        if (_rawSorter->isRunning()) {
            _rawSorter->finish();
            _rawSorter->interrupt();
            try {
                _rawSorter->join();
            }
            catch(const n_u::Exception&) {}
        }
        delete _rawSorter;
    }
    _rawMutex.unlock();

    _procMutex.lock();
    if (_procSorter) {
        if (_procSorter->isRunning()) {
            _procSorter->finish();
            _procSorter->interrupt();
            try {
                _procSorter->join();
            }
            catch(const n_u::Exception&) {}
        }
        delete _procSorter;
    }
    _procMutex.unlock();
}

void SamplePipeline::rawinit()
{
    n_u::Autolock autolock(_rawMutex);
    if (!_rawSorter) {
        if (getRawSorterLength() > 0) {
            _rawSorter = new SampleSorter(_name + "RawSorter",true);
            _rawSorter->setLengthSecs(getRawSorterLength());
        }
        else {
            _rawSorter = new SampleBuffer(_name + "RawBuffer",true);
        }
        _rawSorter->setHeapMax(getRawHeapMax());
        _rawSorter->setHeapBlock(getHeapBlock());
        _rawSorter->setRealTime(getRealTime());
        _rawSorter->setKeepStats(_keepStats);
        ILOG(("RawSorter: length=%f secs, heapMax=%d MB, stats=%d",
            _rawSorter->getLengthSecs(),_rawSorter->getHeapMax()/1000000,
            _rawSorter->getKeepStats()));
	_rawSorter->setRealTimeFIFOPriority(40);
        _rawSorter->start();
    }
}

void SamplePipeline::procinit()
{
    n_u::Autolock autolock(_procMutex);
    if (!_procSorter) {
        if (getProcSorterLength() > 0) {
            _procSorter = new SampleSorter(_name + "ProcSorter",false);
            _procSorter->setLengthSecs(getProcSorterLength());
        }
        else {
            _procSorter = new SampleBuffer(_name + "ProcBuffer",false);
        }
        _procSorter->setHeapMax(getProcHeapMax());
        _procSorter->setHeapBlock(getHeapBlock());
        _procSorter->setRealTime(getRealTime());
        _procSorter->setKeepStats(_keepStats);
        ILOG(("ProcSorter: length=%f secs, heapMax=%d MB, stats=%d",
            _procSorter->getLengthSecs(),_procSorter->getHeapMax()/1000000,
            _procSorter->getKeepStats()));
	_procSorter->setRealTimeFIFOPriority(30);
        _procSorter->start();
    }
}

void SamplePipeline::connect(SampleSource* src) throw()
{
    rawinit();
    procinit();

    SampleSource* rawsrc = src->getRawSampleSource();

    if (rawsrc) {
        SampleTagIterator si = rawsrc->getSampleTagIterator();
        for ( ; si.hasNext(); ) {
            const SampleTag* stag = si.next();
            dsm_sample_id_t rawid = stag->getId() - stag->getSampleId();
#ifdef DEBUG
            cerr << "connect rawid=" << GET_DSM_ID(rawid) << ',' <<
                GET_SPS_ID(rawid) << endl;
            cerr << "connect id=" << GET_DSM_ID(stag->getId()) << ',' <<
                GET_SPS_ID(stag->getId()) << endl;
#endif
            DSMSensor* sensor = Project::getInstance()->findSensor(rawid);
            if (sensor) {
#ifdef DEBUG
                cerr << "sensor=" << sensor->getName() << endl;
#endif
                SampleTagIterator si2 = sensor->getSampleTagIterator();
                for ( ; si2.hasNext(); ) {
                    const SampleTag* stag2 = si2.next();
                    _procSorter->addSampleTag(stag2);
                }
            }
            _rawSorter->addSampleTag(stag);
        }
        rawsrc->addSampleClient(_rawSorter);
    }
    else {
        src = src->getProcessedSampleSource();
        if (!src) return;
        SampleTagIterator si = src->getSampleTagIterator();
        for ( ; si.hasNext(); ) {
            const SampleTag* stag = si.next();
            _procSorter->addSampleTag(stag);
        }
        src->addSampleClient(_procSorter);
    }
}

void SamplePipeline::disconnect(SampleSource* src) throw()
{

    SampleSource* rawsrc = src->getRawSampleSource();

    if (rawsrc) {
        {
            n_u::Autolock autolock(_rawMutex);
            if (!_rawSorter) return;
        }
        rawsrc->removeSampleClient(_rawSorter);
        {
            n_u::Autolock autolock(_procMutex);
            if (!_procSorter) return;
        }
        SampleTagIterator si = rawsrc->getSampleTagIterator();
        for ( ; si.hasNext(); ) {
            const SampleTag* stag = si.next();
            _rawSorter->removeSampleTag(stag);
            dsm_sample_id_t rawid = stag->getId() - stag->getSampleId();
            DSMSensor* sensor = Project::getInstance()->findSensor(rawid);
            if (sensor) {
                SampleTagIterator si2 = sensor->getSampleTagIterator();
                for ( ; si2.hasNext(); ) {
                    const SampleTag* stag2 = si2.next();
                    _procSorter->removeSampleTag(stag2);
                }
            }
        }
    }
    else {
        {
            n_u::Autolock autolock(_procMutex);
            if (!_procSorter) return;
        }
        src = src->getProcessedSampleSource();
        if (!src) return;
        SampleTagIterator si = src->getSampleTagIterator();
        for ( ; si.hasNext(); ) {
            const SampleTag* stag = si.next();
            _procSorter->removeSampleTag(stag);
        }

        src->removeSampleClient(_procSorter);
    }
}

void SamplePipeline::addSampleClient(SampleClient* client) throw()
{
    rawinit();
    procinit();

    list<const SampleTag*> rtags = _rawSorter->getSampleTags();
    list<const SampleTag*>::const_iterator si = rtags.begin();

    // add a client for all possible processed SampleTags.
    for ( ; si != rtags.end(); ++si) {
        const SampleTag* stag = *si;
        dsm_sample_id_t rawid = stag->getId();
        DSMSensor* sensor = Project::getInstance()->findSensor(rawid);
        if (sensor) {
#ifdef DEBUG
            cerr << "addSampleClient sensor=" << sensor->getName() << endl;
#endif
            sensor->addSampleClient(_procSorter);
            stag = sensor->getRawSampleTag();
            _rawSorter->addSampleClientForTag(sensor,stag);
        }
    }
    _procSorter->addSampleClient(client);
}

void SamplePipeline::removeSampleClient(SampleClient* client) throw()
{
    {
        n_u::Autolock autolock(_procMutex);
        if (!_procSorter) return;
    }
    _procSorter->removeSampleClient(client);

    //	If there are no clients of procSorter then clean up.
    if (_procSorter->getClientCount() == 0) {
        {
            n_u::Autolock autolock(_procMutex);
            if (!_rawSorter) return;
        }
        list<const SampleTag*> rtags = _rawSorter->getSampleTags();
        list<const SampleTag*>::const_iterator si = rtags.begin();
        for ( ; si != rtags.end(); ++si) {
            const SampleTag* stag = *si;
            dsm_sample_id_t rawid = stag->getId();
            DSMSensor* sensor = Project::getInstance()->findSensor(rawid);
            if (sensor) {
                sensor->removeSampleClient(_procSorter);
                stag = sensor->getRawSampleTag();
                _rawMutex.lock();
                if (_rawSorter) _rawSorter->removeSampleClientForTag(sensor,stag);
                _rawMutex.unlock();
            }
        }
    }
}

void SamplePipeline::addSampleClientForTag(SampleClient* client,
	const SampleTag* stag) throw()
{
    rawinit();
    procinit();

    dsm_sample_id_t rawid = stag->getId() - stag->getSampleId();
    DSMSensor* sensor = Project::getInstance()->findSensor(rawid);

    if (stag->getSampleId() != 0 && sensor) {
        _procSorter->addSampleClientForTag(client,stag);

        sensor->addSampleClient(_procSorter);

        stag = sensor->getRawSampleTag();
        _rawSorter->addSampleClientForTag(sensor,stag);
    }
}

void SamplePipeline::removeSampleClientForTag(SampleClient* client,
	const SampleTag* stag) throw()
{
    {
        n_u::Autolock autolock(_procMutex);
        if (!_procSorter) return;
    }

    unsigned int rawid = stag->getId() - stag->getSampleId();
    DSMSensor* sensor = Project::getInstance()->findSensor(rawid);

    if (stag->getSampleId() != 0 && sensor) {
        _procSorter->removeSampleClientForTag(client,stag);
    }
    else {
        _rawMutex.lock();
        if (_rawSorter) _rawSorter->removeSampleClientForTag(client,stag);
        _rawMutex.unlock();
    }

    //	If there are no clients of procSorter then clean up.
    if (_procSorter->getClientCount() == 0) {
        sensor->removeSampleClient(_procSorter);
        stag = sensor->getRawSampleTag();
        _rawMutex.lock();
        if (_rawSorter) _rawSorter->removeSampleClientForTag(sensor,stag);
        _rawMutex.unlock();
    }
}
