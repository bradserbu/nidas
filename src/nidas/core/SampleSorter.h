/*
 ********************************************************************
    Copyright 2005 UCAR, NCAR, All Rights Reserved

    $LastChangedDate$

    $LastChangedRevision$

    $LastChangedBy$

    $HeadURL$
 ********************************************************************

*/

#ifndef NIDAS_CORE_SAMPLESORTER_H
#define NIDAS_CORE_SAMPLESORTER_H

#include <nidas/core/SampleSource.h>
#include <nidas/core/SampleClient.h>
#include <nidas/core/SortedSampleSet.h>
#include <nidas/core/DSMTime.h>

#ifdef USE_LOOPER
#include <nidas/core/LooperClient.h>
#endif

#include <nidas/util/Thread.h>
#include <nidas/util/ThreadSupport.h>

namespace nidas { namespace core {

/**
 * A SampleClient that sorts its received samples,
 * using an STL multimap, and then sends the
 * sorted samples onto its SampleClients.
 * One specifies a sorter length in the constructor.
 * Samples whose time-tags are less than the time-tag
 * of the latest sample received, minus the sorter length,
 * are sent on to the SampleClients.
 * This is implemented as a Thread, which must be started,
 * otherwise the sorter will grow and no samples will be
 * sent to clients.
 * This can be a client of multiple DSMSensors, so that the
 * processed samples are sorted in time.
 */
class SampleSorter : public nidas::util::Thread,
	public SampleClient, public SampleSource

#ifdef USE_LOOPER
	, public LooperClient
#endif
{
public:

    /**
     * Constructor.
     * @param sorterLength Length of the SampleSorter, in milliseconds.
     */
    SampleSorter(const std::string& name);

    virtual ~SampleSorter();

    void interrupt();

    bool receive(const Sample *s) throw();

    size_t size() const { return samples.size(); }

    void setLengthMsecs(int val)
    {
        sorterLengthUsec = val * USECS_PER_MSEC;
    }

    int getLengthMsecs() const
    {
        return sorterLengthUsec / USECS_PER_MSEC;
    }


    /**
     * Set the maximum amount of heap memory to use for sorting samples.
     * @param val Maximum size of heap in bytes.
     */
    void setHeapMax(size_t val) { heapMax = val; }

    size_t getHeapMax() const { return heapMax; }

    /**
     * Get the current amount of heap being used for sorting.
     */
    size_t getHeapSize() const { return heapSize; }

    /**
     * @param val If true, and heapSize exceeds heapMax,
     *   then wait for heapSize to be less then heapMax,
     *   which will block any SampleSources that are inserting
     *   samples into this sorter.  If false, then discard any
     *   samples that are received while heapSize exceeds heapMax.
     */
    void setHeapBlock(bool val) { heapBlock = val; }

    bool getHeapBlock() const { return heapBlock; }

    // void setDebug(bool val) { debug = val; }

    /**
     * flush all samples from buffer, distributing them to SampleClients.
     */
    void finish() throw();

    const std::set<const SampleTag*>& getSampleTags() const
    {
        return sampleTags;
    }

    void addSampleTag(const SampleTag* tag)
    	throw(nidas::util::InvalidParameterException)
    {
        sampleTags.insert(tag);
    }

    /**
     * Add a Client for a given SampleTag.
     */
    void addSampleTag(const SampleTag* tag,SampleClient*)
    	throw(nidas::util::InvalidParameterException);

#ifdef USE_LOOPER
    void looperNotify() throw();
#endif

protected:

    /**
     * Thread run function.
     */
    virtual int run() throw(nidas::util::Exception);

    /**
     * Length of SampleSorter, in micro-seconds.
     */
    int sorterLengthUsec;

    SortedSampleSet samples;

    SampleT<char> dummy;

private:

    /**
     * Utility function to decrement the heap size after writing
     * one or more samples. If the heapSize has has shrunk below
     * heapMax then signal any threads waiting on heapCond.
     */
    void inline heapDecrement(size_t bytes);

    nidas::util::Cond sampleSetCond;

    // bool debug;

    /**
     * Limit on the maximum size of memory to use while buffering
     * samples.
     */
    size_t heapMax;

    /**
     * Current heap size, in bytes.
     */
    size_t heapSize;

    /**
     * If heapSize exceeds heapMax, do we wait for heapSize to
     * be less then heapMax, which will block any SampleSources
     * that are inserting samples into this sorter, or if
     * heapBlock is false, then discard any samples that
     * are received while heapSize exceeds heapMax.
     */
    bool heapBlock;

    nidas::util::Cond heapCond;

    size_t discardedSamples;

    int discardWarningCount;

    bool doFlush;

    bool flushed;

    std::set<const SampleTag*> sampleTags;

    std::map<dsm_sample_id_t,SampleClientList> clientsBySampleId;

    nidas::util::Mutex clientMapLock;

    /**
     * No assignment.
     */
    SampleSorter& operator=(const SampleSorter&);

};

}}	// namespace nidas namespace core

#endif
