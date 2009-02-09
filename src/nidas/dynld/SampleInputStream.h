/*
 ********************************************************************
    Copyright 2005 UCAR, NCAR, All Rights Reserved

    $LastChangedDate$

    $LastChangedRevision$

    $LastChangedBy$

    $HeadURL$
 ********************************************************************
*/

#ifndef NIDAS_DYNLD_SAMPLEINPUTSTREAM_H
#define NIDAS_DYNLD_SAMPLEINPUTSTREAM_H

#include <nidas/core/SampleInput.h>
#include <nidas/util/UTime.h>

namespace nidas { namespace dynld {

using namespace nidas::core;	// put this within namespace block

/**
 * An implementation of a SampleInputReader.
 *
 * The readSamples method converts raw bytes from the iochannel
 * into Samples.
 *
 * If a SampleClient has requested processed Samples via
 * addProcessedSampleClient, then SampleInputStream will pass
 * Samples to the respective DSMSensor for processing,
 * and then the DSMSensor passes the processed Samples to
 * the SampleClient:
 *
 * iochannel -> readSamples method -> DSMSensor -> SampleClient
 * 
 * If a SampleClient has requested non-processed Samples,
 * via the simple addSampleClient method, then SampleInput
 * stream passes the Samples straight to the SampleClient:
 *
 * iochannel -> readSamples method -> SampleClient
 *
 */
class SampleInputStream: public nidas::core::SampleInputReader
{

public:

    /**
     * Constructor.
     * @param iochannel The IOChannel that we use for data input.
     *   SampleInputStream will own the pointer to the IOChannel,
     *   and will delete it in ~SampleInputStream(). If 
     *   it is a null pointer, then it must be set within
     *   the fromDOMElement method.
     */
    SampleInputStream(IOChannel* iochannel = 0);

    /**
     * Copy constructor, with a new, connected IOChannel.
     */
    SampleInputStream(const SampleInputStream& x,IOChannel* iochannel);

    /**
     * Create a clone, with a new, connected IOChannel.
     */
    virtual SampleInputStream* clone(IOChannel* iochannel);

    virtual ~SampleInputStream();

    std::string getName() const;

    const std::list<const SampleTag*>& getSampleTags() const
    {
        return _sampleTags;
    }

    void addSampleTag(const SampleTag* stag);

    void readHeader() throw(nidas::util::IOException);

    const SampleInputHeader& getHeader() const { return inputHeader; }

    void addProcessedSampleClient(SampleClient*,DSMSensor*);

    void removeProcessedSampleClient(SampleClient*,DSMSensor* = 0);

    void requestConnection(DSMService*) throw(nidas::util::IOException);

    /**
     * Implementation of ConnectionRequester::connected.
     */
    void connected(IOChannel* iochan) throw();

    nidas::util::Inet4Address getRemoteInet4Address() const;

    void init() throw();

    /**
     * Read a buffer of data, serialize the data into samples,
     * and distribute() samples to the receive() method of my
     * SampleClients and DSMSensors.
     * This will perform only one physical read of the underlying
     * IOChannel and so is appropriate to use when a select()
     * has determined that there is data available on our file
     * descriptor.
     */
    void readSamples() throw(nidas::util::IOException);

    void search(const nidas::util::UTime& tt) throw(nidas::util::IOException);

    /**
     * Read the next sample from the InputStream. The caller must
     * call freeReference on the sample when they're done with it.
     * This method may perform zero or more reads of the IOChannel.
     */
    Sample* readSample() throw(nidas::util::IOException);

    void distribute(const Sample* s) throw();

    // size_t getUnrecognizedSamples() const { return unrecognizedSamples; }

    void close() throw(nidas::util::IOException);

    void newFile() throw(nidas::util::IOException);

    void setFilterBadSamples(bool val)
    {
        _filterBadSamples = val;
    }

    void setMaxDsmId(int val)
    {
        _maxDsmId = val;
        setFilterBadSamples(val < 1024);
    }

    void setMaxSampleLength(unsigned int val)
    {
        _maxSampleLength = val;
        setFilterBadSamples(val < UINT_MAX);
    }

    void setMinSampleTime(const nidas::util::UTime& val)
    {
        _minSampleTime = val.toUsecs();
        setFilterBadSamples(val.toUsecs() > LONG_LONG_MIN);
    }

    void setMaxSampleTime(const nidas::util::UTime& val)
    {
        _maxSampleTime = val.toUsecs();
        setFilterBadSamples(val.toUsecs() < LONG_LONG_MAX);
    }

    void fromDOMElement(const xercesc::DOMElement* node)
	throw(nidas::util::InvalidParameterException);

protected:

    /**
     * Service that has requested my input.
     */
    DSMService* _service;

    IOChannel* _iochan;

    IOStream* _iostream;

    std::map<unsigned int, DSMSensor*> _sensorMap;

    std::map<SampleClient*, std::list<DSMSensor*> > _sensorsByClient;

    nidas::util::Mutex _sensorMapMutex;

    std::list<const SampleTag*> _sampleTags;

private:

    SampleHeader _sheader;

    size_t _headerToRead;

    char* _hptr;

    /**
     * Will be non-null if we have previously read part of a sample
     * from the stream.
     */
    Sample* _samp;

    /**
     * How many bytes left to read from the stream into the data
     * portion of samp.
     */
    size_t _leftToRead;

    /**
     * Pointer into the data portion of samp where we will read next.
     */
    char* _dptr;

    size_t _badInputSamples;

    // size_t unrecognizedSamples;

    /**
     * Copy constructor.
     */
    SampleInputStream(const SampleInputStream&);

    SampleInputHeader inputHeader;

    bool _filterBadSamples;

    unsigned int _maxDsmId;

    size_t _maxSampleLength;

    dsm_time_t _minSampleTime;

    dsm_time_t _maxSampleTime;

};

class SortedSampleInputStream: public SampleInputStream
{
public:
    SortedSampleInputStream(IOChannel* iochannel = 0);
    /**
     * Copy constructor, with a new, connected IOChannel.
     */
    SortedSampleInputStream(const SortedSampleInputStream& x,IOChannel* iochannel);

    virtual ~SortedSampleInputStream();

    /**
     * Create a clone, with a new, connected IOChannel.
     */
    SortedSampleInputStream* clone(IOChannel* iochannel);

    void addSampleClient(SampleClient* client) throw();

    void removeSampleClient(SampleClient* client) throw();

    void addProcessedSampleClient(SampleClient* client, DSMSensor* sensor);

    void removeProcessedSampleClient(SampleClient* client, DSMSensor* sensor = 0);

    void close() throw(nidas::util::IOException);

    void flush() throw();

    /**
     * Set the maximum amount of heap memory to use for sorting samples.
     * @param val Maximum size of heap in bytes.
     * @see SampleSorter::setHeapMax().
     */
    void setHeapMax(size_t val) { heapMax = val; }

    size_t getHeapMax() const { return heapMax; }

    /**
     * @param val If true, and heapSize exceeds heapMax,
     *   then wait for heapSize to be less then heapMax,
     *   which will block any SampleSources that are inserting
     *   samples into this sorter.  If false, then discard any
     *   samples that are received while heapSize exceeds heapMax.
     * @see SampleSorter::setHeapBlock().
     */
    void setHeapBlock(bool val) { heapBlock = val; }

    bool getHeapBlock() const { return heapBlock; }

    /**
     * Set length of SampleSorter, in milliseconds.
     */
    void setSorterLengthMsecs(int val)
    {
        sorterLengthMsecs = val;
    }

    int getSorterLengthMsecs() const
    {
        return sorterLengthMsecs;
    }

    void fromDOMElement(const xercesc::DOMElement* node)
	throw(nidas::util::InvalidParameterException);


private:

    SampleSorter *sorter1;

    SampleSorter *sorter2;

    size_t heapMax;

    bool heapBlock;

    /**
     * No copying.
     */
    SortedSampleInputStream(const SortedSampleInputStream&);

    /**
     * No assignment.
     */
    SortedSampleInputStream& operator=(const SortedSampleInputStream&);

    /**
     * Length of SampleSorter, in milli-seconds.
     */
    int sorterLengthMsecs;

};

}}	// namespace nidas namespace dynld

#endif
