/* -*- mode: C++; indent-tabs-mode: nil; c-basic-offset: 4; -*- */
/* vim: set shiftwidth=4 softtabstop=4 expandtab: */
/*
 ********************************************************************
 ** NIDAS: NCAR In-situ Data Acquistion Software
 **
 ** 2005, Copyright University Corporation for Atmospheric Research
 **
 ** This program is free software; you can redistribute it and/or modify
 ** it under the terms of the GNU General Public License as published by
 ** the Free Software Foundation; either version 2 of the License, or
 ** (at your option) any later version.
 **
 ** This program is distributed in the hope that it will be useful,
 ** but WITHOUT ANY WARRANTY; without even the implied warranty of
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 ** GNU General Public License for more details.
 **
 ** The LICENSE.txt file accompanying this software contains
 ** a copy of the GNU General Public License. If it is not found,
 ** write to the Free Software Foundation, Inc.,
 ** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 **
 ********************************************************************
*/

// #define _XOPEN_SOURCE	/* glibc2 needs this */

#include <ctime>

#include <nidas/core/FileSet.h>
#include <nidas/core/Socket.h>
#include <nidas/dynld/RawSampleInputStream.h>
#include <nidas/core/Project.h>
#include <nidas/core/XMLParser.h>
#include <nidas/core/SamplePipeline.h>
#include <nidas/core/DSMConfig.h>
#include <nidas/core/DSMSensor.h>
#include <nidas/core/Variable.h>
#include <nidas/core/NidasApp.h>
#include <nidas/util/EOFException.h>
#include <nidas/util/Process.h>
#include <nidas/util/Logger.h>
#include <nidas/util/auto_ptr.h>

#include <set>
#include <map>
#include <iostream>
#include <iomanip>
#include <sys/stat.h>

#include <unistd.h>
#include <getopt.h>

using namespace nidas::core;
using namespace nidas::dynld;
using namespace std;

namespace n_u = nidas::util;

class SampleCounter
{
public:
    /**
     * A default constructor is required to use objects as a map element.
     **/
    SampleCounter(dsm_sample_id_t sid = 0, const std::string& sname = "") :
        name(sname),
        id(sid),
        t1s(0),
        t2s(0),
        nsamps(0),
        minlens(0),
        maxlens(0),
        minDeltaTs(0),
        maxDeltaTs(0)
    {
    }

    void
    reset()
    {
        t1s = 0;
        t2s = 0;
        nsamps = 0;
        minlens = 0;
        maxlens = 0;
        minDeltaTs = 0;
        maxDeltaTs = 0;
    }

    bool
    receive(const Sample* samp) throw();

    string name;
    dsm_sample_id_t id;

    dsm_time_t t1s;
    dsm_time_t t2s;
    size_t nsamps;
    size_t minlens;
    size_t maxlens;
    int minDeltaTs;
    int maxDeltaTs;
};


bool
SampleCounter::
receive(const Sample* samp) throw()
{
    dsm_sample_id_t sampid = samp->getId();
    DLOG(("counting sample %d for id ", nsamps)
         << GET_DSM_ID(sampid) << "," << GET_SPS_ID(sampid));
    dsm_time_t sampt = samp->getTimeTag();
    if (nsamps == 0)
    {
        t1s = sampt;
        minDeltaTs = INT_MAX;
        maxDeltaTs = INT_MIN;
    }
    else
    {
        int deltaT = (sampt - t2s + USECS_PER_MSEC/2) / USECS_PER_MSEC;
	minDeltaTs = std::min(minDeltaTs, deltaT);
	maxDeltaTs = std::max(maxDeltaTs, deltaT);
    }
    t2s = sampt;

    size_t slen = samp->getDataByteLength();
    if (nsamps == 0)
    {
        minlens = slen;
        maxlens = slen;
    }
    else
    {
        minlens = std::min(minlens, slen);
        maxlens = std::max(maxlens, slen);
    }
    ++nsamps;
    return true;
}


class CounterClient: public SampleClient 
{
public:

    CounterClient(const list<DSMSensor*>& sensors, bool processed=false);

    virtual ~CounterClient() {}

    void flush() throw() {}

    bool receive(const Sample* samp) throw();

    void printResults(std::ostream& outs);

    void resetResults();

private:

    typedef map<dsm_sample_id_t, SampleCounter> sample_map_t;

    sample_map_t _samples;
};

void
CounterClient::
resetResults()
{
    sample_map_t::iterator si;
    for (si = _samples.begin(); si != _samples.end(); ++si)
    {
        si->second.reset();
    }
}



CounterClient::CounterClient(const list<DSMSensor*>& sensors, bool processed):
    _samples()
{
    list<DSMSensor*>::const_iterator si;
    for (si = sensors.begin(); si != sensors.end(); ++si)
    {
        // Create a SampleCounter for samples from the given sensors.  Raw
        // samples are named by the sensor device, processed samples by the
        // first variable in the first sample tag.
        DSMSensor* sensor = *si;
        string sname = sensor->getDSMConfig()->getName() + ":" +
            sensor->getDeviceName();
        SampleCounter stats(sensor->getId(), sname);
        _samples[stats.id] = stats;

        if (! processed)
        {
            continue;
        }

	// for samples show the first variable name, followed by ",..."
	// if more than one.
	SampleTagIterator ti = sensor->getSampleTagIterator();
	for ( ; ti.hasNext(); ) {
	    const SampleTag* stag = ti.next();
	    if (stag->getVariables().size() > 0)
            {
		string varname = stag->getVariables().front()->getName();
		if (stag->getVariables().size() > 1)
                {
                    varname += ",...";
                }
                SampleCounter pstats(stag->getId(), varname);
                _samples[pstats.id] = pstats;
	    }
	}
    }
}

bool CounterClient::receive(const Sample* samp) throw()
{
    dsm_sample_id_t sampid = samp->getId();

    sample_map_t::iterator it = _samples.find(sampid);
    if (it == _samples.end())
    {
        // When there is no header from which to gather samples ahead of
        // time, just add a SampleCounter instance for any new raw sample
        // that arrives.
        DLOG(("creating counter for sample id ")
             << GET_DSM_ID(sampid) << "," << GET_SPS_ID(sampid));
        SampleCounter ss(sampid);
        _samples[sampid] = ss;
    }
    return _samples[sampid].receive(samp);
}

void CounterClient::printResults(std::ostream& outs)
{
    size_t maxnamelen = 6;
    int lenpow[2] = {5,5};
    int dtlog10[2] = {7,7};

    sample_map_t::iterator si;
    for (si = _samples.begin(); si != _samples.end(); ++si)
    {
        SampleCounter &ss = si->second;
        if (ss.nsamps == 0)
            continue;

	const string& sname = ss.name;
	if (sname.length() > maxnamelen)
            maxnamelen = sname.length();
	size_t m = ss.minlens;
	if (m > 0) {
	    int p = (int)ceil(log10((double)m));
	    lenpow[0] = std::max(lenpow[0],p+1);
	}
	m = ss.maxlens;
	if (m > 0) {
	    int p = (int)ceil(log10((double)m));
	    lenpow[1] = std::max(lenpow[1],p+1);
	}
	int dt = abs(ss.minDeltaTs);
	if (dt > 0 && dt < INT_MAX) {
	    int p = (int)ceil(log10((double)dt+1));
	    dtlog10[0] = std::max(dtlog10[0],p + 2);
	}
	dt = ss.maxDeltaTs;
	if (dt > 0) {
	    int p = (int)ceil(log10((double)dt+1));
	    dtlog10[1] = std::max(dtlog10[1],p + 2);
	}
    }
        
    struct tm tm;
    char tstr[64];
    outs << left << setw(maxnamelen) << (maxnamelen > 0 ? "sensor" : "") <<
    	right <<
    	"  dsm sampid    nsamps |------- start -------|  |------ end -----|    rate" <<
		setw(dtlog10[0] + dtlog10[1]) << " minMaxDT(sec)" <<
		setw(lenpow[0] + lenpow[1]) << " minMaxLen" <<
		endl;

    for (si = _samples.begin(); si != _samples.end(); ++si)
    {
        SampleCounter& ss = si->second;
        if (ss.nsamps == 0)
            continue;

	time_t ut = ss.t1s / USECS_PER_SEC;
	gmtime_r(&ut,&tm);
	strftime(tstr,sizeof(tstr),"%Y %m %d %H:%M:%S",&tm);
	int msec = (int)(ss.t1s % USECS_PER_SEC) / USECS_PER_MSEC;
	sprintf(tstr + strlen(tstr),".%03d",msec);
	string t1str(tstr);
	ut = ss.t2s / USECS_PER_SEC;
	gmtime_r(&ut,&tm);
	strftime(tstr,sizeof(tstr),"%m %d %H:%M:%S",&tm);
	msec = (int)(ss.t2s % USECS_PER_SEC) / USECS_PER_MSEC;
	sprintf(tstr + strlen(tstr),".%03d",msec);
	string t2str(tstr);

        outs << left << setw(maxnamelen) << ss.name
             << right << ' ' << setw(4) << GET_DSM_ID(ss.id) << ' ';

        NidasApp* app = NidasApp::getApplicationInstance();
        app->formatSampleId(outs, ss.id);

        outs << setw(9) << ss.nsamps << ' '
             << t1str << "  " << t2str << ' '
             << fixed << setw(7) << setprecision(2)
             << double(ss.nsamps-1) / (double(ss.t2s - ss.t1s) / USECS_PER_SEC)
             << setw(dtlog10[0]) << setprecision(3)
             << (ss.minDeltaTs < INT_MAX ?
                 (float)ss.minDeltaTs / MSECS_PER_SEC : 0)
             << setw(dtlog10[1]) << setprecision(3)
             << (float)ss.maxDeltaTs / MSECS_PER_SEC
             << setw(lenpow[0]) << ss.minlens
             << setw(lenpow[1]) << ss.maxlens
             << endl;
    }
}

class DataStats
{
public:
    DataStats();

    ~DataStats() {}

    int run() throw();

    int parseRunstring(int argc, char** argv);

    static int main(int argc, char** argv);

    int usage(const char* argv0);

    static void handleSignal(int signum);

private:
    static const int DEFAULT_PORT = 30000;

    static bool _alarm;
    int _count;
    int _period;

    NidasApp app;
    NidasAppArg Period;
    NidasAppArg Count;
};


bool DataStats::_alarm(false);


void
DataStats::handleSignal(int signum)
{
    // The NidasApp handler sets interrupted before calling this handler,
    // so clear that if this is just the interval alarm.
    if (signum == SIGALRM)
    {
        NidasApp::setInterrupted(false);
        _alarm = true;
    }
}


DataStats::DataStats():
    _count(1), _period(0),
    app("data_stats"),
    Period("--period", "<seconds>",
           "Collect statistics for the given number of seconds and then "
           "print the report.", "0"),
    Count("-n,--count", "<count>",
          "When --period specified, generate <count> reports.", "1")
{
    app.setApplicationInstance();
    app.setupSignals();
    app.enableArguments(app.XmlHeaderFile | app.LogConfig |
                        app.SampleRanges | app.FormatHexId |
                        app.FormatSampleId | app.ProcessData |
                        app.Version | app.InputFiles |
                        app.Help | Period | Count);
    app.InputFiles.allowFiles = true;
    app.InputFiles.allowSockets = true;
    app.InputFiles.setDefaultInput("sock:localhost", DEFAULT_PORT);
}


int DataStats::parseRunstring(int argc, char** argv)
{
    // Setup a default log scheme which will get replaced if any logging is
    // configured on the command line.
    n_u::Logger* logger = n_u::Logger::getInstance();
    n_u::LogConfig lc("notice");
    logger->setScheme(logger->getScheme("default").addConfig(lc));

    try {
        ArgVector args(argv+1, argv+argc);
        app.parseArguments(args);
        if (app.helpRequested())
        {
            return usage(argv[0]);
        }
        _period = Period.asInt();
        _count = Count.asInt();

        app.parseInputs(args);
    }
    catch (NidasAppException& ex)
    {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

int DataStats::usage(const char* argv0)
{
    cerr <<
        "Usage: " << argv0 << " [options] [inputURL] ...\n";
    cerr <<
        "Standard options:\n"
         << app.usage() <<
        "Examples:\n" <<
        argv0 << " xxx.dat yyy.dat\n" <<
        argv0 << " file:/tmp/xxx.dat file:/tmp/yyy.dat\n" <<
        argv0 << " -p -x ads3.xml sock:hyper:30000\n" << endl;
    return 1;
}

int DataStats::main(int argc, char** argv)
{
    DataStats stats;
    int result;
    if ((result = stats.parseRunstring(argc, argv)))
    {
        return result;
    }
    NidasApp::setupSignals();

    return stats.run();
}

class AutoProject
{
public:
    AutoProject() { Project::getInstance(); }
    ~AutoProject() { Project::destroyInstance(); }
};

int DataStats::run() throw()
{

    int result = 0;

    try {
        AutoProject aproject;

	IOChannel* iochan = 0;

	if (app.dataFileNames().size() > 0)
        {
            nidas::core::FileSet* fset =
                nidas::core::FileSet::getFileSet(app.dataFileNames());
            iochan = fset->connect();
	}
	else
        {
	    n_u::Socket* sock = new n_u::Socket(*app.socketAddress());
	    iochan = new nidas::core::Socket(sock);
	}

        // Start an alarm here, since the header is not sent until there's
        // a sample to send, so if there are no samples we could block
        // right here reading the header and never get to the readSamples()
        // loop.  However, as soon as the header is read, reset the alarm
        // so it can start again inside the loop which actually reads
        // samples.
        if (_period > 0)
        {
            app.addSignal(SIGALRM, &DataStats::handleSignal);
            alarm(_period);
        }

	SampleInputStream sis(iochan, app.processData());
        sis.setMaxSampleLength(32768);
	// sis.init();
	sis.readInputHeader();
        alarm(0);
        if (_alarm)
        {
            ostringstream outs;
            outs << "Header not received within "
                 << _period << " seconds.";
            throw n_u::Exception(outs.str());
        }
        
	const SampleInputHeader& header = sis.getInputHeader();

	list<DSMSensor*> allsensors;

        string xmlFileName = app.xmlHeaderFile();
	if (xmlFileName.length() == 0)
	    xmlFileName = header.getConfigName();
	xmlFileName = n_u::Process::expandEnvVars(xmlFileName);

	struct stat statbuf;
	if (::stat(xmlFileName.c_str(), &statbuf) == 0 || app.processData())
        {
            n_u::auto_ptr<xercesc::DOMDocument>
                doc(parseXMLConfigFile(xmlFileName));

	    Project::getInstance()->fromDOMElement(doc->getDocumentElement());

            DSMConfigIterator di = Project::getInstance()->getDSMConfigIterator();
	    for ( ; di.hasNext(); )
            {
		const DSMConfig* dsm = di.next();
		const list<DSMSensor*>& sensors = dsm->getSensors();
		allsensors.insert(allsensors.end(),sensors.begin(),sensors.end());
	    }
	}
        XMLImplementation::terminate();

	SamplePipeline pipeline;                                  
        CounterClient counter(allsensors, app.processData());

	if (app.processData()) {
            pipeline.setRealTime(false);                              
            pipeline.setRawSorterLength(0);                           
            pipeline.setProcSorterLength(0);                          

	    list<DSMSensor*>::const_iterator si;
	    for (si = allsensors.begin(); si != allsensors.end(); ++si) {
		DSMSensor* sensor = *si;
		sensor->init();
                //  1. inform the SampleInputStream of what SampleTags to expect
                sis.addSampleTag(sensor->getRawSampleTag());
	    }
            // 2. connect the pipeline to the SampleInputStream.
            pipeline.connect(&sis);

            // 3. connect the client to the pipeline
            pipeline.getProcessedSampleSource()->addSampleClient(&counter);
        }
        else sis.addSampleClient(&counter);

        try {
            int nreports = 0;
            while (!app.interrupted() && ++nreports <= _count)
            {
                if (_period > 0)
                {
                    cout << "....... Collecting samples for "
                         << _period << " seconds "
                         << "......." << endl;
                    alarm(_period);
                }
                while (!_alarm && !app.interrupted())
                {
                    try {
                        sis.readSamples();
                    }
                    catch (n_u::IOException& e)
                    {
                        cerr << e.what()
                             << " (errno=" << e.getErrno() << ")" << endl;
                        if (e.getErrno() != ERESTART && e.getErrno() != EINTR)
                            throw;
                    }
                }
                counter.printResults(cout);
                counter.resetResults();
                _alarm = false;
            }
        }
        catch (n_u::EOFException& e)
        {
            cerr << e.what() << endl;
            counter.printResults(cout);
        }
        catch (n_u::IOException& e)
        {
            if (app.processData())
            {
                pipeline.getProcessedSampleSource()->removeSampleClient(&counter);
                pipeline.disconnect(&sis);
                pipeline.interrupt();
                pipeline.join();
            }
            else
            {
                sis.removeSampleClient(&counter);
            }
            sis.close();
            counter.printResults(cout);
            throw(e);
        }
	if (app.processData())
        {
            pipeline.disconnect(&sis);
            pipeline.flush();
            pipeline.getProcessedSampleSource()->removeSampleClient(&counter);
        }
        else
        {
            sis.removeSampleClient(&counter);
        }
        sis.close();
        pipeline.interrupt();
        pipeline.join();
    }
    catch (n_u::Exception& e) {
        cerr << e.what() << endl;
        XMLImplementation::terminate(); // ok to terminate() twice
	result = 1;
    }
    return result;
}

int main(int argc, char** argv)
{
    return DataStats::main(argc, argv);
}
