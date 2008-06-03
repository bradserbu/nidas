/*
 ********************************************************************
    Copyright 2005 UCAR, NCAR, All Rights Reserved

    $LastChangedDate$

    $LastChangedRevision$

    $LastChangedBy$

    $HeadURL$

    Simple program to "tee" I/O from a serial port to one
    or more pseudo-terminals.
 ********************************************************************

*/

#include <nidas/util/SerialPort.h>
#include <nidas/util/SerialOptions.h>
#include <nidas/util/UTime.h>

#include <vector>
#include <list>

using namespace std;

namespace n_u = nidas::util;

class TeeTTy {
public:
    TeeTTy();
    int parseRunstring(int argc, char** argv);
    int run();
    static int usage(const char* argv0);
private:
    string progname;
    string ttyname;
    n_u::SerialOptions ttyopts;
    list<string> rwptys;
    list<string> roptys;

    bool readonly;
    bool asDaemon;
};

TeeTTy::TeeTTy():readonly(true),asDaemon(true)
{
}

int TeeTTy::parseRunstring(int argc, char** argv)
{
    progname = argv[0];
    string ttyoptstr;
    int iarg = 1;

    for ( ; iarg < argc; iarg++) {
	string arg = argv[iarg];
	if (arg == "-w") {
	    if (++iarg == argc) usage(argv[0]);
	    rwptys.push_back(argv[iarg]);	// user will read and write to/from this pty
	    readonly = false;
	}
	else if (arg == "-f") asDaemon = false;	// don't put in background
	else if (arg[0] == '-') return usage(argv[0]);
	else {
	    if (ttyname.length() == 0) ttyname = argv[iarg];
	    else if (ttyoptstr.length() == 0) ttyoptstr = argv[iarg];
	    else roptys.push_back(argv[iarg]);	// user will only read from this pty
	}
    }

    if (ttyname.length() == 0) return usage(argv[0]);
    if (ttyoptstr.length() == 0) return usage(argv[0]);
    if (rwptys.size() + roptys.size() == 0) return usage(argv[0]);

    try {
        ttyopts.parse(ttyoptstr);
    }
    catch (const n_u::ParseException &e) {
        cerr << e.what();
	return usage(argv[0]);
    }
    return 0;
}

int TeeTTy::usage(const char* argv0)
{
    cerr << "\
Usage: " << argv0 << "[-f] tty ttyopts [ (-w ptyname) | ptyname ] ... ]\n\
  -f: foreground. Don't run as background daemon\n\
  tty: name of serial port to open\n\
  ttyopts: SerialOptions string, see below\n\
  -w ptyname: name of one or more read-write pseudo-terminals\n\
  ptyname: name of one or more read-only pseudo-terminals\n\n\
  ttyopts:\n  " << n_u::SerialOptions::usage() << endl;
    return 1;
}

int TeeTTy::run()
{
    int nloop = 0;
    unsigned int maxread = 0;
    unsigned int minread = 99999999;
    try {
	if (asDaemon && daemon(0,0) < 0) throw n_u::IOException(progname,"daemon",errno);

	fd_set readfds;
	fd_set writefds;
	FD_ZERO(&readfds);
	FD_ZERO(&writefds);

	n_u::SerialPort tty(ttyname);
	tty.setOptions(ttyopts);

	tty.open(readonly ? O_RDONLY : O_RDWR);
	FD_SET(tty.getFd(),&readfds);
	int maxfd = tty.getFd() + 1;
	int maxwfd = 0;

	vector<int> ptyfds;
	vector<string> ptynames;
	struct timeval writeTimeout;

	list<string>::const_iterator li = rwptys.begin();
	for ( ; li != rwptys.end(); ++li) {
	    const string& name = *li;
	    int fd = n_u::SerialPort::createPtyLink(name);

	    FD_SET(fd,&readfds);
	    FD_SET(fd,&writefds);
	    maxfd = std::max(maxfd,fd + 1);
	    maxwfd = std::max(maxwfd,fd + 1);

	    ptyfds.push_back(fd);
	    ptynames.push_back(name);
	}

	// user will only read from these ptys, so we only write to them.
	li = roptys.begin();
	for ( ; li != roptys.end(); ++li) {
	    const string& name = *li;
	    int fd = n_u::SerialPort::createPtyLink(name);
	    FD_SET(fd,&writefds);
	    maxwfd = std::max(maxwfd,fd + 1);

	    ptyfds.push_back(fd);
	    ptynames.push_back(name);
	}

	char buf[1024];
	for (;;) {

	    int nfd;
	    fd_set rfds = readfds;
	    if ((nfd = ::select(maxfd,&rfds,0,0,0)) < 0)
	    	throw n_u::IOException(tty.getName(),"select",errno);

	    if (FD_ISSET(tty.getFd(),&rfds)) {
		nfd--;
		size_t l = tty.read(buf,sizeof(buf));
		if (l > maxread) maxread = l;
		if (l < minread) minread = l;
		if (l > 0) {
		    int nwfd;
		    fd_set wfds = writefds;
		    writeTimeout.tv_sec = 0;
		    writeTimeout.tv_usec = USECS_PER_SEC / 4;
		    if ((nwfd = ::select(maxwfd,0,&wfds,0,&writeTimeout)) < 0)
			throw n_u::IOException(tty.getName(),"select",errno);
		    for (unsigned int i = 0; nwfd > 0 && i < ptyfds.size(); i++)  {
			if (FD_ISSET(ptyfds[i],&wfds)) {
			    nwfd--;
			    int lw = ::write(ptyfds[i],buf,l);
			    if (lw < 0) throw n_u::IOException(ptynames[i],"write",errno);
			    if (lw != (signed)l) cerr << ptynames[i] <<
				" wrote " << lw << " out of " << l << " bytes" << endl;
			}
		    }
		}
	    }

	    for (unsigned int i = 0; nfd > 0 && i < ptyfds.size(); i++)  {
		int fd = ptyfds[i];
		if (FD_ISSET(fd,&rfds)) {
		    nfd--;
		    int l = ::read(fd,buf,sizeof(buf));
		    if (l < 0) {
		        n_u::IOException e(ptynames[i],"read",errno);
			cerr << e.what() << endl;
			::close(fd);
			FD_CLR(fd,&writefds);
			FD_CLR(fd,&readfds);

			// open a new one
			fd = n_u::SerialPort::createPtyLink(ptynames[i]);
			FD_SET(fd,&readfds);
			FD_SET(fd,&writefds);
			maxfd = std::max(maxfd,fd + 1);

			ptyfds[i] = fd;
		    }
		    if (l > 0) tty.write(buf,l);
		}
	    }
	    if (!(nloop++  % 100))
	    	cerr << "nloop=" << nloop << " maxread=" << maxread << " minread=" << minread << endl;
	}
    }
    catch(n_u::IOException& ioe) {
	cerr << ioe.what() << endl;
	return 1;
    }
    return 0;
}
int main(int argc, char** argv)
{
    TeeTTy tee;
    int res;
    if ((res = tee.parseRunstring(argc,argv)) != 0) return res;
    tee.run();
}
