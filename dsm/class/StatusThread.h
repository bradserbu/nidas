/*
 ********************************************************************
    Copyright 2005 UCAR, NCAR, All Rights Reserved

    $LastChangedDate$

    $LastChangedRevision$

    $LastChangedBy$

    $HeadURL$
 ********************************************************************

*/

#ifndef DSM_STATUSTHREAD_H
#define DSM_STATUSTHREAD_H

#include <atdUtil/Thread.h>

#include <iostream> // cerr

namespace dsm {

/**
 * A thread that runs periodically checking and multicasting
 * the status of a DSMEngine.
 */
class StatusThread: public atdUtil::Thread
{
public:
    /**
     * Constructor.
     */
    StatusThread(const std::string& name);

    ~StatusThread();

    int run() throw(atdUtil::Exception);

protected:
};

}

#endif
