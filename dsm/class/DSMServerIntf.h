/*
 ********************************************************************
    Copyright 2005 UCAR, NCAR, All Rights Reserved

    $LastChangedDate$

    $LastChangedRevision$

    $LastChangedBy$

    $HeadURL$
 ********************************************************************
*/

#ifndef DSMSERVERINTF_H
#define DSMSERVERINTF_H

#include <iostream>
#include <XmlRpcThread.h>

using namespace XmlRpc;

namespace dsm {

/// gets a list of DSMs and their locations from the configuration
class GetDsmList : public XmlRpcServerMethod
{
public:
  GetDsmList(XmlRpcServer* s) : XmlRpcServerMethod("GetDsmList", s) {}
  void execute(XmlRpcValue& params, XmlRpcValue& result);
  std::string help() { return std::string("help GetDsmList"); }
};

/**
 * A thread that provides XML-based Remote Procedure Calls
 * to web interfaces from the DSMServer.
 */
class DSMServerIntf : public XmlRpcThread
{
public:
  DSMServerIntf() : XmlRpcThread("DSMServerIntf") {}

  int run() throw(atdUtil::Exception);
};

}

#endif
