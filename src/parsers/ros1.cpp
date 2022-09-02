/*
 *
 *
 * 
 */

// STL
#include <algorithm>

// ros_curses
#include "parsers/ros1.h"

namespace ros_curses
{

ROS1Parser::ROS1Parser()
{
  // override default host / port information from Master URI, if given
  if (const char* master_uri = getenv("ROS_MASTER_URI"); master_uri)
    if (std::string uri = std::string(master_uri); std::regex_match(uri, _re_master))
    {
      // get rid of the 'http://'
      uri = std::string(uri.begin() + 7, uri.end());
      _host = uri.substr(0, uri.find(":"));
      _port = std::stoi(uri.substr(uri.find(":") + 1, uri.size()));
    }
    else
      throw std::runtime_error("Can't process ROS_MASTER_URI '" + uri + "'; please check for typos.");
}

bool ROS1Parser::connected()
{
  // I am a stub.
  return false;
}

std::optional<ComputationalGraph> ROS1Parser::poll()
{
  // do nothing if not connected
  if (!connected())
    return std::nullopt;

  // construct XMLRPC request to ROS master
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = "ros_curses";

  if (!execute("getSystemState", args, result, payload))
  {
    return std::nullopt;
  }

  // I am a stub.
  return std::nullopt;
}

bool ROS1Parser::execute(const std::string& method, const XmlRpc::XmlRpcValue& request, XmlRpc::XmlRpcValue& response, XmlRpc::XmlRpcValue& payload)
{
  // // connect to a client
  // XmlRpc::XmlRpcClient *c = XMLRPCManager::instance()->getXMLRPCClient(_host, _port, "/");

  // // execute call
  // bool result = c->execute(method.c_str(), request, response);

  // // validate result
  // if (!XMLRPCManager::instance()->validateXmlrpcResponse(method, response, payload))
  //   result = false;

  // // always release client
  // XMLRPCManager::instance()->releaseXMLRPCClient(c);

  // // indicate success
  // return result;
  return false;
}

} // namespace ros_curses
