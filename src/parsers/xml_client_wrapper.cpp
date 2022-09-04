/*
 *
 *
 * 
 */

// STL
#include <algorithm>

// ros_curses
#include "parsers/xml_client_wrapper.h"

namespace ros_curses::ros1
{

XMLClientWrapper::XMLClientWrapper(const std::string& uri)
{
  // check if a URI was provided; otherwise we assume this is a connection to the ROS master
  if (uri.size() == 0)
  {
    // override default host / port information from Master URI, if given
    if (const char* master_uri = getenv("ROS_MASTER_URI"); master_uri)
      _uri = std::string(master_uri);
    else
      // otherwise, just use the default ('http://localhost:11311)
      _uri = _default_master_uri;
  }

  // validate URI and split into host / port pair
  if (!extract_uri(_uri, _host, _port))
    throw std::runtime_error("Can't process URI '" + _uri + "'; please check for typos.");

  // connect to the core client
  _client = std::make_unique<XmlRpcClient>(_host.c_str(), _port, "/");
}
bool XMLClientWrapper::execute(const std::string& method, const XmlRpc::XmlRpcValue& request, XmlRpc::XmlRpcValue& response, XmlRpc::XmlRpcValue& payload)
{
  // execute call
  if (!_client->execute(method.c_str(), request, response))
    return false;

  // validate result
  if (   (response.getType() != XmlRpcValue::TypeArray)
      || (response.size() != 2 && response.size() != 3)
      || (response[0].getType() != XmlRpcValue::TypeInt)
      || (response[1].getType() != XmlRpcValue::TypeString) )
    return false;

  // get response component information
  const int status_code = response[0];
  const std::string status_string = response[1];

  // continue validation
  if (status_code != 1)
    return false;

  // format response
  if (response.size() > 2)
  {
    payload = response[2];
  }
  else
  {
    std::string empty_array = "<value><array><data></data></array></value>";
    int offset = 0;
    payload = XmlRpcValue(empty_array, &offset);
  }

  // indicate success, if we got this far
  return true;
}

} // namespace ros_curses::ros1
