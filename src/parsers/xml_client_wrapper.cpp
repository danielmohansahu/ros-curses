/*
 *
 *
 * 
 */

// STL
#include <algorithm>
#include <assert.h>

// ros_curses
#include "parsers/xml_client_wrapper.h"

namespace ros_curses::ros1
{

XMLClientWrapper::XMLClientWrapper()
{
  // override default host / port information from Master URI, if given
  if (const char* master_uri = getenv("ROS_MASTER_URI"); master_uri)
  {
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
