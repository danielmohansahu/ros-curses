/*
 *
 *
 * 
 */

// STL
#include <algorithm>
#include <assert.h>

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

  // connect our client
  _client = std::make_unique<XmlRpcClient>(_host.c_str(), _port, "/");
}

std::optional<ComputationalGraph> ROS1Parser::poll()
{
  // set current graph to disconnected; we'll update if it's active
  if (_graph)
    _graph->set_connected(false);

  if (auto new_graph = get_system_state(); new_graph)
    if (get_system_params(*new_graph))
      if (get_message_types(*new_graph))
      {
        // success! merge this new data (or replace, for initial poll)
        if (!_graph)
          _graph = new_graph;
        else
          _graph->merge(*new_graph);
        _graph->set_connected(true);
      }

  // always return the latest and greatest
  return _graph;
}

std::optional<ComputationalGraph> ROS1Parser::get_system_state()
{
  // poll for the current system state (all published and subscribed topics / services and their nodes)
  //  @TODO doesn't this ignore nodes who haven't published anything? How does that work? Are they guaranteed
  //  to publish to /rosout?

  // construct XMLRPC request to ROS master
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = "ros_curses";

  if (execute("getSystemState", args, result, payload))
    return ComputationalGraph(xml_to_stl(payload[0]), xml_to_stl(payload[1]), xml_to_stl(payload[2]));
  return std::nullopt;
}

bool ROS1Parser::get_system_params(ComputationalGraph& graph)
{
  // I am a stub
  return true;
}

bool ROS1Parser::get_message_types(ComputationalGraph& graph)
{
  // I am a stub
  return true;
}

bool ROS1Parser::execute(const std::string& method, const XmlRpc::XmlRpcValue& request, XmlRpc::XmlRpcValue& response, XmlRpc::XmlRpcValue& payload)
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

std::vector<std::pair<std::string, std::vector<std::string>>> ROS1Parser::xml_to_stl(XmlRpc::XmlRpcValue& xml) const
{
  // sanity checks
  assert(xml.getType() == XmlRpcValue::TypeArray);

  // initialize result
  std::vector<std::pair<std::string, std::vector<std::string>>> result;

  // perform conversion from XML to STL vectors/strings
  for (size_t i = 0; i != xml.size(); ++i)
  {
    // more sanity checks
    assert(xml[i].size() == 2);
    assert(xml[i][0].getType() == XmlRpcValue::TypeString);
    assert(xml[i][1].getType() == XmlRpcValue::TypeArray);

    // prep subvector
    auto& it = result.emplace_back(xml[i][0], std::vector<std::string>());

    // iterate through subvector
    for (size_t j = 0; j != xml[i][1].size(); ++j)
    {
      assert(xml[i][1][j].getType() == XmlRpcValue::TypeString);
      it.second.push_back(static_cast<std::string>(xml[i][1][j]));
    }
  }

  // return result
  return result;
}

} // namespace ros_curses
