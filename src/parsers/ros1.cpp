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

namespace ros_curses::ros1
{

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
  args[0] = _name;

  if (_client->execute("getSystemState", args, result, payload))
    return ComputationalGraph(_client->xml_to_stl<std::vector<std::pair<std::string,std::vector<std::string>>>>(payload[0]),
                              _client->xml_to_stl<std::vector<std::pair<std::string,std::vector<std::string>>>>(payload[1]),
                              _client->xml_to_stl<std::vector<std::pair<std::string,std::vector<std::string>>>>(payload[2]));
  return std::nullopt;
}

bool ROS1Parser::get_system_params(ComputationalGraph& graph)
{
  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = _name;
  params[1] = "/";    // root namespace of all parameters

  if (!_client->execute("getParam", params, result, payload))
    return false;

  // convert to map of strings
  const auto param_map = flatten_params("", payload);

  // hand these off to the graph and return success
  graph.merge_params(param_map);
  return true;
}

bool ROS1Parser::get_message_types(ComputationalGraph& graph)
{
  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = _name;

  if (!_client->execute("getTopicTypes", params, result, payload))
    return false;

  // convert result to a vector of string pairs
  const auto topic_types = _client->xml_to_stl<std::vector<std::pair<std::string,std::string>>>(payload);

  // update this information in the graph
  graph.set_topic_types(topic_types);
  return true;
}

std::unordered_map<std::string, std::string> ROS1Parser::flatten_params(const std::string& ns, XmlRpc::XmlRpcValue xml) const
{
  // initialize result
  std::unordered_map<std::string, std::string> result;

  // handle terminal (primitve) types:
  if (xml.getType() == XmlRpc::XmlRpcValue::TypeString)
    result.emplace(ns, static_cast<std::string>(xml));
  else if (xml.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
    result.emplace(ns, std::to_string(static_cast<bool>(xml)));
  else if (xml.getType() == XmlRpc::XmlRpcValue::TypeInt)
    result.emplace(ns, std::to_string(static_cast<int>(xml)));
  else if (xml.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    result.emplace(ns, std::to_string(static_cast<double>(xml)));
  else if (xml.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    // recurse down, building our map
    result.reserve(xml.size());
    for (auto it = xml.begin(); it != xml.end(); ++it)
      for (auto kv : flatten_params(ns + "/" + static_cast<std::string>(it->first), it->second))
        result.emplace(kv.first, kv.second);
  }
  else if (xml.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    // recurse down, building our map
    result.reserve(xml.size());
    for (size_t i = 0; i != static_cast<size_t>(xml.size()); ++i)
      for (auto kv : flatten_params(ns, xml[i]))
        result.emplace(kv.first, kv.second);
  }
  else
  {
    // we only expect terminal types or Structs
    assert(false);
  }
  return result;
}

} // namespace ros_curses::ros1
