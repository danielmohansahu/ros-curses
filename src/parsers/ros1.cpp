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
  args[0] = _name;

  if (execute("getSystemState", args, result, payload))
    return ComputationalGraph(xml_to_stl<std::vector<std::pair<std::string,std::vector<std::string>>>>(payload[0]),
                              xml_to_stl<std::vector<std::pair<std::string,std::vector<std::string>>>>(payload[1]),
                              xml_to_stl<std::vector<std::pair<std::string,std::vector<std::string>>>>(payload[2]));
  return std::nullopt;
}

bool ROS1Parser::get_system_params(ComputationalGraph& graph)
{
  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = _name;
  params[1] = "/";    // root namespace of all parameters

  if (!execute("getParam", params, result, payload))
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

  if (!execute("getTopicTypes", params, result, payload))
    return false;

  // convert result to a vector of string pairs
  const auto topic_types = xml_to_stl<std::vector<std::pair<std::string,std::string>>>(payload);

  // update this information in the graph
  graph.set_topic_types(topic_types);
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

std::unordered_map<std::string, std::string> ROS1Parser::flatten_params(const std::string& ns, XmlRpc::XmlRpcValue xml) const
{
  // initialize result
  std::unordered_map<std::string, std::string> result;

  // handle terminal (primitve) types:
  if (xml.getType() == XmlRpcValue::TypeString)
    result.emplace(ns, static_cast<std::string>(xml));
  else if (xml.getType() == XmlRpcValue::TypeBoolean)
    result.emplace(ns, std::to_string(static_cast<bool>(xml)));
  else if (xml.getType() == XmlRpcValue::TypeInt)
    result.emplace(ns, std::to_string(static_cast<int>(xml)));
  else if (xml.getType() == XmlRpcValue::TypeDouble)
    result.emplace(ns, std::to_string(static_cast<double>(xml)));
  else if (xml.getType() == XmlRpcValue::TypeStruct)
  {
    // recurse down, building our map
    result.reserve(xml.size());
    for (auto it = xml.begin(); it != xml.end(); ++it)
      for (auto kv : flatten_params(ns + "/" + static_cast<std::string>(it->first), it->second))
        result.emplace(kv.first, kv.second);
  }
  else if (xml.getType() == XmlRpcValue::TypeArray)
  {
    // recurse down, building our map
    result.reserve(xml.size());
    for (size_t i = 0; i != xml.size(); ++i)
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

template <typename T>
T ROS1Parser::xml_to_stl(XmlRpc::XmlRpcValue xml) const
{

  // check what type we want to cast this as
  if constexpr (std::is_same_v<T, std::string>)
  {
    // if this can be cast a string directly, do that
    if (xml.getType() == XmlRpcValue::TypeString)
      return static_cast<std::string>(xml);
    else if (xml.getType() == XmlRpcValue::TypeBoolean)
      return std::to_string(static_cast<bool>(xml));
    else if (xml.getType() == XmlRpcValue::TypeInt)
      return std::to_string(static_cast<int>(xml));
    else if (xml.getType() == XmlRpcValue::TypeDouble)
      return std::to_string(static_cast<double>(xml));

    // otherwise, we expect it to be a nested structure, which we'll resolve recursively
    assert(xml.getType() == XmlRpcValue::TypeStruct);
    return static_cast<std::string>(xml.begin()->first) + "/" + xml_to_stl<std::string>(xml.begin()->second);
  }
  else if constexpr (is_pair<T>::value)
  {
    // pairs can only be created from TypeArrays of size 2
    assert(xml.getType() == XmlRpcValue::TypeArray && xml.size() == 2);

    // cast as a pair
    return std::pair(xml_to_stl<typename T::first_type>(xml[0]), xml_to_stl<typename T::second_type>(xml[1]));
  }  
  else if constexpr (is_vector<T>::value)
  {
    // vectors are a little more complicated - we can only conver them from TypeArrays
    assert(xml.getType() == XmlRpcValue::TypeArray);

    // initialize result
    T result;
    result.reserve(xml.size());

    // recursively populate this vector
    for (size_t i = 0; i != xml.size(); ++i)
      result.emplace_back(xml_to_stl<typename T::value_type>(xml[i]));
    return result;
  }
  else if constexpr (is_unordered_map<T>::value)
  {
    // maps can only be created from TypeStructs
    assert(xml.getType() == XmlRpcValue::TypeStruct);

    // initialize result
    T result;
    result.reserve(xml.size());

    // // recursively populate this struct
    for (auto it = xml.begin(); it != xml.end(); ++it)
      result.emplace(xml_to_stl<typename T::key_type>(it->first), xml_to_stl<typename T::mapped_type>(it->second));
    return result;
  }
  else
  {
    static_assert(always_false<T>, "Invalid XML type conversion requested.");
  }
}

} // namespace ros_curses
