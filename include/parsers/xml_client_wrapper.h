/*
 *
 *
 * 
 */

#pragma once

// STL
#include <string>
#include <optional>
#include <unordered_map>
#include <regex>
#include <type_traits>
#include <assert.h>

// XMLRPC
#include <xmlrpcpp/XmlRpc.h>

namespace ros_curses::ros1
{

// convenience type for constexpr assertions
template <typename...> inline constexpr bool always_false = false; 

// default false evaluations for constexpr type handling
template<typename T> struct is_vector : public std::false_type {};
template<typename T> struct is_pair : public std::false_type {};
template<typename T> struct is_unordered_map : public std::false_type {};

// specialization to check if something is a std::vector
template<typename T, typename A>
struct is_vector<std::vector<T, A>> : public std::true_type {};

// specialization to check if something is a std::pair
template<typename T1, typename T2>
struct is_pair<std::pair<T1, T2>> : public std::true_type {};

// specialization to check if something is a std::unordered_map
template<typename K, typename T, typename H, typename KE, typename A>
struct is_unordered_map<std::unordered_map<K, T, H, KE, A>> : public std::true_type {};

/* Wrapper around general processing of the XMLRpc Client class.
 */
class XMLClientWrapper
{
 private:
  // convenience typdefs
  using XmlRpcClient = XmlRpc::XmlRpcClient;
  using XmlRpcValue = XmlRpc::XmlRpcValue;

  // ROS connection information
  std::regex _re_uri {"^http://(\\w+):([0-9]+)$"};

  // common static information
  const static inline std::string _default_master_uri {"http://localhost:11311"};

  // our connection information
  std::string _uri;
  std::string _host;
  size_t _port;

  // persistent xml client
  std::unique_ptr<XmlRpcClient> _client;

  // convenience method for URI construction
  inline std::string construct_uri(const std::string& host, const size_t port) const { return "http://" + host + ":" + std::to_string(port); };

  // convenience method for URI extraction
  bool extract_uri(const std::string& uri, std::string& host, size_t& port) const
  {
    if (!std::regex_match(uri, _re_uri))
      return false;
    
    // get rid of the 'http://'
    std::string tmp = std::string(uri.begin() + 7, uri.end());
    host = tmp.substr(0, tmp.find(":"));
    port = std::stoi(tmp.substr(tmp.find(":") + 1, tmp.size()));
    return true;
  }

 public:
  /* Constructor; if the user doesn't provide a URI we'll connect to the ROS Master
   */
  explicit XMLClientWrapper(const std::string& uri = "");
  XMLClientWrapper(const std::string& host, const size_t port) : XMLClientWrapper(construct_uri(host, port)) {};

  // simple accessors
  std::string uri() const { return _uri; };
  std::string host() const { return _host; };
  size_t port() const { return _port; };

  /* Execute a call to the XMLRPC server (ROS master)
   * 
   * Adapted from:
   *  https://github.com/ros/ros_comm/blob/f5fa3a168760d62e9693f10dcb9adfffc6132d22/clients/roscpp/src/libros/master.cpp
   */
  bool execute(const std::string& method, const XmlRpcValue& request, XmlRpcValue& response, XmlRpcValue& payload);

  /* Convert the given XML representation to the STL equivalent. 
   */
  template <typename T>
  T xml_to_stl(XmlRpc::XmlRpcValue xml) const
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
      for (size_t i = 0; i != static_cast<size_t>(xml.size()); ++i)
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

}; // namespace ROS1Parser

} // namespace ros_curses::ros1
