/*
 *
 *
 * 
 */

#pragma once

// STL
#include <string>
#include <optional>
#include <regex>
#include <type_traits>

// XMLRPC
#include <xmlrpcpp/XmlRpc.h>

// ros_curses
#include "../computational_graph.h"

namespace ros_curses
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

/* A class to poll ROS for Computational Graph information.
 */
class ROS1Parser
{
 private:
  // convenience typdefs
  using XmlRpcClient = XmlRpc::XmlRpcClient;
  using XmlRpcValue = XmlRpc::XmlRpcValue;

  // local latest and greatest CG
  std::optional<ComputationalGraph> _graph;

  // our identifier
  const static inline std::string _name {"ros_curses"};

  // ROS connection information
  std::regex _re_master {"^http://(\\w+):([0-9]+)$"};
  std::string _host {"localhost"};
  size_t _port {11311};

  // persistent xml client
  std::unique_ptr<XmlRpcClient> _client;

 public:
  ROS1Parser();

  /* Update our current graph and return rendered CG.
   */
  std::optional<ComputationalGraph> poll();

 private:

  /* Query the ros master for current publishers, subscribers, and services.
   */
  std::optional<ComputationalGraph> get_system_state();

  /* Query the ros master for all ROS parameters.
   */
  bool get_system_params(ComputationalGraph& graph);

  /* Query the ros master for the types of active messages / services.
   */
  bool get_message_types(ComputationalGraph& graph);

  /* Execute a call to the XMLRPC server (ROS master)
   * 
   * Adapted from:
   *  https://github.com/ros/ros_comm/blob/f5fa3a168760d62e9693f10dcb9adfffc6132d22/clients/roscpp/src/libros/master.cpp
   */
  bool execute(const std::string& method, const XmlRpcValue& request, XmlRpcValue& response, XmlRpcValue& payload);

  /* Convert the given XML representation to the STL equivalent. 
   */
  template <typename T>
  T xml_to_stl(XmlRpc::XmlRpcValue xml) const;

}; // namespace ROS1Parser

} // namespace ros_curses
