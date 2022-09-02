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

// XMLRPC
#include <xmlrpcpp/XmlRpc.h>

// ros_curses
#include "../computational_graph.h"

namespace ros_curses
{

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

  /* Convert the given XML vector of string/vector<string> to the STL equivalent. 
   */
  std::vector<std::pair<std::string, std::vector<std::string>>> xml_to_stl(XmlRpcValue& xml) const;

}; // namespace ROS1Parser

} // namespace ros_curses
