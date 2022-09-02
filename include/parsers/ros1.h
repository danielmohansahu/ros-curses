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

  // local latest and greatest CG
  std::optional<ComputationalGraph> _graph;

  // ROS connection information
  std::regex _re_master {"^http://(\\w+):([0-9]+)$"};
  std::string _host {"localhost"};
  size_t _port {11311};

 public:
  ROS1Parser();

  /* Return whether or not we have a connection.
   */
  bool connected();

  /* Update our current graph and return rendered CG.
   */
  std::optional<ComputationalGraph> poll();

 private:

  /* Execute a call to the XMLRPC server (ROS master)
   * 
   * Adapted from:
   *  https://github.com/ros/ros_comm/blob/f5fa3a168760d62e9693f10dcb9adfffc6132d22/clients/roscpp/src/libros/master.cpp
   */
  bool execute(const std::string& method, const XmlRpc::XmlRpcValue& request, XmlRpc::XmlRpcValue& response, XmlRpc::XmlRpcValue& payload);


}; // namespace ROS1Parser

} // namespace ros_curses
