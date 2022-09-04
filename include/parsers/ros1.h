/*
 *
 *
 * 
 */

#pragma once

// STL
#include <string>
#include <optional>
#include <memory.h>

// ros_curses
#include "xml_client_wrapper.h"
#include "../computational_graph.h"

// XMLRPC
#include <xmlrpcpp/XmlRpc.h>

namespace ros_curses::ros1
{

/* A class to poll ROS for Computational Graph information.
 */
class ROS1Parser
{
 private:

  // local latest and greatest CG
  std::optional<ComputationalGraph> _graph;

  // our identifier
  const static inline std::string _name {"ros_curses"};

  // persistent xml client to ROS master
  std::unique_ptr<XMLClientWrapper> _master;

 public:
  ROS1Parser() : _master(new XMLClientWrapper()) {};

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

  /* Flatten the hierarchical set of params into a list.
   */
  std::unordered_map<std::string, std::string> flatten_params(const std::string& ns, XmlRpc::XmlRpcValue xml) const;

}; // namespace ROS1Parser

} // namespace ros_curses::ros1
