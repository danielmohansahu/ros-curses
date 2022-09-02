/*
 *
 *
 * 
 */

#pragma once

// STL
#include <random>
#include <vector>
#include <string>
#include <optional>

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

 public:
  ROS1Parser();

  /* Return whether or not we have a connection.
   */
  bool connected();

  /* Update our current graph and return rendered CG.
   */
  std::optional<ComputationalGraph> poll();

}; // namespace ROS1Parser

} // namespace ros_curses
