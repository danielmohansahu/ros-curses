/*
 *
 *
 *
 */

#pragma once

// STL
#include <optional>

// ros_curses
#include "base/panel_list.h"

namespace ros_curses::panels
{

/* A Panel to list all available nodes.
 */
class NodeListPanel final : public PanelList<ComputationalGraph::Node>
{
 public:

  NodeListPanel() : PanelList<ComputationalGraph::Node>("ROS Nodes", Action::SELECT_NODE) {};

}; // class NodeListPanel

} // namespace ros_curses::panels