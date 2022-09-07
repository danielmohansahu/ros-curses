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

/* A Panel subclass to display all available services.
 */
class ServiceListPanel : public PanelList<ComputationalGraph::Service>
{
 private:

 public:

  ServiceListPanel() : PanelList<ComputationalGraph::Service>("ROS Services", Action::SELECT_SERVICE) {};

}; // class ServiceListPanel

} // namespace ros_curses::panels