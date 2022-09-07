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

/* A Panel subclass used as a placeholder.
 */
class ParamListPanel final : public PanelList<ComputationalGraph::Param>
{
 private:

 public:

  ParamListPanel() : PanelList<ComputationalGraph::Param>("ROS Params", ROSType::PARAMETER, Action::SELECT_PARAM) {};

}; // class ParamListPanel

} // namespace ros_curses::panels