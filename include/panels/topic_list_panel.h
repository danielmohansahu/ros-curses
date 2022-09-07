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
class TopicListPanel : public PanelList<ComputationalGraph::Topic>
{
 private:

 public:

  TopicListPanel() : PanelList<ComputationalGraph::Topic>("ROS Topics", Action::SELECT_TOPIC) {};

}; // class TopicListPanel

} // namespace ros_curses::panels