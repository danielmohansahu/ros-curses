/*
 *
 *
 * 
 */

#pragma once

// STL
#include <optional>

// ros_curses
#include "panel_base.h"

namespace ros_curses::panels
{

/* A Panel subclass used as a placeholder.
 */
class TopicListPanel : public PanelBase
{
 private:

 public:

  // constructor
  TopicListPanel() : PanelBase(true) {}

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>& graph) override;

  /* Update currently activated item.
   */
  virtual void select(const std::optional<std::string>& selection) override { _scroll.select(selection); };

}; // class TopicListPanel

} // namespace ros_curses::panels