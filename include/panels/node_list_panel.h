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

/* A Panel to list all available nodes.
 */
class NodeListPanel : public PanelBase
{
 private:

 public:

  NodeListPanel() : PanelBase(true) {}

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>& graph) override;

  /* Update currently activated item.
   */
  virtual void select(const std::optional<std::string>& selection) override { _scroll.select(selection); };

  /* Update the current filter.
   */
  virtual void update_filter(const char& c = '\0') override 
  {
    // initialize filter
    if (!_filter)
    {
      _filter = "";
      _scroll.resize(_rows - BORDER * 4);
    }

    // @TODO handle special characters (backspace, etc.)
    if (c != '\0')
      _filter->push_back(c);
  };

}; // class NodeListPanel

} // namespace ros_curses::panels