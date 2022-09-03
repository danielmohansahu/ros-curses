/*
 *
 *
 * 
 */

#pragma once

// ros_curses
#include "panel_base.h"

namespace ros_curses::panels
{

/* A Panel subclass to display param information.
 */
class ParamInfoPanel : public PanelBase
{
 private:

  // the node we're currently highlighting
  std::optional<std::string> _active_param;

 public:

  // constructor
  ParamInfoPanel() : PanelBase(true) {}

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>& graph) override;

  /* Update currently activated param.
   */
  virtual void select(const std::optional<std::string>& param) override { _active_param = param; };

}; // class ParamInfoPanel

} // namespace ros_curses::panels