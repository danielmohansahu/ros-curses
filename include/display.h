/*
 *
 *
 *
 */

#pragma once

// STL
#include <vector>
#include <memory>
#include <unordered_map>
#include <optional>

// curses
#include <curses.h>
#include <panel.h>

// ros_curses
#include "types.h"
#include "computational_graph.h"
#include "panels/panel_base.h"
#include "panels/header_panel.h"

namespace ros_curses
{

/* Top level display class, encapsulating all curses objects and handling.
 */
class Display
{
 private:

  // all available panels
  std::unordered_map<PanelNames, std::unique_ptr<panels::PanelBase>> _panels;
  std::unique_ptr<panels::HeaderPanel> _header_panel;

  // currently active window (user selected)
  PanelNames _active {PanelNames::INITIALIZATION};
  PanelNames _last_active {_active};

  // miscellaneous formatting data
  static const inline uint8_t HEADER_ROWS {4};
  static const inline uint8_t HELP_COLS {50};

 public:
  Display();
  ~Display();

  /* Core polling method; called every loop with updates.
   */
  Action process(const std::optional<ComputationalGraph>& graph);

  /* Set the given panels as visible, hiding everything else, and activate 'first'.
   */
  void show_displays(const PanelNames first, const PanelNames second = PanelNames::NONEXISTENT);

 private:

  /* Switch to the target active display.
   */
  void activate(const PanelNames panel);

  /* Toggle the given panel as visible over everything else or hidden.
   */
  void supersede(const PanelNames panel, const bool supersede);

  /* Process user input, returning the desired action if not applicable to visualization.
   */
  Action process_user_input();

  /* Cycle through active displays.
   */
  void cycle_displays(const bool reverse = false);

  /* Switch between left and right hand active panels.
   */
  void switch_displays();

  /* Handle a window resize event.
   */
  void resize();

}; // class Display

} // namespace ros_curses