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

// curses
#include <curses.h>
#include <panel.h>

// ros_curses
#include "types.h"
#include "updater.h"

namespace curses
{

/* Core encapsulation of a Curses Panel object.
 */
class Panel
{
 private:
  // core curses window object underlying this class
  WINDOW* _window;

  // core curses panel object underlying this class
  PANEL* _panel;

  // boolean indicating if we're active or not
  bool _active {false};

  // completely redraw this panel, clearing all information
  void redraw();

 public:
  // constructors
  Panel() : Panel(0, 0, 0, 0) {};
  Panel(size_t rows, size_t cols, size_t origin_y, size_t origin_x);

  /****************************** Core Panel API *****************************/

  /* Set this panel to be the topmost panel available and highlight appropriately.
   */
  void set_active();

  /* Indicate that we're inactive, optionally completely hiding.
   */
  void set_inactive(const bool hide);

  /* Write the given data to our panel.
   */
  void write(const std::vector<ros_curses::LineDatum>& lines);

  /* Resize to the given dimensions and move to the given location.
   */
  void move_and_resize(const size_t rows, const size_t cols, const size_t y, const size_t x);

  /************************* Window Pass-through API *************************/

  /* Get user input to the given panel, if any.
   */
  int get_ch() { return wgetch(&(*_window)); };

  /* Add a string to the given line (mostly for debugging)
   */
  void debug(const std::string& str)
  {
    [[maybe_unused]] int minx, maxy, miny, maxx;
    getbegyx(_window, miny, minx); getmaxyx(_window, maxy, maxx);
    mvwaddstr(_window, maxy - 2, minx + 1, str.c_str());
  }

}; // class Panel

/* Top level display class, encapsulating all curses objects and handling.
 */
class Display
{
 private:
  // convenience typedefs
  using Action = ros_curses::Action;
  using PanelNames = ros_curses::PanelNames;

  // all available panels
  std::unordered_map<PanelNames, Panel> _panels;

  // interface used to update the display on active panels
  std::shared_ptr<ros_curses::Updater> _updater; 

  // currently active window (user selected)
  PanelNames _active {PanelNames::INITIALIZATION};
  PanelNames _last_active {_active};

  // miscellaneous formatting data
  static const inline uint8_t HEADER_ROWS {3};
  static const inline uint8_t HELP_COLS {50};

  // miscellaneous cached information (should be minimal...)
  std::string _header_status {""};

 public:
  Display(const std::shared_ptr<ros_curses::Updater>& updater);
  ~Display();

  /* Switch to the target active display, optionally hiding the previous display.
   */
  void activate(const PanelNames panel, const bool hide = false);

  /* Core polling method; called every loop with updates.
   */
  ros_curses::Action process(const std::unordered_map<ros_curses::PanelNames,std::vector<ros_curses::LineDatum>>& updates);

  /* Update header, including a status message.
   */
  void update_header(const std::string& status = "");

 private:

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

  /* Write help screen information.
   */
  void update_help_panel();

}; // class Display

/*
 */


} // namespace ros_curses