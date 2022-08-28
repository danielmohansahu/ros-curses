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
  std::unique_ptr<WINDOW> _window;

  // core curses panel object underlying this class
  std::unique_ptr<PANEL> _panel;

 public:
  // no default constructor
  Panel() = delete;
  Panel(size_t rows, size_t cols, size_t origin_y, size_t origin_x);

  /****************************** Core Panel API *****************************/

  /* Set this panel to be the topmost panel available and highlight appropriately.
   */
  void set_active();

  /* Set this panel to be invisible.
   */
  void hide();

  /* Set this panel to be visible.
   */
  void show();

  /* Write the given data to our panel.
   */
  void write(const std::vector<ros_curses::LineDatum>& lines);

}; // class Panel

/* Top level display class, encapsulating all curses objects and handling.
 */
class Display
{
 private:
  // convenience typedefs
  using Action = ros_curses::Action;

  // all available panels
  std::unordered_map<std::string, Panel> _panels;

  // interface used to update the display on active panels
  std::shared_ptr<ros_curses::Updater> _updater; 

 public:
  Display(const std::shared_ptr<ros_curses::Updater>& updater);
  ~Display();

  /* Process user input, returning the desired action if not applicable to visualization.
   */
  Action process_user_input();

}; // class Display

/*
 */


}; // namespace ros_curses