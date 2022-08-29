/*
 *
 *
 * 
 */

#pragma once

// STL
#include <string>
#include <vector>

// curses
#include "curses.h"
#include "panel.h"

// ros_curses
#include "../computational_graph.h"

namespace ros_curses::panels
{

/* Abstract base class for a Panel.
 *
 * This class implements some of the core curses-specific
 * panel / window handling to let child classes focus on 
 * customization.
 */
class PanelBase
{
 protected:
  // core curses window object underlying this class
  WINDOW* _window;

  // core curses panel object underlying this class
  PANEL* _panel;

  // boolean indicating if we're active or not
  bool _active {false};

  // size metadata
  size_t _rows {0};
  size_t _cols {0};

  // current scroll amount
  int _scroll_value {0};

  // completely redraw this panel, clearing all information
  void redraw();

 public:
  // constructors
  PanelBase() : PanelBase(0, 0, 0, 0) {};
  PanelBase(size_t rows, size_t cols, size_t origin_y, size_t origin_x);

  /******************************* Subclass API ******************************/

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  virtual void render(const std::optional<ComputationalGraph>& graph) = 0;

  /* Handle 'up' keystroke.
   */
  virtual void handle_key_up() = 0;

  /* Handle 'down' keystroke.
   */
  virtual void handle_key_down() = 0;

  /* Handle 'enter' keystroke.
   */
  virtual void handle_enter() = 0;

  /****************************** Core Panel API *****************************/

  /* Mark this panel as either active (current cursor location) or inactive.
   */
  void set_active(const bool active);

  /* Mark this panel as either visible or hidden.
   */
  void set_visible(const bool visible);

  /* Resize to the given dimensions and move to the given location.
   */
  void move_and_resize(const size_t rows, const size_t cols, const size_t y, const size_t x);

  /************************* Window Pass-through API *************************/

  /* Get user input to the given panel, if any.
   */
  int get_ch() { return wgetch(&(*_window)); };

  /* Add a string to the bottom of our window (mostly for debugging)
   */
  void debug(const std::string& str) { mvwaddstr(_window, _rows - 2, _cols + 1, str.c_str()); }

}; // class PanelBase

} // namespace ros_curses::panels