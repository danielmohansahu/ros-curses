/*
 *
 *
 * 
 */

#pragma once

// STL
#include <string.h>
#include <string>
#include <vector>
#include <optional>
#include <algorithm>

// curses
#include "curses.h"
#include "panel.h"

// ros_curses
#include "../computational_graph.h"

namespace ros_curses::panels
{

/* Helper class to calculate the visible part of a scrolling region.
 */
class ScrollRegion
{
 private:
  // constraints
  size_t _size {0};   // number of rows available

 public:
  /* Update available region size.
   */
  void update_size(const size_t size) { _size = size; }

  /* Returns whether or not scrolling is required.
   */
  bool scroll_required(const size_t length) const { return length > _size; }

  /* Find the index of the given object in the given list, and return the shifted index.
   */
  template <typename T>
  std::optional<size_t> shift(const std::optional<T>& item, const std::vector<T>& items, const int shift)
  {
    // if the incoming vector is empty we can't do anything
    if (items.size() == 0)
      return std::nullopt;

    // handle inputs that we can't find
    if (!item || std::find(items.begin(), items.end(), *item) == items.end())
      return 0;

    // find item's current location
    size_t idx = std::distance(items.begin(), std::find(items.begin(), items.end(), *item));

    // check if we need to shift
    if (shift != 0 && items.size() > 1)
    {
      // wrap negative shifting
      int shift_pos = shift;
      while (shift_pos < 0)
        shift_pos += items.size() - 1;

      // get element _shift elements away from current selection    
      idx = (idx + shift_pos) % (items.size() - 1);
    }
    // return resulting index
    return idx;
  }

  /* Calculate the visible indices in [0,length-1] subject to the start index constraint.
   *
   * Args:
   *    length  : the number of rows we want to display
   *    idx     : the starting index we want to display
   */
  std::pair<size_t,size_t> range_from_start(const size_t length, const size_t idx) const
  {
    // if we have enough space to show everything, return full range indices
    if (length <= _size)
      return {0, length - 1};
    
    // if [idx, idx + _size] is valid, return that
    if (idx + _size < length)
      return {idx, idx + _size - 1};

    // just show the end
    return {length - _size, length - 1};
  }

  /* Calculate the visible indices in [0,length-1] subject to keeping the given element visible.
   *
   * Args:
   *    length    : the number of rows we want to display
   *    start_idx : the starting index we last displayed
   *    req_idx   : the required index that must be displayed
   */
  std::pair<size_t,size_t> range_from_selection(const size_t length, const size_t start_idx, const size_t req_idx) const
  {
    // get previously displayed range as a starting point
    auto [begin, end] = range_from_start(length, start_idx);

    // modify such that 'req_idx' is included
    if (req_idx < begin)
      // shift back to include 'req_idx'
      return {begin - (begin - req_idx), end - (begin - req_idx)};
    else if (req_idx > end - 1)
      // shift forward to include 'req_idx'
      return {begin + (req_idx - end + 1), end + (req_idx - end + 1)};
    else
      // no shift needed!
      return {begin, end};
  }

}; // class ScrollRegion

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

  // hardcoded border size
  const uint8_t BORDER {1};

  // current scroll amount
  int _scroll_value {0};

  // completely redraw this panel, clearing all information
  void redraw();

  // redraw the border
  void draw_border();

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
  virtual void set_active(const bool active);

  /* Mark this panel as either visible or hidden.
   */
  virtual void set_visible(const bool visible);

  /* Resize to the given dimensions and move to the given location.
   */
  virtual void move_and_resize(const size_t rows, const size_t cols, const size_t y, const size_t x);

  /*************************** Useful Utilities API **************************/

  /* Get user input to the given panel, if any.
   */
  int get_ch() { return wgetch(&(*_window)); };

  /* Add a string to the bottom of our window (mostly for debugging)
   */
  void debug(const std::string& str) { mvwaddstr(_window, _rows - 2, _cols + 1, str.c_str()); }

  /* Write a single line to our window.
   */
  void print_line(const size_t row, const std::string& str, const int format = A_NORMAL);

  /* Write a single center-aligned line to our window.
   */
  void print_line_center(const size_t row, const std::string& str, const int format = A_NORMAL);

}; // class PanelBase

} // namespace ros_curses::panels