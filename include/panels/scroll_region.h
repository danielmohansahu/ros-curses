/*
 *
 *
 * 
 * 
 */

#pragma once

// STL
#include <string>
#include <vector>
#include <optional>
#include <any>
#include <algorithm>
#include <numeric>
#include <assert.h>

namespace ros_curses
{

/* Class used in calculating the visible part of a scrollable region.
 */
class ScrollRegion
{
 private:

  // whether or not we have selectable items
  const bool _selectable {true};

  // number of lines available for display
  size_t _size {0};

  // last "start" index (top of the page)
  size_t _start_index {0};

  // last unique selection
  std::any _selection;

 public:
  // specify mode during construction
  explicit ScrollRegion(const bool selectable) : _selectable(selectable) {}

  // completely reset all state variables (except size)
  void reset()
  {
    _selection.reset();
    _start_index = 0;
  }

  // update our current size
  void resize(const size_t size) { _size = size; }

  // return true if we need to scroll
  bool scroll_required(const size_t length) const { return length > _size; }

  /* Get the updated bounds of our visible region for 'selectable' mode.
   *
   * Args:
   *    selectables: Vector of unique selectable items
   *    selectable_indices: The indices of our selectable items in the broader vector (with non-selectable entries)
   *    step: The amount and direction to increment our current selection
   *    page: Amount and direction to page
   * 
   * Returns:
   *    Tuple containing the start index, end signal, and selected index in the vector of selectable_indices.
   */
  template <typename T>
  std::tuple<size_t,size_t,size_t> update(const std::vector<T>& selectables, const std::vector<size_t>& selectable_indices, const int step = 0, const int page = 0)
  {
    // sanity check
    assert(_selectable);
    assert(selectables.size() > 0);

    // find the index of our current selection (or set it to the first element)
    if (!_selection.has_value()
        || std::find(selectables.begin(), selectables.end(), std::any_cast<T>(_selection)) == selectables.end())
      _selection = selectables[0];

    // find the index of the current selection
    size_t selection_idx = std::distance(selectables.begin(), std::find(selectables.begin(), selectables.end(), std::any_cast<T>(_selection)));

    // check if the user wants to move the selection index
    selection_idx = std::clamp(static_cast<int>(selection_idx) + step, 0, static_cast<int>(selectables.size()) - 1);
    selection_idx = std::clamp(static_cast<int>(selection_idx) + static_cast<int>(_size) * page, 0, static_cast<int>(selectables.size()) - 1);
    _selection = selectables[selection_idx];

    // find the selection index in the full text region
    size_t required_idx = std::distance(selectable_indices.begin(), std::find(selectable_indices.begin(), selectable_indices.end(), selection_idx));

    // handle trivial case : we have enough space
    if (!scroll_required(selectable_indices.size()))
    {
      // reset starting index
      _start_index = 0;
      return {_start_index, selectable_indices.size(), required_idx};
    }

    // shift the starting index location until the required index is in view
    if (required_idx < _start_index)
      // shift back to include 'req_idx'
      _start_index -= (_start_index - required_idx);
    else if (required_idx > (_start_index + _size - 1))
      // shift forward to include 'req_idx'
      _start_index += (required_idx - (_start_index + _size - 1));
    // else no shift needed!

    // return indices
    return {_start_index, _start_index + _size, required_idx};
  }

  /* Get the updated bounds of our visible region for non-selectable mode.
   *
   * Args:
   *    length: The amount of lines we wish to display.
   *    step: The amount and direction to increment our current selection
   *    page: Amount and direction to page
   * 
   * Returns:
   *    Start index and end signal of viewable region.
   */
  std::pair<size_t,size_t> update(const size_t length, const int step = 0, const int page = 0)
  {
    // sanity check
    assert(!_selectable);

    // handle trivial case : we have enough space
    if (!scroll_required(length))
    {
      // reset starting index
      _start_index = 0;
      return {_start_index, length};
    }

    // we have to scroll; check if the user also wants to move the indices
    _start_index = std::clamp(static_cast<int>(_start_index) + step, 0, static_cast<int>(length - _size));
    _start_index = std::clamp(static_cast<int>(_start_index) + static_cast<int>(_size) * page, 0, static_cast<int>(length - _size));

    // return the visible section from _start_index to _start_index + _size
    return {_start_index, _start_index + _size};
  }

}; // class ScrollRegion

} // namespace ros_curses