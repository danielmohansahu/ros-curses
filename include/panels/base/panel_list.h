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

/* Abstract base class for List Panels.
 */
template <typename ElementType>
class PanelList : public PanelBase
{
 private:
  // title of this list display
  const std::string _title;

  // The action to perform on selection
  const Action _action;

 public:

  PanelList(const std::string& title, const Action& action)
    : PanelBase(/*selectable=*/true), _title(title), _action(action) {}

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>& graph) override
  {
    // start from a blank slate
    redraw();
    mvwaddnstr(_window, 0, 1, _title.c_str(), _cols - 2 * BORDER);

    // if we don't have a graph we can't do anything
    if (!graph)
      return NULL_ACTION;

    // if we don't have topics we also can't do anything
    std::vector<std::string> all_elements;
    if (all_elements = graph->list<ElementType>(); all_elements.size() == 0)
      return NULL_ACTION;

    // optionally filter elements
    std::vector<std::string> elements;
    elements.reserve(all_elements.size());
    if (_filter && _filter->size() != 0)
    {
      for (const auto& element : all_elements)
        if (element.find(*_filter) != std::string::npos)
          elements.emplace_back(element);
    }
    else
      elements.swap(all_elements);

    // add a little blurb if we're filtering
    if (_filter)
      print_line(_rows - BORDER * 2, "Showing matches for: '" + *_filter + "'");

    // if we filtered everything there's not much to do
    if (elements.size() == 0)
      return NULL_ACTION;

    // construct a list of indices for the visible region
    std::vector<size_t> indices(elements.size());
    std::iota(indices.begin(), indices.end(), 0);

    // get the start -> end of the scrollable region, as well as current selection index
    const auto [begin, end, selection_idx, highlight_idx] = _scroll.update(elements, indices, elements.size());

    // iterate through visible section of items and print
    for (size_t i = begin; i != end; ++i)
      print_line(BORDER + i - begin, " - " + elements[i], format(i == highlight_idx, !graph->map<ElementType>().at(elements[i])->active));

    // add a little blurb if we're scrolling
    if (_scroll.scroll_required(elements.size()))
      print_line_center(BORDER + end - begin, (end < elements.size()) ? "-- more --" : "-- end --");

    // redraw border, in case it got borked
    draw_border();

    // write panel name over border
    mvwaddnstr(_window, 0, 1, _title.c_str(), _cols - 2 * BORDER);

    // indicate we want to display information about this topic
    return {_action, elements[selection_idx]};
  };

  /* Update currently activated item.
   */
  virtual void select(const std::optional<std::string>& selection) override
  {
    _scroll.select(selection);
  };

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
  }

}; // class PanelList

} // namespace ros_curses::panels