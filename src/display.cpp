/*
 *
 *
 */

// ros_curses
#include "display.h"
#include "panels/help_panel.h"
#include "panels/initialization_panel.h"
#include "panels/node_info_panel.h"
#include "panels/node_list_panel.h"
#include "panels/service_info_panel.h"
#include "panels/service_list_panel.h"
#include "panels/param_info_panel.h"
#include "panels/param_list_panel.h"
#include "panels/topic_info_panel.h"
#include "panels/topic_list_panel.h"

namespace ros_curses
{

Display::Display()
{
  // curses initialization
  initscr();              // initialize screens
  start_color();          // allow color processing
  cbreak();               // ???
  noecho();               // don't echo user input to screen
  keypad(stdscr, true);   // properly interpret keypad / arrow keys
  nodelay(stdscr, true);  // don't block on calls for user input
  curs_set(0);            // don't show the cursor
  leaveok(stdscr, true);  // don't move the cursor unless explicitly told

  // instantiate panels via std::unordered_map's autoconstruction
  _panels.emplace(PanelNames::INITIALIZATION, new panels::InitializationPanel());
  _panels.emplace(PanelNames::HELP, new panels::HelpPanel());
  _panels.emplace(PanelNames::NODEINFO, new panels::NodeInfoPanel());
  _panels.emplace(PanelNames::NODELIST, new panels::NodeListPanel());
  _panels.emplace(PanelNames::TOPICINFO, new panels::TopicInfoPanel());
  _panels.emplace(PanelNames::TOPICLIST, new panels::TopicListPanel());
  _panels.emplace(PanelNames::SERVICEINFO, new panels::ServiceInfoPanel());
  _panels.emplace(PanelNames::SERVICELIST, new panels::ServiceListPanel());
  _panels.emplace(PanelNames::PARAMINFO, new panels::ParamInfoPanel());
  _panels.emplace(PanelNames::PARAMLIST, new panels::ParamListPanel());

  // maintain header panel separately
  _header_panel = std::make_unique<panels::HeaderPanel>();

  // resize to fit current display
  resize();

  // start with only the INITIALIZATION screen active
  show_displays(PanelNames::INITIALIZATION);
}

Display::~Display()
{
  // curses shutdown
  endwin();
}

void Display::activate(const PanelNames panel)
{
  // save last state to support toggling
  _last_active = _active;
  _active = panel;

  // set states of appropriate panels
  _panels.at(_last_active)->set_active(false);
  _panels.at(_active)->set_active(true);

  // indicate in header
  _header_panel->set_active(_active);
}

void Display::supersede(const PanelNames panel, const bool supersede)
{
  _panels[panel]->set_visible(supersede);
  activate(supersede ? panel : _last_active);
}

ros_curses::Action Display::process(const std::optional<ros_curses::ComputationalGraph>& graph)
{
  // core method to be called every loop

  // check if we've been initialized (latched boolean)
  if (graph && !_initialized)
  {
    // first time initialization
    _initialized = true;
    show_displays(PanelNames::NODELIST, PanelNames::NODEINFO);
  }

  // check if we're uninitialized
  if (!_initialized)
  {
    _panels.at(PanelNames::INITIALIZATION)->render(graph);
    _header_panel->render(graph);
  }
  else
  {
    // we expect to always receive valid data after initialization
    assert(graph);

    // process incoming data
    for (auto& kv : _panels)
      if (kv.second->visible())
        process_action(kv.second->render(graph));
    _header_panel->set_status(graph->connected() ? "connected" : "disconnected");
    _header_panel->render(graph);
  }

  // process user input
  const Action action = process_user_input();

  // perform refresh of all windows / panels
  update_panels();
  doupdate();

  // return action
  return action;
}

void Display::process_action(const ActionPacket& packet)
{
  // perform the given action requested by a panel
  if (const auto& [action, data] = packet; action == Action::NONE)
    return;
  else if(action == Action::EXIT)
    // @TODO should this be allowed?
    return;
  else if (action == Action::SELECT_NODE)
    _panels[PanelNames::NODEINFO]->select(data);
  else if (action == Action::SELECT_TOPIC)
    _panels[PanelNames::TOPICINFO]->select(data);
  else if (action == Action::SELECT_SERVICE)
    _panels[PanelNames::SERVICEINFO]->select(data);
  else if (action == Action::SELECT_PARAM)
    _panels[PanelNames::PARAMINFO]->select(data);
  else if (action == Action::DISPLAY_NODE)
  {
    show_displays(PanelNames::NODEINFO, PanelNames::NODELIST);
    _panels[PanelNames::NODELIST]->select(data);
    _panels[PanelNames::NODEINFO]->select(data);
  }
  else if (action == Action::DISPLAY_TOPIC)
  {
    show_displays(PanelNames::TOPICINFO, PanelNames::TOPICLIST);
    _panels[PanelNames::TOPICLIST]->select(data);
    _panels[PanelNames::TOPICINFO]->select(data);
  }
  else if (action == Action::DISPLAY_SERVICE)
  {
    show_displays(PanelNames::SERVICEINFO, PanelNames::SERVICELIST);
    _panels[PanelNames::SERVICELIST]->select(data);
    _panels[PanelNames::SERVICEINFO]->select(data);
  }
  else if (action == Action::DISPLAY_PARAM)
  {
    show_displays(PanelNames::PARAMINFO, PanelNames::PARAMLIST);
    _panels[PanelNames::PARAMLIST]->select(data);
    _panels[PanelNames::PARAMINFO]->select(data);
  }
  else
    // shouldn't get here
    assert(false);
}

ros_curses::Action Display::process_user_input()
{
  // initialize action to be NONE
  Action action = Action::NONE;

  // handle user input in active display
  const int ch = _panels.at(_active)->get_ch();

  // until we've been initialized, all the user can do is quit
  if (!_initialized)
    return (ch == int('q')) ? Action::EXIT : Action::NONE;

  // otherwise handle navigation commands (valid for all modes)
  switch (ch)
  {
    case ERR:           // no user input; do nothing
      return action;
    case KEY_RESIZE:    // terminal resize event
      resize();
      return action;
    case KEY_UP:        // move selection up
      _panels.at(_active)->handle_key_up();
      return action;
    case KEY_DOWN:      // move selection down
      _panels.at(_active)->handle_key_down();
      return action;
    case KEY_PPAGE:     // page selection up
      _panels.at(_active)->handle_page_up();
      return action;
    case KEY_NPAGE:     // page selection up
      _panels.at(_active)->handle_page_down();
      return action;
    case KEY_ENTER:
    case int('\n'):
    case int('\r'):
      process_action(_panels.at(_active)->handle_enter());
      return action;
    case KEY_LEFT:      // move selection left
    case KEY_RIGHT:     // move selection right
      switch_displays();
      return action;
    case int('\t'):     // tab between panel display options
      cycle_displays();
      return action;
    default:
      // continue allowing evaluation
      break;
  }

  // if we're in "filter" mode the user can pass through most alpha-numeric
  //  characters which are normally reserved. Note that this will be cleared
  //  by certain navigation actions above
  if (_panels.at(_active)->filtering())
    switch (ch)
    {
      case 27:          // escape - exit filter
        _panels.at(_active)->clear_filter();
        return action;
      default:
        // pass through
        _panels.at(_active)->update_filter(ch);
        return action;
    }

  // if we're not filtering, allow other alphanumeric commands
  switch (ch)
  {
    case int('q'):      // user requested we exit the current screen
      if (_active != PanelNames::HELP)
        return Action::EXIT;
    case int('h'):      // toggle help screen
    case int('?'):      // toggle help screen
      (_active != PanelNames::HELP) ? supersede(PanelNames::HELP, true) : supersede(PanelNames::HELP, false);
      break;
    case int('/'):      // start filtering on current panel, if supported
      _panels.at(_active)->update_filter();
    default:
      break;
  }

  // indicate that no user action is required.
  return action;
}

void Display::show_displays(const PanelNames first, const PanelNames second)
{
  // hide all panels that aren't the given two
  for (auto& kv : _panels)
    if (kv.first == first || kv.first == second)
      kv.second->set_visible(true);
    else
      kv.second->set_visible(false);
  
  // activate the first panel
  activate(first);
}

void Display::cycle_displays(const bool reverse)
{
  // cycle through displays
  switch (_active)
  {
    case PanelNames::NODELIST:
    case PanelNames::NODEINFO:
      (reverse) ? show_displays(PanelNames::PARAMLIST, PanelNames::PARAMINFO) : show_displays(PanelNames::TOPICLIST, PanelNames::TOPICINFO);
      break;
    case PanelNames::TOPICLIST:
    case PanelNames::TOPICINFO:
      (reverse) ? show_displays(PanelNames::NODELIST, PanelNames::NODEINFO) : show_displays(PanelNames::SERVICELIST, PanelNames::SERVICEINFO);
      break;
    case PanelNames::SERVICELIST:
    case PanelNames::SERVICEINFO:
      (reverse) ? show_displays(PanelNames::TOPICLIST, PanelNames::TOPICINFO) : show_displays(PanelNames::PARAMLIST, PanelNames::PARAMINFO);
      break;
    case PanelNames::PARAMLIST:
    case PanelNames::PARAMINFO:
      (reverse) ? show_displays(PanelNames::SERVICELIST, PanelNames::SERVICEINFO) : show_displays(PanelNames::NODELIST, PanelNames::NODEINFO);
      break;
    default:
      // do nothing; these panels can't be cycled!
      break;
  }
}

void Display::switch_displays()
{
  // update to activate the left or right display
  switch(_active)
  {
    case PanelNames::NODELIST: activate(PanelNames::NODEINFO); break;
    case PanelNames::NODEINFO: activate(PanelNames::NODELIST); break;
    case PanelNames::TOPICLIST: activate(PanelNames::TOPICINFO); break;
    case PanelNames::TOPICINFO: activate(PanelNames::TOPICLIST); break;
    case PanelNames::SERVICELIST: activate(PanelNames::SERVICEINFO); break;
    case PanelNames::SERVICEINFO: activate(PanelNames::SERVICELIST); break;
    case PanelNames::PARAMLIST: activate(PanelNames::PARAMINFO); break;
    case PanelNames::PARAMINFO: activate(PanelNames::PARAMLIST); break;
    default: break;
  }
}

void Display::resize()
{
  // get terminal size
  int rows, cols;
  getmaxyx(stdscr, rows, cols);

  // HEADER panel gets the top few rows
  _header_panel->move_and_resize(HEADER_ROWS, cols, 0, 0);

  // INITIALIZATION screen gets the full size except for a few header rows
  _panels.at(PanelNames::INITIALIZATION)->move_and_resize(rows - HEADER_ROWS, cols, HEADER_ROWS, 0);

  // HELP panel gets a fixed width in the middle of the screen
  _panels.at(PanelNames::HELP)->move_and_resize(4 * rows / 5, HELP_COLS, rows / 10, (cols - HELP_COLS) / 2);

  // LIST displays get the left half of the screen
  _panels.at(PanelNames::NODELIST)->move_and_resize(rows - HEADER_ROWS, cols / 2, HEADER_ROWS, 0);
  _panels.at(PanelNames::TOPICLIST)->move_and_resize(rows - HEADER_ROWS, cols / 2, HEADER_ROWS, 0);
  _panels.at(PanelNames::SERVICELIST)->move_and_resize(rows - HEADER_ROWS, cols / 2, HEADER_ROWS, 0);
  _panels.at(PanelNames::PARAMLIST)->move_and_resize(rows - HEADER_ROWS, cols / 2, HEADER_ROWS, 0);

  // INFO displays get the right half of the screen
  _panels.at(PanelNames::NODEINFO)->move_and_resize(rows - HEADER_ROWS, (cols + 1) / 2, HEADER_ROWS, cols / 2);
  _panels.at(PanelNames::TOPICINFO)->move_and_resize(rows - HEADER_ROWS, (cols + 1) / 2, HEADER_ROWS, cols / 2);
  _panels.at(PanelNames::SERVICEINFO)->move_and_resize(rows - HEADER_ROWS, (cols + 1) / 2, HEADER_ROWS, cols / 2);
  _panels.at(PanelNames::PARAMINFO)->move_and_resize(rows - HEADER_ROWS, (cols + 1) / 2, HEADER_ROWS, cols / 2);
}

} // namespace ros_curses