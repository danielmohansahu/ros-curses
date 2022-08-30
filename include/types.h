/* Common datatypes used throughout the project.
 *
 *
 * 
 */

#pragma once

#include <string>
#include <optional>

namespace ros_curses
{

/* Enumeration of all available panels.
 */
enum PanelNames
{
  NONEXISTENT,
  INITIALIZATION,
  HEADER,
  HELP,
  NODELIST,
  NODEINFO,
  TOPICLIST,
  TOPICINFO,
  SERVICELIST,
  SERVICEINFO,
  PARAMLIST,
  PARAMINFO
}; // enum PanelNames

/* Enumeration of all possible actions requested by users and panels.
 */
enum Action
{
  NONE,               // do nothing
  EXIT,               // exit program (or current panel)
  SELECT_NODE,        // display summary information for the given NODE
  SELECT_TOPIC,       // display summary information for the given TOPIC
  SELECT_SERVICE,     // display summary information for the given SERVICE
  SELECT_PARAM,       // display summary information for the given PARAM
  DISPLAY_NODE,       // switch to NODE panels, highlighting the given NODE
  DISPLAY_TOPIC,      // switch to TOPIC panels, highlighting the given TOPIC
  DISPLAY_SERVICE,    // switch to SERVICE panels, highlighting the given SERVICE
  DISPLAY_PARAM       // switch to PARAM panels, highlighting the given PARAM
}; // enum Action

/* Structure dictating a desired action and required information.
 */
using ActionPacket = std::pair<Action, std::optional<std::string>>;

// pre-constructed objects for common actions
const static inline ActionPacket NULL_ACTION {Action::NONE, std::nullopt};
const static inline ActionPacket EXIT_ACTION {Action::EXIT, std::nullopt};

/* String representation of a single line, with formatting information.
 */
struct LineDatum
{
  const std::string text;
  const int format;

  // only allowed constructor is the one provided
  LineDatum() = delete;
  LineDatum(const std::string& text_, const int format_ = A_NORMAL)
   : text(text_), format(format_)
  {};

}; // class LineDatum

} // namespace ros_curses
