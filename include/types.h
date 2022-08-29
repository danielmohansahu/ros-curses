/* Common datatypes used throughout the project.
 *
 *
 * 
 */

#pragma once

#include <string>

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

/* Enumeration of all possible actions requested by users.
 */
enum Action
{
  NONE,
  EXIT,
  INCREMENT,
  DECREMENT
}; // enum Action

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
