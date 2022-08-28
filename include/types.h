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
  INITIALIZATION,
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
  const uint8_t indentations;
  const bool highlight;

  // only allowed constructor is the one provided
  LineDatum() = delete;
  LineDatum(const std::string& text_, const uint8_t indentations_=0, const bool highlight_=false)
   : text(text_), indentations(indentations_), highlight(highlight_)
  {};

}; // class LineDatum

} // namespace ros_curses
