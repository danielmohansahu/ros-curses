/* Common datatypes used throughout the project.
 *
 *
 * 
 */

#pragma once

#include <string>

namespace ros_curses
{

enum Action
{
  NONE,
  EXIT
}; // enum Action

/* String representation of a single line, with formatting information.
 */
class LineDatum
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

}; // namespace ros_curses
