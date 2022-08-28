/*
 *
 *
 * 
 */

#pragma once

// STL
#include <vector>
#include <optional>

// ros_curses
#include "types.h"

namespace ros_curses
{

/* Base Class for a general Updater interface to produce panel-specific information.
 */
class Updater
{
 private:
  
 public:

  /* Core method - return rendered data for the given panel identifier.
   */
  virtual std::optional<std::vector<LineDatum>> update(const std::string& panel) = 0;
};

} // namespace ros_curses