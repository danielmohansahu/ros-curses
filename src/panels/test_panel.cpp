/*
 *
 *
 * 
 */

// ros_curses
#include "panels/test_panel.h"

namespace ros_curses::panels
{

ActionPacket TestPanel::render(const std::optional<ComputationalGraph>& graph)
{
    return NULL_ACTION;
}

void TestPanel::handle_key_up()
{

}

void TestPanel::handle_key_down()
{

}

} // namespace ros_curses::panels