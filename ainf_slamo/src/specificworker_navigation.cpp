#include "specificworker.h"

#include <algorithm>
#include <limits>

void SpecificWorker::slot_new_target(QPointF pos)
{
    if (editing_room_layout_)
        return;

    if (!room_ai.is_loc_initialized() || !nav_manager_.path_planner().is_ready())
    {
        qWarning() << "Cannot set target: room not initialized or planner not ready";
        return;
    }

    const Eigen::Vector2f target(pos.x(), pos.y());
    nav_manager_.plan_to_target(target);
    start_episode("goto_point", target);
}

void SpecificWorker::send_velocity_command(float adv, float side, float rot)
{
    try
    {
        omnirobot_proxy->setSpeedBase(side*1000.f, adv*1000.f, rot);
    }
    catch (const Ice::Exception &e)
    {
        static int err_count = 0;
        if (++err_count % 100 == 1)
            qWarning() << "OmniRobot proxy error:" << e.what();
    }
}
