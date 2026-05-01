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

    // If clicked inside a furniture polygon, use goto_object for standoff + alignment
    for (const auto& fp : layout_manager_.furniture())
    {
        if (fp.vertices.size() < 3) continue;
        bool inside = false;
        const size_t n = fp.vertices.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++)
        {
            if (((fp.vertices[i].y() > target.y()) != (fp.vertices[j].y() > target.y())) &&
                (target.x() < (fp.vertices[j].x() - fp.vertices[i].x()) *
                               (target.y() - fp.vertices[i].y()) /
                               (fp.vertices[j].y() - fp.vertices[i].y()) + fp.vertices[i].x()))
                inside = !inside;
        }
        if (inside)
        {
            const std::string name = fp.label.empty() ? fp.id : fp.label;
            if (nav_manager_.goto_object(name))
            {
                const auto& path = nav_manager_.current_path();
                const Eigen::Vector2f target_m = path.empty() ? target : path.back();
                start_episode("goto_object", target_m, nav_manager_.nav_target_object_name());
                return;
            }
        }
    }

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
