#ifndef ROOM_LAYOUT_H
#define ROOM_LAYOUT_H

#include "localization_types.h"
#include <string>
#include <vector>

// Known room polygon (corners in world frame, counter-clockwise) loaded from a CSV: id,x,y,type.
class RoomLayout
{
public:
    // Throws std::runtime_error on a missing file or fewer than 3 corners.
    explicit RoomLayout(const std::string &csv_path);

    const std::vector<Corner> &corners() const;           // world frame, CSV order
    std::vector<LineSegment> walls() const;               // corner i -> i+1, support = 0
    Eigen::AlignedBox2f bounds() const;

private:
    std::vector<Corner> corners_;
};

#endif
