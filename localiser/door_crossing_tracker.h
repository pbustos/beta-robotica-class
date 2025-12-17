//
// Created by pbustos on 14/12/25.
//

#ifndef LOCALISER_DOOR_CROSSING_TRACKER_H
#define LOCALISER_DOOR_CROSSING_TRACKER_H

// Door crossing struct to track used door in crossing situations

        struct DoorCrossing
        {
            int leaving_room_index = -1;
            int leaving_door_index = -1;
            int entering_room_index = -1;
            int entering_door_index = -1;
            bool valid = false;                             // only true if both leaving and entering data are set
            DoorCrossing() = default;                       // attribs take default values
            DoorCrossing(int room_index, int door_index)    // leaving room and door are initialized
                : leaving_room_index{room_index}, leaving_door_index{door_index} { valid = true; }
            Eigen::Vector2f leaving_door_center{0.f, 0.f};   // local coordinate of tracked door

            // track the leaving door center in local coordinates
            void track_entering_door(const Doors &doors)
            {
                if (doors.empty()) return;
                // find the door whose center is closest to the leaving door center in local coordinates
                leaving_door_center = std::ranges::min_element(doors, [d =leaving_door_center](const auto &a, const auto &b)
                { return (a.center()-d).norm() < (b.center()-d).norm();})->center();
            }

            // // compute and store entering room and door indices using the tracked leaving_door_center
            void set_entering_data(int room_index, const std::vector<NominalRoom> &nom_rooms)          // compute door index from leaving_door_center
            {
                entering_room_index = room_index;
                // find the door in nominal_rooms[room_index] whose center is closest to leaving_door_center transformed to global
                const auto &nominal_doors = nom_rooms[room_index].doors;
                if (nominal_doors.empty())
                {
                    qWarning() << __FUNCTION__ << "empty nominal doors for room" << room_index;
                    return;
                }
                const auto closest_door = std::ranges::min_element(nominal_doors, [this](const auto &a, const auto &b)
                { return (a.center_global() - leaving_door_center).norm() < (b.center_global() - leaving_door_center).norm(); });
                entering_door_index = static_cast<int>(std::distance(nominal_doors.begin(), closest_door));
            }
        };
#endif //LOCALISER_DOOR_CROSSING_TRACKER_H