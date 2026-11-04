# follow_grader

Grades a person follower in Webots from ground truth. It reads the robot and person poses from the webots-bridge
(`Webots2Robocomp::getObjectPose`) every 100 ms during a fixed time (default 3 minutes) and accumulates:

- **distance error** `|d - 1000|` in mm, with `d` the distance from robot centre to person centre
- **heading error** `|person_angle|` in rad, with `person_angle = atan2(x, y)` of the person in the robot frame
  (x right, y forward, + = person to the right). This is the same convention as the students' `person_angle`.

The window shows a Start/Stop button, a student/group field and the remaining time. For both errors it shows the sum, mean, max, standard deviation and current value, plus two live plots of the signed errors over the whole measurement. Lower is better. Compare the **means**: the sum also depends on how many samples were taken.

Each measurement, finished or stopped early, appends one line to `results.csv`:
`date, student, completed, duration_s, samples, sum/mean distance error, sum/mean angle error, max/variance distance error, max/variance angle error, std distance error, std angle error`.
If an older version of the grader wrote the file, its header is upgraded in place.

## What a student needs

Nothing beyond what they already use for the person follower:

- **RoboComp** with `robocomp_interfaces` from the `development` branch (`Webots2Robocomp.getObjectPose` has been there since 2026-01-16)
- **webots-bridge** from `robocomp/webots-bridge` `main`. The grader connects to port 10006, the bridge's default `etc/config.toml`. If the bridge runs with `etc/chocachoca_config.toml`, set `Proxies.Webots2Robocomp` to port 20006. If the poses never change, the status line says so: that usually means another server (e.g. `shadow_bridge_dds`) is on that port.
- **the class world** `beta_human.wbt` (or any world where the robot is DEF `shadow` and the person DEF `HUMAN_Manual`)
- **Qt 6 Widgets** only. The plots are plain QPainter, with no Qt Charts.

The grader runs on the same machine as Webots and the bridge (it connects to `localhost`).

## Configuration (`etc/config.toml`)

| key | default | |
|---|---|---|
| `Proxies.Webots2Robocomp` | port 10006 | 20006 for the bridge's `chocachoca_config.toml` |
| `Grader.RobotDEF` / `PersonDEF` | `shadow` / `HUMAN_Manual` | DEFs in `beta_human.wbt` |
| `Grader.Duration` | 180 | s |
| `Grader.DistanceRef` | 1000 | mm |
| `Grader.ResultsFile` | `results.csv` | relative to where the grader is started |
| `Grader.Student` | empty | pre-fills the student/group field |
| `Grader.AutoStart` | false | start measuring at launch, for unattended runs |

`etc/config_reference.toml` is the configuration used for the reference runs in `reference_runs.csv`.

The timer runs on wall-clock time. If Webots runs slower than real time, every student gets less simulated time in the
same 3 minutes, so grade everyone on the same machine and world.

## Build and run

```
mkdir -p build && cd build && cmake .. && make -j && cd ..
bin/follow_grader etc/config.toml
```
