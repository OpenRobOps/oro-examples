# ORO examples

Working examples of [ORO](https://github.com/OpenRobOps/oro) with real and simulated robots.
Each folder is self-contained: it has its own README with how to run it.

| Example | Robot | What it shows |
|---|---|---|
| [turtlebot-gazebo-sim](turtlebot-gazebo-sim/) | TurtleBot3 Burger: fake, Gazebo or real | Teleop, waypoints, missions, SLAM + Nav2, ORO actions, statuses and incidents |
| [flatland-nav2-sim](flatland-nav2-sim/) | Simulated diff-drive AMR in a 2D office (Flatland) | Nav2 on a known map, battery + charging docks, camera, ROS diagnostics, ORO dashboards and actions |

Both examples use the same ORO object ids (dashboards, data sources, ModuleStates, footprint):
run one example per ORO instance, or re-apply its config when you switch.

Clone with submodules (Flatland builds from upstream source):

```bash
git clone --recurse-submodules https://github.com/OpenRobOps/oro-examples.git
```

Maintainer: Irina Terebiznik
