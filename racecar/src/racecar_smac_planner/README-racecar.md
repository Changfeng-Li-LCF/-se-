# Racecar Smac optimization

Based on Navigation2 tag `1.1.18`, `nav2_smac_planner`, Apache-2.0.
Upstream: https://github.com/ros-navigation/navigation2/tree/1.1.18/nav2_smac_planner

The custom plugin ID is `racecar_smac_planner/SmacPlannerHybrid`.
Namespaces are isolated from the system package. Hybrid A* motion primitives,
costs, collision checks, smoothing and turning-radius policy are unchanged.

`src/node_hybrid.cpp` replaces eager OMPL distance-table evaluation with lazy
evaluation of the same discrete entries. A radius/model/table change invalidates
the whole bounded table; the planner's existing mutex protects access. No table
of historical radii grows in memory. Difficult searches can still take longer;
this is not a real-time execution guarantee.

`fast/frontier_kernel.cpp` implements the existing frontier selector's bounded
BFS traversal, with identical neighbor ordering. The Python wrapper remains in
`~/racecar-tests/s-curve-test/fast_frontier_kernel.py`. It uses an independently
spawned process, one outstanding request, and one cached map-mask snapshot.

Build without colcon on this car:

```bash
source /opt/ros/humble/setup.bash
cmake -S ~/racecar/src/racecar_smac_planner -B ~/racecar/build/racecar_smac_planner -DCMAKE_INSTALL_PREFIX=$HOME/racecar/install/racecar_smac_planner -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build ~/racecar/build/racecar_smac_planner -j2
cmake --install ~/racecar/build/racecar_smac_planner
source ~/racecar/install/setup.bash
```

The deployment adds the standard colcon package registration/environment files
to the isolated install prefix, so existing workspace setup commands discover
this plugin. Rebuilding in place preserves those files. Normal colcon builds
generate their own registration.

`bench_lazy_distance` compares the stock and custom distance calculations for
12,006 samples, both Dubins and Reeds-Shepp, and three radii. It publishes no ROS
messages. The separate offline Python integration test uses saved maps and a
synthetic obstacle map, isolated ROS domain 187, a planner only, and no vehicle
controller or driver. Recorded results accompany the optimization report.

Rollback requires restoring the backed-up Python files and shared nav.yaml /
nav_carto.yaml together. Leaving the unused isolated package installed is safe.
The system package in `/opt/ros/humble` is not modified.
