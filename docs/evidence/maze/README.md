# Maze warehouse acceptance evidence

The updated warehouse completed `Deliver all three parcels` from a fresh HOME pose in **123.044 seconds after intent acceptance**. All six navigation goals succeeded; there were three pickups, three drops, and no application retries in this run.

- [Results, including actual Gazebo transfer poses](results.json)
- [Execution journal](delivery.jsonl)
- [Warehouse ready](maze-ready.png)
- [All parcels delivered](maze-delivered.png)
- [22 tests on the ROS runtime's Python 3.12](tests-ros.log)

The same 22 automated tests also passed on Python 3.14 in the source environment. They cover existing cargo/cancellation behavior plus unreachable overlap poses, incorrect pickup/drop orientation, true facing direction, wrapped yaw, aisle connectivity, shelf detours, and generated collision geometry.

## Captured configuration

These images show the same 18 × 14 m warehouse used in the recorded run: seven shelf sections, three shelf pickup positions, and three delivery pedestals. Robot approach circles and facing arrows are separate from the parcel storage positions. The complete configuration is embedded in the first event of [the execution journal](delivery.jsonl); the editable version is [config/warehouse.json](../../../config/warehouse.json).

![Fresh warehouse with parcels waiting on the shelves](maze-ready.png)

*Before the mission: the robot is at HOME and the three parcels are waiting at their shelf positions.*

![Completed mission with all three parcels at their delivery pedestals](maze-delivered.png)

*After the mission: all three parcels are delivered, and the robot remains at a separate approach position in front of the final pedestal.*

## Independent pose check

A read-only test observer subscribed to the ROS `/warehouse/status` topic and Gazebo's `/world/warehouse_mvp/dynamic_pose/info` transport topic. For each transition to onboard or delivered, it compared the latest actual `turtlebot3_waffle` pose with the configured approach and parcel positions.

All six actual poses passed the same checks used by the supervisor. The largest actual approach-position error was **0.132 m**, the largest actual facing error was **0.177 rad (10.1°)**, and the smallest conservative actual parcel clearance was **0.422 m**. Samples are near the published state transition rather than a synchronized physics-step trace.

The supervisor's TF-based measurements are recorded in `delivery.jsonl`; the independent samples are in `ground_truth_transfers` in `results.json`. Maximum observed localization error over the sampled run was **0.148 m** in position and **0.088 rad** in heading.

## Scope

The first larger-scene attempt ended with deferred deliveries after an AMCL pose jump. The run recorded here followed a localization adjustment for the expanded shelf layout. The initial failed attempt remains in ignored development artifacts and is not included as a success.

This is one successful development acceptance run, not a reliability benchmark. Pickup and drop are logical transfers; the parcel markers have no grasping or attachment physics. The screenshots contain only the application scene, with no desktop account names or paths.
