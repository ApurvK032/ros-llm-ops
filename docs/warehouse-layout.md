# Maze warehouse and parcel approach poses

The warehouse is **18 × 14 m (252 m²)**, about 3.15 times the original floor area. Seven shelf sections form staggered rows, L-shaped corners, and connecting aisles. Pickup stations sit on shelf faces; three delivery pedestals sit along the east side.

![Warehouse layout with separate robot stopping poses and parcel positions](warehouse-layout.svg)

## Stop in front and face the parcel

Each station has two separate locations in [the configuration](../config/warehouse.json):

- `waypoints`: the robot's approach pose `[x, y, yaw]`, in metres and radians.
- `stations.<name>.parcel_position`: the parcel's visible storage position `[x, y, z]`.

The default approach poses are **1.0 m from the parcel centres**. Ground circles show where the robot should stop; arrows show its required facing direction. Parcels remain on their shelf or pedestal until the supervisor confirms a transfer. The cyan active-goal ring marks the approach pose.

For example, PICK_A stores its parcel at `[-5.3, -1.5, 0.78]`. The robot navigates to `[-6.3, -1.5, 0.0]`: one metre west, facing east toward the parcel. PICK_B faces north and PICK_C faces west, so the robot must turn appropriately at different shelf faces.

Before either pickup or drop, the supervisor requires:

| Check | Limit |
| --- | --- |
| Nav2 action result | Succeeded |
| Distance from the robot to its approach pose | At most 0.20 m |
| Heading error from the configured approach orientation | At most 0.22 rad, about 12.6° |
| Facing error toward the parcel from the measured robot position | At most 0.25 rad, about 14.3° |
| Clearance between conservative robot/parcel footprints | At least 0.20 m |

The clearance calculation uses a 0.22 m robot radius and the parcel's circumscribed horizontal radius. At the ideal approach pose, the conservative gap is about 0.55 m. It is checked independently of position tolerance, so loosening the position check does not permit a transfer when the measured pose overlaps a parcel.

Nav2 uses tighter goal tolerances (0.12 m position and 0.10 rad heading) to leave room for the supervisor's measured checks. A navigation success with an unsuitable pose does not transfer cargo; the existing retry/defer policy applies.

Pickup and drop remain logical operations. There is no arm or physical grasping. The displayed parcel moves onto the robot only after the arrival checks pass.

## Shared geometry

Shelf and pedestal footprints are used by the Gazebo collisions, the Nav2 occupancy map, and the inflated A* planning grid. Rack visuals include bases, decks, and uprights; the complete shelf footprint is reserved as an obstacle. Stored pickup parcels lie within these reserved shelf footprints, so navigation does not route through them.

HOME, robot spawn, initial localization, and the overview camera are derived from configuration. A fresh simulator launch regenerates the world, maps, Nav2 parameters, and GUI camera settings.

## Run it

Start a fresh demo with `bash scripts/demo.sh`, or `bash scripts/wsl.sh browser` from the two-distribution WSL setup. A running simulator must be restarted to load changed geometry.

Enter `Deliver all three parcels` in the agent terminal. Watch the robot navigate the shelf aisles, stop on an approach circle, rotate toward the parcel, and then update cargo. The same front-of-station rule applies when placing a parcel at a delivery pedestal.

The automated suite checks aisle reachability, shelf detours, generated collisions, facing direction, wrapped yaw, rejection of overlapping transfers, and preservation of the existing cargo/cancellation rules. Real simulation evidence is kept separately from those tests.


## Simulation validation

A full mission completed all three pickups and drops across six successful Nav2 goals in 123.044 seconds. A read-only observer compared ROS status changes with the Gazebo robot pose. Every actual transfer pose passed the same approach, heading, facing, and clearance checks. See [measurements and captures](evidence/maze/README.md).

The first larger-layout attempt exposed an AMCL pose jump after a turn between similar shelf faces. The simulation now uses lower odometry motion-noise values, 1,000–3,000 particles, 120 laser beams, and more frequent updates. The laser scan was checked against the generated geometry before changing these settings. Parameter definitions are documented in [Nav2's Jazzy AMCL guide](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/). This tuning was validated in the recorded simulation run; it is not a general reliability result.
