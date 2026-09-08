# Stage 1 acceptance evidence

On 8 September 2026, the local model interpreted `Deliver all three parcels` and the robot completed three pickups and three drops from a fresh HOME pose. The run took **63.779 seconds** after intent acceptance. All six Nav2 goals succeeded; the largest measured arrival error was **0.2379 m** against a **0.35 m** tolerance.

- [Machine-readable results](results.json)
- [Full delivery journal](delivery.jsonl)
- [Gazebo after delivery](gazebo-delivered.png)
- [Ready panel](panel-ready.png), [onboard panel](panel-onboard.png), [completed panel](panel-complete.png)
- `snapshot-001.json` through `snapshot-008.json`: all three parcels progress through waiting, onboard and delivered with Gazebo connected. Any subsequent snapshot records session closure.
- [Python 3.14 tests](stage1-tests-source.log), [Python 3.12 tests](stage1-tests-ros.log): 15 tests passed in each environment.
- [Headless launch result](results.json): Nav2/localization ready with all three windows disabled; a late ROS subscriber received the three parcel boxes and seven station captions in the map frame.
- Raw simulator logs are omitted from the public evidence because they contain local environment paths. Native shutdown diagnostics remain documented in the runbook.

The snapshots were checked against station coordinates and reported robot pose: waiting/delivered boxes remain at their assigned stations, onboard boxes stay within the cargo rack offsets, and the final goal/queue are empty. The panel images and Gazebo frame are captures from the real running applications.

This is a successful development check, not a reliability benchmark. The final cosmetic edit raises the onboard caption above station labels; the recorded cargo and goal behavior is unchanged. Earlier interrupted development runs remain in ignored artifacts and are not counted as successful acceptance runs.
