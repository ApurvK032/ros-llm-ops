Runs that achieved the expected result, and their median mission time:

| Scenario | Supervisor alone | Playbook | qwen3.5:9b | qwen3.5:4b |
| --- | --- | --- | --- | --- |
| `amcl_crash` | 0 of 3 | 0 of 5 | 0 of 5 | 0 of 5 |
| `bad_parameter` | 0 of 3 | 5 of 5, 127.4 s | 5 of 5, 136.3 s | 5 of 5, 138.6 s |
| `blocked_aisle` | 3 of 3, 146.4 s | 5 of 5, 146.1 s | 5 of 5, 146.1 s | 5 of 5, 146.6 s |
| `blocked_station` | 3 of 3, 498.4 s | 5 of 5, 104.4 s | 5 of 5, 111.4 s | 5 of 5, 115.3 s |
| `controller_crash` | 0 of 3 | 5 of 5, 145.5 s | 5 of 5, 149 s | 5 of 5, 148.4 s |
| `distractor_lidar` | 0 of 3 | 5 of 5, 140.9 s | 5 of 5, 146.5 s | 2 of 5, 158.1 s |
| `double_fault` | 0 of 3 | 2 of 5, 148.6 s | 0 of 5 | 0 of 5 |
| `lidar_dropout` | 0 of 3 | 3 of 5, 133.2 s | 5 of 5, 140 s | 5 of 5, 144.5 s |
| `lidar_frozen` | 0 of 3 | 5 of 5, 138.3 s | 4 of 5, 149.9 s | 3 of 5, 159.1 s |
| `localization_lost` | 0 of 3 | 5 of 5, 140.4 s | 5 of 5, 147.4 s | 5 of 5, 147.4 s |
| `nominal` | 3 of 3, 123.1 s | 5 of 5, 123.2 s | 5 of 5, 123.3 s | 5 of 5, 123.4 s |
| `planner_crash` | 0 of 3 | 5 of 5, 143.9 s | 5 of 5, 151.8 s | 5 of 5, 150.3 s |
| `speed_limit` | 3 of 3, 159.2 s | 5 of 5, 126.7 s | 5 of 5, 136.2 s | 5 of 5, 140.4 s |
| **all** | **12 of 39** | **55 of 65** | **54 of 65** | **50 of 65** |

Interruptions of the mission, and actions carried out that the scenario did not need:

| Scenario | Playbook | qwen3.5:9b | qwen3.5:4b |
| --- | --- | --- | --- |
| `amcl_crash` | 2.4 per run; 1 unneeded in 1 of 5 | 1.0 per run; 0 unneeded in 0 of 5 | 2.6 per run; 0 unneeded in 0 of 5 |
| `bad_parameter` | 1.0 per run; 0 unneeded in 0 of 5 | 1.0 per run; 0 unneeded in 0 of 5 | 1.0 per run; 0 unneeded in 0 of 5 |
| `blocked_aisle` | 0.0 per run; 0 unneeded in 0 of 5 | 0.0 per run; 0 unneeded in 0 of 5 | 0.0 per run; 0 unneeded in 0 of 5 |
| `blocked_station` | 1.0 per run; 0 unneeded in 0 of 5 | 1.0 per run; 0 unneeded in 0 of 5 | 1.0 per run; 0 unneeded in 0 of 5 |
| `controller_crash` | 1.4 per run; 0 unneeded in 0 of 5 | 1.2 per run; 0 unneeded in 0 of 5 | 1.0 per run; 0 unneeded in 0 of 5 |
| `distractor_lidar` | 2.0 per run; 5 unneeded in 5 of 5 | 2.0 per run; 5 unneeded in 5 of 5 | 2.6 per run; 4 unneeded in 4 of 5 |
| `double_fault` | 3.0 per run; 0 unneeded in 0 of 5 | 3.0 per run; 0 unneeded in 0 of 5 | 3.0 per run; 0 unneeded in 0 of 5 |
| `lidar_dropout` | 1.8 per run; 0 unneeded in 0 of 5 | 1.0 per run; 0 unneeded in 0 of 5 | 1.0 per run; 0 unneeded in 0 of 5 |
| `lidar_frozen` | 1.0 per run; 0 unneeded in 0 of 5 | 1.4 per run; 0 unneeded in 0 of 5 | 1.8 per run; 0 unneeded in 0 of 5 |
| `localization_lost` | 1.0 per run; 0 unneeded in 0 of 5 | 1.0 per run; 0 unneeded in 0 of 5 | 1.0 per run; 0 unneeded in 0 of 5 |
| `nominal` | 0.2 per run; 1 unneeded in 1 of 5 | 0.2 per run; 1 unneeded in 1 of 5 | 0.0 per run; 0 unneeded in 0 of 5 |
| `planner_crash` | 1.4 per run; 0 unneeded in 0 of 5 | 1.2 per run; 0 unneeded in 0 of 5 | 1.0 per run; 0 unneeded in 0 of 5 |
| `speed_limit` | 1.0 per run; 5 unneeded in 5 of 5 | 1.0 per run; 5 unneeded in 5 of 5 | 1.0 per run; 5 unneeded in 5 of 5 |
