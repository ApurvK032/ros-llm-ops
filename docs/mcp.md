# The diagnostic checks over MCP

The checks the recovery agent uses are also available to any [Model Context Protocol](https://modelcontextprotocol.io)
client: an IDE assistant, a chat client, or another agent. Point the server at a directory of runs and ask
"why did this mission fail?" in whatever client you already use.

```bash
python3 -B -m warehouse_agent.mcp_server docs/evidence/scenarios
```

The server speaks JSON-RPC over standard input and output, needs no ROS and no dependency, and is read-only:
no tool starts, stops or changes anything. To register it with a client, give it the command and the two paths:

```json
{
  "mcpServers": {
    "warehouse-agent": {
      "command": "python3",
      "args": ["-B", "-m", "warehouse_agent.mcp_server", "/path/to/ros-llm-ops/docs/evidence/scenarios"],
      "env": {"PYTHONPATH": "/path/to/ros-llm-ops"}
    }
  }
}
```

## Tools

| Tool | Returns |
| --- | --- |
| `list_runs` | Every run under the directory: path, scenario, how the mission ended, finished recording or live |
| `health_report` | What each check found unusual in one run; the place to start |
| `mission_summary` | Each navigation goal with its outcome, deferred parcels, how the run ended |
| `node_health`, `lidar_status`, `motion_status`, `localization_status`, `parameter_changes`, `obstacles`, `planning_status`, `nav2_warnings` | One check in full, as described in [diagnosis](diagnosis.md) |
| `rule_based_diagnosis` | The fixed rules' answer: category, component, evidence |
| `check_action` | What the recovery gate would say about an action for this run, and why. Nothing is carried out |

A run is named by its path below the directory the server was started on, and a path that leaves that directory
is refused. Runs still under way are served from the live monitor's latest snapshot, so the same questions can
be asked of a mission while it is running.

## What an exchange looks like

Against the committed recordings, a client that calls `health_report` with
`{"run": "bad_parameter/20261110T040727Z"}` gets (five of the nine entries shown):

```json
{
 "mission": [
  "pickup P2 to PICK_B started at 14.6 s: failed after 186.4 s",
  "pickup P2 to PICK_B started at 201.0 s: failed after 178.0 s",
  "drop P1 to DROP_A started at 379.0 s: failed after 178.0 s",
  "drop P1 to DROP_A started at 557.0 s: cancelled after 162.6 s"
 ],
 "node_health": "nothing unusual",
 "lidar_status": "nothing unusual",
 "parameter_changes": [
  "/velocity_smoother parameter max_velocity was set to [0.0, 0.0, 0.0] at 23.3 s"
 ],
 "planning_status": [
  "the controller aborted path following 81 time(s)"
 ]
}
```

and `check_action` for two candidate fixes:

```json
{"action": "restore_parameter", "target": "/velocity_smoother max_velocity", "permitted": true,
 "why": "this parameter changed at runtime", "carried_out": false}
{"action": "restart_process", "target": "controller_server", "permitted": false,
 "why": "controller_server is not a process this run started", "carried_out": false}
```

The gate's answer is the one the recovery agent would get; the difference is that here nothing follows from it.

## Scope

- Tools only: no resources, prompts or sampling. Protocol versions 2024-11-05 to 2025-06-18.
- Checked with the official Python client library (`mcp` 2.3.0) against the committed recordings, and by unit
  tests that need no client ([`tests/test_mcp_server.py`](../tests/test_mcp_server.py)).
- The recovery actions themselves are deliberately not exposed. They change a running robot, and the only
  caller that may do that is the recovery agent, through the gate and the supervisor.
