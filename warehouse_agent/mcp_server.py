"""Model Context Protocol server: the diagnostic checks as tools any MCP client can call.

Point it at a directory of scenario runs. A client (an IDE assistant, a chat client, another agent) can list the
runs, read one run's health report, open any single check, get the rule-based diagnosis, and ask what the
recovery gate would say about an action. Everything is read-only: no tool here touches a robot or a mission.

    python3 -B -m warehouse_agent.mcp_server artifacts/scenarios

The protocol is JSON-RPC 2.0, one message per line on standard input and output; this file implements the small
part a tools-only server needs, with no dependency.
"""

import json
import sys
from pathlib import Path

from .diagnosis import TOOLS, Incident, rule_based
from .recovery import ACTIONS, gate

VERSIONS = ("2025-06-18", "2025-03-26", "2024-11-05")
RUN = {"type": "string", "description": "A run directory, as returned by list_runs"}


class Refused(ValueError):
    """The request cannot be served as asked; the text goes back to the client."""


def describe(function):
    return " ".join(function.__doc__.split())


def tool_list():
    tools = [{"name": "list_runs", "description": "Scenario runs under the server's directory: path, scenario, "
              "how the mission ended, and whether the run is finished or still live.",
              "inputSchema": {"type": "object", "properties": {}}}]
    tools += [{"name": name, "description": describe(function),
               "inputSchema": {"type": "object", "properties": {"run": RUN}, "required": ["run"]}}
              for name, function in TOOLS.items()]
    tools.append({"name": "rule_based_diagnosis", "description": describe(rule_based),
                  "inputSchema": {"type": "object", "properties": {"run": RUN}, "required": ["run"]}})
    tools.append({"name": "check_action", "description": "Ask the recovery gate whether an action would be "
                  "permitted for this run, and why. Nothing is carried out. Actions: " + "; ".join(
                      f"{name}: {text}" for name, text in ACTIONS.items()),
                  "inputSchema": {"type": "object", "required": ["run", "action"], "properties": {
                      "run": RUN, "action": {"type": "string", "enum": list(ACTIONS)},
                      "target": {"type": "string", "description": "What the action applies to; may be empty"}}}})
    return tools


class Server:
    def __init__(self, root):
        self.root = Path(root).resolve()

    def runs(self):
        found = []
        for journal in sorted(self.root.rglob("journal.jsonl")):
            run = journal.parent
            finished, live = (run / "evidence.json").exists(), (run / "live_evidence.json").exists()
            if not finished and not live:
                continue
            entry = {"run": str(run.relative_to(self.root)), "evidence": "recording" if finished else "live monitor"}
            result = run / "result.json"
            if result.exists():
                saved = json.loads(result.read_text(encoding="utf-8"))
                entry.update(scenario=saved["scenario"]["name"], ended=saved["outcome"]["result"],
                             delivered=saved["outcome"]["delivered"],
                             recovery=(saved.get("recovery") or {}).get("mode"))
            found.append(entry)
        return found

    def incident(self, name):
        if not isinstance(name, str) or not name:
            raise Refused("'run' must be a run directory from list_runs")
        run = (self.root / name).resolve()
        if run != self.root and self.root not in run.parents:
            raise Refused("that path is outside the directory this server was started on")
        if (run / "evidence.json").exists():
            return Incident(run)
        if (run / "live_evidence.json").exists():
            return Incident(run, live=True)
        raise Refused(f"{name} has no evidence; run 'python3 -m warehouse_agent diagnose --rules {name}' first")

    def call(self, name, arguments):
        if name == "list_runs":
            return self.runs()
        if name in TOOLS:
            return TOOLS[name](self.incident(arguments.get("run")))
        if name == "rule_based_diagnosis":
            return rule_based(self.incident(arguments.get("run")))["diagnosis"]
        if name == "check_action":
            incident = self.incident(arguments.get("run"))
            # The parcels as the mission last recorded them; a live run also has the supervisor's own status.
            state = next((e["state"] for e in reversed(incident.events) if isinstance(e.get("state"), dict)), {})
            status = incident.evidence.get("mission_status") or state
            permitted, why, _ = gate(arguments.get("action"), arguments.get("target", ""), incident, status,
                                     incident.run)
            return {"action": arguments.get("action"), "target": arguments.get("target", ""),
                    "permitted": permitted, "why": why, "carried_out": False}
        raise Refused(f"no tool named {name}")

    def handle(self, message):
        """Answer one JSON-RPC message. Returns the reply, or None for a notification."""
        method, identifier = message.get("method"), message.get("id")
        if identifier is None:
            return None
        params = message.get("params") or {}

        def reply(result=None, error=None):
            body = {"error": error} if error else {"result": result}
            return {"jsonrpc": "2.0", "id": identifier, **body}

        if method == "initialize":
            asked = params.get("protocolVersion")
            return reply({"protocolVersion": asked if asked in VERSIONS else VERSIONS[0],
                          "capabilities": {"tools": {}},
                          "serverInfo": {"name": "warehouse-agent", "version": "0.0.1"},
                          "instructions": "Read-only diagnostics for recorded or running warehouse-robot "
                                          "missions. Start with list_runs, then health_report for one run."})
        if method == "ping":
            return reply({})
        if method == "tools/list":
            return reply({"tools": tool_list()})
        if method == "tools/call":
            try:
                output = self.call(params.get("name"), params.get("arguments") or {})
                return reply({"content": [{"type": "text", "text": json.dumps(output, indent=1)}], "isError": False})
            except (Refused, OSError, KeyError, json.JSONDecodeError) as exc:
                text = str(exc) if isinstance(exc, Refused) else f"{type(exc).__name__}: {exc}"
                return reply({"content": [{"type": "text", "text": text}], "isError": True})
        return reply(error={"code": -32601, "message": f"Method not found: {method}"})


def serve(server, lines=sys.stdin, out=sys.stdout):
    for line in lines:
        if not line.strip():
            continue
        try:
            message = json.loads(line)
        except json.JSONDecodeError:
            answer = {"jsonrpc": "2.0", "id": None, "error": {"code": -32700, "message": "Parse error"}}
        else:
            answer = server.handle(message) if isinstance(message, dict) else {
                "jsonrpc": "2.0", "id": None, "error": {"code": -32600, "message": "Invalid Request"}}
        if answer is not None:
            out.write(json.dumps(answer, allow_nan=False) + "\n")
            out.flush()


def main():
    if len(sys.argv) != 2 or not Path(sys.argv[1]).is_dir():
        print("usage: python3 -B -m warehouse_agent.mcp_server <directory of runs>", file=sys.stderr)
        return 2
    serve(Server(sys.argv[1]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
