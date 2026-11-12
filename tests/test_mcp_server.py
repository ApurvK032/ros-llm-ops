"""The MCP server's protocol handling and tools, on synthetic runs (no ROS, no client library)."""

import io
import json
import tempfile
import unittest
from pathlib import Path

from test_diagnosis import healthy_evidence, journal

from warehouse_agent.mcp_server import Server, serve, tool_list


def request(identifier, method, **params):
    return {"jsonrpc": "2.0", "id": identifier, "method": method, "params": params}


class Case(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name) / "runs"
        evidence = healthy_evidence()
        evidence["parameter_changes"] = [{"t": 20.0, "node": "/velocity_smoother",
                                          "changed": [{"name": "max_velocity", "value": [0.0, 0.0, 0.0]}]}]
        self.add_run("bad_parameter/one", evidence, [(0, 90, "PICK_B", "failed")])
        self.add_run("nominal/one", healthy_evidence(), [(0, 30, "PICK_B", "succeeded")], live=True)
        (self.root / "empty").mkdir()
        (self.root / "empty/journal.jsonl").write_text("")
        self.server = Server(self.root)

    def add_run(self, name, evidence, goals, live=False):
        run = self.root / name
        run.mkdir(parents=True)
        (run / ("live_evidence.json" if live else "evidence.json")).write_text(json.dumps(evidence))
        (run / "journal.jsonl").write_text("\n".join(json.dumps(e) for e in journal(goals)) + "\n")
        if not live:
            (run / "result.json").write_text(json.dumps({
                "scenario": {"name": name.split("/")[0]}, "outcome": {"result": "timeout", "delivered": []}}))

    def call(self, name, **arguments):
        reply = self.server.handle(request(7, "tools/call", name=name, arguments=arguments))
        self.assertEqual((reply["jsonrpc"], reply["id"]), ("2.0", 7))
        content = reply["result"]["content"][0]
        self.assertEqual(content["type"], "text")
        return reply["result"]["isError"], content["text"]


class Protocol(Case):
    def test_initialize_agrees_a_version_and_offers_tools_only(self):
        result = self.server.handle(request(1, "initialize", protocolVersion="2025-03-26"))["result"]
        self.assertEqual((result["protocolVersion"], result["capabilities"]), ("2025-03-26", {"tools": {}}))
        self.assertEqual(result["serverInfo"]["name"], "warehouse-agent")
        newer = self.server.handle(request(1, "initialize", protocolVersion="2099-01-01"))["result"]
        self.assertEqual(newer["protocolVersion"], "2025-06-18")

    def test_notifications_get_no_reply_and_unknown_methods_an_error(self):
        self.assertIsNone(self.server.handle({"jsonrpc": "2.0", "method": "notifications/initialized"}))
        self.assertEqual(self.server.handle(request(2, "ping"))["result"], {})
        self.assertEqual(self.server.handle(request(3, "resources/list"))["error"]["code"], -32601)

    def test_every_tool_is_listed_with_a_schema(self):
        tools = self.server.handle(request(4, "tools/list"))["result"]["tools"]
        self.assertEqual(tools, tool_list())
        names = [tool["name"] for tool in tools]
        self.assertEqual(names[:3], ["list_runs", "health_report", "mission_summary"])
        self.assertEqual(names[-2:], ["rule_based_diagnosis", "check_action"])
        for tool in tools:
            self.assertTrue(tool["description"])
            self.assertEqual(tool["inputSchema"]["type"], "object")

    def test_one_message_per_line_and_a_broken_line_is_a_parse_error(self):
        lines = [json.dumps(request(1, "ping")) + "\n", "\n", "{not json\n",
                 json.dumps({"jsonrpc": "2.0", "method": "notifications/initialized"}) + "\n", "[1]\n"]
        out = io.StringIO()
        serve(self.server, lines, out)
        replies = [json.loads(line) for line in out.getvalue().splitlines()]
        self.assertEqual([r.get("error", {}).get("code") for r in replies], [None, -32700, -32600])


class Tools(Case):
    def test_runs_are_listed_with_how_they_ended(self):
        failed, text = self.call("list_runs")
        self.assertFalse(failed)
        self.assertEqual(json.loads(text), [
            {"run": "bad_parameter/one", "evidence": "recording", "scenario": "bad_parameter", "ended": "timeout",
             "delivered": [], "recovery": None},
            {"run": "nominal/one", "evidence": "live monitor"}])

    def test_a_check_reports_what_it_found_in_a_run(self):
        failed, text = self.call("parameter_changes", run="bad_parameter/one")
        self.assertFalse(failed)
        self.assertEqual(json.loads(text)["unusual"],
                         ["/velocity_smoother parameter max_velocity was set to [0.0, 0.0, 0.0] at 20.0 s"])
        failed, text = self.call("health_report", run="nominal/one")
        self.assertEqual(json.loads(text)["parameter_changes"], "nothing unusual")
        failed, text = self.call("rule_based_diagnosis", run="bad_parameter/one")
        self.assertEqual(json.loads(text)["category"], "bad_parameter")

    def test_the_gate_answers_without_carrying_anything_out(self):
        failed, text = self.call("check_action", run="bad_parameter/one", action="restore_parameter",
                                 target="/velocity_smoother max_velocity")
        verdict = json.loads(text)
        self.assertEqual((verdict["permitted"], verdict["carried_out"]), (True, False))
        failed, text = self.call("check_action", run="nominal/one", action="restore_parameter",
                                 target="/velocity_smoother max_velocity")
        self.assertFalse(json.loads(text)["permitted"])
        failed, text = self.call("check_action", run="nominal/one", action="format_disk")
        self.assertEqual((failed, json.loads(text)["why"]), (False, "not one of the permitted actions"))

    def test_bad_requests_come_back_as_tool_errors(self):
        for arguments, expected in (({"run": "../../etc"}, "outside the directory"),
                                    ({"run": "missing/run"}, "has no evidence"), ({}, "must be a run directory")):
            failed, text = self.call("health_report", **arguments)
            self.assertTrue(failed)
            self.assertIn(expected, text)
        failed, text = self.call("delete_everything")
        self.assertEqual((failed, text), (True, "no tool named delete_everything"))


if __name__ == "__main__":
    unittest.main()
