"""The demo launcher waits for Nav2, restarts the simulation when Nav2 aborts its bringup, and gives up cleanly.

Runs the real scripts/demo.sh against a fake simulator, agent, curl and ollama; no ROS needed.
"""

import os
import shutil
import stat
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FAKE_SIM = """#!/usr/bin/env bash
n=$(cat artifacts/attempts 2>/dev/null || echo 0); n=$((n+1)); echo $n > artifacts/attempts
if [[ "{mode}" == exit ]]; then echo "A warehouse simulation is already running on this ROS domain."; exit 2; fi
if (( n <= {aborts} )); then
  echo "[lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup."
else
  printf '[INFO] [lifecycle_manager_navigation]: \\033[34m\\033[1mManaged nodes are active\\033[0m\\n'
fi
trap 'exit 0' INT TERM
while true; do sleep 0.1; done
"""


@unittest.skipUnless(shutil.which("bash") and shutil.which("setsid"), "needs bash and setsid")
class DemoLauncherTests(unittest.TestCase):
    def launch(self, aborts=0, mode="run"):
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            (work/"scripts").mkdir()
            (work/"artifacts").mkdir()
            (work/"bin").mkdir()
            shutil.copy(ROOT/"scripts/demo.sh", work/"scripts/demo.sh")
            files = {"scripts/run_sim.sh": FAKE_SIM.format(aborts=aborts, mode=mode),
                     "scripts/run_agent.sh": '#!/usr/bin/env bash\necho "agent started: $*"\n',
                     "bin/curl": "#!/bin/sh\nexit 0\n", "bin/ollama": "#!/bin/sh\nexit 0\n"}
            for name, text in files.items():
                (work/name).write_text(text)
                (work/name).chmod((work/name).stat().st_mode | stat.S_IEXEC)
            env = {**os.environ, "PATH": f"{work/'bin'}{os.pathsep}{os.environ['PATH']}"}
            result = subprocess.run(["bash", "scripts/demo.sh", "--command", "Deliver all three parcels"],
                                    cwd=work, env=env, capture_output=True, text=True, timeout=60)
            attempts = int((work/"artifacts/attempts").read_text())
            logs = sorted(p.name for p in (work/"artifacts").glob("simulation.attempt*.log"))
            return result.returncode, result.stdout + result.stderr, attempts, logs

    def test_starts_the_agent_once_nav2_is_active(self):
        code, output, attempts, logs = self.launch()
        self.assertEqual((code, attempts, logs), (0, 1, []))
        self.assertIn("agent started: --command Deliver all three parcels", output)

    def test_restarts_the_simulation_when_nav2_aborts_its_bringup(self):
        code, output, attempts, logs = self.launch(aborts=2)
        self.assertEqual((code, attempts), (0, 3))
        self.assertEqual(logs, ["simulation.attempt1.log", "simulation.attempt2.log"])
        self.assertEqual(output.count("Nav2 aborted its bringup"), 2)
        self.assertIn("agent started", output)

    def test_gives_up_after_three_aborted_bringups(self):
        code, output, attempts, logs = self.launch(aborts=3)
        self.assertEqual((code, attempts), (1, 3))
        self.assertIn("Simulation not ready: Nav2 aborted its bringup (attempt 3)", output)
        self.assertNotIn("agent started", output)

    def test_does_not_retry_when_the_simulator_exits(self):
        code, output, attempts, _ = self.launch(mode="exit")
        self.assertEqual((code, attempts), (1, 1))
        self.assertIn("the simulator exited", output)
        self.assertIn("already running", output)


if __name__ == "__main__":
    unittest.main()
