"""Plant known bugs in a scratch copy of the supervisor and check that the property tests catch every one.

A test suite that never fails proves little; this shows the property tests and the journal checker detect real
mistakes. Needs Hypothesis in the running interpreter. Exit code 1 if any planted bug survives.

    python3 scripts/mutation_check.py
"""

import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MISSION, PLANNER, STATE = "warehouse_agent/mission.py", "warehouse_agent/planner.py", "warehouse_agent/state.py"

# (description, [(file, original code, buggy replacement), ...])
MUTANTS = [
    ("cargo moves while the model is still interpreting", [(MISSION,
     "            if self.language_pending and not self.cancel_requested:\n                return\n",
     "            pass\n")]),
    ("planning ignores a pending request", [(MISSION,
     "        if self.paused or self.language_pending:\n            return\n",
     "        if self.paused:\n            return\n")]),
    ("cargo moves after a cancel", [(MISSION,
     "            if cancelled or self.paused or parcel.disposition is not Disposition.ACTIVE:\n",
     "            if self.paused:\n")]),
    ("arrival checks skipped", [(MISSION,
     'if outcome == "succeeded" and arrival["accepted"]:',
     'if outcome == "succeeded":')]),
    # The state machine alone would refuse this, so the bug has to break both layers to be observable.
    ("onboard cargo can be cancelled (supervisor check and transition table both broken)", [
     (MISSION, "            onboard = [pid for pid in ids if self.parcels[pid].state is Physical.ONBOARD]\n",
      "            onboard = []\n"),
     (STATE, '"cancel": ({(Physical.AWAITING_PICKUP, Disposition.ACTIVE)}, (None, Disposition.CANCELLED)),',
      '"cancel": ({(Physical.AWAITING_PICKUP, Disposition.ACTIVE), (Physical.ONBOARD, Disposition.ACTIVE)},'
      ' (None, Disposition.CANCELLED)),')]),
    ("deferral without a retry", [(MISSION,
     "if self.attempts[key] >= 2:",
     "if self.attempts[key] >= 1:")]),
    ("planner forgets drops for onboard parcels", [(PLANNER,
     '                remaining.append(Stop(pid, "pickup", parcel.pickup))\n'
     '            remaining.append(Stop(pid, "drop", parcel.drop))\n',
     '                remaining.append(Stop(pid, "pickup", parcel.pickup))\n'
     '                remaining.append(Stop(pid, "drop", parcel.drop))\n')]),
]

def tracked_files():
    listed = subprocess.run(["git", "ls-files", "-co", "--exclude-standard"], cwd=ROOT,
                            capture_output=True, text=True, check=True).stdout.split("\n")
    return [name for name in listed if name and (ROOT/name).is_file()]


def run_mutant(files, edits):
    with tempfile.TemporaryDirectory() as scratch:
        scratch = Path(scratch)
        for name in files:
            (scratch/name).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT/name, scratch/name)
        for path, original, buggy in edits:
            source = (scratch/path).read_text()
            if source.count(original) != 1:
                raise SystemExit(f"Mutation target not found exactly once in {path}; update MUTANTS:\n{original}")
            (scratch/path).write_text(source.replace(original, buggy))
        result = subprocess.run([sys.executable, "-B", "-m", "unittest", "discover", "-s", "tests",
                                 "-p", "test_properties.py"], cwd=scratch, capture_output=True, text=True)
        output = result.stdout + result.stderr
        if "Hypothesis is not installed" in output:
            raise SystemExit("Install Hypothesis first: pip install hypothesis")
        codes = set(re.findall(r"code='([A-Z_]+)'", output)) | set(re.findall(r"(IllegalTransition)", output))
        return result.returncode != 0, sorted(codes)


def main():
    files = tracked_files()
    survived = 0
    for description, edits in MUTANTS:
        caught, codes = run_mutant(files, edits)
        survived += not caught
        print(f"{'caught  ' if caught else 'SURVIVED'} {description}" + (f"  ({', '.join(codes)})" if codes else ""),
              flush=True)
    print(f"{len(MUTANTS)-survived}/{len(MUTANTS)} planted bugs caught")
    return 1 if survived else 0


if __name__ == "__main__":
    raise SystemExit(main())
