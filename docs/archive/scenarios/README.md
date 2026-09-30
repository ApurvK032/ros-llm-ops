# Development scenario contracts

`development.json` defines five initial mission examples and five mid-mission change examples with expected outcomes. These are fixtures for the upcoming implementation, not executable mission scenarios yet. The expected fields belong to the evaluator and must never be included in a model prompt.

Coordinates use `[row, column]`, 0-based indexing, four-neighbor motion. The empty grid makes early state tests easy to reason about. Add an obstacle map and controlled navigation failure fixture with CORE-02; the original planner's obstacle-world baseline remains separate.

Mid-mission examples describe a precondition snapshot and the event boundary where the request occurs. When the harness exists, reach that precondition through acknowledged events, or mark a deliberately synthetic setup explicitly. Record the actual state at request receipt and at revision activation.

The five examples do not exhaust the test matrix. Add duplicate/late events, two overlapping requests, stale map assumptions, unknown IDs, invalid model output, journal-write failures, and unreachable destinations as listed in `docs/backlog.md`.
