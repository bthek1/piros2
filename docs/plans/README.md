# Plans

Build plans live here, filed by status:

- **`in-progress/`** — where every new plan starts.
- **`completed/`** — where a plan moves once its work is done. Moving the
  file *is* the status change; fix inbound links when it moves.

Reference docs (what things are and how they behave) belong in
[docs/info/](../info/), not here. A plan is a build order: it says what gets
built, in what sequence, and how each step is proved.

## Stable phases

Every plan is structured as numbered phases (P0, P1, …), each ending with
something you can run and check before the next starts. Once a phase is
written, its number and scope stay fixed — record progress by annotating the
phase (dates, ✓ marks, what actually happened), never by renumbering or
reshuffling, so that "P2" means the same thing in every doc, commit message,
and conversation that mentions it.

Current plans:

| Plan | Status |
| --- | --- |
| [cloud-fusion-plan.md](in-progress/cloud-fusion-plan.md) | In progress — P0–P1 done 2026-07-30 (`cloud_fusion` node fusing, verified on `bags/static1`); P2 hysteresis next |
| [perception-plan.md](completed/perception-plan.md) | Closed 2026-07-29 — P0–P2 done, P3 plumbing verified; the sweep and map were not run |
| [ansible-plan.md](completed/ansible-plan.md) | Completed 2026-07-24 — kept as the build log |
