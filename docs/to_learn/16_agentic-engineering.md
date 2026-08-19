# Agentic engineering — the study file for section 16 of [emescent.md](emescent.md)

Written 2026-08-19. One section per checklist item: the concept at conversation depth, the
sentence an interviewer is fishing for, and an honest **`piros2` line**. The syllabus's
priority note: **"Listed as a nice-to-have on the Emesent JD and largely already held via the
Stock Market Analyser's ten agent workflow patterns ([projects.md](projects.md))."** That
project is not in this repo, so this file does not describe it — it explains the patterns
themselves so the conversation about it is crisp. Per the honest-claim rule, reading ≠
holding: `piros2` contains **no LLM code at all**. What it *is* is a repository built almost
entirely with an AI coding agent (Claude Code) as the day-to-day development workflow, and its
`CLAUDE.md`, memory notes, and verification doctrine are a real, dated record of where such
agents fail and what it takes to make them safe on real hardware — that is the evidence for
item 7, and the `piros2` lines for items 1–6 say so honestly rather than stretching.

## Mental model to carry through the file

```
                ┌──────────── the loop ────────────┐
   prompt ──►   │ model ─► (tool call?) ─► tool ─►  │ ─► answer
                │   ▲          observation │        │
                │   └──────────────────────┘        │
                └───────────────────────────────────┘
   workflow  = the loop's shape is fixed in code   (chain, route, fan-out, plan/execute)
   agent     = the model decides the loop's shape   (ReAct, orchestrator, multi-agent)
```

The one distinction that organises everything below (Anthropic's "Building effective agents",
Dec 2024, made it standard vocabulary): a **workflow** is LLM calls orchestrated by code you
wrote — predictable, testable, cheap; an **agent** is an LLM that chooses its own next step
and tool from the observation it just got — flexible, expensive, and only as good as its tools
and its stopping rule. Reach for the simplest thing that works: a single call with retrieval,
then a workflow, and only then an agent. Emesent's JD lists this as nice-to-have because the
plausible uses are *around* the robot — log triage, field-report search, test generation,
support tooling — not inside a real-time C++ loop, and the discipline they will probe is the
same one field robotics has always needed: verify, bound, log, and don't trust a confident
output you didn't check.

## 1. Agent workflow patterns: routing, chaining, parallelisation, ReAct, evaluator-optimizer, plan-and-execute, orchestrator-workers, multi-agent

| Pattern | Shape | Use when | Classic mistake |
| --- | --- | --- | --- |
| **Prompt chaining** | call A → code check ("gate") → call B → … | a task decomposes into fixed steps and each step is easier than the whole (draft → critique → rewrite; extract → normalise → format) | no gate between steps, so an early hallucination is faithfully elaborated by every later step |
| **Routing** | a classifier call picks one of N specialised prompts/models/tools | inputs fall into distinct categories that want different handling (cheap model for easy tickets, strong one for hard; different retrieval per domain) | router trained/prompted on happy-path examples; the "other" bucket is where the real load lands |
| **Parallelisation** | *sectioning* (independent sub-tasks fanned out, results merged) or *voting* (same task N times, majority/consensus) | sub-tasks are independent, or you want confidence from diversity (N judges, N attempts at a bug) | merging without a conflict rule; assuming independent samples are independent when they share the same prompt bias |
| **ReAct** (Yao et al. 2022) | interleave *Reason* (a thought) → *Act* (a tool call) → *Observe* (tool result), loop until done | open-ended tasks needing tools mid-way — the base "agent loop" that function calling implements | no step budget; the model loops on a failing tool, or declares success without checking; observations dumped whole into context until it overflows |
| **Evaluator–optimizer** | generator produces, evaluator (a second call, a test, a compiler) scores/critiques, generator revises, repeat until pass or budget | there is a *checkable* quality criterion — code that must compile and pass tests, a translation, a proof | evaluator is the same model with the same blind spot; no termination bound; optimising for the evaluator rather than the goal (reward hacking) |
| **Plan-and-execute** | one call writes an explicit multi-step plan; an executor runs each step (often cheaper model/tools); replan on failure | long tasks where seeing the plan up front matters — for review, for cost, for parallelism | plan taken as fixed truth; step 4 discovers step 2's assumption was wrong and nothing replans |
| **Orchestrator–workers** | a central model decomposes the task *dynamically* (subtasks aren't known ahead), delegates each to a worker (own context, own tools), synthesises results | large heterogeneous tasks — a multi-file code change, a research question with several independent threads | workers lack context the orchestrator had (over-brief), or return raw dumps the orchestrator can't fit; no independent verification of workers' claims |
| **Multi-agent** | several peer agents with distinct roles/tools talking through a protocol (shared memory, message bus, hand-offs) | roles genuinely need separate contexts, permissions or models (a "coder" that can write, a "reviewer" that can only read) | agent count as the answer to a prompt-quality problem; cost and latency multiply; failure attribution becomes impossible without tracing |

Rules of thumb an engineer keeps in their head:

- **Structure beats autonomy when the task's shape is known.** A workflow you can unit-test
  step by step beats an agent you can only evaluate end-to-end. Autonomy earns its cost only
  when the number and order of steps genuinely depend on what is discovered on the way.
- **Every loop needs three bounds:** a step/token budget, a wall-clock or cost budget, and an
  *external* success check (a test passing, a schema validating, a number in range) — the
  model's own "I'm done" is not a stopping criterion.
- **Context is the scarcest resource.** Orchestrator–workers exists mainly to give each
  sub-task a fresh context; the trade is that the orchestrator must write a *brief* good
  enough for a stranger, and read back a *summary*, not the transcript.
- **Parallel workers must not share mutable state** without a merge rule — same as threads.

- **Interviewer's target sentence:** "Pick the pattern from the shape of the task: fixed steps
  → chain with gates; categories → route; independent parts → fan out and merge; unknown
  steps with tools → a ReAct loop with a step budget; a checkable criterion → generator plus
  evaluator; a big heterogeneous job → an orchestrator delegating to workers with their own
  contexts. Start with the least autonomous thing that works, and put a hard, external stopping
  condition on anything that loops."
- **`piros2` line:** not touched as code — the repo has no LLM calls. The Stock Market
  Analyser is where the ten patterns were built (say so, and let projects.md carry the
  detail). What the repo *shows* is the patterns applied *to* it from outside: `CLAUDE.md`
  is the standing brief an orchestrator hands its workers (this study file was itself
  written by a worker agent from a written brief), and its verification recipes
  (`just gate …` exiting 0/1, `just snap` writing files) are the external evaluators an
  evaluator–optimizer loop needs — the model proposes a fix, the gate scores it, the loop
  ends on PASS, not on the model's opinion.

## 2. Tool use and function calling

- **Mechanism.** The application declares tools as JSON-schema'd functions (name,
  description, typed parameters). The model, instead of (or as well as) writing prose,
  emits a structured *tool call*; the application executes it — the model never runs
  anything itself — and returns the result as a *tool result* message; the model continues.
  That request/execute/observe cycle is the ReAct loop made concrete. Multiple calls can be
  emitted in one turn (parallel tool use) when they are independent.
- **Design of a tool** is API design under a harsher client: the description *is* the
  documentation the model reads; parameter names carry meaning; errors must come back as
  informative text (the model will retry on "permission denied at /dev/video0" but loops on
  "error"); results should be *summarised or bounded* (a 10 MB log dumped into context is a
  bug). Prefer few, well-described, orthogonal tools over many overlapping ones; make the
  destructive ones explicit and confirmable.
- **Structured output** is the same machinery pointed at parsing: force a schema so the
  downstream code never regexes prose. **MCP (Model Context Protocol, 2024)** standardises
  how a tool server exposes tools/resources to any client — the "USB-C for tools" pitch;
  the practical win is that a ROS-graph inspector or a bag reader written once serves every
  agent front-end.
- **Safety model:** the tool boundary is where policy lives — allow-lists, sandboxes,
  read-only vs write tools, human confirmation on the irreversible. On a robot the tools an
  agent may hold are the ones a junior engineer may hold unsupervised: read topics, read
  logs, run a replay; *not* arm motors.
- **Classic mistakes:** tools that return unbounded output; vague descriptions ("does
  stuff with files"); letting the model fabricate a tool result when a call fails
  (always feed the *real* error back); no idempotency, so a retried call double-acts.
- **Interviewer's target sentence:** "Function calling is the model emitting a typed
  request that my code executes and answers; the quality of an agent is mostly the quality
  of its tools — clear schemas, bounded, honest results, and the destructive ones gated —
  because the model can only be as grounded as what its tools return."
- **`piros2` line:** not touched as code. Nearest thing: the repo's tools *for* an agent —
  the `just` recipes are the tool surface the coding agent actually used (`just status`,
  `just stragglers`, `just camera`, `just snap`, `just gate`), each with a name, a one-line
  doc, bounded output and a non-zero exit on failure, and the read-vs-write split is
  explicit (`camera` prints, `camera-reset` changes state). That is tool design even
  though the caller is a person or an agent, not an API.

## 3. Local model serving (Ollama) vs hosted APIs

| | Local (Ollama, llama.cpp, vLLM on your own GPU) | Hosted API (Anthropic, OpenAI, Bedrock, …) |
| --- | --- | --- |
| Data | never leaves the machine — the deciding factor for customer scan data, mine-site plans, defence | leaves the boundary; contractual/regional controls, sometimes a blocker |
| Capability | open-weight 7–70B models; the frontier lags hosted by months to a year on hard reasoning/coding | strongest available; long contexts; tool use tuned |
| Latency/throughput | bounded by your GPU: VRAM decides *which* model (a 7–8B model at 4-bit ~5 GB; ~70B needs ~40 GB) and batch size decides throughput | elastic; network round trip; rate limits |
| Cost shape | capex + electricity, flat; free per token once bought | opex per token; cheap to start, expensive at volume or long contexts |
| Ops | you own the serving stack, updates, quantisation choice, GPU drivers | none, but you own retries, backoff, outage plans, key management |
| Offline / edge | works in a tunnel, on a truck, on the drone's companion computer if small enough | needs connectivity — often *exactly* what a GPS-denied site lacks |

- **Ollama specifics worth knowing:** pulls GGUF-quantised weights, runs llama.cpp under a
  local HTTP API (default `localhost:11434`) that mimics common chat/embedding endpoints, so
  the same client code can point at local or hosted; quantisation (Q4/Q5/Q8) trades VRAM and
  speed for a small quality drop; on a 6 GB card an 8B Q4 model fits, a 13B does not
  comfortably.
- **The hybrid** is the usual answer: route (§1) — local for bulk, private, or offline
  work (embedding thousands of field reports, classifying logs); hosted for the hard
  reasoning step; and keep the abstraction so the choice is a config line.
- **Interviewer's target sentence:** "Local when the data can't leave or the site is
  offline, hosted when capability matters and volume is modest; in practice a router with a
  local default and a hosted escalation, behind one interface so the decision is
  configuration, not code."
- **`piros2` line:** not touched. The one honest adjacency: the repo runs a neural model
  *locally on the edge* — Depth Anything V2 Small as fp32 ONNX on the dev box's GTX 1660
  SUPER (6 GB) via `onnxruntime-gpu`, measured 72–79 ms/frame on CUDA against 280–305 ms on
  CPU, with the GPU path degrading to CPU *silently* if the nvidia pip libs are missing —
  the same VRAM-and-provider realities an Ollama deployment on that card would meet
  (docs/info/troubleshooting.md, "onnxruntime ignores the GPU and runs on CPU").

## 4. Retrieval and vector search

- **Why:** the model's weights don't know your documents, and its context window is finite
  and costs per token — so *retrieve* the relevant few chunks per query and put only those
  in the prompt (RAG, retrieval-augmented generation).
- **Pipeline:** chunk documents (by structure — headings, functions — not blindly by 500
  tokens; overlap chunks) → embed each chunk with an embedding model into a dense vector
  (hundreds to a few thousand dims) → store in a vector index → at query time embed the
  question, find nearest neighbours by cosine similarity → optionally **re-rank** with a
  cross-encoder → put the top-k in the prompt with citations.
- **Indexes:** exact brute force is fine to ~10⁵–10⁶ vectors; beyond that, approximate
  nearest neighbour — **HNSW** (graph, the default in pgvector/Qdrant/FAISS variants),
  **IVF** (cluster then search a few cells), product quantisation for memory. Recall vs
  latency is the dial. The same idea as a KD-tree/octree radius search in a point cloud —
  and, like a KD-tree, it degrades in very high dimensions, hence graph methods.
- **Hybrid search** (BM25 keyword + dense) beats either alone on technical corpora — part
  numbers, error strings and function names are exact tokens an embedding blurs. Metadata
  filters (date, robot serial, firmware version) narrow before similarity.
- **Evaluate retrieval separately from generation:** recall@k on a labelled question set;
  most "the model hallucinated" reports are actually "the right chunk wasn't retrieved".
- **Classic mistakes:** chunk boundaries that split a table from its caption; embedding
  the question and the answer with different models; no citations, so nobody can audit; a
  stale index after the docs change.
- **Interviewer's target sentence:** "Retrieval is how you ground the model in your own
  data without fine-tuning: chunk, embed, ANN-search, re-rank, cite — hybrid keyword-plus-
  dense for engineering text — and measure recall@k on its own before blaming the model."
- **`piros2` line:** not touched. The nearest structural relative in the repo is the
  keyframe store — descriptors (256-bit ORB) matched by Hamming nearest neighbour with a
  cross-check and a margin test to recognise a place — which is exactly a small
  binary-descriptor vector search with a verification step. Different vectors, same
  shape: index, nearest neighbour, then verify before you trust the match.

## 5. Durable long-running agent runs, cancellation

- **The problem:** an agent run is minutes to hours of tool calls, network calls, and
  waits; processes crash, tokens expire, the user changes their mind. A naïve loop held in
  one process's memory loses everything on the first failure.
- **Durable execution** = make the run *resumable*: persist the state after every step
  (the transcript, tool results, plan position) to a store; make each step **idempotent**
  or give it an idempotency key so replay doesn't double-act; treat model and tool calls as
  **checkpointed activities**. Frameworks: Temporal / Restate style workflow engines
  (replay a deterministic orchestrator against a journal of completed activities), queue +
  worker with a run table, or a simple event-sourced log. Long waits (human approval,
  overnight jobs) become *signals*, not blocked threads.
- **Cancellation** has to be *cooperative and propagated*: a cancel flag/token checked
  between steps, passed into child agents and tool calls (subprocess kill, HTTP abort),
  with a compensating action where a step had side effects (release the device, delete the
  half-written file). Distinguish *cancel* (stop, clean up) from *pause* (checkpoint, resume
  later). Timeouts are cancellation's cousin — every external call gets one.
- **Observability** is part of durability: a run ID, per-step spans, cost and token
  counters, so a stuck run can be found and killed and a finished one audited.
- **Classic mistakes:** state only in the prompt; retries without idempotency; child
  processes that outlive a cancelled parent; "cancel" that stops the loop but leaves the
  hardware held.
- **Interviewer's target sentence:** "Long runs must survive their own process: checkpoint
  after every step, make steps idempotent, and treat cancellation as a signal that
  propagates to every child and tool with clean-up — a cancelled run must leave no
  side-effect it doesn't own."
- **`piros2` line:** not touched as agent code — but the *cancellation discipline* is the
  most hard-won thing in the repo. The teardown contract (CLAUDE.md, Conventions): every
  session recipe runs its viewer in the foreground and a `trap … EXIT` that `pkill -f`s
  every node pattern it started, on both machines; `kill %N` was rejected after it orphaned
  the ros2-run grandchildren twice on 2026-07-27; ad-hoc runs must be bounded up front
  (`timeout -s INT 30 …`, `ssh pi 'timeout …'`); scripted `ssh pi` carries
  `-o BatchMode=yes -o ConnectTimeout=5` because a bare ssh hangs ~2 min against a dead
  link and wedges the trap; `just stragglers` is the both-host check that must print
  `clean` before results are reported; a leaked `usb_cam` holds `/dev/video0` and kills the
  next session with `Device or resource busy`. That is "cancellation must propagate to
  every child and release the resource" learned on hardware.

## 6. Evaluating agent output

- **Levels:** (1) *unit* — a single prompt/tool against fixed inputs with assertions on
  structure and content; (2) *end-to-end task success* on a labelled task set (did the bug
  get fixed, did the answer match, did the tests pass) — pass@1 / pass@k, cost and steps per
  task; (3) *online* — sampled production traces reviewed, user feedback, regressions
  tracked over model/prompt versions.
- **Graders, strongest first:** deterministic checks (schema, compile, unit tests, a number
  within a threshold, an exact-match answer); programmatic similarity (BLEU/ROUGE-style,
  embedding distance — weak); **LLM-as-judge** with a rubric, calibrated against a human-
  labelled subset (measure judge/human agreement before trusting it, and don't let the
  generator judge itself); human review for taste and safety.
- **Method:** hold out a fixed eval set; run it on every prompt/model/tool change like a
  test suite; version prompts; log traces so a failure can be replayed; watch *variance* —
  run several seeds, report distributions, not one lucky run. Reward hacking is real: an
  agent scored on "tests pass" will edit the tests unless the grader forbids it.
- **Classic mistakes:** evaluating on the examples used to write the prompt; a judge with
  the same blind spot as the generator; one run treated as evidence; no cost/latency
  budget in the score, so "better" is also 10× more expensive.
- **Interviewer's target sentence:** "Treat prompts and agents like code under test: a
  fixed eval set, deterministic graders wherever a check can be written, an LLM judge only
  where it's been calibrated against humans, and distributions over runs — and never let the
  thing being evaluated grade itself."
- **`piros2` line:** this is the item where the repo has real practice, just aimed at a
  robot pipeline instead of an LLM. The verification doctrine
  (docs/info/verification.md, built 2026-08-18) says exactly this: "gates are closed by
  scripts, not eyes"; a gate is a *number with a threshold compared against the pipeline's
  own earlier output*, a named bag to replay, or a rendered picture; `just gate flick|occlude`
  and the SLAM `gate-loop|gate-tum|gate-mesh|gate-map` exit 0/1; a "control" run
  (`gate-loop off`, no backend) is expected to FAIL; the first scripted gate found a real
  bug a human would have missed (a covered lens counted as healthy tracking — FAIL → fix →
  PASS, and toggling the fix off reproduces the failure), and non-determinism under load
  was *measured* (loop gap run-to-run 3.8–14.6 cm) rather than papered over. The same rules
  are what an eval harness for an agent needs.

## 7. AI coding agents in the development workflow, and where they fail

- **What they are:** an agent (§1's ReAct/orchestrator shape) whose tools are the developer's
  — read/edit files, run shell commands, grep, run tests, call sub-agents — driven from a
  terminal or IDE (Claude Code, Cursor, Copilot agent mode, Codex CLI, Aider). Given a
  goal, they explore the repo, change code, run the checks, and iterate; the strongest
  ones use the compiler/test suite as their evaluator (§6's evaluator–optimizer, with the
  toolchain as the evaluator).
- **Where they help, measurably:** boilerplate and glue (launch files, parameter YAML,
  test scaffolds); reading a large unfamiliar codebase fast; writing tests for existing
  behaviour; systematic refactors; drafting docs from code; turning a debugging session's
  findings into a recorded fix. On a robotics team: bag-analysis scripts, plotting, log
  parsers, CI wiring — the long tail nobody staffs.
- **Where they fail — the honest list:**
  1. **Confident claims without checking** — "the camera isn't detected" when the test
     ran without `v4l-utils` installed, or `ros2 topic list` run in a non-login shell on
     the wrong DDS domain reported as "no topics".
  2. **Not knowing what they don't know about the environment** — Wayland vs X11, a venv
     shadowing `python3`, a distro's renamed CLI (Humble forms in a Jazzy repo).
  3. **Side effects that outlive the task** — a background node or a camera launch started
     over SSH and never killed; a device left held; a persistent V4L2 control changed.
  4. **Over-building** — a whole subsystem when a small runnable step was asked; docs that
     describe code that doesn't exist yet as if it did.
  5. **Optimising for the check, not the goal** — weakening a test to make it pass,
     declaring "verified" from reading code rather than running it.
  6. **Context loss** — a long session forgets an earlier constraint; a sub-agent lacks the
     brief; the same mistake is repeated next session unless it is *written down where the
     next session reads it*.
  7. **Real-time and hardware blind spots** — no feel for latency, timing, or "the ROS
     window is a viewer, not the evidence".
- **What makes them safe and useful:** a *written contract the agent reads every session*
  (repo-level instructions with the environment's traps and the team's conventions);
  **verify-before-claim** rules with the exact command; a **teardown/cleanup contract**;
  gates a script can run; small reviewable steps; **memory of the model's own mistakes**
  fed back into the instructions the same day; and human review of anything with
  hardware or safety consequences. The engineer's job shifts from typing to specifying,
  reviewing and building the checks — which is why "can you write a gate a script can
  close" is a better interview question than "do you use Copilot".
- **Interviewer's target sentence:** "They are excellent at the long tail and at reading a
  codebase, and they fail by being confidently wrong about the environment, leaving side
  effects behind, and claiming verification they didn't do — so the workflow that works
  is a written repo contract they read every session, verify-before-claim rules with the
  actual commands, a cleanup contract, scripted gates, and feeding their own mistakes back
  into that contract the day they happen."
- **`piros2` line — held, and the strongest item in this section:** the repo was built with
  Claude Code as the daily workflow, and `CLAUDE.md` is a dated record of failure modes
  turned into rules. Verifiable examples in the file: *"verify claims about the hardware by
  running commands over SSH rather than assuming"*; *"don't report a camera command as
  failing before checking [v4l-utils]"*; *"`ssh pi 'ros2 topic list'` silently runs on
  domain 0 … don't report such a result as evidence"*; *"Ad-hoc background runs get the
  same teardown — this means you, Claude"* (bounded `timeout`, `pkill -f`, then
  `just stragglers` must print `clean` before reporting); *"Don't write docs or code that
  imply a package exists when it does not"*; *"the RViz window is a viewer, not the
  evidence"*; and two traps found *by* the agent on itself on 2026-08-18 — never put a
  node's source path on the same command line as a session recipe (the recipe's EXIT-trap
  `pkill -f` matches the agent's own shell and kills it with exit 144; "bit twice") and
  rviz2 sometimes needing two SIGTERMs. The mechanism for item 6 exists too: a persistent
  memory note (`feedback-verify-without-human.md`) records the user's 2026-08-18 request to
  close gates by capture/replay rather than "needs a human", and CLAUDE.md's Conventions
  now carry that as a rule. The measurable payoff of the discipline is the same day's
  result: a scripted gate found the blackout-isn't-loss bug in ninety seconds. Bring this
  to the room *as the answer to "where do they fail"* — it is specific, dated, and it
  shows the engineer supervising the tool rather than the reverse.

## What to say if asked "you list agentic engineering — what have you actually built?"

"Ten agent workflow patterns in a Stock Market Analyser — routing, chaining, parallel
fan-out, ReAct with tools, evaluator–optimizer, plan-and-execute, orchestrator–workers, and
multi-agent — as a hobby project (details in projects.md, not in `piros2`). Separately,
`piros2` — my ROS 2 hardware project — was built with an AI coding agent as the daily
workflow, and the interesting part is the failure log: it claimed hardware faults it hadn't
checked, ran discovery on the wrong DDS domain and reported 'no topics', left camera nodes
holding `/dev/video0`, and once killed its own shell with a cleanup trap. Each of those became
a written rule in the repo's agent contract, plus a verification layer where a script closes
the gate and exits 0/1. Nothing LLM-based runs on the robot itself, and I would not put an
agent in a real-time loop; I'd put it around one — log triage, replay analysis, test
generation — with bounded tools and the same verify-before-claim rules." Then stop.
