# CI/CD, deployment, infrastructure — the study file for section 13 of [emescent.md](emescent.md)

Written 2026-08-19. One section per checklist bullet: the concept, the sentence an
interviewer is fishing for, and an honest **`piros2` line**.

The honest headline for this section: **`piros2` has no CI service** — there is no
`.github/` directory in the repo, and the test suite runs locally through `just test` or the
VSCode Testing sidebar. What it *does* have is the deployment half done properly and by
hand: idempotent Ansible provisioning of two machines, a scripted rsync-and-build deploy to
the Pi, checksum-pinned model artefacts, a documented venv escape hatch, and a rejected-Docker
decision recorded with its reasons. So the answer to "tell me about your CI/CD" here is
"deployment yes, continuous integration no, and I know exactly which pieces are missing" —
which is a better answer than a badge in a README nobody reads.

Emesent relevance: the job ad names CI/regression, Docker, AWS and GitHub explicitly; their
product ships **firmware to a fleet** (Cortex 4.1.3, Commander 2.3 — versioned releases with
release notes, installed by customers over Wi-Fi in a mine), and their customers push
**10–50 GB scans** into desktop and now cloud software. That is the shape of the infrastructure
conversation they will want.

## Mental model: the path from a commit to a robot in a mine

```
commit → CI (build both arches, lint, unit tests, replay regressions) → artefact
      → release (versioned, signed, release-noted) → distribution (download / OTA)
      → install on device (A/B partition, verify, switch) → health check → rollback if bad
                                                          ↑
                                              telemetry/logs say which build is in the field
```

Every stage is a place a robot company gets hurt: a build that only compiles on x86, a
regression nobody ran because it needed hardware, an update that bricks a payload 600 m
underground, and — the one people forget — **not knowing which version is on which unit**.

## 1. GitHub Actions, runners, matrix builds

- **Model:** a workflow (`.github/workflows/*.yml`) triggers on events (`push`,
  `pull_request`, `schedule`, `workflow_dispatch`), runs **jobs** on **runners**
  (GitHub-hosted VMs or self-hosted machines you own), each job being a sequence of steps —
  shell commands or reusable **actions**. Jobs are parallel by default; `needs:` sequences
  them; `if:` gates them.
- **Matrix builds** expand one job definition across a parameter grid — for robotics
  typically `{ros_distro: [jazzy, rolling]} × {arch: [amd64, arm64]}` — with `fail-fast:
  false` so one red cell doesn't cancel the rest, and `include`/`exclude` for the awkward
  combinations. This is how you catch "works on the dev box, doesn't compile for the Pi".
- **Self-hosted runners** are the robotics-specific answer to two problems: hardware
  (a GPU for CUDA builds; a real device for HIL) and **arm64 native builds** — the
  alternative being QEMU emulation via `docker/setup-qemu-action`, which works and is
  roughly 5–10× slower for C++ compilation. Security caveat that interviewers like: never
  run self-hosted runners on public-repo pull requests without gating, because a PR can
  execute arbitrary code on your machine.
- **Concurrency and cost hygiene:** `concurrency:` groups to cancel superseded runs,
  `paths:` filters so a docs change doesn't rebuild the world, `timeout-minutes` on every job.
- **Interviewer's target sentence:** "Matrix over distro and architecture with fail-fast off,
  self-hosted or QEMU for arm64, and concurrency groups plus path filters so the queue stays
  honest — the point of the matrix in robotics is catching the cross-compile break before the
  robot does."
- **`piros2` line:** **not present — no `.github/` at all.** Tests run via `just test`
  (colcon test plus result aggregation) or the VSCode Testing sidebar, and both report
  identically because the accommodations were made deliberately (`.vscode/ros.env` for the
  ROS python path, `pytest.ini` for importlib mode and disabling launch_testing plugins, and
  per-package linter tests anchored on `__file__` rather than the CWD, because
  `ros2 pkg create`'s generated form is CWD-dependent). The suite is green at 199 tests and
  needs **no hardware and no model weights** — which is precisely what makes it CI-ready; the
  only missing piece is the workflow file and an arm64 runner or QEMU leg.

## 2. Build caching for C++

- **Why it dominates robotics CI:** a clean `colcon build` of a ROS 2 workspace with C++
  packages is minutes-to-tens-of-minutes; without caching every PR pays it, and people stop
  waiting for CI.
- **ccache / sccache:** hash the preprocessed source plus compiler flags, reuse the object
  file on a hit. `sccache` adds shared cloud storage (S3/GCS/Redis), which is what makes it
  work across ephemeral runners. Enable via `CMAKE_CXX_COMPILER_LAUNCHER=ccache`. Watch the
  hit rate — it is the metric; a hit rate near zero usually means the cache key includes
  something volatile (an absolute path, a timestamp, `__DATE__`).
- **Layered caching:** cache the *dependency* build separately from your own source
  (upstream workspace built once, overlay rebuilt per commit) — the ROS analogue of a Docker
  layer split. `colcon build --packages-up-to` and `--packages-select` keep incremental
  builds honest, and `colcon test --event-handlers console_direct+` keeps the output readable.
- **The other lever:** build in parallel but bound it (`--parallel-workers`, `MAKEFLAGS=-jN`)
  — CI runners with 2 cores and a `-j$(nproc)` that reports 32 is a classic OOM.
- **Interviewer's target sentence:** "ccache or sccache with a shared backend, keyed so the
  hit rate is actually high, plus splitting the dependency build from the overlay — otherwise
  C++ CI is slow enough that people route around it."
- **`piros2` line:** all packages are `ament_python`, so there is no compilation to cache —
  builds use `colcon build --symlink-install` specifically so Python and launch-file edits
  apply *without* a rebuild. The moment a package became `ament_cmake` this whole section
  would start mattering, and that is a fair thing to say rather than pretend otherwise.

## 3. Docker, multi-stage builds, image size

- **Multi-stage:** one stage with the toolchain and dev headers builds the artefacts; a
  final `FROM` a slim runtime base copies only what runs. For ROS: build in `ros:jazzy-ros-base`,
  ship from `ros-core` or a plain Ubuntu with only the runtime libraries, and drop the
  `build/`, `log/` and source trees.
- **Size levers, in order of payoff:** multi-stage; `--no-install-recommends` and cleaning
  `/var/lib/apt/lists` **in the same RUN layer** (otherwise the deleted files still occupy the
  layer); ordering layers so the volatile ones (your source) are last so the cache holds;
  `.dockerignore`. For robotics the real win is usually not shipping ROS desktop, rviz and
  every simulator into a payload image.
- **The robotics-specific pain:** devices (`--device /dev/video0`), networking (DDS multicast
  needs `--network host` or careful configuration — bridge networking is a classic reason
  nodes discover nothing across containers), GPU (`--gpus all` and matched driver/userspace
  versions), and real-time (`--cap-add SYS_NICE` for scheduling priorities). Containers
  isolate userspace, not the kernel — so RT tuning is still host business.
- **Multi-arch:** `docker buildx build --platform linux/amd64,linux/arm64` producing a
  manifest list, either on native runners or under QEMU.
- **Interviewer's target sentence:** "Multi-stage with a runtime base, clean apt in the same
  layer, volatile layers last — and in robotics the config that actually breaks is DDS
  discovery under bridge networking, device passthrough and GPU driver/userspace version
  match."
- **`piros2` line:** **Docker was considered for the Pi and explicitly rejected** — both
  machines run ROS 2 natively from apt, and CLAUDE.md carries a standing instruction not to
  reintroduce container instructions. The reasoning is the honest kind: for a two-machine
  learning setup where the point is understanding the stack, a container adds a discovery,
  device and GPU layer between you and every bug. I'd argue the opposite way for a fleet
  product, where reproducibility beats transparency — and being able to argue both directions
  is the actual answer.

## 4. Containerised development, devcontainers

- **What it buys:** one declared environment (`.devcontainer/devcontainer.json` + image),
  so a new engineer is productive in minutes and "works on my machine" stops being a
  category of bug. VS Code (and now others) attach into the container; extensions,
  toolchains and ROS live inside.
- **What it costs in robotics specifically:** GUI applications (rviz2, rqt) need X/Wayland
  forwarding or a VNC sidecar; hardware needs passthrough; DDS across the container boundary
  needs configuration; and GPU work needs the container toolkit plus a matching driver.
  These are all soluble and all fiddly, which is why plenty of robotics teams still develop
  natively and containerise only for CI and deployment.
- **The middle path many teams land on:** native development, containerised CI, containerised
  deployment — the container is a *distribution* mechanism rather than a development one.
- **Interviewer's target sentence:** "Devcontainers standardise the toolchain and are worth it
  for onboarding, but GUI, device and DDS passthrough are real friction, so a common split is
  develop native, build and ship in a container."
- **`piros2` line:** development is native on both machines by decision (above), and the
  environment is standardised a different way — **Ansible**, with six roles (`ros2_apt`,
  `ros2_install`, `ros2_env`, `camera`, `workspace`, `wifi`) and `site.yml`, idempotent to
  `changed=0` on a clean rerun, with machine-specific values in `group_vars` rather than
  hard-coded in roles. That is the same goal (declared, reproducible environment) reached
  with configuration management instead of images, and it is a legitimate answer for
  bare-metal fleets — a mine payload is not a Kubernetes pod.

## 5. Cross-compilation and toolchains

- **The three ways to get arm64 binaries:** (1) **build on the target** — simplest, slow,
  and fine when the target is a Pi 5 and the code is small; (2) **cross-compile** with a
  toolchain file and a **sysroot** (the target's headers and libraries, usually rsynced off a
  real device or built with a distro tool) — fast, and the setup is where the pain lives:
  every dependency must exist for the target, `pkg-config`/CMake must look in the sysroot not
  the host, and anything that runs a compiled tool during the build (code generators — and
  ROS 2 does generate code) needs a host build too; (3) **emulate** with QEMU binfmt inside a
  container — trivially correct, 5–10× slower.
- **CMake specifics worth knowing:** `CMAKE_TOOLCHAIN_FILE` setting `CMAKE_SYSTEM_NAME`,
  the compiler triple (`aarch64-linux-gnu-gcc`), `CMAKE_SYSROOT`, and
  `CMAKE_FIND_ROOT_PATH_MODE_{PROGRAM,LIBRARY,INCLUDE}` so `find_package` searches the
  sysroot for libraries and the host for programs. ROS 2's cross-compilation story
  historically involved `ros_cross_compile` (Docker + QEMU under the hood).
- **The related trap:** ABI. Mixing a library built with a different `_GLIBCXX_USE_CXX11_ABI`,
  a different libstdc++ version, or different `-march` flags produces link errors at best and
  silent misbehaviour at worst.
- **Interviewer's target sentence:** "Native-on-target for small workspaces, a sysroot-based
  toolchain when build time matters — remembering host tools for code generation — and QEMU
  when correctness matters more than speed; the recurring trap is `find_package` reaching into
  the host instead of the sysroot."
- **`piros2` line:** **option (1)** — the Pi builds its own copy. The repo lives at
  `~/Documents/piros2` on the dev box and `~/piros2` on the Pi, kept in step with an rsync
  that excludes `build`, `install`, `log` and the 99 MB depth-model directory (inference is
  dev-box-only — the Pi never needs the weights), and the Ansible `workspace` role does the
  same as part of a run. `colcon build` runs as the login user on each machine, never under
  `sudo`. Python-only packages make this cheap; with C++ nodes I would expect to need either
  a sysroot or a self-hosted arm64 runner.

## 6. Artifact and release management, versioning

- **What an artefact is:** a built, immutable, versioned thing you can point at — a `.deb`, a
  container image digest, a firmware `.bin`, a tarball, a model file. The property that
  matters is that it is **reproducible and identifiable**: given a version you can say
  exactly which commit it came from, and given a device you can say which version it runs.
- **Versioning:** semver for libraries and APIs (breaking.feature.fix); for a robot product
  what matters more is **compatibility matrices** — Cortex 4.x with Commander 2.x with Spot
  firmware 3.3.x and Hovermap 3.1+ is a real published example, and it is exactly the sort of
  thing that has to be encoded and checked at install time rather than lived in a support
  engineer's head. ROS 2 packages carry a version in `package.xml`; `bloom` + `catkin_pkg`
  turn them into distro releases.
- **Provenance:** build metadata baked in (git SHA, build date, distro), signed artefacts,
  an SBOM if you sell into regulated or defence customers — and Emesent now has defence
  customers, which raises this from hygiene to requirement.
- **Large binaries** do not belong in git: Git LFS, an artefact store (Artifactory, GitHub
  Releases, S3), or a fetch-with-checksum script. Model weights are the canonical example.
- **Interviewer's target sentence:** "Immutable versioned artefacts with the commit baked in,
  a compatibility matrix between the payload firmware and the apps that talk to it enforced at
  install time, and large binaries in an artefact store with a checksum rather than in git."
- **`piros2` line:** small but real and verified: **`just fetch-model` downloads the Depth
  Anything V2 Small ONNX weights and checks them against a pinned SHA-256** — it re-verifies
  an existing file, skips the download on a match, and `sha256sum --check`s after downloading
  — and the weights are git-ignored, as are `build/`, `install/`, `log/`, bags, `datasets/`,
  `captures/` and `meshes/`. That is the artefact rule in miniature: the code is in git, the
  99 MB binary is fetched and verified. There is no release process because there are no
  releases — the deploy unit is "the repo, rsynced".

## 7. Over-the-air and field update strategies

- **The constraint that shapes everything:** a failed update on a device you cannot reach is
  the worst outcome in the product. Underground, "cannot reach" is literal.
- **A/B (dual-bank) updates:** two system slots; write the inactive one, verify (checksum,
  signature), flip a boot flag, reboot into it. If the new slot fails a health check within a
  watchdog window, the bootloader falls back to the old slot. This is the standard for
  embedded Linux — Mender, RAUC, SWUpdate, Balena, or Android's update_engine.
- **Delta updates** to save bandwidth; **atomicity** so a power cut mid-write leaves a valid
  system; **signing** so a payload only accepts images from you; **staged rollout** (canary a
  small percentage, watch telemetry, widen) so a bad build hits ten units and not ten
  thousand.
- **Health checks are the crux:** an update is not "installed", it is "installed and proven
  working" — which requires the device to self-test after boot and *actively confirm*, with
  the bootloader treating silence as failure. A watchdog that reboots to the old slot on no
  confirmation is the whole safety net.
- **Application vs system updates:** shipping a container or a package is far less risky than
  a kernel/rootfs change; many robotics fleets split the two cadences deliberately.
- **Interviewer's target sentence:** "A/B slots with signed, atomic writes and an
  affirmative post-boot health check — silence means roll back — plus staged rollout, because
  the unrecoverable case is a device you can't physically reach."
- **`piros2` line:** no OTA — updates are `rsync` + `colcon build` over SSH, or an Ansible
  run. But the repo has a genuinely relevant **field-recovery** story: the Pi's Wi-Fi link died
  twice in two days while the OS ran on undisturbed, so the Ansible `wifi` role now installs
  an **escalation-ladder watchdog** on a 60-second timer — reassociate → reload the
  `brcmfmac` driver → reboot, the reboot **guarded by a 10-minute uptime floor and a 1-hour
  cooldown** so it cannot boot-loop — with `journalctl -t wifi-watchdog` as the flight
  recorder and thresholds in `group_vars`. It was drilled: the drill reproduced the AP's
  `status_code=16` rejection and recovered unaided at T+426 s **via the driver-reload rung**,
  proving reassociation alone measurably could not clear it. That is the same design
  vocabulary as OTA rollback — escalate, guard against loops, and prove the recovery path by
  exercising it rather than assuming it.

## 8. Rollback and recovery

- **The general principle:** every change needs an undo whose cost you know *before* you make
  the change. Rollback of code (previous artefact), of configuration (declarative, versioned),
  of data (backups, migrations that are reversible or additive), and of state (a robot that
  has physically moved cannot be rolled back — only recovered).
- **Forward-fix vs rollback:** rollback is preferred when the previous state is known-good and
  reachable; forward-fix when the change was one-way (a database migration, a filesystem
  format). Knowing which category you're in *before* deploying is the discipline.
- **The robotics-specific version** is failsafe design: return-to-home, rally points, safe-hold
  behaviours, and a manual override that always wins. Emesent's chain is a good example to cite
  — comms loss goes to a rally point, an autonomy error returns along the *previously flown
  safe path* rather than a straight line, and the RC's mode switch takes control back from the
  payload at any moment.
- **Recovery of *state*, not just software,** is where robotics diverges from web: the machine
  is somewhere, holding something, at some battery level. Recovery procedures have to be
  written for a physical world with a person in it.
- **Interviewer's target sentence:** "Know the undo before you ship the change; prefer rollback
  when the old state is reachable and forward-fix when it isn't — and on a robot the equivalent
  layer is failsafes: rally point, return along the flown path, and manual override that always
  wins."
- **`piros2` line:** the repo's rollbacks are git plus idempotent Ansible (rerun to converge),
  and its runtime recovery is deliberate: **sessions tear themselves down** — the viewer runs
  in the foreground so closing the window ends the recipe, and a `trap … EXIT` `pkill -f`s
  every node pattern the recipe started, which also covers Ctrl-C; `just stragglers` sweeps both
  machines and prints `clean` per host. The reason it is strict is a real failure mode: a leaked
  `usb_cam` holds the camera's exclusive capture and every later session dies with
  `Device or resource busy`. There is also a documented frozen fallback — `piros2_world` is kept
  as the known-good stack while all new work lands in the `piros2_world_mesh` fork, which is
  version-control-as-rollback at the package level.

## 9. AWS: EC2, S3, IAM, deployment patterns

- **EC2** — VMs; the robotics-relevant bits are instance families (GPU `g`/`p` for training and
  inference, `c` for compute, `m` for general), spot instances for batch processing at a
  fraction of the cost with the risk of interruption, and AMIs as the machine artefact.
- **S3** — object storage; the concepts that matter for spatial data are **storage classes**
  (Standard → Infrequent Access → Glacier tiers, with retrieval time and cost trade-offs),
  **lifecycle policies** to move old scans down automatically, **multipart upload** (essential
  for tens-of-GB objects, and it resumes), **presigned URLs** so a device or a customer uploads
  directly without credentials, and **transfer acceleration** for long-haul uploads. Egress is
  the cost that surprises people: storing a petabyte is cheap, shipping it out repeatedly is not.
- **IAM** — the model to be able to sketch: principals, policies (identity- vs resource-based),
  roles assumed by services, least privilege, and **instance profiles / IRSA** so nothing ever
  holds a long-lived access key. The single most valuable sentence: "devices get short-lived
  credentials via a role, never a baked-in key" (AWS IoT Core's credential provider exists
  exactly for this).
- **Deployment patterns:** blue/green and canary behind a load balancer; infrastructure as code
  (Terraform/CDK/CloudFormation) so the environment is reviewable and reproducible; queues
  (SQS) and events (S3 notifications → Lambda) for asynchronous processing pipelines.
- **Interviewer's target sentence:** "S3 with lifecycle policies and multipart/presigned uploads
  for the big scans, roles and short-lived credentials rather than keys on devices, infrastructure
  as code, and spot instances for the batch reconstruction work where interruption is cheap."
- **`piros2` line:** not used — everything here is two machines on a LAN, deliberately. The
  nearest thing is the deliberate **data-locality decision**: the Pi never receives the depth
  model (excluded from the rsync) because inference happens on the dev box's GPU; images cross
  the network compressed and exactly once (a relay fans the stream out locally) after raw
  streaming and per-subscriber unicast copies were measured saturating the Wi-Fi. That is the
  same reasoning cloud cost forces on you, applied to a 2.4 GHz link instead of an egress bill.

## 10. Cloud ingest of large spatial datasets

- **The scale to have in mind:** a Hovermap scan is roughly 1 GB per minute raw (a 10-minute
  scan produced over 11 GB, cut to ~1 GB by a 2024 compression change with a 44-second offload);
  raw captures run 10–50 GB and the device stores 512 GB. A mine site generates this weekly.
  Point clouds are the worst kind of big data: large, dense, and useless in a database row.
- **Ingest patterns:** upload direct to object storage with presigned multipart (never through
  an application server); checksum and manifest per upload so partial transfers are detectable;
  idempotent keys so a retry doesn't duplicate; a queue-triggered processing pipeline; and an
  intake that records provenance (device serial, firmware version, operator, site, time).
- **Serving them is a different problem from storing them:** clouds get converted into
  **streamable, level-of-detail** formats — COPC (Cloud Optimised Point Cloud, LAZ with an
  octree so a viewer can range-request only what it needs), Entwine/EPT, 3D Tiles, potree —
  so a browser can show a billion points without downloading them. That is the technology
  behind every "share a link to the point cloud" feature, including the Aura Cloud beta.
- **Formats and tooling:** LAS/LAZ (with PDAL as the swiss-army processing tool), E57 for
  scan-plus-imagery interchange, PLY for meshes. Compression and a spatial index are what turn
  a file into a service.
- **Edge decisions matter more than cloud ones:** what you upload at all. Onboard processing
  that produces a decimated preview for the field and defers the full cloud to Wi-Fi at the
  surface is a bandwidth strategy, not just a feature.
- **Interviewer's target sentence:** "Direct-to-object-storage multipart uploads with manifests
  and provenance, then convert to a level-of-detail format like COPC or 3D Tiles for serving —
  because storing a 30 GB cloud is easy and letting someone *look* at it is the actual product."
- **`piros2` line:** the volumes here are small (a 24 s bag ≈ 36 MiB MCAP; a saved mesh reached
  723k triangles) and everything stays local, but the *shape* is familiar: the offline
  reconstruction pipeline in `tools/recon/` converts bags into a TUM-layout keyframe export
  (CRC dup-skip, depth as 16-bit millimetre PNGs, poses in a separate rewritable
  `groundtruth.txt` so a better trajectory can be swapped in), and the outputs (`datasets/`,
  `captures/`, `meshes/`, `bags/`) are all git-ignored by policy. The decimation lesson is
  there too: the live mesher hard-caps triangles and *says in a warning what it dropped* rather
  than silently truncating — the same honesty a level-of-detail pipeline needs.

## 11. Git workflow, branching, code review practice

- **Branching models:** trunk-based (short-lived branches, merge daily, feature flags for
  unfinished work) versus GitFlow (long-lived develop/release/hotfix branches). Trunk-based
  wins for most product teams because long branches make integration pain quadratic; GitFlow's
  release branches still make sense when you must maintain multiple shipped firmware versions
  at once — which is the case for a hardware company with customers on older payload firmware.
- **Hygiene that reviewers actually notice:** small PRs (a reviewer's attention is roughly 200–400
  lines before quality collapses), one logical change per commit, imperative commit subjects that
  say *why*, a linear-ish history (rebase for cleanliness, merge commits for shared branches),
  and never rewriting published history.
- **Review as a practice, not a gate:** review for correctness, boundaries and maintainability,
  not style (automate style — clang-format, ruff, the ament linters, ideally in a pre-commit
  hook); ask questions rather than issue commands; approve with nits rather than blocking on
  taste; and reserve blocking for defects, missing tests and API decisions that will be
  expensive to undo. The reviewer's most valuable question in robotics is usually "how does this
  behave when the sensor stops?".
- **CI's role in review:** the machine checks what a machine can (build, lint, tests, coverage
  delta), so humans spend their attention on design. That division is the argument for having
  CI at all.
- **Interviewer's target sentence:** "Short-lived branches off trunk, small PRs with one logical
  change, style automated so review is about design and failure behaviour — and release branches
  only where you genuinely have to support multiple shipped firmware versions."
- **`piros2` line:** honest and small: this is a single-author learning repo, so there are no
  pull requests and no review process to describe — commits go to `main`. What it *does* have,
  and what is transferable, is written convention that survives contact: plans are structured as
  **stable phases** (P0, P1, … — once written, a phase's number and scope never change; progress
  is recorded by annotating it, so "P2" means the same thing in every commit message and doc),
  docs are split by kind with a plan's *location* encoding its status (`in-progress/` →
  `completed/`), and decisions are recorded with dates and rejected alternatives. That is the
  reviewable-artefact instinct applied to documentation instead of to PRs — see
  [17_software-engineering-practice.md](17_software-engineering-practice.md).

## What to say if asked "what does your CI/CD look like?"

"There isn't one — this is a single-author repo with no `.github/`, and I'd rather say that
than dress up a badge. What I did build is the deployment and reproducibility half:
Ansible provisions both machines idempotently from `group_vars` with no machine-specific
values in the roles, the workspace syncs and builds on the target, the 99 MB model is
fetched and SHA-256-verified rather than committed, and the risky runtime paths have real
recovery — a watchdog with an escalation ladder and anti-boot-loop guards, and session
teardown traps that stop orphaned nodes holding the camera. The suite is 199 tests that need
no hardware and no weights, so wiring it to Actions with an arm64 leg is a small job I
haven't done. If you want to know where I'd start on your side: making the replay regressions
run per-commit, because they're the ones that catch the failures a unit test can't."
