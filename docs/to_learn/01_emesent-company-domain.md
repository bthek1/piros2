# Emesent — company and product domain

**Section 1 of [emescent.md](emescent.md), researched from the web on 2026-08-18** (the day
before the 08-19 panel). Four parallel research passes, ~200 fetches; every fact cluster
carries its source and date. Where sources disagree, the discrepancy is stated rather than
smoothed over. Things that could not be verified are collected at the end — do not quote
them in the room.

**Read this file top-down once, then the "one-liners" at the end just before the panel.**

## The 60-second version

Emesent is a Brisbane robotics company (spun out of CSIRO Data61 in **Nov 2018**) that
sells **Hovermap** — a LiDAR SLAM payload that is *also an autopilot*: clip it to a drone
and the drone can hold position, avoid walls and fly itself beyond line of sight and beyond
radio range in places with no GPS (mine stopes, ore passes, tunnels). The same box scans
handheld, on a backpack, on a vehicle or LHD, and on Boston Dynamics Spot. Its SLAM is
CSIRO's **Wildcat** (continuous-time LiDAR-inertial); its onboard software is called
**Cortex**; the tablet app is **Commander**; desktop post-processing is **Aura** (with an
**Aura Cloud** sharing layer in beta since June 2026). Customers are mostly **underground
hard-rock mines** (200+ sites: BHP, Rio Tinto, Glencore, Barrick, Newmont, Northern
Star…), buying stope reconciliation, convergence monitoring and "don't send a person there"
inspection; **defence** and **AEC/geospatial** (the new **GX1** scanner, Feb 2026) are the
second and third legs. It raised **A$25M (≈US$17M) in July 2026** and won an
**A$2M CRC-P grant on 12 Aug 2026 for "Cortex AI"** — an open, hardware-agnostic autonomy
platform for GPS-denied robots. Team CSIRO Data61 (CSIRO + Emesent + Georgia Tech) was
**runner-up at the DARPA SubT Final, Sept 2021**, tied on points, lost on a tiebreak.

## 1. History: CSIRO Data61 spinout, 2018, what was commercialised

- **Founders:** Dr **Stefan Hraspspeebar** (CEO until Nov 2023, now Chief Strategy Officer) and
  Dr **Farid Kendoul** (CTO), after ~a decade in CSIRO's Robotics and Autonomous Systems
  Group (Data61). What was commercialised: the Hovermap payload — SLAM-based LiDAR mapping
  plus omnidirectional collision avoidance and GPS-denied flight, aimed first at underground
  mining. Sources: [Emesent team](https://www.emesent.com/our-team),
  [CSIRO Emesent page](https://www.csiro.au/en/research/technology-space/robotics/Emesent).
- **Seed, Nov 2018:** **A$3.5M** led by Main Sequence Ventures (manager of the CSIRO
  Innovation Fund) with mining executive Andy Greig; Archangel Ventures also named. The
  team went from 7 to ~25, then ~40 staff and 100+ units within 18 months. *The CSIRO
  overview page says "A$4.5M"; CSIRO's own 2018 news release and IM-Mining say A$3.5M —
  use A$3.5M.* [CSIRO news 2018-11](https://www.csiro.au/en/news/all/news/2018/november/underground-mines-drone-startup),
  [IM-Mining 2018-11-05](https://im-mining.com/2018/11/05/csiro-drone-autonomy-spin-emesent-finds-financial-backing/).
- **Series A, Feb 2022:** **US$23M (A$32M)**, oversubscribed, led by Perennial Partners
  with Tiger Global, TELUS Ventures, Main Sequence, Archangel. Headcount ~130, 300+
  customers in 40+ countries. [InnovationAus 2022-02-19](https://www.innovationaus.com/emesent-lands-32m-to-drive-autonomous-drone-tech/).
- **Jul 2026 raise:** **A$25M ≈ US$17M** — A$10M (US$7M) venture debt from the National
  Reconstruction Fund Corporation (its first deep-tech venture-debt deal) + A$15M (US$10M)
  equity from Main Sequence, QIC Ventures, Orion Resource Partners, Hostplus, NGS Super.
  Uses: Wacol (Brisbane) manufacturing expansion, **Cortex AI**, **Aura Cloud**.
  [Emesent PR 2026-07-08](https://www.emesent.com/news/emesent-secures-25-million-aud-to-accelerate-autonomous-intelligence-platform),
  [Geo Week News](https://www.geoweeknews.com/articles/emesent-secures-17-million-to-accelerate-autonomous-intelligence-platform/).
- **12 Aug 2026 (last week):** **A$2M CRC-P grant** for a A$5.1M project with QUT and EPE
  to build **Cortex AI** — an "open, modular, hardware-agnostic" autonomy platform for
  GPS-denied aerial and ground robots, 12+ early adopters; pitch line: building GPS-denied
  navigation from scratch takes "5+ years and several million dollars"; claimed
  GPS-denied-autonomy market "$9B by 2030".
  [IM-Mining 2026-08-12](https://im-mining.com/2026/08/12/emesent-wins-government-backing-for-open-modular-autonomy-platform-for-gps-denied-environments/).
  **This is the freshest thing to know walking in — the JD's "Cortex" is being turned into
  a product/platform, not just firmware.**
- **Not acquired.** No evidence of Emesent buying or being bought 2025–26; every source
  describes an independent private company raising money. If "acquisition" was heard, it
  was probably the NRFC deal or a distributor partnership.
- **DARPA Subterranean Challenge:** Team CSIRO Data61 = CSIRO + Emesent + Georgia Tech
  (+ BIA5 building robots) — one of 11 funded teams, the only Australian one. Drones were
  DJI M210s carrying Hovermap running Emesent's commercial AL2 autonomy; ground robots
  (BIA5 Titan tracked, Ghost Vision60 quadrupeds) carried the "CatPack"; **every platform
  ran Wildcat SLAM as a decentralised multi-agent SLAM** (drift ≪0.05% of distance in the
  Tunnel Circuit). Tunnel Circuit Aug 2019: 7 points; Urban Feb 2020: most-accurate
  artefact report; **Final, Louisville Mega Cavern, 21–24 Sept 2021: tied top score (23
  points) with CERBERUS, 2nd on tiebreak (46 s), US$1M prize**, six robots including two
  Hovermap drones. [arXiv 2104.09053](https://arxiv.org/abs/2104.09053),
  [CSIRO news Sept 2021](https://www.csiro.au/en/news/All/News/2021/September/Australia-claims-historic-top-two-spot-in-the-Robot-Olympics).
- **Timeline of milestones:** 2019 first product (retro-named Hovermap 100), Australian
  Good Design Award, world-first fully autonomous BVLOS underground flight 600 m below
  surface in WA · 2020-07 AL2 launched; DJI M300 plug-and-play · 2022-02 Hovermap ST ·
  early 2023 ST-X · 2024-02 Spot integration and Freefly Astro (NDAA-friendly US drone) ·
  2025-04 fully autonomous *exploration* (Cortex 4.0 / Commander 2.1) · 2025-09 Aura 2.0 ·
  2026-02 GX1 at Geo Week (won "Pitch the Press") · 2026-04 US Army xTech|Live winner
  (US$100k, one of 6 of 76; demo to 101st Airborne May 2026) and Teledyne FLIR Defense
  payload certification · 2026-06 Aura Cloud public beta · 2026-07 raise · 2026-08 CRC-P.
  [5-year post](https://www.emesent.com/news/2024/05/01/emesent-celebrates-5-years-of-autonomously-mapping-the-inaccessible),
  [xTech](https://www.emesent.com/news/us-army-selects-emesent-hovermap-among-six-winners-for-soldier-readiness-enhancement).
- **Scale and people (2026):** 200+ mine sites, 40+ countries, 45–50 resellers; **109
  staff in Australia (Jul 2026)**, ~130 total — flat since the 2022 Series A. HQ Milton,
  Brisbane + Wacol manufacturing; US office Littleton/Denver, CO; UK regional office.
  **Execs:** CEO **Charles Miller** (ex-COO, CEO since Nov 2023); Hrabar CSO; Kendoul CTO;
  Ewen Cameron CCO; David Nucifora CFO; **Tim Schier EVP Engineering & Product**; David
  Raffelt Director of Engineering; Ryan Palfrey Director Product Management. Board addition
  Christofer Catania Nov 2024. [team page](https://www.emesent.com/our-team).

## 2. Hovermap product line and what it does

| Model | Launched | LiDAR | Range | Points/s | Accuracy | Weight | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Hovermap (HF1 / "100") | 2019 | Velodyne VLP-16 Lite, 16 ch, dual return | 100 m | 300k | ±30 mm (±15 mm indoor post-processed) | 1.8 kg | DJI A3 drones (M600/M210); dropped from Cortex support Aug 2025 |
| Hovermap ST | Feb 2022 | 16 ch rotating, dual return | 0.5–100 m | 600k | ±20 mm | 1.6 kg | IP65, 512 GB, 360°×290° FOV; Automated Ground Control introduced |
| **Hovermap ST-X** | early 2023 | **32 ch** rotating, triple return (vendor undisclosed) | **0.5–300 m** | 640k single / **1.92M** triple | ±15 mm general, ±10 mm indoor/underground, ±5 mm change detection | 1.57 kg | IP65, 512 GB (~4 h), Class 1 eye-safe; ~4 h runtime; the current flagship |
| **Emesent GX1** | Feb 2026 | same-class 32 ch + **RTK GNSS + 4×20 MP cameras** | 300 m | 1.92M | **5–10 mm global** | — | handheld/backpack/pole/vehicle only — a surveyor product, not a drone payload |

Sources: [KB specs](https://knowledge.emesent.com/articles/hovermap-specifications),
[Hovermap series](https://www.emesent.com/emesent-product/hovermap-series/),
[HF1 sheet](https://monsenengineering.com/wp-content/uploads/sites/7/2020/07/Emesent-Hovermap-HF1-spec-sheet-2.pdf),
[GX1 launch](https://www.gpsworld.com/new-emesent-gx1-is-all-in-one-slam-lidar-rtk-and-360-imagery-scanner/).
Naming: "Hovermap 100" is real (the original); "Hovermap 200" does not exist; current names
are ST, ST-X (sometimes styled STX) and GX1. The ST-X LiDAR OEM is not published — its
spec fingerprint (640k/1.92M pts/s at 300 m, 32 ch) matches the Hesai XT32 family, but that
is an inference. IMU specs are not published; colourisation is a 360° camera add-on
(GoPro Hero/MAX clip-on, or GX1's built-in cameras).

- **What it does:** onboard real-time SLAM on the payload's embedded computer, live point
  cloud streamed to the tablet; post-processing/georeferencing in Aura. **Automated Ground
  Control** (auto-detect retroreflective targets, constellation-match to known coordinates)
  since ST. Drift figure on the spec sheet ±0.03%. The ST-X white paper (rev 1.1, Feb 2024)
  measured **6.7 mm 1σ / 13.9 mm 2σ vs a Leica RTC360** on a 2.5-min walk *without* GCPs,
  and states plainly that "like any SLAM system… accuracy is likely to decrease in less
  favourable environments and during longer transits".
  [white paper](https://frontierprecision.com/wp-content/uploads/2025/10/Frontier-Precision-Emesent-Hovermap-ST-X-Precision-and-Accuracy-White-Paper.pdf).
- **Autonomy levels** (Emesent's own naming): **AL1 (2018–19), now "Pilot Assist"** —
  GPS-denied position hold + velocity control + omnidirectional collision avoidance, within
  line of sight. **AL2 (Jul 2020), "Autonomy"** — beyond line of sight *and beyond comms*:
  tap waypoints on the live cloud, autonomous path planning, return home; the first
  plug-and-play GPS-denied BVLOS payload. **Explore (Cortex 4.0, Apr 2025)** — drag a 3D
  bounding box and it explores it, fitting gaps down to 2.4 m horizontal / 1.75 m vertical
  on M300/M350, budgeting battery. [Geo Week 2020-07](https://www.geoweeknews.com/news/emesent-achieves-level-2-autonomy-for-hovermap-powered-by-slam-technology),
  [mission modes KB](https://knowledge.emesent.com/articles/mission-modes).
- **Platforms:** DJI M300/M350 (fitting kit, RC Plus); Freefly Astro / Astro Max (US-made
  alternative to DJI, ST-X); Boston Dynamics **Spot** via a 5.9 kg forward cage powered from
  Spot's GXP 24 V port (Cortex 3.1+, Spot 3.3.x); vehicle/RTK mount; **LHD mount**
  (magnetic, on mine loaders); protective cage for lowering down voids; backpack; pole;
  handheld; Teledyne FLIR SkyRanger R70 / SkyRaider R80D / SUGV / MUVE R430 (defence, 2026).
  [platforms KB](https://knowledge.emesent.com/articles/hovermap-platforms),
  [Spot KB](https://knowledge.emesent.com/articles/spot2).
- **Pricing (reseller-listed only; Emesent says "request pricing"):** ST-X **CAD$68,600**
  (Candrone); rental US$4,000; plan tiers Core / Mapping / Autonomy — autonomy is a
  licensed feature upgrade. [Candrone](https://candrone.com/products/emesent-hovermap-st-x).
- **Software brands:** **Cortex** = onboard software (next section); **Commander** =
  Android tablet / DJI RC Plus / Freefly Pilot Pro mission app (2.3, Jun 2026: GX1 support,
  PPK, FPV/3PV viewports); **Aura** = Windows desktop post-processing (2.0 Sept 2025, 2.2
  Jun 2026); **Aura Cloud** = browser sharing (beta Jun 2026). "Aura Enterprise" — no such
  product found.

## 3. Cortex: software and control architecture, handheld through to autonomous

- **The definition, in Emesent's words** (Software Engineer, Capture team job ad, live on
  Lever Aug 2026): *"Cortex is the software and control architecture at the heart of every
  Emesent scanner — it's what takes you from handheld mapping all the way to autonomously
  controlling robotic systems connected to it."* Product page: pre-installed on every
  Hovermap, does live SLAM, 3D perception, path planning and control, auto-detects the
  platform it is mounted on (M300/M350, Astro, Spot, handheld/vehicle) and self-configures.
  Renamed "Emesent Cortex" at release 3.2.2 — before that it was just "Hovermap firmware".
  Also runs on the GX1. [job ad](https://jobs.lever.co/Emesent/5ad8355e-4b17-4344-a09d-d44c3a385662),
  [Cortex page](https://www.emesent.com/emesent-product/cortex/),
  [release notes](https://knowledge.emesent.com/articles/release-notes-combined).
- **Tech stack from the current job ad (verified Aug 2026):** C++14/17+ for
  "performance-critical, real-time robotics components"; Python 3 for tooling, test
  harnesses, analysis; "ROS or ROS 2 … in a development or production capacity"; "develop
  and extend our ROS-based architecture considering compute constraints of the hardware
  platform"; Linux; modules named: hardware interface, navigation, mapping, perception,
  localisation & sensor-fusion pipelines, a simulation pipeline (Gazebo / Isaac Sim
  nice-to-have); CI/regression, Docker, AWS, GitHub; "embedded or edge compute platforms".
  A 2022 ROS Discourse ad called it "our ROS based autonomy stack … high-level autonomy and
  lidar mapping for UAVs in GPS-denied environments". *Inference:* "ROS or ROS 2" plus
  2018-era CSIRO code means ROS 1 heritage with a migration in progress — a fair thing to
  ask about. **The compute module (Jetson vs x86) is not public — don't assert Jetson.**
  [ROS Discourse ad](https://discourse.openrobotics.org/t/software-and-embedded-systems-engineers-emesent-australia/25938).
- **SLAM lineage:** Wildcat — Ramezani, Khosoussi, Catt, Moghadam, Williams, Borges,
  Pauling, Kottege, *"Wildcat: Online Continuous-Time 3D Lidar-Inertial SLAM"*,
  [arXiv 2205.12595](https://arxiv.org/abs/2205.12595) (May 2022): continuous-time
  (spline) lidar-inertial odometry + pose-graph optimisation, single- and multi-agent, built
  for SubT. Companion: Hudson et al., *"Heterogeneous Ground and Air Platforms, Homogeneous
  Sensing"*, [arXiv 2104.09053](https://arxiv.org/abs/2104.09053). The KB says features must
  be within ~40 m; the manual warns "SLAM slip" kills position hold and return-to-home.
  [SLAM KB](https://knowledge.emesent.com/articles/emesent-slam). *(Section 2 of the
  syllabus — continuous-time SLAM — is the thing to hold a conversation about.)*
- **Control chain (user manual UM-024 rev 3.1, Dec 2025):**
  - *Mapping (AL0):* SLAM + recording only; any carrier, drone flown by DJI Pilot 2.
  - *Pilot Assist (AL1):* the LiDAR is integrated into the drone's flight control —
    position hold, velocity control, and **Shield**, a virtual elliptical bubble with
    configurable clearance that grows forward with speed and shrinks beside structures.
    Shield is passive (won't dodge moving objects).
  - *Autonomy (AL2):* beyond LOS and comms; waypoints on the live cloud or a bounding-box
    Explore zone; Cortex plans, explores, budgets battery, returns along the
    previously-flown safe path (RTH "may not be the shortest path"); a *rally point* is the
    fallback on error/comms loss. Navigation source auto-selects **SLAM → GPS → INS**
    (Cortex 3.0); INS-only tolerated ~10 s before RTH.
  - *Handover to the flight controller:* on M300/M350 Hovermap talks over the **DJI
    Onboard SDK** serial port (230400 baud, "Enable API Control"); the RC's flight-mode
    switch has a "Hovermap" position — flip out and DJI regains control (Atti mode without
    GPS). **Cortex issues velocity/position commands; DJI keeps the inner attitude loop.**
    Freefly Astro via Freefly's API (Cortex 3.2+).
  - Maps stored onboard, uploaded on return to Wi-Fi/Long Range Radio (~500 m).
  [manual](https://knowledge3.emesent.com/hubfs/UM-024%20-%20Emesent%20Hovermap%20User%20Manual%20-%203.1.pdf),
  [SDK activation](https://knowledge.emesent.com/articles/dji-m300-m350-and-hovermap-sdk-activation).
- **Onboard vs Aura processing (a real design trade-off to discuss):** onboard uses 0.1 m
  voxels, a subset of rings, no global optimisation — a 15-min mission in ~80 s vs ~1,200 s
  in Aura; median stope error <1.1 cm, **but a 22 km vehicle test drifted >90 m**. Aura's
  re-process adds global optimisation and non-rigid drift correction.
  [comparison KB](https://knowledge.emesent.com/aura-and-hovermap-onboard-processing-comparison).
- **Release cadence:** Cortex 4.1.3 (30 Mar 2026, SLAM-stability fix during autonomous
  flight); 4.1 + Commander 2.2 (Dec 2025); 4.0 + Commander 2.1 (Apr 2025, Explore, onboard
  processing); 3.4 (Feb 2025); Commander 2.3 (Jun 2026).
  [Commander notes](https://knowledge.emesent.com/emesent-commander-release-notes).
- **Cortex AI (Aug 2026):** the CRC-P-funded next step — same stack opened up as an OEM
  autonomy platform for other people's aerial and ground robots (see §1).

## 4. Handheld vs drone vs ground-robot mapping modes

Same payload, quick-release between carriers; the mode decides how much of Cortex is
engaged (mapping only → pilot assist → autonomy).

- **Handheld / backpack / pole / cage (Mapping mode):** hold still ~10 s at start (IMU
  init), move "slow and fluid"; cage lowered down shafts/ore passes on a winch (100 m raise
  in ~20 min; raises >1,000 m scanned on tether); pole for ceiling voids; GoPro for colour;
  Backpack RTK for georeferenced walking scans; Automated GCP targets. Uses: construction
  progress, scan-to-BIM, tunnels, plant interiors, forensics, abandoned workings.
- **Vehicle / LHD:** suction or magnetic mounts, keep <20 km/h, Vehicle RTK outdoors;
  **Hovermap LHD** — magnetic mount on load-haul-dump loaders with M12 mine-network
  power/data for near-real-time stope evaluation via onboard processing ("plug in, start,
  stop … anyone can do it" — Byrnecut / Mining Plus). Vehicle-mounted convergence scans of
  >200 m production drifts in minutes.
- **Drone (AL1/AL2/Explore):** <5 m/s; AL1 for near-structure inspection (bridges, towers,
  telecom); AL2 for stopes, ore passes (a 120 m descent flight, 2020), abandoned workings.
  Stope setup <5 min, ~5-min flight; results "75% faster" with onboard processing (Mount
  Isa, Glencore Kidd, Jul 2025); trans-continental remote AL2 demo South Africa→Australia
  (2020). [stope blog](https://www.emesent.com/blog/2025/07/11/transforming-underground-surveying-fully-autonomous-stope-mapping).
- **Spot / ground robot:** Spot Cage, official launch Geo Week Feb 2024. **Today Hovermap
  on Spot is mapping, not Cortex-driven autonomy** — the operator drives Spot or Spot replays
  an *Autowalk* mission with embedded start/stop-scan actions; "10× faster than TLS"; 500 m
  LRR teleop; an "Early Adopter Program" for advanced ground autonomy was announced.
  *Inference:* the SubT ground-robot autonomy exists in-house, but the shipped Spot product
  delegates navigation to Boston Dynamics — and Cortex AI is presumably where that changes.
  [Spot PR](https://emesent.com/2024/02/06/hovermap-integration-with-boston-dynamics-spot-robot/),
  [Spot page](https://www.emesent.com/emesent-product/high-speed-survey-with-boston-dynamics-spot/).
- **Which industry uses which:** mining → drone + LHD + cage; tunnels/civil → handheld,
  vehicle, drone AL1 (Brierley road tunnel: 1-hour night scan, 85% less inspection time;
  historic rail tunnel: >2,000 ft of inaccessible tunnel in one field day; culverts on Spot
  for Caltrans; the Silkyara tunnel-collapse rescue in India, Nov 2023, 41 workers); AEC →
  handheld/backpack/GX1; defence → drone + handheld + Teledyne FLIR ground robots. An
  independent Czech Technical University 120 m tunnel study ranked ST-X top for accuracy vs
  Trimble X7 / FARO / NavVis / GeoSLAM. [Coptrz on the CTU study](https://coptrz.com/blog/emesent-hovermap-st-x-surpasses-traditional-tls-in-independent-tunnel-survey/),
  [Caltrans](https://bostondynamics.com/case-studies/spot-inspects-confined-spaces-for-caltrans/),
  [Silkyara](https://emesent.com/2024/05/01/an-indian-tunnel-collapse-case-study/).

## 5. Mining and tunnel inspection: what customers buy and why

**The best single map of the domain is Emesent's own "20+ Ways" underground use-case paper
(Aug 2022)** — [PDF](https://emesent.com/wp-content/uploads/2022/08/20-Ways-UG-Mining-Use-Case.pdf).
Verified use cases from it and the case studies:

- **Development:** over/under-break vs design; development pickups and cut volumes
  (pre/post-blast scans → in-situ volume, bulked volume, bulking factor); **convergence
  monitoring** (changes >5 mm claimed detectable; repeat scans differenced into
  closure/expansion heat maps); **structure detection** (clouds into Maptek PointStudio /
  Sirovision / CloudCompare for dip, azimuth, persistence — BHP Olympic Dam picks the
  structures controlling final stope shape); shotcrete thickness (pre/post scan vs invoiced
  volume); heading re-entry after failure; rock-bolt inventory; falls of ground (LKAB Kiruna
  2019 seismic event: km of damaged drifts mapped in days, 30+ scans over 1.2 km).
- **Stopes — the CMS replacement:** stope shape and volume, reconciliation (tonnes, grade to
  correct ROM stockpile, end-of-month), blast performance over time, over/under-break
  back-analysis ("in a three-minute flight… over-break the legacy CMS could not see"),
  backfill height/volume (replaces bucket counts), statutory void models, brow deformation,
  **drawpoint hang-up inspection**. Numbers: traditional cavity monitoring system (CMS) up to
  ~3 h per stope vs ~15 min with Hovermap; setup + flight <10 min; CMS point density falls
  to single points/m² with range while Hovermap holds; >80 structures picked from one
  stope scan (peer-reviewed SAIMM 2020 paper — Jones, Sofonia, Canales, Hrabar, Kendoul,
  [scielo](https://scielo.org.za/scielo.php?script=sci_arttext&pid=S2225-62532020000100009)).
  Emesent still sells a **CMS-mount** for voids drones can't fly.
- **Vertical infrastructure:** vent raises (fly if >4 m diameter, else winch cage), ore-pass
  inspection (Petra Diamonds abandoned an ore pass rather than remediate: "That saved us
  millions"), raisebore as-builts (±50 mm heat maps), decommissioned shafts (Olympic Dam,
  40 m winch). Cost claim: raise scanning "<1% of raise-bore/orepass excavation cost".
- **Who buys:** mine surveyors ("we're not holding up the production team as long" —
  Senior Surveyor, Mount Isa), geotech teams (Kidd Operations, 3,000 m deep, high stress:
  structural analysis, convergence, ground-support design), drill-and-blast/geology for
  reconciliation, contractors (Byrnecut, Mining Plus). Named customers: BHP, Barrick
  (Kibali, Bulyanhulu), Rio Tinto (Argyle), Glencore (Mount Isa, Kidd), Northern Star,
  Newcrest, Newmont (Tanami), Anglo American, Evolution (Mungari — replaced CMS with
  Hovermap + Deswik, processing 30 → 5 min), LKAB.
  [mining page](https://www.emesent.com/industry/mining/),
  [LHD](https://www.emesent.com/emesent-product/hovermap-lhd/).
- **The dollar logic (inference, but the one to voice):** (a) production time — a stope
  brow tied up 3 h vs 15 min; (b) dilution/reconciliation — over-break is paying to process
  waste, under-break is lost ore, mis-assigned stockpile grade compounds; (c) capex
  avoidance — deciding to abandon a raise or ore pass on evidence; (d) safety/liability —
  statutory void models, no surveyor at an unsupported brow, no re-entry into failed
  headings. Emesent's own framing: "informed decision-making, improved safety, certainty and
  savings, savings on equipment and consultancy". **Emesent publishes no % dilution
  reduction — don't quote one.**
- **Beyond mining:** tunnels/civil (above); **defence** — US Army xTech|Live (Apr 2026),
  Teledyne FLIR certification, dismounted GPS-denied nav, CBRN, subterranean; **AEC/topo**
  via GX1; telecom/energy/forestry are marketed but had no dated case study with numbers —
  treat as secondary.

## 6. GPS-denied environments: what breaks, and why it matters commercially

- **Why GNSS fails:** it needs line of sight to ≥4 satellites; rock, floors, ship hulls
  block it outright; under canopy / urban canyon the signal is attenuated and reflected
  (**multipath**) so a fix is degraded or biased. Underground there is simply no signal.
- **Why a stock drone then can't even hover:** the flight controller fuses GNSS (position),
  magnetometer (heading), barometer (altitude) and a downward vision/ToF system. Underground:
  no GNSS; the **magnetometer is disturbed** by rebar, mesh, steel sets, machinery and
  magnetic ore ("compass error" → ATTI mode → the drone drifts); the barometer is fine but
  useless for XY; downward optical flow needs light and texture — dark, dusty, uniform rock
  kills it. Radio doesn't propagate through rock and the pilot can't see the aircraft, so
  manual BVLOS is impractical.
- **LiDAR SLAM as the position source:** Hovermap is "a LiDAR mapping payload but also an
  advanced autopilot": SLAM gives *relative* pose to the flight controller → position hold
  and velocity control with no GPS; layered above: omnidirectional collision avoidance
  (1 mm wires claimed), waypoints (AL2), then bounding-box exploration — all "beyond
  communications range … no additional infrastructure". Loop closure, GCPs and (GX1) RTK
  give the global tie. [autonomy page](https://www.emesent.com/autonomy).
- **Failure modes even for LiDAR SLAM (the interview-grade part):** geometric
  **degeneracy** in long uniform tunnels/corridors — scan matching is unconstrained along
  the tunnel axis and "fails silently"; mitigations: tightly coupled LiDAR-inertial odometry
  (the IMU carries the unconstrained axis), degeneracy detection (eigenvalue /
  optimisation-based), intensity features, dynamic-object filtering; plus dust/smoke (returns
  off particulates), glass and high-reflectivity surfaces (the SAIMM paper: survey spheres
  saturated the sensor), moving vehicles, and drift on long transits — mitigated by loop
  closure, control points/targets, RTK/PPK, Automated Ground Control. Emesent's own numbers:
  KB says features must be within ~40 m; onboard processing drifted >90 m over 22 km of
  vehicle travel; "SLAM slip" is a named failure that cancels position hold and RTH.
- **Why the market exists:** GPS-denied = underground mining, tunnels, indoor industrial,
  ship holds/tanks (Flyability's territory), defence subterranean/urban. Regulation: surface
  BVLOS is heavily gated; underground is private enclosed airspace with no third parties
  overhead — the practical constraint is capability, not the regulator (*inference*; Emesent
  still calls it RPAS and insists on training). Proof point: DARPA SubT (§1). Cortex AI's
  pitch quantifies the build-vs-buy: "5+ years and several million dollars" to build
  GPS-denied navigation from scratch. DJI dependence is a visible commercial risk — hence
  Freefly Astro, Teledyne FLIR, and the defence push.

## 7. Point cloud deliverables and downstream spatial data workflows

- **Formats:** ST-X spec lists `.las`, `.laz`, `.ply`, `.dxf`, `E57`, per-point attributes
  intensity, range, time, return number, ring number, RGB (optional). Aura writes
  full-resolution LAZ 1.4 plus an optional subsampled cloud, a **trajectory** file (XYZ),
  and a `.prj` (OGC WKT) when output is UTM/WGS84. PTS not mentioned anywhere.
  [specs](https://knowledge.emesent.com/articles/hovermap-specifications),
  [generating a cloud](https://knowledge.emesent.com/generating-a-point-cloud-in-emesent-aura).
- **Data volumes:** raw scans typically 10–50 GB; a 10-min scan produced >11 GB raw
  (~1.1 GB/min), cut to ~1 GB and a 44 s offload after the Sept 2024 firmware compression
  update; ST-X stores 512 GB (>4 h).
  [transfer speeds](https://emesent.com/2024/09/04/emesent-supercharges-hovermap-data-transfer-speeds/).
- **Aura features:** SLAM re-processing with auto-tuned parameters (reduces SLAM slip),
  **non-rigid drift correction**, non-rigid merge of multiple scans, surface noise reduction,
  colour-scale filtering (elevation/intensity/time/range), measurement, unlimited processing
  (no per-m² fees). Paid extensions: **360° Colorization** (auto people-masking),
  **Automated Ground Control**, **Change Detection & Convergence Monitoring** (mesh the
  reference scan, measure point-to-mesh distance of the later one).
  [Aura](https://www.emesent.com/emesent-product/emesent-aura/),
  [change detection](https://knowledge.emesent.com/change-monitoring-and-change-detection).
- **Georeferencing:** GCPs (≥3; CSV `ID,X,Y,Z,Radius`; auto-matched retroreflective 25/50 cm
  circular targets *or* user-selected paint marks/features), RTK (M300/M350), PPK (GX1
  integrated; ST-X with Emlid RS2/RS3, Trimble R10/R12/R980, Leica GS18 via RINEX). Rigid
  pre-alignment error reported, then SLAM correction; **check points** for independent
  validation; **PDF + CSV accuracy reports** with per-GCP error and RMS. This is what turns
  *relative* accuracy (the ±10 mm figure) into *absolute* accuracy on the mine grid.
  [GCP workflow](https://knowledge.emesent.com/articles/process-scan-data-with-gcps-p1-9).
- **Aura 2.0 (9 Sept 2025):** concurrent-user licensing (no dongle), user-selected targets,
  check-point validation, redesigned measurement tools, data-validation warnings; quotes from
  LKAB and Newmont Tanami. **Aura 2.2** (Jun 2026): PPK, GX1, Aura Cloud. **Aura Cloud
  (public beta 3 Jun 2026):** "first cloud-native layer" — browser-link sharing of clouds +
  360° panos to non-technical stakeholders; processing still happens in Aura, not the cloud.
  [Aura Cloud](https://www.emesent.com/news/emesent-launches-aura-cloud-3d-data-sharing-and-visualization-for-every-stakeholder-in-the-workflow).
- **Third-party integrations Emesent names:** mining — **Deswik** (partnership Jul 2020:
  Deswik.CAD / AdvSurvey → coordinate to mine grid via survey stations → solid → boolean
  out development → stope reconciliation; a co-developed Process Map does it "in minutes"),
  **Maptek** (PointStudio, Sirovision), **Pointerra 3D**, **Trimble Business Center**; AEC —
  TBC, Agisoft Metashape, Bentley ContextCapture/MicroStation, Cintoo Cloud, Pointerra,
  PointCab, Prevu3D, Autodesk Revit (scan-to-BIM guides). Not on Emesent's lists but reached
  via LAS/E57: Micromine, Surpac, Leapfrog, Vulcan, ReCap/Navisworks, CloudCompare, Cyclone,
  ArcGIS/QGIS. [mining software](https://www.emesent.com/compatible-software-for-mining),
  [Deswik blog](https://www.emesent.com/blog/2021/02/02/processing-your-hovermap-data-in-deswik),
  [AEC software](https://emesent.com/compatible-software-for-aec).
- **The typical workflow, end to end:** capture (handheld / drone AL0–AL2 / Spot / vehicle)
  → offload → Aura SLAM re-process (+ non-rigid drift correction; loop closure is what makes
  relative accuracy hold) → georeference to mine grid / site datum via GCP / RTK / PPK (RMS +
  check points in the report) → clean / subsample / colourise → export LAZ / E57 → Deswik /
  Maptek / TBC / Revit for solids, volumes, CMS-style reconciliation, change detection →
  share via Aura Cloud / Pointerra / Cintoo.

## 8. Competitors: NavVis, Leica BLK, Exyn, Flyability (and the rest)

| | Who | Flagship kit | Positioning vs Emesent | Recent |
| --- | --- | --- | --- | --- |
| **NavVis** | private, Munich; CEO Felix Reinshagen | VLX 3 (2×32-layer, 5 mm, 8.5 kg), MLX handheld (Sept 2024, ~£30.9k), IVION cloud | indoor/industrial "spatial twin" data for AEC and factories — no autonomy, no mining | **US$85M Series D, 6 Aug 2026** (The Jordan Company); 1,500+ customers, >1 bn m² captured 2025 |
| **Leica BLK (Hexagon)** | Leica Geosystems, Heerbrugg | BLK2GO handheld (GrandSLAM), BLK2GO PULSE (Oct 2023), BLK360 G2, **BLK ARC** (Spot payload; first certified reality-capture payload for Spot, Sept 2024), **BLK2FLY** (Sept 2021, "first autonomous flying laser scanner", GNSS/outdoor-oriented) | AEC ecosystem breadth (Cyclone, Reality Cloud Studio, HxDR); not underground BVLOS | — |
| **Exyn Technologies** | Philadelphia, est. 2014, UPenn GRASP; **IPO'd on Nasdaq 15 May 2026** (EXYN, ~US$19.4M gross) — *not acquired* | **Nexys** modular payload (handheld/backpack/aerial/ground), ExynAero drone, ExynPak, ExynAI; self-defined **"Level 4A" autonomy** (Apr 2021: set a volume, drone explores at 2+ m/s) | the closest analogue — autonomous GPS-denied aerial mapping for mining — but tiny: Q1 2026 revenue US$1.19M, net loss US$3.24M, US$7.4M cash | Exyn Defense subsidiary Jun 2026 |
| **Flyability** | Lausanne, private (Series C CHF 15M, 2022) | **Elios 3** caged drone (Ouster OS0-32; **Surveying payload** OS0-128 + FARO Connect, ±6 mm, Oct 2023), FlyAware SLAM, Inspector / Flyability Cloud, UT and RAD payloads | confined-space *inspection* (tanks, boilers, sewers, ship holds), pilot-flown, not autonomous mapping | 2025: +50% battery, tether, Smart RTH; ABS class approval; JLR warehouse pilot 2026 |
| **FARO / GeoSLAM** | FARO bought GeoSLAM Sept 2022 (£22M+); **AMETEK bought FARO** (Jul 2025, ~US$920M) | ZEB Horizon → **FARO Orbis** hybrid SLAM + flash (Hesai XT-32; 5 mm SLAM / 2 mm flash) | handheld survey; no autonomy | — |
| **Kaarta** | Pittsburgh, CMU LOAM lineage | Stencil, Contour | private, ~US$8M raised, quiet | — |
| **Skydio X10** | US | visual-inertial GPS-denied flight, NightSense | camera-based inspection drone, not a survey LiDAR mapper | — |

Sources: [NavVis Series D](https://www.navvis.com/blog/navvis-raises-usd-85m-series-d-round-to-supply-the-data-foundation-for-physical-ai),
[NavVis device matrix](https://knowledge.navvis.com/docs/device-comparison-matrix),
[BLK ARC on Spot](https://leica-geosystems.com/en-us/about-us/news-room/news-overview/2024/09/leica-blk-arc-now-the-first-certified-reality-capture-device-available-for-boston-dynamics-spot),
[Exyn Q1 2026](https://www.globenewswire.com/news-release/2026/07/06/3322787/0/en/Exyn-Technologies-Reports-First-Quarter-2026-Financial-Results-Successfully-Completed-Initial-Public-Offering-and-Continued-Expansion-of-ExynAI-Across-Commercial-and-Government-Mar.html),
[Exyn AL4](https://www.exyn.com/news/exyn-drones-achieve-autonomy-level-4),
[Flyability 2025 review](https://www.flyability.com/blog/2025-review-flyability),
[Elios 3 surveying](https://www.flyability.com/news/elios-3-surveying-payload),
[AMETEK–FARO](https://www.privsource.com/acquisitions/deal/ametek-acquires-faro-technologies-x7S8Ye).

**How Emesent differentiates (grounded in the above):**
1. **Autonomy beyond comms range in unmapped underground space** (AL2 waypoints, Explore,
   omnidirectional avoidance, RTH along the flown path) — a decade of CSIRO research and the
   SubT runner-up; NavVis/Leica don't do it, Flyability is pilot-flown, Exyn does but is
   ~1/20th the size.
2. **Mining heritage:** 200+ sites, CMS replacement, Deswik/Maptek workflows, convergence
   monitoring — versus NavVis/Leica's AEC-indoor focus and Flyability's inspection focus.
3. **One sensor-agnostic payload** across DJI / Freefly / Spot / vehicle / LHD / cage /
   handheld, now defence-certified (Teledyne FLIR).
4. **A vertical software stack** Commander → Cortex → Aura → Aura Cloud with GCP/RTK/PPK and
   accuracy reports built in — and, from Aug 2026, Cortex AI as an OEM autonomy platform.

## Things that could NOT be verified — don't say them

- Tenacious Ventures as an investor (no source at all).
- Any acquisition of or by Emesent.
- The ST-X LiDAR vendor (Hesai XT32 is a spec-fingerprint inference).
- The onboard compute module (Jetson vs x86).
- "Aura Enterprise" as a product.
- "Hovermap 200".
- "Three 2025 Mining Technology Excellence Awards" (Tracxn only; the confirmed 2025 item is
  Prospect Awards finalist, IIoT category).
- Any published % dilution reduction; NavVis VLX 3 "US$100k" and Flyability "US$25k" (third-
  party price guesses).
- Whether Cortex is on ROS 1 or ROS 2 in production (the ad says "ROS or ROS 2").

## One-liners for the room

- "SLAM is the GPS replacement for the autopilot, not just a mapping output — Cortex hands
  velocity commands to DJI's inner loop over the Onboard SDK."
- "The customer buys minutes at the brow and confidence in the reconciliation, not point
  clouds — 3 hours of CMS became 15 minutes."
- "Long straight tunnels are the classic degenerate case for scan matching — the IMU carries
  the unconstrained axis; I'd want to know how Cortex detects degeneracy."
- "Onboard processing gives you a stope in 80 seconds at <1.1 cm, but 90 m of drift over
  22 km — the local-vs-global optimisation trade-off is where Aura earns its keep."
- "Wildcat is continuous-time — a spline trajectory rather than discrete keyframes — which is
  what lets one SLAM run on a spinning-LiDAR drone and a walking Spot alike."
- "Cortex AI is last week's news: an A$2M CRC-P to open the stack to other people's robots
  — the JD's Cortex is becoming a platform."
- "Exyn is the nearest competitor and just IPO'd at ~US$5M/yr revenue; NavVis raised
  US$85M two weeks ago but for indoor twins, not autonomy."
- Honest boundary (per emescent.md's rule), **updated 2026-08-19**: `piros2`'s fork now *is*
  SLAM — always-on loop detection, a hand-written SE(3) pose-graph backend owning `map → odom`
  checked against `g2o`, a TSDF that rebuilds when the graph moves, a persistent graph — gated
  against RTAB-Map and TUM ground truth ([02_SLAM.md](02_SLAM.md) has the numbers). Say it as
  "monocular RGB-D-style SLAM in one room, hand-written backend", and say the limits in the same
  breath: mono depth scaled by tape measure, no IMU, no LiDAR, a palindrome bag rather than a
  walked loop.
