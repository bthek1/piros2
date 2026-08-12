# Wi-Fi watchdog plan — the Pi heals its own link

> **Done 2026-08-12 — planned, built and drilled in one day.** All five
> phases ran; every rung and both reboot guards have live journal
> evidence. The drill's headline: the mesh rejected rung 1's
> reassociation with `status_code=16` — **incident 1's signature,
> reproduced under control** — and rung 2's driver reload recovered
> the link unaided at T+426 s. The ladder's escalation is necessary,
> not paranoia. Per-phase annotations below are the build log.

> **Planned, written 2026-08-12.** Twice in two days the Pi became
> unreachable and stayed that way until someone power-cycled it. The
> journals show the OS never crashed either time — the Wi-Fi
> association died and could not recover on its own. This plan makes
> the Pi detect and repair its own link (an escalation-ladder watchdog,
> provisioned by Ansible), and makes both sides of the codebase honest
> about outages while one is in progress: fast failure, visible
> staleness, and no orphaned camera process holding the device.

## The incident record (why this exists)

Measured on 2026-08-12, from the Pi's own journals:

- **Incident 1** (recovered by reboot, Pi-time 2026-08-11 11:44): the
  mesh node `…:d6:83` rejected re-association **100 times**
  (`CTRL-EVENT-ASSOC-REJECT status_code=16`, `auth_failures=87`);
  wpa_supplicant looped through 5-minute `SSID-TEMP-DISABLED` cycles
  for hours. A reboot reconnected in seconds.
- **Incident 2** (recovered by power cycle, 2026-08-12 03:34): the
  link died *silently* ~10 minutes after the last camera session — no
  deauth, no driver error, just 15 hours of NTP timeouts — and never
  re-associated. The OS stayed healthy throughout: journal written
  continuously, load ~0.03, zero OOM, `throttled=0x0`.
- Signal is not the problem: RSSI −45 dBm, 433 Mbps link at the time
  of diagnosis. The suspect pairing is the Pi's `brcmfmac` driver ×
  mesh band-steering (`THEKKEL_MESH`), a known-bad combination; the
  same boot also logged `brcmf_set_channel … fail, reason -52` scan
  spam.
- Collateral on the dev box: the session trap's `ssh pi pkill` hung
  ~2 minutes on the dead host (fixed 2026-08-12 with `ConnectTimeout`),
  and the orphaned `usb_cam` kept the camera LED lit and `/dev/video0`
  held — the next launch died with usb_cam's `char*` abort.

## What already exists that this builds on

| Have | Where |
| --- | --- |
| Passwordless sudo on the Pi | needed by the watchdog and `wpa_cli` |
| Role/`group_vars` discipline, idempotent `site.yml` | `ansible/` (ansible plan, 2026-07-24) |
| Fast-fail session ssh (`ConnectTimeout=5`), local-first `world` trap | justfile (2026-08-12) |
| `just stragglers` aware of an unreachable Pi | justfile (2026-08-12) |
| STALE banners measured on the receipt clock | `dashboard` (world plan) |
| The incident evidence itself | journals, boots −2/−1; this file |

## The honest scope

- **This is a mitigation, not a cure.** The root cause lives between
  Broadcom firmware and the mesh's client steering; neither is ours to
  fix. An Ethernet cable remains the real fix and would obsolete P1–P2
  wholesale — this plan exists because the cable doesn't yet.
- **Network changes on the Pi are the riskiest edits in this repo**
  (CLAUDE.md): a bad one leaves the machine needing a keyboard and
  monitor. Every phase below states its rollback, and P1–P3 should be
  run while someone is physically near the Pi.
- **The watchdog cannot out-argue the AP.** Incident 1 shows the mesh
  rejecting the client until a fresh firmware session appeared; that is
  why the ladder ends in a (guarded) reboot rather than pretending
  `wpa_cli reassociate` will always do.
- **Outage behaviour is reap, not limp.** When the link dies
  mid-session, the Pi-side camera launch dies with its ssh (P3) —
  deterministic, no held device, camera LED off — and the operator
  reruns `just world` once the watchdog restores the link. That
  matches the repo's fail-loudly camera rules; a session that
  silently half-survives an outage is how the orphan leak happened.

## Changes at a glance

```
ansible/
├── roles/wifi/                          # NEW role, robot only
│   ├── tasks/main.yml                   # P1: iw + power-save unit; P2: watchdog
│   ├── templates/wlan0-powersave.service.j2   # P1
│   ├── templates/wifi-watchdog.sh.j2    # P2: the escalation ladder
│   ├── templates/wifi-watchdog.service.j2     # P2 (oneshot)
│   └── templates/wifi-watchdog.timer.j2 # P2 (every 60 s)
├── group_vars/robot.yml                 # P1/P2: wifi_* interface + thresholds
└── site.yml                             # role wired in after ros2_env
justfile                                 # P0: `just wifi`; P3: keepalive + -tt ssh
docs/info/networking.md                  # per-phase: incident record, design, drill results
docs/info/troubleshooting.md             # P4: new symptom entries
CLAUDE.md                                # P4: the Wi-Fi constraint bullet learns the truth
```

No `src/` package changes are expected: the ROS nodes already behave
correctly during an outage (RELIABLE subscriptions simply starve, the
dashboard banners it) — P3 *verifies* that claim with a drill instead
of assuming it.

## P0 — See the link before touching it ✓ (2026-08-12)

> Built as planned (one quoting fix: the gateway awk needed single
> quotes so the remote shell didn't eat `$3`). Healthy-link output
> verified live: COMPLETED, RSSI −47, 0% gateway loss, power-save
> honestly `unknown` until P1 — then `off` after it.

Visibility first, and a phase that changes nothing on the Pi. A new
status recipe:

- `just wifi` — one screen, over the existing fast-fail ssh:
  - association state + BSSID + frequency (`sudo wpa_cli -i wlan0
    status`),
  - signal and link rate (`sudo wpa_cli -i wlan0 signal_poll`),
  - this boot's damage count: `journalctl -b | grep -c ASSOC-REJECT`
    and the last `SSID-TEMP-DISABLED` line if any,
  - gateway reachability (`ping -c3` to the gateway from the Pi),
  - power-save state — `iw` isn't installed until P1, so print
    `power_save: unknown (iw not installed — P1)` rather than lying.
- Degrades honestly when the Pi is down: prints `pi unreachable`
  (same pattern as `stragglers`).
- networking.md gains a **Wi-Fi reliability** section opening with the
  incident record above — the doc is the durable home for it, this
  plan file is the working copy. *(Landed early, 2026-08-12, same day
  as the plan — with the CLAUDE.md constraint bullet and P4's two
  troubleshooting entries. P0 and P4 now only update those homes with
  what the phases measure.)*

**Runnable check:** `just wifi` on a healthy link shows COMPLETED,
RSSI ≈ −45, zero rejects; yank the Pi's power and it prints
`pi unreachable` instead of hanging.

The ROS lesson here is a systems lesson: the layers under DDS.
Discovery silence can mean domain mismatch, RMW mismatch, interface
pinning — or, as these two incidents prove, no link at all. `just
wifi` sits below `just status` in that debugging stack.

## P1 — Power-save off (the smallest network change first) ✓ (2026-08-12)

> Built as planned (`wifi-powersave.service`, not interface-named — the
> interface is templated inside). Deploy → `Power save: off`; rebooted
> the Pi → still off with the unit active. Persistence proven, not
> assumed.

Wi-Fi power-save is a documented contributor to silent brcmfmac drops
— exactly incident 2's signature (dead link, zero log lines, ARP
still answering occasionally). Low risk, possibly high reward, and a
rehearsal of the role plumbing before P2 does anything bolder:

- New Ansible role `wifi` (robot only), first tasks:
  - `iw` installed by apt (Ubuntu Server ships without it — same class
    of gap as `v4l-utils`, noted in CLAUDE.md).
  - `wlan0-powersave.service`: a oneshot after `network-online.target`
    running `iw dev {{ wifi_interface }} set power_save off`;
    `wifi_interface: wlan0` lives in `group_vars/robot.yml`, never
    hard-coded in the role.
- `just wifi` upgraded to print the real power-save state.

**Runnable check:** `just deploy-pi` (idempotent: second run
`changed=0`), then reboot the Pi and confirm `just wifi` shows
`power_save: off` — the reboot proves persistence, not just the
one-shot. **Rollback:** disable the unit; power-save returns on next
boot.

## P2 — The watchdog: an escalation ladder on a timer ✓ (2026-08-12)

> Built with two reality corrections the plan's draft got wrong: the
> wpa unit on this netplan/networkd system is
> **`netplan-wpa-wlan0.service`** (not `wpa_supplicant@wlan0`), and the
> Pi 5's driver is **`brcmfmac_wcc`** stacked on `brcmfmac` — rung 2
> removes both, then modprobes `brcmfmac_wcc` (which pulls the base).
> Force-tests all passed with journal evidence: rung 1 reassociated
> (ssh even survived the blip), rung 2 recovered in ~65 s with both
> modules back, rung 3 genuinely rebooted and left its marker; the
> uptime guard suppressed at 365 s < 600, the cooldown guard at
> 676 s < 3600. The real drill ran combined with P3's (see below) —
> recovery unaided at T+426 s, with the escalation genuinely needed.
>
> **Post-drill bug, caught by the closing `just wifi`:** rung 2's
> driver reload resets the radio's power-save to its default ON, and
> P1's oneshot only ran at boot. Fixed the systemd-idiomatic way —
> the unit is now `WantedBy=` the *device* unit (every `wlan0`
> appearance re-runs it; `RemainAfterExit` dropped so the re-trigger
> isn't swallowed) plus an explicit re-assert at the end of rung 2 —
> and regression-tested: power-save stays off across a forced driver
> reload. Always re-check assumptions the recovery path can undo.

The core of the plan. A systemd **timer** (not cron — journald
logging, dependency ordering, `systemctl list-timers` visibility; the
same reasoning the repo applies picking idiomatic tools) fires
`wifi-watchdog.service` every 60 s. The script pings the **gateway**
(reachability of the AP is the question — pinging the internet would
conflate an ISP outage with a link failure) and keeps a consecutive-
failure count in `/run` (tmpfs: state resets on boot, exactly right).

The ladder, each rung taken only after `wifi_fail_threshold` (default
3) consecutive failures at the rung below, all thresholds in
`group_vars/robot.yml`:

1. **Reassociate** — `networkctl reconfigure wlan0` and restart
   `wpa_supplicant@wlan0`. Clears wpa_supplicant's temp-disable/ignore
   list (incident 1's 5-minute doom loop) without touching the driver.
2. **Reload the driver** — `modprobe -r brcmfmac_wcc brcmfmac &&
   modprobe brcmfmac`. A fresh firmware session without a reboot; the
   strongest medicine short of one.
3. **Reboot** — guarded: never within 10 minutes of boot, at most one
   watchdog reboot per hour (marker file under `/var/lib`, which
   survives the reboot it causes — `/run` would not). Incidents 1 and
   2 both ended only at reboot; pretending rung 2 always suffices
   would be dishonest. The guard is what separates a watchdog from a
   boot loop.

Every action logs one loud line to the journal
(`logger -t wifi-watchdog`), so `journalctl -t wifi-watchdog` is the
flight recorder. A `WATCHDOG_FORCE_RUNG=<n>` env override lets each
rung be exercised on a healthy link without faking an outage.

**Runnable check:** deploy; `systemctl list-timers` shows the timer;
force each rung in order over ssh and watch the link drop and return
(rung 2 severs the ssh session — reconnect and read the journal;
that is the test passing, not a hang). Then the real drill:
`sudo systemctl stop wpa_supplicant@wlan0` from a screen/tmux-less
ssh, walk away, and verify by the journal that the watchdog restored
the link unaided within ~3 minutes. **Rollback:** `systemctl disable
--now wifi-watchdog.timer` — the ladder never fires again; nothing
else in the system depends on it.

## P3 — Both sides graceful through an outage ✓ (2026-08-12)

> All seven camera launchers now run
> `ssh -tt -o ServerAliveInterval=5 -o ServerAliveCountMax=3 … </dev/null`
> — the stdin redirect matters: it keeps ssh's tty games off the local
> terminal while `-tt` still forces the remote pty that makes sshd HUP
> the launch on disconnect. Mechanism pre-tested in isolation (remote
> survives normal operation; killed client → remote reaped), plus the
> sshd `ClientAlive 15×4` drop-in for silent link deaths. The combined
> drill, camera streaming through the new launcher: link killed at
> T+0 (`systemctl stop netplan-wpa-wlan0`), local ssh dead on
> keepalives in ~15 s ("Timeout, server 192.168.2.17 not responding"),
> camera **fully reaped** and `/dev/video0` free on recheck; watchdog
> counted 3 failures → rung 1 → the mesh rejected reassociation with
> `status_code=16` (incident 1's exact signature, reproduced) →
> failures 4–6 → rung 2 driver reload → **link back at T+426 s with no
> human involved**. Steps 1–3 of the drill script ran with a bare
> camera session rather than full `just world` (RViz adds nothing to
> the outage mechanics); dev-side nodes' STALE behaviour was already
> live-proven during the 2026-08-12 morning outage, when the world
> session ran against a dead Pi without crashing.

The dev box got its fast-fail half on 2026-08-12 (ConnectTimeout on
every session ssh, local-first `world` trap, reworded health-check
message). This phase finishes the job for a link that dies
*mid-session*:

- **Dev side notices within seconds, not TCP-forever:** the
  long-running camera ssh in session recipes gains
  `-o ServerAliveInterval=5 -o ServerAliveCountMax=3` — a dead link
  kills the ssh in ~15 s, the terminal says so, and the recipe's
  existing machinery does the rest.
- **Pi side reaps instead of leaking:** the camera ssh allocates a
  tty (`-tt`), so when the connection dies sshd HUPs the remote
  session and `ros2 launch` shuts the camera down — LED off, device
  released, no `char*` abort on the next launch. (If `-tt` garbles
  the launch logs unacceptably, the fallback is a stdin-EOF wrapper
  on the remote command; decide by trying, record the outcome here.)
- **The ROS layer needs no code change — prove it:** RELIABLE
  subscriptions starve silently by design; the dashboard's
  receipt-clock STALE banners are the honest indicator (never
  `header.stamp` — the 0.73 s fault). The drill verifies no dev-box
  node crashes or needs restarting across the outage window.

**Runnable check — the full outage drill,** run once end-to-end and
recorded in networking.md:

1. `just world`, streams live.
2. Kill the link (`sudo systemctl stop wpa_supplicant@wlan0` on the
   Pi).
3. Within ~15 s: dev terminal reports the camera ssh dead; within
   ~30 s the camera LED is **off** (the reap worked); dashboard panels
   go STALE; no dev-box node dies.
4. Within ~3 min: watchdog restores the link (journal shows the rung
   used); `just wifi` healthy again.
5. Rerun `just world` — camera comes up clean on the first try (no
   held device, no leftover state).

## P4 — Teach the docs what we learned ✓ (2026-08-12)

> Flips applied the same day: networking.md's mitigation list and the
> troubleshooting entries now describe the built watchdog (journal tag
> `wifi-watchdog`), CLAUDE.md's bullet carries the drill result, and
> ansible.md documents the `wifi` role. This file moved to
> `completed/` — the move is the status change.

> **2026-08-12, ahead of the build:** the two troubleshooting entries,
> the CLAUDE.md constraint bullet, and networking.md's incident-record
> section all landed the day the plan was written (they document what
> is already true, watchdog marked as planned). P4's remaining job is
> to update them once P1–P3 run: flip "planned" to built, add the
> drill results and the watchdog's journal tag.

- troubleshooting.md, two new symptom entries:
  - *"Pi unreachable but the camera LED is on"* → the link died, the
    OS didn't; check `journalctl -t wifi-watchdog` first, then the
    ASSOC-REJECT count; the LED means an orphaned pre-P3 session or
    (post-P3) a session whose reap failed — `just stragglers`.
  - *"usb_cam dies at startup with `terminate … instance of
    'char*'`"* → device already held (the orphan case) or transient
    post-boot state — `just camera`, then retry once.
- CLAUDE.md's "Pi is on Wi-Fi" constraint bullet gains the measured
  truth: the link self-heals via the watchdog since P2, outages
  reap the camera session since P3, and Ethernet is still the real
  fix when a cable reaches the shelf.
- networking.md's Wi-Fi reliability section closes with the drill
  results and the watchdog's tuning knobs.

**Runnable check:** none — this phase is prose; its gate is that every
claim in it cites a phase that ran.

## Out of scope, recorded so nobody wonders

- **Fixing the mesh.** Disabling band-steering for the Pi's MAC or
  pinning it to one node happens in the router UI, not this repo;
  worth trying, impossible to automate from here.
- **Ethernet.** The moment a cable exists, `eth0` gets a netplan
  entry with route priority over `wlan0` and P1–P2 become dormant
  insurance. Not planned because it needs hands, not code.
- **A second radio / USB Wi-Fi dongle.** More brcmfmac-class driver
  surface, not less.
- **LTE/other out-of-band management.** Out of all proportion to a
  learning project with the Pi ten steps away.
