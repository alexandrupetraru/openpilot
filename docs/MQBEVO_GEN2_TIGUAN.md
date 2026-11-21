# Volkswagen Tiguan Mk3 (2025-26, MQB Evo Gen 2) — camera-integrated harness

This branch adds a native platform for the third-generation Tiguan and fixes the MQB Evo Gen 2
bring-up issues that made infiniteCable2 master unusable on it (`Unknown Vehicle Variant` = CAN
error) and made Intelligent Cruise Button Management walk the set speed down on its own.

## Install (Custom Software)

Repository: `https://github.com/alexandrupetraru/openpilot`, branch `tiguan-mk3`.
The `opendbc_repo` submodule points at `https://github.com/alexandrupetraru/opendbc` (same branch name).

Branch `tiguan-mk3-jetlink` is `tiguan-mk3` plus Jetlink (below); its opendbc is the same `tiguan-mk3` commit.
`tiguan-mk3` stays as the fallback. Custom Software URL: `alexandrupetraru/tiguan-mk3-jetlink`.

## Jetlink: the large driving model on a connected computer or phone (branch `tiguan-mk3-jetlink`)

[Jetlink](https://github.com/zoompilot/jetlink) (v0.9.0, `jetlink_repo`) runs openpilot's large driving model on a
computer or phone on the comma's USB-C port. Off by default; with it off the branch behaves as `tiguan-mk3` (same
selfdrived output on replayed drives, same model output).

**Setup.** Settings → Models → **Jetlink**: *USB* for a Jetson Orin Nano Super, a Linux PC with an NVIDIA GPU, an
Apple-silicon Mac or an Android phone (experimental); *iPhone* for an iPhone/iPad with USB-C (TestFlight app, keep it
on screen and the phone unlocked). Change it only when parked. Turning it on turns ADB off (the link uses the port).
* Power the comma 3X from the car harness only, and the computer from its own supply: never let the host's port power
  the comma. A USB 3 cable (USB 2 is over the 50 ms frame budget on a Mac). Engine start and start-stop can brown a
  host out; the comma then simply falls back.
* Pick the big model in the Jetlink app / on the host. On a curvature-steered VW the model's request goes almost
  straight to the rack, so prefer one users rate as well-centred (Lebowski on a Mac/iPhone; a 766 MB model on a Jetson).

**Driving.** The big model swaps in only while openpilot and lateral (MADS) are fully off: when **Big Model Ready /
Re-engage to switch** shows, turn them off and on again; the second after a swap refuses engagement (a MADS enable in
it waits and resumes on its own), then a chime. If the link drops or lags while engaged (5 late frames in a row, more
than 20 in 10 s, a 0.2 s silence or modeld dropping frames), the built-in model takes over **on the same frame** and
**TAKE CONTROL: Big Model Lost / Using built-in model** warns for 5 s: nothing disengages, steering continues (its change
is rate-limited as always), and ICBM does not raise the set speed during the warning. The big model comes back only
after you disengage. **Smooth Big Model** (Settings → Models, shown with the link on) adds 0.1 s of curvature smoothing
for the big model only; leave it off unless the big model weaves on straight roads (Smooth Steer already smooths).

**Checkpoint drills** (empty straight road, hands on the wheel): unplug the cable while engaged; quit or lock the host
app while engaged; load the host until it lags. Each: the warning shows, steering carries on smoothly, nothing
disengages, the set speed is not raised during the warning.

## ICBM curve speed (branch `tiguan-mk3-jetlink`, 2026-11-21)

Decided on the owner's drives (7 drives, ~2 h, Cluj city and local roads up to 72 km/h), replayed closed loop:

* **SCC-Map camera gate** (stock ACC + ICBM only, `smart_cruise_control/camera_gate.py`). The map's slow points on
  junctions or wrong road matches were the main "slows down too much": 6 of 11 map slowdowns were on roads driven
  straight (down to the 20 km/h floor; three at the same spot near 46.7500, 23.516 in two drives). The camera's
  near-field read now vetoes a map target once its point is within ~3 s (4 s on the Jetlink big model) and the camera
  has seen a straight road there for 0.5 s; it lifts at once when the camera sees a bend. On the logs: all 9 straight-
  road map events vetoed, all 5 real map curves kept. In a replay of a phantom 100 m ahead at 54 km/h the set speed
  bottoms at 42 instead of 30 km/h; a real curve is followed exactly as before. Logged as
  `longitudinalPlanSP.smartCruiseControl.map.cameraVeto` / `targetDistance`.
* **Restore after a limiter: 1.5 s instead of 2 s, counted from the last drop.** Replayed, 1.5 s saves ~10 s of waiting
  under a higher target over the 2 h with no more +/- flicker bursts (1.0 s doubled them). The wait no longer restarts
  each time the set speed catches up with a target that keeps rising (a curve exit in small steps).
* **Not changed: SCC-Vision's 4 s deceleration lead.** Shortening it would have taken one logged city bend (R ~44 m,
  already 2.3 m/s^2) at ~2.7 m/s^2: the car reaches a lowered set speed late (the stock ACC lags), and the lead is what
  makes up for that. To be revisited with highway / country drives and with the big model.
* **Owner settings to try:** a livelier ACC driving profile in the car (the stock ACC accelerated at a median
  0.14 m/s^2 while ICBM raised the set speed); check the car's own predictive / curve ACC is not also active, as it
  stacks with SCC.

**Where it will not do what you might expect**
* A map phantom still *starts* a slowdown: its point is far when the map asks (~6-7 s at 50-60 km/h), beyond what the
  camera can confirm; the gate ends it once the point is near. A persistent phantom spot shows as a short dip.
* A trusted camera that misreads a real curve as straight inside its near field vetoes the map until it sees the bend
  (replayed: 2 km/h less slowdown when the bend is seen 2 s out). Vision's own curve slowdown and lateral control run
  on the same read.
* For 5 s after modeld starts or after any Jetlink swap / hand-back the gate never vetoes (the model's history is
  reset): phantoms then slow the car as before.
* After a Jetlink hand-back ICBM does not raise the set speed for 5 s (lowering still works); a pick-up due then
  waits for the warning to end.
* ICBM never presses below 7 km/h or at a standstill, never while you press the accelerator or for 1.5 s after a stalk
  press, and only with the stock ACC engaged: with MADS lateral and ACC off there is no curve slowdown at all.
* Pick-up stays bounded by ICBM's press rate (one press per 0.2 s, ~5 km/h per s) and the car's own ACC acceleration.
* Fast curves: vision sees a bend late at speed (the model reads far curvature low); map data is what slows early, and
  the gate leaves far map points alone.

## Vehicle selection

Settings → Vehicle → Volkswagen → **Volkswagen Tiguan 2025-26** (platform `VOLKSWAGEN_TIGUAN_MK3`).
Until the radar firmware below is added to the database the car is not auto-fingerprinted from
its VIN, so select it manually once; the selection persists across reboots (not across reinstalls).

## What changed and why

| Area | Problem on infiniteCable2 master | Fix |
|---|---|---|
| carstate | `SMLS_01` (stalk), `EA_01` (Emergency Assist) and the MADS `SMLS_01.FAS_Taster` read were unconditional. The CAN parser auto-subscribes every message it is asked for and a message that never arrives invalidates the parser → `canError` | reads gated on `MQB_EVO_GEN2` / `STOCK_EA_PRESENT`; `SMLS_01` not subscribed on Gen 2, blinkers from `Blinkmodi_02` |
| DBC (`vw_mqbevo_2024`) | `MEB_Side_Assist_01` carried a 2-bit `Motion_State` at bit 171 of a 16-byte message; `ESC_50` used the Gen 1 1-bit `Standstill` | layout aligned with the validated Gen 2 capture (`Standstill` bit 86; `ESC_50.Motion_State` 171/2) |
| ICBM button frame | `create_acc_buttons_control` named 5 signals and zeroed the other 12 (`GRA_Typ468` button-type coding, `GRA_TravelAssist`, …) | stock frame carried through; only `Setzen`/`Wiederaufnahme`/`Abbrechen` asserted; driver bits `Hoch`/`Runter`/`Zeitluecke` always cleared (never echoed to the ACC) |
| ICBM echo | on a camera harness openpilot reads its own transmitted `GRA_ACC_01` back and treated it as a driver set/resume press | carstate ignores set/resume bits for 5 frames after each own transmission |
| ICBM dispatch | `MQB_EVO` cars fell through to `mqbcan` | `(MEB \| MQB_EVO)` like the rest of the port |
| button timers | module-level `CRUISE_BUTTON_TIMER` dict was aliased by two classes; a timer whose release event is missed synthesised long-press speed changes forever | `.copy()`; timer dropped after 15 s |
| vehicle list | generic "MQBevo Gen 2" entries were hand-edited into `car_list.json` | generated from `platform_list.py` (test passes again) |

Panda safety was not modified. The bits it inspects in `GRA_ACC_01` (13 cancel, 16 set, 19 resume)
are set exactly as before; see `opendbc/car/volkswagen/tests/test_mqbevo_gen2.py::TestGraAccButtonFrame`.

## Findings from the first drive logs (2026-09-06)

* **ICBM walked the set speed down** because the stalk goes to the car, which applies its own step: on this
  Tiguan a `+` tap jumps to the next multiple of 10 km/h (21 → 40 in the log) while openpilot assumed +1 and
  then clipped to its 30 km/h floor. ICBM "corrected" the car back down to 30, one press at a time. openpilot
  now adopts the car's resulting set speed for 1.5 s after any driver button (`cruise_ext.py`,
  `sync_v_cruise_with_car`). Without a driver press openpilot stays authoritative, so limiters still work.
* **"Steering Fault May Be Imminent"** came from the HCA-status watchdog counting a harmless 10–20 ms
  `active → ready → active` blip that the EPS produces once per second (a stock camera 1 Hz message is
  forwarded just before it; steering resumes on the next frame). The watchdog now counts only excursions that
  are sustained (≥ 50 ms) or in rapid succession (< 250 ms apart), which still catches the flapping it was
  written for. Turning **Lane Assist off in the car** is still recommended on Gen 2: the stock request
  reaches the EPS over Ethernet and cannot be blocked by the harness.
* The car's ACC minimum set speed is 20 km/h, which is what `MAX 20` on the HUD shows at a standstill.
* Do not enable sunnypilot Longitudinal Control (alpha) on Gen 2: there are no decoded radar objects.

## Second update (2026-09-07): hold set speed, 20 km/h floor, route data on bus 0

* **ICBM: Hold Set Speed** (new toggle, on by default). ICBM tracks the set speed you selected, lowered only by
  Smart Cruise Control / Speed Limit Assist, and restores it afterwards. The stock radar ACC keeps its own
  acceleration and lead following; the car is never asked for more than you set. Previously ICBM chased the
  planner's MPC speed, which produced a dip after every `+`, a slow ramp, ±1–8 km/h hunting and vision-lead
  following through button presses. ICBM also pauses for 1.5 s after any stalk press and openpilot no longer
  adopts the car's reaction to ICBM's own presses.
* **ICBM minimum set speed 20 km/h on MQB Evo Gen 2** (Tiguan 2025-26, RS3 2026): the ACC accepts 20, so curve
  logic can act in tight city corners. Other cars keep 30.
* **VW route data (PSD_04/05/06) is read from bus 0** on camera-integrated cars. The car's own map delivers
  speed limits (50/30/20 zones in the drive logs) to the "Car" speed-limit source; camera sign recognition is
  not on CAN on Gen 2 and stays unavailable. Use **Car First** with OSM as fallback.

## Third update (2026-09-07): Speed Limit Assist as a cap under your set speed

On a stock-ACC car SLA used the "confirmation" flow: every new limit gave you 5 s to press `-`/`+`, otherwise it
went inactive, and any set-speed change also cancelled it (below 80 km/h a confirmation was always required).
When it did activate it replaced your set speed with the limit. Now, with ICBM, SLA works like the car's own
speed-limit response: your set speed is the ceiling, `limit + offset` is applied underneath it automatically, and
the target goes back up to your set speed (never above) when the limit rises. Curves from SCC Vision / Map are
applied on top (lowest wins). If no limit is known, the last known one keeps applying ("Auto adjusting to last speed limit").

### Verified against the 2026-09-07 drive logs (routes 9, a, b)

* Hold mode works as designed: openpilot's set speed stays where you put it, the car's set speed only moves when
  SCC Vision / Map lowers it (e.g. 70 -> 28 in a bend, then back to 70). The "Car" speed-limit source delivers
  50 / 30 / 20 zones from the car's map on bus 0.
* SLA confirmed broken as described: 70 set in a 50 zone for minutes with SLA `inactive`; a 30 zone at 70 km/h
  (route 9, 817 s) only produced a `preActive` prompt. Fixed by the cap mode above.
* Found and fixed from the same logs: ICBM kept pressing `+` (= RESUME on VW) while braking from 34 to 1 km/h
  (route a, 175-181 s). ICBM now never presses below 7 km/h or while the car reports standstill. Raising the set
  speed back up now waits 2 s after a limiter clears (no more 70 -> 54 -> 70 bursts when Vision flickers), lowering
  is immediate. Map "limits" of 5 km/h on parking areas are ignored by the cap (anything under 20 km/h).
* Note: the car reports `ESC_50.Motion_State` = 1 at standstill and 0 when moving; the code expects 3, so
  `cruiseState.standstill` is never set on this car. Left unchanged on purpose: a correct flag would also enable
  openpilot's automatic RESUME after a lead pulls away (controlsd), which is a behaviour change to decide on separately.

## Driver monitoring timings (2026-09-08)

Alert timers doubled by owner choice (`selfdrive/monitoring/policy.py`), balancing comfort against the upstream
EU-regulation margin. No alerts below 10 km/h as before.

| Policy | Alert 1 | Alert 2 (orange, chime) | Alert 3 (red, continuous) |
|---|---|---|---|
| Vision (face seen, looking away / eyes closed / phone) | 10 s (was 5) | 16 s (was 8) | 26 s (was 13) |
| Wheel-touch (no face, or model uncertain 10 s) | 10 s (was 5) | 30 s (was 15) | 50 s (was 25) |

Unchanged: recovery when you look back (proportionally the same), wheel touch resets in wheel-touch policy, red
escalation (5 s of ignored red -> force decel, ICBM walks the set speed to 20 km/h, steering assist suspended),
lockout after two reds or one ignored red (1 / 5 / 15 / 30 min). Side effect of the scaling: after an orange
"no face" alert, being visible refills the wheel-touch budget in ~16 s instead of ~8 s.

## Recommended settings for stock ACC + ICBM

* Cruise: ICBM **on**, Hold Set Speed **on**, Smart Cruise Control Vision on / Map off until Romania OSM is
  downloaded, Speed Limit **Warning** first; Customize Source → **Car First**; then Assist with Fixed +10.
* infiniteCable: "VW: Lateral Correction (Recommended)" on; the four "VW: Speed Limit …" toggles off
  until the car's reported limits have been checked in Info mode.
* sunnypilot Longitudinal Control (alpha) **off** — Gen 2 has no decoded radar objects.

## Verify on the car

1. Engage at 50 km/h with ICBM on and every limiter off: the set speed must hold.
2. Blinkers, blind-spot (side assist), cluster set-speed display, traffic-sign display in Info mode.
3. `Motor_54.Engine_On` is decoded from the Gen 1 Evo layout and is not yet confirmed on Gen 2. It only
   masks transient TSK faults while the engine is off; if ACC faults are ever hidden, report it.

## Enable VIN auto-fingerprinting

Capture the radar firmware once (device SSH):

```bash
cd /data/openpilot && python3 -c "
from openpilot.common.params import Params; from openpilot.cereal import messaging, car
cp = messaging.log_from_bytes(Params().get('CarParamsPersistent'), car.CarParams)
[print(fw.ecu, hex(fw.address), fw.fwVersion) for fw in cp.carFw]"
```

Add the `fwdRadar` (0x757) string to `CAR.VOLKSWAGEN_TIGUAN_MK3` in
`opendbc/car/volkswagen/fingerprints.py`. VIN `WVGZZZCT…` (WMI `WVG`, chassis `CT`) then matches without
manual selection; `test_vin_matching_selects_the_right_gen2_platform` already covers the VIN side.
