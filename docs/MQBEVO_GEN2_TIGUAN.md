# Volkswagen Tiguan Mk3 (2025-26, MQB Evo Gen 2) — camera-integrated harness

This branch adds a native platform for the third-generation Tiguan and fixes the MQB Evo Gen 2
bring-up issues that made infiniteCable2 master unusable on it (`Unknown Vehicle Variant` = CAN
error) and made Intelligent Cruise Button Management walk the set speed down on its own.

## Install (Custom Software)

Repository: `https://github.com/alexandrupetraru/openpilot`, branch `tiguan-mk3`.
The `opendbc_repo` submodule points at `https://github.com/alexandrupetraru/opendbc` (same branch name).

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
