#!/usr/bin/env python3
from cereal import car, log
from common.params import Params
from common.realtime import Priority, config_realtime_process
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.longitudinal_planner import Planner
from selfdrive.controls.lib.lateral_planner import LateralPlanner
from selfdrive.hardware import TICI
import cereal.messaging as messaging


def _is_e2e_xstate(xs: int) -> bool:
  # e2eCruise(2), e2eStop(3), softHold(4), e2eCruisePrepare(5)
  return int(xs) in (2, 3, 4, 5)


def plannerd_thread(sm=None, pm=None):
  config_realtime_process(5 if TICI else 2, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  CP = car.CarParams.from_bytes(params.get("CarParams", block=True))
  cloudlog.info("plannerd got CarParams: %s", CP.carName)

  use_lanelines = not params.get_bool('EndToEndToggle')
  wide_camera = params.get_bool('EnableWideCamera') if TICI else False

  cloudlog.event("e2e mode", on=use_lanelines)

  longitudinal_planner = Planner(CP)
  lateral_planner = LateralPlanner(CP, use_lanelines=use_lanelines, wide_camera=wide_camera)

  if sm is None:
    sm = messaging.SubMaster(
      ['carState', 'controlsState', 'radarState', 'modelV2', 'carControl', 'longitudinalPlan', 'lateralPlan'],
      poll=['radarState', 'modelV2'],
      ignore_avg_freq=['radarState']
    )

  if pm is None:
    pm = messaging.PubMaster(['longitudinalPlan', 'lateralPlan'])

  while True:
    sm.update()

    if sm.updated['modelV2']:
      # lateral
      lateral_planner.update(sm)
      lateral_planner.publish(sm, pm)

      # longitudinal
      longitudinal_planner.update(sm)

      # ✅ planner에서 msg를 받아서 source를 덮어쓰기
      msg = longitudinal_planner.publish(sm, pm, return_msg=True)
      lp = msg.longitudinalPlan

      # ✅ 요청사항: "첫번째" = ACC/E2E만 구분해서 UI 표시용 source 강제
      if _is_e2e_xstate(int(lp.xState)):
        lp.longitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource.e2e
      else:
        lp.longitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource.cruise

      pm.send('longitudinalPlan', msg)


def main(sm=None, pm=None):
  plannerd_thread(sm, pm)


if __name__ == "__main__":
  main()
