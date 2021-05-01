from collections import namedtuple
from cereal import car
from common.realtime import DT_CTRL
from selfdrive.controls.lib.drive_helpers import rate_limit
from common.numpy_fast import clip, interp
from selfdrive.car import create_gas_command
from selfdrive.car.honda import hondacan
from selfdrive.car.honda.values import CruiseButtons, CAR, VISUAL_HUD, HONDA_BOSCH, CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.kegman_conf import kegman_conf
kegman = kegman_conf()

VisualAlert = car.CarControl.HUDControl.VisualAlert

def actuator_hystereses(brake, braking, brake_steady, v_ego, car_fingerprint):
  # hyst params
  brake_hyst_on = 0.02     # to activate brakes exceed this value
  brake_hyst_off = 0.005   # to deactivate brakes below this value
  brake_hyst_gap = 0.01    # don't change brake command for small oscillations within this value

  #*** hysteresis logic to avoid brake blinking. go above 0.1 to trigger
  if (brake < brake_hyst_on and not braking) or brake < brake_hyst_off:
    brake = 0.
  braking = brake > 0.

  # for small brake oscillations within brake_hyst_gap, don't change the brake command
  if brake == 0.:
    brake_steady = 0.
  elif brake > brake_steady + brake_hyst_gap:
    brake_steady = brake - brake_hyst_gap
  elif brake < brake_steady - brake_hyst_gap:
    brake_steady = brake + brake_hyst_gap
  brake = brake_steady

  if (car_fingerprint in (CAR.ACURA_ILX, CAR.CRV, CAR.CRV_EU)) and brake > 0.0:
    brake += 0.15

  return brake, braking, brake_steady


def brake_pump_hysteresis(apply_brake, apply_brake_last, last_pump_on_state, ts):
  # If calling for more brake, turn on the pump
  if (apply_brake > apply_brake_last):
    pump_on = True
  
  # if calling for the same brake, leave the pump alone. It was either turned on 
  # previously while braking, or it was turned off previously when apply_brake
  # dropped below the last value. In either case, leave it as-is.
  # Necessary because when OP is lifting its foot off the brake, we'll come in here
  # twice with the same brake value due to the timing.
  if (apply_brake == apply_brake_last):
    pump_on = last_pump_on_state

  if (apply_brake < apply_brake_last):
    pump_on = False

  last_pump_on_state = pump_on

  return pump_on, last_pump_on_state


def process_hud_alert(hud_alert):
  # initialize to no alert
  fcw_display = 0
  steer_required = 0
  acc_alert = 0

  # priority is: FCW, steer required, all others
  if hud_alert == VisualAlert.fcw:
    fcw_display = VISUAL_HUD[hud_alert.raw]
  elif hud_alert == VisualAlert.steerRequired:
    steer_required = VISUAL_HUD[hud_alert.raw]
  else:
    acc_alert = VISUAL_HUD[hud_alert.raw]

  return fcw_display, steer_required, acc_alert


HUDData = namedtuple("HUDData",
                     ["pcm_accel", "v_cruise",  "car",
                     "lanes", "fcw", "acc_alert", "steer_required", "dist_lines", "dashed_lanes"])


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.apply_brake_last = 0
    self.last_pump_on_state = False
    self.packer = CANPacker(dbc_name)
    self.new_radar_config = False

    self.params = CarControllerParams(CP)

  def update(self, enabled, CS, frame, actuators,
             pcm_speed, pcm_override, pcm_cancel_cmd, pcm_accel,
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert):

    P = self.params

    # *** apply brake hysteresis ***
    brake, self.braking, self.brake_steady = actuator_hystereses(actuators.brake, self.braking, self.brake_steady, CS.out.vEgo, CS.CP.carFingerprint)

    # *** no output if not enabled ***
    if not enabled and CS.out.cruiseState.enabled:
      # send pcm acc cancel cmd if drive is disabled but pcm is still on, or if the system can't be activated
      pcm_cancel_cmd = True

    # *** rate limit after the enable check ***
    self.brake_last = rate_limit(brake, self.brake_last, -2., DT_CTRL)

    # vehicle hud display, wait for one update from 10Hz 0x304 msg
    if hud_show_lanes and CS.lkMode:
      hud_lanes = 1
    else:
      hud_lanes = 0

    if enabled:
      if hud_show_car:
        hud_car = 2
      else:
        hud_car = 1
    else:
      hud_car = 0

    fcw_display, steer_required, acc_alert = process_hud_alert(hud_alert)

    hud = HUDData(int(pcm_accel), int(round(hud_v_cruise)), hud_car,
                  hud_lanes, fcw_display, acc_alert, steer_required, CS.read_distance_lines, CS.lkMode)

    # **** process the car messages ****

    # steer torque is converted back to CAN reference (positive when steering right)
    apply_steer = int(interp(-actuators.steer * P.STEER_MAX, P.STEER_LOOKUP_BP, P.STEER_LOOKUP_V))

    lkas_active = enabled and not CS.steer_not_allowed and CS.lkMode #and not CS.left_blinker_on and not CS.right_blinker_on  # add LKAS button to toggle steering
    if apply_steer > 229 and False:
      apply_steer_orig = apply_steer
      apply_steer = (apply_steer - 229) * 2 + apply_steer
      if apply_steer > 240:
        self.apply_steer_over_max_counter += 1
        if self.apply_steer_over_max_counter > 3:
          apply_steer = apply_steer_orig
          self.apply_steer_over_max_counter = 0
      else:
        self.apply_steer_over_max_counter = 0
    elif apply_steer < -229 and False:
      apply_steer_orig = apply_steer
      apply_steer = (apply_steer + 229) * 2 + apply_steer
      if apply_steer < -240:
        self.apply_steer_over_max_counter+= 1
        if self.apply_steer_over_max_counter > 3:
          apply_steer = apply_steer_orig
          self.apply_steer_over_max_counter = 0
      else:
        self.apply_steer_over_max_counter = 0
    else:
      self.apply_steer_over_max_counter = 0
    lkas_active = enabled and not CS.steer_not_allowed
    
    # Send CAN commands.
    can_sends = []

    # Send steering command.
    idx = frame % 4
    can_sends.append(hondacan.create_steering_control(self.packer, apply_steer,
      lkas_active, CS.CP.carFingerprint, idx, CS.CP.isPandaBlack, CS.CP.openpilotLongitudinalControl))

    # Send dashboard UI commands.
    if (frame % 10) == 0:
      idx = (frame//10) % 4
      can_sends.extend(hondacan.create_ui_commands(self.packer, pcm_speed, hud, CS.CP.carFingerprint, CS.is_metric, idx, CS.CP.isPandaBlack, CS.CP.openpilotLongitudinalControl, CS.stock_hud))

    if not CS.CP.openpilotLongitudinalControl:
      if (frame % 2) == 0:
        idx = frame // 2
        can_sends.append(hondacan.create_bosch_supplemental_1(self.packer, CS.CP.carFingerprint, idx, CS.CP.isPandaBlack))
      # If using stock ACC, spam cancel command to kill gas when OP disengages.
      if pcm_cancel_cmd:
        can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.CANCEL, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
      elif CS.out.cruiseState.standstill:
        can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.RES_ACCEL, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))

    else:
      # Send gas and brake commands.
      if (frame % 2) == 0:
        idx = frame // 2
        ts = frame * DT_CTRL
        
        if CS.CP.carFingerprint in HONDA_BOSCH:
          pass # TODO implement
        else:
          apply_gas = clip(actuators.gas, 0., 1.)
          apply_brake = int(clip(self.brake_last * P.BRAKE_MAX, 0, P.BRAKE_MAX - 1))
        
          pump_on, self.last_pump_on_state = brake_pump_hysteresis(apply_brake, self.apply_brake_last, self.last_pump_on_state, ts)
          # Do NOT send the cancel command if we are using the pedal. Sending cancel causes the car firmware to
          # turn the brake pump off, and we don't want that. Stock ACC does not send the cancel cmd when it is braking.
        
        if CS.CP.enableGasInterceptor:
          pcm_cancel_cmd = False
        
        can_sends.append(hondacan.create_brake_command(self.packer, apply_brake, pump_on,
          pcm_override, pcm_cancel_cmd, hud.fcw, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack, CS.stock_brake))
        self.apply_brake_last = apply_brake


        if CS.CP.enableGasInterceptor:
          # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
          # This prevents unexpected pedal range rescaling
          can_sends.append(create_gas_command(self.packer, apply_gas, idx))

    return can_sends
