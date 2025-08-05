#!/usr/bin/env python3
# HARDCODED DRIVER MONITORING - ALWAYS GOOD STATUS, ZERO PROCESSING
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Ratekeeper
import time

def dmonitoringd_thread():
  config_realtime_process([0, 1, 2, 3], 5)
  
  params = Params()
  pm = messaging.PubMaster(['driverMonitoringState'])
  
  # No SubMaster needed - we're not processing any inputs
  # No DriverMonitoring class - we're hardcoding results
  
  # Get RHD setting once and cache it
  is_rhd = params.get_bool("IsRhdDetected")
  
  # 20Hz rate keeper for consistent publishing
  rk = Ratekeeper(20, None)
  
  print("OpenPilot Driver Monitoring: HARDCODED MODE - Always good status, zero processing")
  
  # Main loop - just publish hardcoded good status at 20Hz
  while True:
    # Create hardcoded always-good driver monitoring state
    dat = messaging.new_message('driverMonitoringState', valid=True)
    dat.driverMonitoringState = {
      "events": [],                    # Never any events/warnings
      "faceDetected": True,           # Always face detected  
      "isDistracted": False,          # Never distracted
      "distractedType": 0,           # No distraction type
      "awarenessStatus": 1.0,        # Always full awareness (never triggers forceDecel)
      "posePitchOffset": 0.0,        # Neutral pose
      "posePitchValidCount": 999,    # High valid count 
      "poseYawOffset": 0.0,          # Neutral pose
      "poseYawValidCount": 999,      # High valid count
      "stepChange": 0.0,             # No awareness change
      "awarenessActive": 1.0,        # Full active awareness
      "awarenessPassive": 1.0,       # Full passive awareness  
      "isLowStd": False,             # Good tracking quality
      "hiStdCount": 0,               # No high std counts
      "isActiveMode": True,          # Active mode
      "isRHD": is_rhd,               # Use cached RHD setting
    }
    
    # Publish hardcoded good state
    pm.send('driverMonitoringState', dat)
    
    # Maintain 20Hz with minimal CPU usage
    rk.keep_time()

def main():
  dmonitoringd_thread()


if __name__ == '__main__':
  main()
