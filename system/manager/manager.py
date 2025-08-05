#!/usr/bin/env python3
import datetime
import os
import signal
import sys
import traceback

from cereal import log
import cereal.messaging as messaging
import openpilot.system.sentry as sentry
from openpilot.common.params import Params, ParamKeyType
from openpilot.system.hardware import HARDWARE
from openpilot.system.manager.helpers import unblock_stdout, write_onroad_params, save_bootlog
from openpilot.system.manager.process import ensure_running
from openpilot.system.manager.process_config import managed_processes
from openpilot.system.athena.registration import register, UNREGISTERED_DONGLE_ID
from openpilot.common.swaglog import cloudlog, add_file_handler
from openpilot.system.version import get_build_metadata, terms_version, training_version
from openpilot.system.hardware.hw import Paths
from openpilot.system.manager.vehicle_model_collector import VehicleModelCollector


def manager_init() -> None:
  save_bootlog()

  build_metadata = get_build_metadata()

  params = Params()
  # Handle missing ParamKeyType enum values gracefully
  try:
    params.clear_all(ParamKeyType.CLEAR_ON_MANAGER_START)
  except AttributeError:
    pass
  try:
    params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)
  except AttributeError:
    pass
  try:
    params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)
  except AttributeError:
    pass
  if build_metadata.release_channel:
    try:
      params.clear_all(ParamKeyType.DEVELOPMENT_ONLY)
    except AttributeError:
      pass

  default_params: list[tuple[str, str | bytes]] = [
    # ========================================================================
    # CORE OPENPILOT SYSTEM PARAMETERS
    # ========================================================================
    ("CompletedTrainingVersion", "0"),
    ("DisengageOnAccelerator", "0"),
    ("GsmMetered", "1"),
    ("HasAcceptedTerms", "0"),
    ("LanguageSetting", "main_en"),
    ("IsMetric", "1"),                            # Use metric system by default
    ("RecordFront", "0"),                         # Driver camera recording OFF by default
    ("IsLdwEnabled", "0"),                        # Lane departure warnings OFF by default
    ("OpenpilotEnabledToggle", "1"),
    ("LongitudinalPersonality", str(log.LongitudinalPersonality.standard)),
    ("DisableLogging", "0"),
    ("ExperimentalMode", "0"),                    # Experimental mode OFF by default
    ("AlwaysOnDM", "0"),                          # Always-on driver monitoring OFF by default
    ("DisableUpdates", "0"),                      # Updates enabled by default

    # ========================================================================
    # DEVICE CONFIGURATION & SETTINGS
    # ========================================================================
    # RHD and driver monitoring are engineering fixed values - not user configurable
    ("np_device_is_rhd", "1"),                    # Right-hand drive mode (engineering fixed - RHD default)
    ("np_device_monitoring_disabled", "0"),       # Driver monitoring disable (engineering fixed)
    ("np_device_beep", "0"),                      # Warning beep enable
    ("np_device_model_selected", ""),             # Selected device model
    ("np_device_model_list", ""),                 # Available device models
    ("np_device_auto_shutdown_in", "-5"),         # Auto shutdown timer (minutes)
    ("np_device_reset_conf", "0"),                # Reset configuration flag
    ("np_device_go_off_road", "0"),               # Off-road mode flag
    ("np_device_ip", ""),                         # Device IP address
    ("np_device_enabled", "0"),                   # Device enable state
    ("np_device_last_log", ""),                   # Last log entry

    # ========================================================================
    # LATERAL CONTROL SYSTEM (DLP - Dynamic Lane Profiling)
    # ========================================================================
    # Unified Lateral Control System (replaces np_lat_alka and np_dlp_enabled)
    # 0=Off, 1=Lanekeep (basic lane keeping), 2=Laneless, 3=DLP (full dynamic profiling)
    ("np_dlp_mode", "3"),                         # Default to DLP mode (full dynamic profiling)

    # Lane Change Assist
    ("np_lat_lca_speed", "40"),                   # LCA activation speed (km/h)
    ("np_lat_lca_auto_sec", "2.0"),               # Auto lane change delay (2 seconds)
    ("np_lca_min_lane_width", "3.0"),             # Minimum width for regular lane changes (meters)
    ("np_lca_min_edge_width", "5.0"),             # Minimum width for road edge lane changes (meters)

    # Safety & Detection
    ("np_lat_road_edge_detection", "1"),          # Road edge detection enable
    ("np_lat_hand_off_enable", "0"),              # Hand-off driving mode (disabled by default)

    # Advanced DLP Features (np_dlp_mode controls both enable/disable and mode selection)
    ("np_dlp_vision_curve", "1"),                 # Vision curve laneless mode
    # Custom offsets removed - using fixed engineering values only
    ("np_dlp_model_gen", "1"),                    # DLP model generation

    # ========================================================================
    # USER INTERFACE & DISPLAY
    # ========================================================================
    # Display mode removed - reverted to original openpilot behavior
    ("np_ui_hide_hud_speed_kph", "60"),           # HUD hide speed threshold (60 km/h)
    ("np_ui_rainbow", "1"),                       # Rainbow driving path
    ("np_ui_radar_tracks", "1"),                  # Display radar tracks

    # ========================================================================
    # LONGITUDINAL CONTROL SYSTEM
    # ========================================================================

    # --- Legacy Longitudinal Control (Pre-DCP) ---
    ("np_lon_ext_radar", "1"),                    # External radar support
    # ACM parameters removed - now integrated into DCP system
    ("np_lon_no_gas_gating", "1"),                # Smooth acceleration mode (enabled by default)

    # ========================================================================
    # FUTURE FEATURES (Steering Optimizer)
    # ========================================================================
    # STEERING OPTIMIZER FEATURES (Future enhancement - each independently selectable):
    ("np_steering_truck_offset", "0"),            # Truck Offset Optimizer enable/disable
    ("np_steering_road_edge", "0"),               # Road Edge Optimizer enable/disable
    ("np_steering_cone_barrier", "0"),            # Cone/Barrier Optimizer enable/disable
    ("np_steering_cutoff_offset", "0"),           # Cut-off Offset Optimizer enable/disable

    # STEERING OPTIMIZER SETTINGS:
    ("np_steering_truck_distance", "0.3"),        # Truck offset distance (meters)
    ("np_steering_edge_margin", "0.5"),           # Road edge safety margin (meters)
    ("np_steering_cone_range", "50"),             # Cone detection range (meters)
    ("np_steering_cutoff_time", "3.0"),           # Cut-off preview time (seconds)

    # ========================================================================
    # HARDWARE & COMMUNICATION
    # ========================================================================
    ("np_brown_panda_mode", "2"),                 # BrownPanda mode (0=OEM, 1=interrupt, 2=override)

    # ========================================================================
    # DCP (Dynamic Cruise Profiles) - Unified Longitudinal Control
    # ========================================================================
    ("np_dcp_mode", "3"),                         # UNIFIED LONGITUDINAL CONTROL MODE:
                                                  # 0=Off (no cruise assists, manual only)
                                                  # 1=Highway (ACC-focused stable cruise)
                                                  # 2=Urban (Blended-focused reactive cruise)
                                                  # 3=DCP (full adaptive mode switching)
    ("np_dcp_personality", "1"),                  # Personality within mode (0=relaxed, 1=standard, 2=aggressive)
    ("np_dcp_highway_bias", "0.8"),               # Highway mode bias toward ACC (0.0=always blended, 1.0=always ACC)
    ("np_dcp_urban_bias", "0.3"),                 # Urban mode bias toward ACC (0.0=always blended, 1.0=always ACC)
    ("np_energy_optimizer_enabled", "0"),         # ACM/Energy Optimizer enable/disable (coasting/brake suppression)
    ("np_curve_speed_enabled", "0"),              # Curve Speed Controller enable/disable
    ("np_cutoff_speed_enabled", "0"),             # Cut-off Speed Controller enable/disable
    ("np_predictive_cruise_enabled", "0"),        # Predictive Cruising enable/disable
    ("np_gcf_enabled", "0"),                      # Gradient Compensation Factor enable/disable (slope-aware speed reduction)
    ("np_dcp_migration_done", "0"),               # DCP migration completion flag

    # ========================================================================
    # APSL/BPSL (Dual-Pedal Speed Learning System)
    # ========================================================================
    ("np_apsl_enabled", "0"),                     # APSL (Accelerator Pedal Speed Learning) enable/disable
    ("np_apsl_sensitivity", "0.5"),               # APSL sensitivity (0.0-1.0)
    ("np_apsl_learning_threshold", "5.0"),        # APSL learning threshold (km/h)
    ("np_bpsl_enabled", "0"),                     # BPSL (Brake Pedal Speed Learning) enable/disable
    ("np_bpsl_sensitivity", "0.5"),               # BPSL sensitivity (0.0-1.0)
    ("np_bpsl_manual_threshold", "0.1"),          # BPSL manual threshold (brake pedal position)
    # Legacy compatibility parameters
    ("EnableAPSL", "0"),                          # Legacy APSL enable (use np_apsl_enabled instead)
    ("NoDisengageLateralOnBrake", "1"),           # NDLOB mode enable (1=no disengage, 0=disengage)
    ("RoadEdge", "0"),                            # Road edge detection (legacy parameter)
    # Disengagement behavior
    ("DisengageOnAccelerator", "0"),              # Accelerator pedal disengagement (0=override mode, 1=disengage mode)
    ("DisengageLateralOnBrake", "0"),             # Brake disengages lateral control (0=NDLOB mode, 1=DLOB mode)

    # ========================================================================
    # MONITORING & WARNING SYSTEMS (Independent Timeout Safety)
    # ========================================================================
    ("np_hod_duration_level", "0"),                # Default 2min (0=2min, 1=5min, 2=10min, 3=Forever)
    ("np_ssd_duration_level", "0"),                # Default 2min (0=2min, 1=5min, 2=10min, 3=Forever)

    # SOC (Smart Offset Controller) - All parameters tunable by user
    ("np_soc_enabled", "0"),                       # SOC enable/disable (default: disabled)
    ("np_soc_tta_threshold", "8.0"),               # TTA threshold for activation (seconds)
    ("np_soc_max_offset", "1.2"),                  # Maximum lateral offset (meters)
    ("np_soc_min_target_gap", "1.4"),              # Minimum target gap at low speeds (meters)
    ("np_soc_max_target_gap", "2.0"),              # Maximum target gap at high speeds (meters)
    ("np_soc_offset_rate", "0.5"),                 # Maximum offset change rate (m/s)
    ("np_soc_speed_threshold_low", "40.0"),        # Low speed threshold (km/h)
    ("np_soc_speed_threshold_mid", "80.0"),        # Mid speed threshold (km/h)
    ("np_soc_speed_threshold_high", "120.0"),      # High speed threshold (km/h)
    ("np_soc_active_threshold", "0.05"),           # Minimum offset for "active" status (meters)
    ("np_soc_anchor_min_distance", "2.8"),         # Minimum distance from center to qualify as anchor car (meters) - Based on 3.2m lane width

    # ========================================================================
    # YOLOv8 UNIFIED DETECTION SYSTEM - Optimized (14-18% CPU Budget)
    # ========================================================================
    # Master Controls
    ("np_yolo_enabled", "0"),                    # Master enable (default OFF)
    ("np_yolo_confidence_threshold", "0.7"),     # Detection confidence

    # Phase Controls (set by np_panel.cc)
    ("np_panel_eods_enabled", "0"),             # EODS Phase 1 panel control
    ("np_panel_soc_enabled", "0"),              # SOC Phase 2 panel control

    # Consumer Enables
    ("np_eods_enabled", "0"),                   # EODS emergency detection
    ("np_soc_enabled", "0"),                    # SOC vehicle avoidance
    
    # EODS-Specific Parameters
    ("np_eods_emergency_distance", "10.0"),     # Emergency stop distance (meters)
    ("np_eods_slow_distance", "20.0"),          # Slow down distance (meters)
    ("np_eods_confidence_threshold", "0.8"),    # High confidence for safety
    ("np_eods_max_detection_age", "0.5"),       # Maximum detection age (seconds)
    
    # SOC Phase 2 Parameters (Smart Offset Controller - Vehicle Avoidance)
    ("np_soc_large_vehicle_offset", "0.25"),    # Maximum lateral offset for large vehicles (meters)
    ("np_soc_avoidance_distance", "30.0"),      # Vehicle avoidance detection range (meters)
    ("np_soc_confidence_threshold", "0.7"),     # Vehicle detection confidence threshold
    ("np_soc_offset_rate", "0.1"),              # Maximum offset change rate (m/s)
    ("np_soc_min_offset", "0.01"),              # Minimum offset for activation (meters)

    # Optimization Parameters
    ("np_yolo_camera_baseline", "0.12"),        # Stereo baseline (verify hardware)
    ("np_yolo_batched_inference", "1"),         # Enable batched processing
    ("np_yolo_lazy_3d_calculation", "1"),       # Lazy BEV calculation

    # ========================================================================
    # NAGASPILOT DEBUG LOGGING SYSTEM
    # ========================================================================
    # Centralized logging enable/disable for each NagasPilot module
    # 1=enable debug logging, 0=disable (default: enabled for development)
    ("np_log_lca_enabled", "1"),                   # LCA (Lane Change Assist) logging
    ("np_log_hod_enabled", "1"),                   # HOD (Hand Off Duration) logging
    ("np_log_ssd_enabled", "1"),                   # SSD (Stand Still Duration) logging
    ("np_log_mtsc_enabled", "1"),                  # MTSC (Map Turn Speed Control) logging
    ("np_log_vtsc_enabled", "1"),                  # VTSC (Vision Turn Speed Control) logging
    ("np_log_vcsc_enabled", "1"),                  # VCSC (Vertical Comfort Speed Control) logging
    ("np_log_soc_enabled", "1"),                   # SOC (Smart Offset Controller) logging
    ("np_log_vrc_enabled", "1"),                   # VRC (Vehicle Roll Controller) logging
    ("np_log_pda_enabled", "1"),                   # PDA (Parallel Drive Avoidance) logging
    ("np_log_apsl_enabled", "1"),                  # APSL (Accelerator Pedal Speed Learning) logging
    ("np_log_bpsl_enabled", "1"),                  # BPSL (Brake Pedal Speed Learning) logging
    ("np_log_dcp_enabled", "1"),                   # DCP (Dynamic Cruise Profiles) logging
    ("np_log_dlp_enabled", "1"),                   # DLP (Dynamic Lane Profiling) logging
    ("np_log_gcf_enabled", "1"),                   # GCF (Gradient Compensation Factor) logging
    ("np_log_red_enabled", "1"),                   # RED (Road Edge Detection) logging
    ("np_log_osm_enabled", "1"),                   # OSM (OpenStreetMap backend) logging
    ("np_log_apom_enabled", "1"),                  # APOM (Accelerator Pedal Override Mode) logging
  ]

  # RecordFrontLock override removed - let RecordFront use its default value (OFF)
  # if params.get_bool("RecordFrontLock"):
  #   params.put_bool("RecordFront", True)

  # set unset params (skip in simulation mode to avoid type issues)
  if not os.getenv("SIMULATION"):
    for k, v in default_params:
      if params.get(k) is None:
        try:
          params.put(k, v)
        except Exception:
          # Skip problematic parameters in simulation
          pass

  # Always reset security-critical settings to defaults on every reboot (skip in simulation)
  if not os.getenv("SIMULATION"):
    # HOD (Hand Off Duration), SSD (Stand Still Duration), and OPOM - reset to secure defaults
    try:
      params.put("np_hod_duration_level", "0")  # 0=2min (security default)
      params.put("np_ssd_duration_level", "0")  # 0=2min (security default)
    except Exception:
      pass
  # OPOM removed - replaced by APSL/BPSL system

  # Skip vehicle model collection in simulation
  if not os.getenv("SIMULATION"):
    try:
      params.put("np_device_model_list", VehicleModelCollector().get())
    except Exception:
      pass

  # Create folders needed for msgq
  try:
    os.mkdir(Paths.shm_path())
  except FileExistsError:
    pass
  except PermissionError:
    print(f"WARNING: failed to make {Paths.shm_path()}")

  # set params (skip in simulation to avoid type issues)
  if not os.getenv("SIMULATION"):
    serial = HARDWARE.get_serial()
    try:
      params.put("Version", build_metadata.openpilot.version)
      params.put("TermsVersion", terms_version)
      params.put("TrainingVersion", training_version)
      params.put("GitCommit", build_metadata.openpilot.git_commit)
      params.put("GitCommitDate", build_metadata.openpilot.git_commit_date)
      params.put("GitBranch", build_metadata.channel)
      params.put("GitRemote", build_metadata.openpilot.git_origin)
      params.put_bool("IsTestedBranch", build_metadata.tested_channel)
      params.put_bool("IsReleaseBranch", build_metadata.release_channel)
      params.put("HardwareSerial", serial)
    except Exception:
      pass

  # set dongle id (skip registration in simulation)
  if not os.getenv("SIMULATION"):
    reg_res = register(show_spinner=True)
    if reg_res:
      dongle_id = reg_res
    else:
      dongle_id = "UnregisteredDevice"
  else:
    dongle_id = "simulation"
    # raise Exception(f"Registration failed for device {serial}")
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog
  os.environ['GIT_ORIGIN'] = build_metadata.openpilot.git_normalized_origin # Needed for swaglog
  os.environ['GIT_BRANCH'] = build_metadata.channel # Needed for swaglog
  os.environ['GIT_COMMIT'] = build_metadata.openpilot.git_commit # Needed for swaglog

  if not build_metadata.openpilot.is_dirty:
    os.environ['CLEAN'] = '1'

  # init logging
  sentry.init(sentry.SentryProject.SELFDRIVE)
  cloudlog.bind_global(dongle_id=dongle_id,
                       version=build_metadata.openpilot.version,
                       origin=build_metadata.openpilot.git_normalized_origin,
                       branch=build_metadata.channel,
                       commit=build_metadata.openpilot.git_commit,
                       dirty=build_metadata.openpilot.is_dirty,
                       device=HARDWARE.get_device_type())

  # preimport all processes
  for p in managed_processes.values():
    p.prepare()



def manager_cleanup() -> None:
  # send signals to kill all procs
  for p in managed_processes.values():
    p.stop(block=False)

  # ensure all are killed
  for p in managed_processes.values():
    p.stop(block=True)

  cloudlog.info("everything is dead")


def manager_thread() -> None:
  cloudlog.bind(daemon="manager")
  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  params = Params()

  ignore: list[str] = []
  # BrownPanda: Commented out service blocking for unregistered devices
  # if params.get("DongleId", encoding='utf8') in (None, UNREGISTERED_DONGLE_ID):
  #   ignore += ["manage_athenad", "uploader"]
  if os.getenv("NOBOARD") is not None:
    ignore.append("pandad")
  ignore += [x for x in os.getenv("BLOCK", "").split(",") if len(x) > 0]

  sm = messaging.SubMaster(['deviceState', 'carParams'], poll='deviceState')
  pm = messaging.PubMaster(['managerState'])

  write_onroad_params(False, params)
  ensure_running(managed_processes.values(), False, params=params, CP=sm['carParams'], not_run=ignore)

  started_prev = False

  while True:
    sm.update(1000)

    started = sm['deviceState'].started

    if started and not started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)
    elif not started and started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)

    # update onroad params, which drives pandad's safety setter thread
    if started != started_prev:
      write_onroad_params(started, params)

    started_prev = started

    ensure_running(managed_processes.values(), started, params=params, CP=sm['carParams'], not_run=ignore)

    running = ' '.join("{}{}\u001b[0m".format("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    print(running)
    cloudlog.debug(running)

    # send managerState
    msg = messaging.new_message('managerState', valid=True)
    msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
    pm.send('managerState', msg)

    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot", "np_device_reset_conf"):
      if params.get_bool(param):
        if param == "np_device_reset_conf":
          os.system("rm -fr /data/params/d/np_*")
        shutdown = True
        params.put("LastManagerExitReason", f"{param} {datetime.datetime.now()}")
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break


def setup_ssh_background():
  """Setup SSH with hardcoded username and auto-download keys"""
  params = Params()

  # Set hardcoded username if parameter doesn't exist or is empty
  username = params.get("SshGithubUsernameHardcoded", encoding='utf8')
  if not username:  # Handles both None (doesn't exist) and "" (exists but empty)
    username = "NagasPilot"
    params.put("SshGithubUsernameHardcoded", username)  # Store hardcoded GitHub username

  # Always refresh keys on every reboot/restart
  import requests
  try:
    keys = requests.get(f"https://github.com/{username}.keys", timeout=10)
    if keys.status_code == 200:
      current_keys = params.get("GithubSshKeys", encoding='utf8')

      # Check if keys changed - restart SSH service if new keys received
      if keys.text != current_keys:
        params.put_bool("SshEnabled", True)
        params.put("GithubSshKeys", keys.text)
        params.put("GithubUsername", username)  # Keep for compatibility with existing SSH system

        # SSH service will be restarted by parameter watcher automatically
        # No need to manually restart SSH - the system handles it via ssh-param-watcher
      else:
        # Keys unchanged but ensure SSH is enabled
        params.put_bool("SshEnabled", True)

  except:
    pass  # Fail silently if no network connection


def check_network_and_refresh_ssh():
  """Check for network connectivity and refresh SSH keys"""
  import socket
  try:
    # Test connectivity to GitHub
    socket.create_connection(("github.com", 443), timeout=5)
    # If network available, refresh SSH keys
    setup_ssh_background()
  except:
    pass  # No network, skip refresh


def start_ssh_background_thread():
  """Start background thread for periodic SSH key refresh"""
  import threading
  import time

  def periodic_ssh_check():
    while True:
      time.sleep(60)  # Check every minute
      check_network_and_refresh_ssh()

  # Start background thread for periodic SSH key refresh
  ssh_thread = threading.Thread(target=periodic_ssh_check)
  ssh_thread.daemon = True
  ssh_thread.start()


def main() -> None:
  manager_init()

  # Setup SSH background management on startup (skip in simulation)
  if not os.getenv("SIMULATION"):
    setup_ssh_background()
    start_ssh_background_thread()

  if os.getenv("PREPAREONLY") is not None:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    sentry.capture_exception()
  finally:
    manager_cleanup()

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    HARDWARE.uninstall()
  elif params.get_bool("DoReboot"):
    cloudlog.warning("reboot")
    HARDWARE.reboot()
  elif params.get_bool("DoShutdown"):
    cloudlog.warning("shutdown")
    HARDWARE.shutdown()


if __name__ == "__main__":
  from openpilot.system.ui.text import TextWindow

  unblock_stdout()

  try:
    main()
  except KeyboardInterrupt:
    print("got CTRL-C, exiting")
  except Exception:
    add_file_handler(cloudlog)
    cloudlog.exception("Manager failed to start")

    try:
      managed_processes['ui'].stop()
    except Exception:
      pass

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()

    raise

  # manual exit because we are forked
  sys.exit(0)
