#pragma once

#include <string>
#include <unordered_map>

inline static std::unordered_map<std::string, uint32_t> keys = {
    // ========================================================================
    // AUTHENTICATION & SECURITY
    // ========================================================================
    {"AccessToken", CLEAR_ON_MANAGER_START | DONT_LOG},
    {"AssistNowToken", PERSISTENT},
    {"GithubSshKeys", PERSISTENT},
    {"GithubUsername", PERSISTENT},
    {"SshGithubUsernameHardcoded", PERSISTENT},
    {"HasAcceptedTerms", PERSISTENT},

    // ========================================================================
    // DEVICE HARDWARE & IDENTIFICATION
    // ========================================================================
    {"AdbEnabled", PERSISTENT},
    {"BootCount", PERSISTENT},
    {"DongleId", PERSISTENT},
    {"HardwareSerial", PERSISTENT},
    {"InstallDate", PERSISTENT},

    // ========================================================================
    // SYSTEM CONTROL & LIFECYCLE
    // ========================================================================
    {"DoReboot", CLEAR_ON_MANAGER_START},
    {"DoShutdown", CLEAR_ON_MANAGER_START},
    {"DoUninstall", CLEAR_ON_MANAGER_START},
    {"ForcePowerDown", PERSISTENT},
    {"DisablePowerDown", PERSISTENT},
    {"DisableUpdates", PERSISTENT},

    // ========================================================================
    // CALIBRATION & CAMERA SYSTEM
    // ========================================================================
    {"CalibrationParams", PERSISTENT},
    {"CameraDebugExpGain", CLEAR_ON_MANAGER_START},
    {"CameraDebugExpTime", CLEAR_ON_MANAGER_START},

    // ========================================================================
    // VEHICLE PARAMETERS & CONTROL STATE
    // ========================================================================
    {"CarBatteryCapacity", PERSISTENT},
    {"CarParams", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"CarParamsCache", CLEAR_ON_MANAGER_START},
    {"CarParamsPersistent", PERSISTENT},
    {"CarParamsPrevRoute", PERSISTENT},
    {"ControlsReady", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},

    // ========================================================================
    // OPENPILOT CORE FEATURES & MODES
    // ========================================================================
    {"AlphaLongitudinalEnabled", PERSISTENT | DEVELOPMENT_ONLY},
    {"AlwaysOnDM", PERSISTENT},
    {"CompletedTrainingVersion", PERSISTENT},
    {"DisengageOnAccelerator", PERSISTENT},
    {"DisengageLateralOnBrake", PERSISTENT},
    {"ExperimentalMode", PERSISTENT},
    {"ExperimentalModeConfirmed", PERSISTENT},

    // ========================================================================
    // LOGGING & DIAGNOSTICS
    // ========================================================================
    {"CurrentBootlog", PERSISTENT},
    {"CurrentRoute", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"DisableLogging", PERSISTENT},

    // ========================================================================
    // VERSION CONTROL & BUILD INFO
    // ========================================================================
    {"GitBranch", PERSISTENT},
    {"GitCommit", PERSISTENT},
    {"GitCommitDate", PERSISTENT},
    {"GitDiff", PERSISTENT},
    {"GitRemote", PERSISTENT},

    // ========================================================================
    // NETWORK & CONNECTIVITY
    // ========================================================================
    {"GsmApn", PERSISTENT},
    {"GsmMetered", PERSISTENT},
    {"GsmRoaming", PERSISTENT},

    // ========================================================================
    // ATHENA CLOUD SERVICES
    // ========================================================================
    {"AthenadPid", PERSISTENT},
    {"AthenadUploadQueue", PERSISTENT},
    {"AthenadRecentlyViewedRoutes", PERSISTENT},
    {"ApiCache_Device", PERSISTENT},
    {"ApiCache_InstallationGuideStats", PERSISTENT},

    // ========================================================================
    // FIRMWARE & HARDWARE INTERFACE
    // ========================================================================
    {"FirmwareQueryDone", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},

    // ========================================================================
    // SYSTEM STATE & STATUS FLAGS
    // ========================================================================
    {"IsDriverViewEnabled", CLEAR_ON_MANAGER_START},
    {"IsEngaged", PERSISTENT},
    {"IsLdwEnabled", PERSISTENT},
    {"IsMetric", PERSISTENT},
    {"IsOffroad", CLEAR_ON_MANAGER_START},
    {"IsOnroad", PERSISTENT},
    {"IsRhdDetected", PERSISTENT},
    {"IsReleaseBranch", CLEAR_ON_MANAGER_START},
    {"IsTakingSnapshot", CLEAR_ON_MANAGER_START},
    {"IsTestedBranch", CLEAR_ON_MANAGER_START},

    // ========================================================================
    // USER PREFERENCES & LOCALIZATION
    // ========================================================================
    {"LanguageSetting", PERSISTENT},

    // ========================================================================
    // SYSTEM MONITORING & LAST STATE TRACKING
    // ========================================================================
    {"LastAthenaPingTime", CLEAR_ON_MANAGER_START},
    {"LastGPSPosition", PERSISTENT},
    {"LastManagerExitReason", CLEAR_ON_MANAGER_START},
    {"LastOffroadStatusPacket", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"LastPowerDropDetected", CLEAR_ON_MANAGER_START},
    {"LastUpdateException", CLEAR_ON_MANAGER_START},
    {"LastUpdateTime", PERSISTENT},

    // ========================================================================
    // LIVE CONTROL PARAMETERS & TUNING
    // ========================================================================
    {"LiveDelay", PERSISTENT},
    {"LiveParameters", PERSISTENT},
    {"LiveParametersV2", PERSISTENT},
    {"LiveTorqueParameters", PERSISTENT | DONT_LOG},
    {"LocationFilterInitialState", PERSISTENT},

    // ========================================================================
    // LONGITUDINAL CONTROL SYSTEM (CORE OPENPILOT)
    // ========================================================================
    {"LongitudinalManeuverMode", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"LongitudinalPersonality", PERSISTENT},

    // ========================================================================
    // NETWORK & CONNECTIVITY ADVANCED
    // ========================================================================
    {"NetworkMetered", PERSISTENT},

    // ========================================================================
    // VEHICLE COMMUNICATION & OBD
    // ========================================================================
    {"ObdMultiplexingChanged", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"ObdMultiplexingEnabled", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},

    // ========================================================================
    // DEBUGGING & DEVELOPMENT
    // ========================================================================
    {"JoystickDebugMode", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},

    // ========================================================================
    // OFFROAD ERROR & STATUS ALERTS
    // ========================================================================
    {"Offroad_BadNvme", CLEAR_ON_MANAGER_START},
    {"Offroad_CarUnrecognized", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"Offroad_ConnectivityNeeded", CLEAR_ON_MANAGER_START},
    {"Offroad_ConnectivityNeededPrompt", CLEAR_ON_MANAGER_START},
    {"Offroad_IsTakingSnapshot", CLEAR_ON_MANAGER_START},
    {"Offroad_NeosUpdate", CLEAR_ON_MANAGER_START},
    {"Offroad_NoFirmware", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"Offroad_Recalibration", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"Offroad_StorageMissing", CLEAR_ON_MANAGER_START},
    {"Offroad_TemperatureTooHigh", CLEAR_ON_MANAGER_START},
    {"Offroad_UnofficialHardware", CLEAR_ON_MANAGER_START},
    {"Offroad_UpdateFailed", CLEAR_ON_MANAGER_START},

    // ========================================================================
    // OPENPILOT CORE SYSTEM CONTROL
    // ========================================================================
    {"OnroadCycleRequested", CLEAR_ON_MANAGER_START},
    {"OpenpilotEnabledToggle", PERSISTENT},

    // ========================================================================
    // PANDA HARDWARE INTERFACE
    // ========================================================================
    {"PandaHeartbeatLost", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"PandaSomResetTriggered", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"PandaSignatures", CLEAR_ON_MANAGER_START},

    // ========================================================================
    // COMMA SERVICES & SUBSCRIPTIONS
    // ========================================================================
    {"PrimeType", PERSISTENT},

    // ========================================================================
    // RECORDING & DATA COLLECTION
    // ========================================================================
    {"RecordFront", PERSISTENT},
    {"RecordFrontLock", PERSISTENT},  // for the internal fleet
    {"RouteCount", PERSISTENT},

    // ========================================================================
    // SECURITY & ENCRYPTION
    // ========================================================================
    {"SecOCKey", PERSISTENT | DONT_LOG},

    // ========================================================================
    // DEVELOPMENT & SSH ACCESS
    // ========================================================================
    {"SshEnabled", PERSISTENT},

    // ========================================================================
    // TRAINING & TERMS SYSTEM
    // ========================================================================
    {"TermsVersion", PERSISTENT},
    {"TrainingVersion", PERSISTENT},

    // ========================================================================
    // GPS & POSITIONING
    // ========================================================================
    {"UbloxAvailable", PERSISTENT},

    // ========================================================================
    // SOFTWARE UPDATE SYSTEM
    // ========================================================================
    {"SnoozeUpdate", CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION},
    {"UpdateAvailable", CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION},
    {"UpdateFailedCount", CLEAR_ON_MANAGER_START},
    {"UpdaterAvailableBranches", PERSISTENT},
    {"UpdaterCurrentDescription", CLEAR_ON_MANAGER_START},
    {"UpdaterCurrentReleaseNotes", CLEAR_ON_MANAGER_START},
    {"UpdaterFetchAvailable", CLEAR_ON_MANAGER_START},
    {"UpdaterNewDescription", CLEAR_ON_MANAGER_START},
    {"UpdaterNewReleaseNotes", CLEAR_ON_MANAGER_START},
    {"UpdaterState", CLEAR_ON_MANAGER_START},
    {"UpdaterTargetBranch", CLEAR_ON_MANAGER_START},
    {"UpdaterLastFetchTime", PERSISTENT},

    // ========================================================================
    // VERSION INFORMATION
    // ========================================================================
    {"Version", PERSISTENT},

    // ========================================================================
    // DEVICE CONFIGURATION & SETTINGS
    // ========================================================================
    {"np_device_enabled", PERSISTENT},
    {"np_device_last_log", CLEAR_ON_ONROAD_TRANSITION},
    {"np_device_reset_conf", CLEAR_ON_MANAGER_START},
    {"np_device_is_rhd", PERSISTENT},
    {"np_device_monitoring_disabled", PERSISTENT},
    {"np_device_beep", PERSISTENT},
    {"np_device_model_selected", PERSISTENT},
    {"np_device_model_list", PERSISTENT},
    {"np_device_go_off_road", CLEAR_ON_MANAGER_START},
    {"np_device_auto_shutdown_in", PERSISTENT},
    {"np_device_ip", CLEAR_ON_MANAGER_START},

    // ========================================================================
    // HARDWARE - Protocol Translator
    // ========================================================================
    {"np_brown_panda_mode", PERSISTENT},

    // ========================================================================
    // DCP (Dynamic Cruise Profiles) - Longitudinal Control System
    // ========================================================================
    {"np_dcp_mode", PERSISTENT},
    {"np_dcp_personality", PERSISTENT},
    {"np_dcp_highway_bias", PERSISTENT},
    {"np_dcp_urban_bias", PERSISTENT},
    {"np_dcp_migration_done", PERSISTENT},
    {"np_energy_optimizer_enabled", PERSISTENT},
    {"np_curve_speed_enabled", PERSISTENT},
    {"np_cutoff_speed_enabled", PERSISTENT},
    {"np_predictive_cruise_enabled", PERSISTENT},
    {"np_gcf_enabled", PERSISTENT},

    // ========================================================================
    // DLP (Dynamic Lane Profiling) - Lateral Control System
    // ========================================================================
    {"np_lat_lca_speed", PERSISTENT},
    {"np_lat_lca_auto_sec", PERSISTENT},
    {"np_lca_min_lane_width", PERSISTENT},
    {"np_lca_min_edge_width", PERSISTENT},
    {"np_lat_road_edge_detection", PERSISTENT},
    {"np_lat_hand_off_enable", PERSISTENT},
    {"np_dlp_mode", PERSISTENT},
    {"np_dlp_vision_curve", PERSISTENT},
    // Custom offsets removed - using fixed engineering values only
    {"np_dlp_model_gen", PERSISTENT},

    // ========================================================================
    // FOUNDATION SYSTEM PARAMETERS (Filter Layer Architecture)
    // ========================================================================
    {"np_dcp_safety_fallback", PERSISTENT},
    {"np_dcp_fallback_enabled", PERSISTENT},
    {"np_dlp_fallback_enabled", PERSISTENT},

    // ========================================================================
    // SPEED CONTROLLER PARAMETERS (Filter Layers)
    // ========================================================================
    // VTSC (Vision Turn Speed Controller)
    {"np_vtsc_enabled", PERSISTENT},
    {"np_vtsc_max_lateral_accel", PERSISTENT},
    {"np_vtsc_min_speed", PERSISTENT},
    {"np_vtsc_curve_threshold", PERSISTENT},

    // MTSC (Map Turn Speed Controller)
    {"np_mtsc_enabled", PERSISTENT},
    {"np_mtsc_lookahead_distance", PERSISTENT},
    {"np_mtsc_speed_limit_offset", PERSISTENT},
    {"np_mtsc_min_speed_reduction", PERSISTENT},

    // APSL (Accelerator Pedal Speed Learning) - Dual-Pedal Speed Learning System
    {"EnableAPSL", PERSISTENT},
    {"NoDisengageLateralOnBrake", PERSISTENT},
    {"RoadEdge", PERSISTENT},
    {"np_apsl_enabled", PERSISTENT},
    {"np_apsl_sensitivity", PERSISTENT},
    {"np_apsl_learning_threshold", PERSISTENT},

    // BPSL (Brake Pedal Speed Learning) - Dual-Pedal Speed Learning System
    {"EnableBPSL", PERSISTENT},
    {"np_bpsl_enabled", PERSISTENT},
    {"np_bpsl_sensitivity", PERSISTENT},
    {"np_bpsl_manual_threshold", PERSISTENT},

    // OSM (OpenStreetMap) Offline Support
    {"np_osm_enabled", PERSISTENT},
    {"np_osm_region", PERSISTENT},

    // VCSC (Vertical Comfort Speed Controller) - Kalman Filter Enhanced
    {"np_vcsc_enabled", PERSISTENT},
    {"np_vcsc_comfort_threshold", PERSISTENT},
    {"np_vcsc_jerk_threshold", PERSISTENT},
    {"np_vcsc_min_reduction", PERSISTENT},
    {"np_vcsc_max_reduction", PERSISTENT},
    {"np_vcsc_min_speed_ratio", PERSISTENT},
    {"np_vcsc_max_decel_rate", PERSISTENT},
    {"np_vcsc_accel_weight", PERSISTENT},
    {"np_vcsc_jerk_weight", PERSISTENT},
    {"np_vcsc_uncertainty_weight", PERSISTENT},
    {"np_vcsc_speed_scaling", PERSISTENT},
    {"np_vcsc_min_confidence", PERSISTENT},
    {"np_vcsc_uncertainty_factor", PERSISTENT},
    {"np_vcsc_debug_enabled", PERSISTENT},
    {"np_vcsc_log_interval", PERSISTENT},

    // PDA (Parallel Drive Avoidance)
    {"np_pda_enabled", PERSISTENT},
    {"np_pda_min_tta", PERSISTENT},


    // ========================================================================
    // MONITORING & WARNING SYSTEMS (Independent Timeout Safety)
    // ========================================================================
    // HOD (Hand Off Duration) - Duration selector (0=2min, 1=5min, 2=10min, 3=Forever/Override)
    {"np_hod_enabled", PERSISTENT},
    {"np_hod_duration_level", PERSISTENT},

    // SSD (Stand Still Duration) - Duration selector (0=2min, 1=5min, 2=10min, 3=Forever/Override)
    {"np_ssd_enabled", PERSISTENT},
    {"np_ssd_duration_level", PERSISTENT},

    // SOC (Smart Offset Controller)
    {"np_soc_enabled", PERSISTENT},
    {"np_soc_tta_threshold", PERSISTENT},
    {"np_soc_max_offset", PERSISTENT},
    {"np_soc_min_target_gap", PERSISTENT},
    {"np_soc_max_target_gap", PERSISTENT},
    {"np_soc_offset_rate", PERSISTENT},
    {"np_soc_speed_threshold_low", PERSISTENT},
    {"np_soc_speed_threshold_mid", PERSISTENT},
    {"np_soc_speed_threshold_high", PERSISTENT},
    {"np_soc_active_threshold", PERSISTENT},
    {"np_soc_anchor_min_distance", PERSISTENT},  // Real world: 3.2m lane width

    // ========================================================================
    // EODS (EMERGENCY OBJECT DETECTION SYSTEM) PARAMETERS
    // ========================================================================
    {"np_eods_enabled", PERSISTENT},                     // Enable EODS system
    {"np_eods_emergency_distance", PERSISTENT},          // Emergency stop distance (meters)
    {"np_eods_slow_distance", PERSISTENT},               // Slow down distance (meters)
    {"np_eods_monitor_distance", PERSISTENT},            // Monitor distance (meters)
    {"np_eods_confidence_threshold", PERSISTENT},        // Minimum confidence threshold (0.0-1.0)
    {"np_eods_high_confidence_threshold", PERSISTENT},   // High confidence threshold (0.0-1.0)
    {"np_eods_response_delay", PERSISTENT},              // Response delay (seconds)
    {"np_eods_emergency_timeout", PERSISTENT},           // Emergency override timeout (seconds)
    {"np_eods_max_decel", PERSISTENT},                   // Maximum emergency deceleration (m/s²)
    {"np_eods_slow_factor", PERSISTENT},                 // Slow down speed factor (0.0-1.0)
    {"np_eods_enable_alerts", PERSISTENT},               // Enable UI alerts
    {"np_eods_alert_threshold", PERSISTENT},             // Alert distance threshold (meters)

    // ========================================================================
    // YOLOV8 DAEMON PARAMETERS
    // ========================================================================
    {"np_yolo_enabled", PERSISTENT},                     // Enable YOLOv8 daemon
    {"np_yolo_confidence_threshold", PERSISTENT},        // YOLOv8 confidence threshold
    {"np_yolo_phase1_enabled", PERSISTENT},              // Enable EODS Phase 1
    {"np_yolo_phase2_enabled", PERSISTENT},              // Enable SOC Phase 2

    // ========================================================================
    // SYSTEM COORDINATION PARAMETERS
    // ========================================================================
    {"np_master_safety_enabled", PERSISTENT},
    {"np_system_health_monitoring", PERSISTENT},

    // ========================================================================
    // DEBUG LOGGING PARAMETERS (1=enable, 0=disable)
    // ========================================================================
    {"np_log_lca_enabled", PERSISTENT},     // LCA (Lane Change Assist) logging
    {"np_log_hod_enabled", PERSISTENT},     // HOD (Hand Off Duration) logging
    {"np_log_ssd_enabled", PERSISTENT},     // SSD (Stand Still Duration) logging
    {"np_log_mtsc_enabled", PERSISTENT},    // MTSC (Map Turn Speed Control) logging
    {"np_log_vtsc_enabled", PERSISTENT},    // VTSC (Vision Turn Speed Control) logging
    {"np_log_vcsc_enabled", PERSISTENT},    // VCSC (Vertical Comfort Speed Control) logging
    {"np_log_soc_enabled", PERSISTENT},     // SOC (Smart Offset Controller) logging
    {"np_log_pda_enabled", PERSISTENT},     // PDA (Predictive Dynamic Acceleration) logging
    {"np_log_dcp_enabled", PERSISTENT},     // DCP (Dynamic Cruise Profiles) logging
    {"np_log_dlp_enabled", PERSISTENT},     // DLP (Dynamic Lane Profiling) logging

    // ========================================================================
    // USER INTERFACE & DISPLAY
    // ========================================================================
    // Display mode removed - reverted to original openpilot behavior
    {"np_ui_hide_hud_speed_kph", PERSISTENT},
    {"np_ui_rainbow", PERSISTENT},
    {"np_ui_radar_tracks", PERSISTENT},

    // ========================================================================
    // LONGITUDINAL CONTROL SYSTEM
    // ========================================================================

    // --- Legacy Longitudinal Control (Pre-DCP) ---
    {"np_lon_ext_radar", PERSISTENT},
    {"np_lon_no_gas_gating", PERSISTENT},

    // ========================================================================
    // STEERING OPTIMIZER FEATURES (Future enhancement)
    // ========================================================================
    {"np_steering_truck_offset", PERSISTENT},
    {"np_steering_road_edge", PERSISTENT},
    {"np_steering_cone_barrier", PERSISTENT},
    {"np_steering_cutoff_offset", PERSISTENT},
    {"np_steering_truck_distance", PERSISTENT},
    {"np_steering_edge_margin", PERSISTENT},
    {"np_steering_cone_range", PERSISTENT},
    {"np_steering_cutoff_time", PERSISTENT},

    // ========================================================================
    // TRIP TRACKING SYSTEM
    // ========================================================================
    {"np_trip_lifetime_distance", PERSISTENT},
    {"np_trip_lifetime_time", PERSISTENT},
    {"np_trip_lifetime_interventions", PERSISTENT},
    {"np_trip_current_distance", PERSISTENT},
    {"np_trip_current_time", PERSISTENT},
    {"np_trip_current_interventions", PERSISTENT},


    // ========================================================================
    // YOLO & DETECTION SYSTEMS
    // ========================================================================
    {"np_panel_eods_enabled", PERSISTENT},
    {"np_panel_soc_enabled", PERSISTENT},
    {"np_eods_emergency_distance", PERSISTENT},
    {"np_eods_slow_distance", PERSISTENT},
    {"np_eods_max_detection_age", PERSISTENT},
    {"np_soc_large_vehicle_offset", PERSISTENT},
    {"np_soc_avoidance_distance", PERSISTENT},
    {"np_soc_confidence_threshold", PERSISTENT},
    {"np_soc_min_offset", PERSISTENT},
    {"np_yolo_camera_baseline", PERSISTENT},
    {"np_yolo_batched_inference", PERSISTENT},
    {"np_yolo_lazy_3d_calculation", PERSISTENT},

    // ========================================================================
    // DEBUG LOGGING SYSTEM - Additional Parameters
    // ========================================================================
    {"np_log_vrc_enabled", PERSISTENT},
    {"np_log_gcf_enabled", PERSISTENT},
    {"np_log_red_enabled", PERSISTENT},
    {"np_log_osm_enabled", PERSISTENT},
    {"np_log_apom_enabled", PERSISTENT},
    {"np_log_apsl_enabled", PERSISTENT},
    {"np_log_bpsl_enabled", PERSISTENT},

};
