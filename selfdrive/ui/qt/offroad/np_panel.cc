#include "selfdrive/ui/qt/offroad/np_panel.h"


void NPPanel::add_lateral_toggles() {
  std::vector<std::tuple<QString, QString, QString>> toggle_defs{
    {
      "",
      tr("■ Lateral Control"),
      "",
    },
    {
      "np_lat_hand_off_enable",
      tr("  └ Hand Off Enable"),
      tr("Enable hand-off driving mode")
    },
    {
      "np_lat_road_edge_detection",
      tr("  └ Road Edge Detection"),
      tr("Block lane change when road edge detected")
    },
    {
      "np_dlp_vision_curve",
      tr("  └ Laneless on Curve"),
      tr("Force using Laneless on curvature instead of Lanekeep")
    },
    // Custom offsets removed - using fixed engineering values only
  };

  // Enhanced DLP Mode Selector with fallback information
  std::vector<QString> dlp_mode_texts{tr("Off"), tr("Lanekeep"), tr("Laneless"), tr("DLP")};
  ButtonParamControl* dlp_mode_selector = new ButtonParamControl("np_dlp_mode",
                                        tr("  └ Dynamic Lane Plus (DLP)"),
                                        tr("Off: Fallback to OpenPilot\nLanekeep: Basic lane keeping\nLaneless: No strict lane boundaries\nDLP: Full dynamic lane profile"),
                                        "",
                                        dlp_mode_texts, 200);

  // LCA speed is fixed at 40km/h, but delay timer is user-configurable
  lca_sec_toggle = new ParamDoubleSpinBoxControl("np_lat_lca_auto_sec", tr("  └ LCA Delay (>=40km/h)"), tr("Lane Change Assist delay"), "", 0, 5.0, 0.5, tr(" sec"), tr("Off"));
  

  // DLP Offset Controls removed - using fixed engineering values only

  QWidget *label = nullptr;
  bool has_toggle = false;

  for (auto &[param, title, desc] : toggle_defs) {
    if (param.isEmpty()) {
      label = new LabelControl(title, "");
      addItem(label);
      // Add unified DLP mode selector after header
      addItem(dlp_mode_selector);
      // LCA delay timer is user configurable
      addItem(lca_sec_toggle);
      has_toggle = true;
      continue;
    }

    has_toggle = true;
    auto toggle = new ParamControl(param, title, desc, "", this);
    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);
    addItem(toggle);
    toggles[param.toStdString()] = toggle;
  }

  // DLP offset controls removed - using fixed engineering values only

  // If no toggles were added, hide the label
  if (!has_toggle && label) {
    label->hide();
  }
}

void NPPanel::add_longitudinal_toggles() {
  std::vector<std::tuple<QString, QString, QString>> toggle_defs{
    {
      "",
      tr("■ Longitudinal Control"),
      "",
    },
    // DCP Mode selector will be added separately below
    {
      "np_lon_no_gas_gating",
      tr("  └ Smooth Acceleration"),
      tr("Continue accelerating smoothly through traffic lights and highway merges"),
    },
  };

  // Speed Control Systems
  std::vector<std::tuple<QString, QString, QString>> speed_control_defs{
    {
      "",
      tr("■ Speed Control Systems"),
      "",
    },
    {
      "np_vtsc_enabled",
      tr("  └ VTSC (Vision Speed Control)"),
      tr("Automatically reduce speed before curves using vision detection"),
    },
    {
      "np_mtsc_enabled", 
      tr("  └ MTSC (Map Speed Control)"),
      tr("Use map data to adjust speed for upcoming road conditions"),
    },
    {
      "np_pda_enabled",
      tr("  └ PDA (Parallel Drive Avoidance)"),
      tr("Strategic overtaking optimization and parallel drive avoidance"),
    },
  };

  // DCP Mode selector (enhanced with fallback information)
  std::vector<QString> dcp_mode_texts{tr("Off"), tr("Highway"), tr("Urban"), tr("DCP")};
  ButtonParamControl* dcp_mode_selector = new ButtonParamControl("np_dcp_mode",
                                        tr("  └ Dynamic Cruise Plus (DCP)"),
                                        tr("Off: Fallback to OpenPilot\nHighway: Stable ACC for highways\nUrban: Reactive for city driving\nDCP: Full adaptive cruise control"),
                                        "",
                                        dcp_mode_texts, 200);

  // DCP Personality Settings  
  std::vector<QString> dcp_personality_texts{tr("Relaxed"), tr("Standard"), tr("Aggressive")};
  ButtonParamControl* dcp_personality_selector = new ButtonParamControl("np_dcp_personality",
                                        tr("    └ DCP Personality"),
                                        tr("Relaxed: Conservative following distance\nStandard: Balanced behavior\nAggressive: Sport driving style"),
                                        "",
                                        dcp_personality_texts, 200);

  // DCP Safety Fallback
  auto dcp_safety_fallback = new ParamControl("np_dcp_safety_fallback",
                                             tr("    └ DCP Safety Fallback"),
                                             tr("Enable automatic fallback to OpenPilot when DCP encounters issues"),
                                             "", this);

  QWidget *label = nullptr;
  bool has_toggle = false;

  for (auto &[param, title, desc] : toggle_defs) {
    if (param.isEmpty()) {
      label = new LabelControl(title, "");
      addItem(label);
      // Add DCP mode selector and personality after the longitudinal control label
      addItem(dcp_mode_selector);
      addItem(dcp_personality_selector);
      addItem(dcp_safety_fallback);
      toggles["np_dcp_safety_fallback"] = dcp_safety_fallback;
      continue;
    }
    // DCP mode logic handled by ButtonParamControl

    has_toggle = true;
    auto toggle = new ParamControl(param, title, desc, "", this);
    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);
    addItem(toggle);
    toggles[param.toStdString()] = toggle;
  }

  // Add Speed Control Systems section
  QWidget *speed_label = nullptr;
  bool has_speed_toggle = false;

  for (auto &[param, title, desc] : speed_control_defs) {
    if (param.isEmpty()) {
      speed_label = new LabelControl(title, "");
      addItem(speed_label);
      has_speed_toggle = true;
      continue;
    }

    has_speed_toggle = true;
    auto toggle = new ParamControl(param, title, desc, "", this);
    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);
    addItem(toggle);
    toggles[param.toStdString()] = toggle;
  }

  // Add VTSC parameters
  auto vtsc_lateral_accel = new ParamDoubleSpinBoxControl("np_vtsc_max_lateral_accel",
                                                         tr("    └ VTSC Max Lateral Acceleration"),
                                                         tr("Maximum lateral acceleration for curve speed calculation"),
                                                         "", 1.0, 3.0, 0.1, tr(" m/s²"), "");
  addItem(vtsc_lateral_accel);

  auto vtsc_lookahead = new ParamSpinBoxControl("np_vtsc_lookahead_distance",
                                               tr("    └ VTSC Lookahead Distance"),
                                               tr("Distance ahead to scan for curves"),
                                               "", 50, 500, 25, tr(" m"), "");
  addItem(vtsc_lookahead);

  // Add MTSC parameters
  auto mtsc_lookahead = new ParamSpinBoxControl("np_mtsc_lookahead_distance",
                                               tr("    └ MTSC Lookahead Distance"),
                                               tr("Distance ahead to check map data"),
                                               "", 100, 1000, 50, tr(" m"), "");
  addItem(mtsc_lookahead);

  auto mtsc_speed_reduction = new ParamDoubleSpinBoxControl("np_mtsc_speed_reduction_factor",
                                                           tr("    └ MTSC Speed Reduction Factor"),
                                                           tr("Factor for speed reduction on map-detected conditions"),
                                                           "", 0.5, 1.0, 0.05, "", "");
  addItem(mtsc_speed_reduction);

  // Add PDA parameters
  auto pda_min_tta = new ParamDoubleSpinBoxControl("np_pda_min_tta",
                                                  tr("    └ PDA Minimum TTA"),
                                                  tr("Minimum time-to-approach before PDA activation"),
                                                  "", 5.0, 30.0, 1.0, tr(" sec"), "");
  addItem(pda_min_tta);

  auto pda_speed_boost = new ParamDoubleSpinBoxControl("np_pda_speed_boost_factor",
                                                      tr("    └ PDA Speed Boost Factor"),
                                                      tr("Speed boost factor for performance scenarios"),
                                                      "", 1.0, 1.5, 0.05, "", "");
  addItem(pda_speed_boost);

  // If no toggles were added, hide the label
  if (!has_toggle && label) {
    label->hide();
  }
  if (!has_speed_toggle && speed_label) {
    speed_label->hide();
  }
}

void NPPanel::add_monitoring_toggles() {
  std::vector<std::tuple<QString, QString, QString>> toggle_defs{
    {
      "",
      tr("■ Monitoring & Warning Systems"),
      "",
    },
  };

  // HOD Duration Selector (2min/5min/10min/Forever)
  std::vector<QString> hod_duration_texts{tr("2min"), tr("5min"), tr("10min"), tr("Forever")};
  ButtonParamControl* hod_duration_selector = new ButtonParamControl("np_hod_duration_level",
                                        tr("  └ HOD (Hand Off Duration)"),
                                        tr("2min: Short timeout\n5min: Medium timeout\n10min: Long timeout\nForever: Infinite (disables OpenPilot steering timeout)"),
                                        "",
                                        hod_duration_texts, 200);

  // SSD Duration Selector (2min/5min/10min/Forever)
  std::vector<QString> ssd_duration_texts{tr("2min"), tr("5min"), tr("10min"), tr("Forever")};
  ButtonParamControl* ssd_duration_selector = new ButtonParamControl("np_ssd_duration_level",
                                        tr("  └ SSD (Stand Still Duration)"),
                                        tr("2min: Short timeout\n5min: Medium timeout\n10min: Long timeout\nForever: Infinite (disables OpenPilot standstill timeout)"),
                                        "",
                                        ssd_duration_texts, 200);


  // Add section header
  QWidget *label = new LabelControl(tr("■ Monitoring & Warning Systems"), "");
  addItem(label);
  
  // Add duration selectors directly
  addItem(hod_duration_selector);
  addItem(ssd_duration_selector);
}

void NPPanel::add_ui_toggles() {
  std::vector<std::tuple<QString, QString, QString>> toggle_defs{
    {
      "",
      tr("■ User Interface"),
      "",
    },
    {
      "np_ui_rainbow",
      tr("  └ Rainbow Driving Path"),
      tr("After raining, there will be Rainbow"),
    },
  };
  // Display mode feature removed - screen always on during driving for safety

  auto hide_hud = new ParamSpinBoxControl("np_ui_hide_hud_speed_kph", tr("  └ Hide HUD When Moves above"), tr("To prevent screen burn-in, hide Speed, MAX Speed, and Steering Icons when the car moves.\nOff = Stock Behavior\n1 km/h ≈ 0.6 mph"), "", 0, 120, 5, tr(" km/h"), tr("Off"));

  // OSM Map Region Display (read-only)
  osm_region_display = new LabelControl(tr("  └ Offline MAP data"), "");

  QWidget *label = nullptr;
  bool has_toggle = false;

  for (auto &[param, title, desc] : toggle_defs) {
    if (param.isEmpty()) {
      label = new LabelControl(title, "");
      addItem(label);
      // Display mode setting removed - using standard behavior
      addItem(hide_hud);
      // Add OSM region display
      addItem(osm_region_display);
      has_toggle = true;
      continue;
    }

    has_toggle = true;
    auto toggle = new ParamControl(param, title, desc, "", this);
    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);
    addItem(toggle);
    toggles[param.toStdString()] = toggle;
  }

  // If no toggles were added, hide the label
  if (!has_toggle && label) {
    label->hide();
  }
}

void NPPanel::add_device_toggles() {
  std::vector<std::tuple<QString, QString, QString>> toggle_defs{
    {
      "",
      tr("■ Device Control"),
      "",
    },
  };

  auto auto_shutdown_toggle = new ParamSpinBoxControl("np_device_auto_shutdown_in", tr("  └ Auto Shutdown In"), tr("0 mins = Immediately"), "", -5, 300, 5, tr(" mins"), tr("Off"));

  QWidget *label = nullptr;
  bool has_toggle = false;

  for (auto &[param, title, desc] : toggle_defs) {
    if (param.isEmpty()) {
      label = new LabelControl(title, "");
      addItem(label);
      continue;
    }

    has_toggle = true;
    auto toggle = new ParamControl(param, title, desc, "", this);
    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);
    addItem(toggle);
    toggles[param.toStdString()] = toggle;
  }
  addItem(auto_shutdown_toggle);
  has_toggle = true;

  // If no toggles were added, hide the label
  if (!has_toggle && label) {
    label->hide();
  }
}

void NPPanel::add_safety_control_systems() {
  addItem(new LabelControl(tr("■ Safety Control Systems"), ""));

  // VRC (Vision-based Rate Controller)
  auto vrc_toggle = new ParamControl("np_vrc_enabled",
                                    tr("  └ VRC (Vision Rate Controller)"),
                                    tr("Limit lateral acceleration to prevent unsafe steering rates"),
                                    "", this);
  addItem(vrc_toggle);
  toggles["np_vrc_enabled"] = vrc_toggle;

  auto vrc_lateral_accel_limit = new ParamDoubleSpinBoxControl("np_vrc_lateral_accel_limit",
                                                              tr("    └ VRC Lateral Acceleration Limit"),
                                                              tr("Maximum allowed lateral acceleration for safety"),
                                                              "", 1.5, 4.0, 0.1, tr(" m/s²"), "");
  addItem(vrc_lateral_accel_limit);

  auto vrc_yaw_rate_limit = new ParamDoubleSpinBoxControl("np_vrc_yaw_rate_limit",
                                                         tr("    └ VRC Yaw Rate Limit"),
                                                         tr("Maximum yaw rate for steering safety"),
                                                         "", 0.1, 1.0, 0.05, tr(" rad/s"), "");
  addItem(vrc_yaw_rate_limit);

  // SOC (Safety Offset Controller)
  auto soc_toggle = new ParamControl("np_soc_enabled",
                                    tr("  └ SOC (Safety Offset Controller)"),
                                    tr("Adjust lateral position to avoid potential collisions"),
                                    "", this);
  addItem(soc_toggle);
  toggles["np_soc_enabled"] = soc_toggle;

  auto soc_tta_threshold = new ParamDoubleSpinBoxControl("np_soc_tta_threshold",
                                                        tr("    └ SOC TTA Activation Threshold"),
                                                        tr("Time-to-approach threshold for SOC activation"),
                                                        "", 3.0, 15.0, 0.5, tr(" sec"), "");
  addItem(soc_tta_threshold);

  auto soc_max_offset = new ParamDoubleSpinBoxControl("np_soc_max_lateral_offset",
                                                     tr("    └ SOC Max Lateral Offset"),
                                                     tr("Maximum lateral offset for collision avoidance"),
                                                     "", 0.5, 2.0, 0.1, tr(" m"), "");
  addItem(soc_max_offset);

  auto soc_reaction_time = new ParamDoubleSpinBoxControl("np_soc_reaction_time",
                                                        tr("    └ SOC Reaction Time"),
                                                        tr("Response time for safety offset maneuvers"),
                                                        "", 0.5, 3.0, 0.1, tr(" sec"), "");
  addItem(soc_reaction_time);
}

void NPPanel::add_system_coordination() {
  addItem(new LabelControl(tr("■ System Coordination"), ""));

  // Master Safety Controls
  auto master_safety_toggle = new ParamControl("np_master_safety_enabled",
                                              tr("  └ Master Safety System"),
                                              tr("Enable master safety coordination and override capabilities"),
                                              "", this);
  addItem(master_safety_toggle);
  toggles["np_master_safety_enabled"] = master_safety_toggle;

  auto safety_override_sensitivity = new ParamSpinBoxControl("np_safety_override_sensitivity",
                                                            tr("    └ Safety Override Sensitivity"),
                                                            tr("Sensitivity level for automatic safety interventions"),
                                                            "", 1, 10, 1, "", "");
  addItem(safety_override_sensitivity);

  // System Health Monitoring
  auto health_monitoring_toggle = new ParamControl("np_system_health_monitoring",
                                                  tr("  └ System Health Monitoring"),
                                                  tr("Enable continuous monitoring of all NagasPilot systems"),
                                                  "", this);
  addItem(health_monitoring_toggle);
  toggles["np_system_health_monitoring"] = health_monitoring_toggle;

  auto health_check_interval = new ParamSpinBoxControl("np_health_check_interval",
                                                      tr("    └ Health Check Interval"),
                                                      tr("Frequency of system health checks"),
                                                      "", 100, 5000, 100, tr(" ms"), "");
  addItem(health_check_interval);

  // Emergency Fallback Settings
  auto emergency_fallback_mode = new ParamControl("np_emergency_fallback_enabled",
                                                 tr("  └ Emergency Fallback Mode"),
                                                 tr("Automatically fallback to OpenPilot in emergency situations"),
                                                 "", this);
  addItem(emergency_fallback_mode);
  toggles["np_emergency_fallback_enabled"] = emergency_fallback_mode;

  auto fallback_trigger_threshold = new ParamSpinBoxControl("np_fallback_trigger_threshold",
                                                           tr("    └ Fallback Trigger Threshold"),
                                                           tr("Number of consecutive system failures before fallback"),
                                                           "", 1, 10, 1, "", "");
  addItem(fallback_trigger_threshold);
}

NPPanel::NPPanel(SettingsWindow *parent) : ListWidget(parent) {
  auto cp_bytes = params.get("CarParamsPersistent");

  if (!cp_bytes.empty()) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();
    brand = QString::fromStdString(CP.getBrand());
    vehicle_has_long_ctrl = hasLongitudinalControl(CP);
    vehicle_has_radar_unavailable = CP.getRadarUnavailable();
  }

  // Initialize default parameter values for all NagasPilot systems
  
  // Foundation system defaults (Off by default for safety)
  if (params.get("np_dcp_mode").empty()) params.put("np_dcp_mode", "0");  // Off by default
  if (params.get("np_dcp_personality").empty()) params.put("np_dcp_personality", "1");  // Standard
  if (params.get("np_dlp_mode").empty()) params.put("np_dlp_mode", "0");  // Off by default
  if (params.get("np_dcp_safety_fallback").empty()) params.putBool("np_dcp_safety_fallback", true);

  // Speed control defaults (all disabled by default)
  if (params.get("np_vtsc_enabled").empty()) params.putBool("np_vtsc_enabled", false);
  if (params.get("np_vtsc_max_lateral_accel").empty()) params.put("np_vtsc_max_lateral_accel", "1.9");
  if (params.get("np_vtsc_lookahead_distance").empty()) params.put("np_vtsc_lookahead_distance", "200");
  if (params.get("np_mtsc_enabled").empty()) params.putBool("np_mtsc_enabled", false);
  if (params.get("np_mtsc_lookahead_distance").empty()) params.put("np_mtsc_lookahead_distance", "200");
  if (params.get("np_mtsc_speed_reduction_factor").empty()) params.put("np_mtsc_speed_reduction_factor", "0.8");
  if (params.get("np_pda_enabled").empty()) params.putBool("np_pda_enabled", false);
  if (params.get("np_pda_min_tta").empty()) params.put("np_pda_min_tta", "15.0");
  if (params.get("np_pda_speed_boost_factor").empty()) params.put("np_pda_speed_boost_factor", "1.2");

  // Safety control defaults (enabled by default for safety)
  if (params.get("np_vrc_enabled").empty()) params.putBool("np_vrc_enabled", true);
  if (params.get("np_vrc_lateral_accel_limit").empty()) params.put("np_vrc_lateral_accel_limit", "2.5");
  if (params.get("np_vrc_yaw_rate_limit").empty()) params.put("np_vrc_yaw_rate_limit", "0.5");
  if (params.get("np_soc_enabled").empty()) params.putBool("np_soc_enabled", true);
  if (params.get("np_soc_tta_threshold").empty()) params.put("np_soc_tta_threshold", "8.0");
  if (params.get("np_soc_max_lateral_offset").empty()) params.put("np_soc_max_lateral_offset", "1.0");
  if (params.get("np_soc_reaction_time").empty()) params.put("np_soc_reaction_time", "1.5");

  // System coordination defaults (enabled by default)
  if (params.get("np_master_safety_enabled").empty()) params.putBool("np_master_safety_enabled", true);
  if (params.get("np_safety_override_sensitivity").empty()) params.put("np_safety_override_sensitivity", "5");
  if (params.get("np_system_health_monitoring").empty()) params.putBool("np_system_health_monitoring", true);
  if (params.get("np_health_check_interval").empty()) params.put("np_health_check_interval", "1000");
  if (params.get("np_emergency_fallback_enabled").empty()) params.putBool("np_emergency_fallback_enabled", true);
  if (params.get("np_fallback_trigger_threshold").empty()) params.put("np_fallback_trigger_threshold", "3");

  // OSM system defaults (Thailand default for SEA region)
  if (params.get("np_osm_region").empty()) params.put("np_osm_region", "thailand");

  // Set fixed LCA values (not user configurable for safety)
  params.put("np_lat_lca_speed", "40");      // 40 km/h threshold (actual value used by desire_helper.py)
  params.put("np_lat_lca_auto_sec", "2.0");  // 2 second auto timer (matches manager.py)
  
  // Set LCA lane width parameters (safety defaults, could be made configurable in future)
  if (params.get("np_lca_min_lane_width").empty()) params.put("np_lca_min_lane_width", "3.0");        // 3.0m minimum for regular lanes
  if (params.get("np_lca_min_edge_width").empty()) params.put("np_lca_min_edge_width", "5.0"); // 5.0m minimum for road edge lanes

  add_lateral_toggles();
  add_longitudinal_toggles();
  add_monitoring_toggles();
  add_ui_toggles();
  add_safety_control_systems();
  add_system_coordination();
  add_device_toggles();

  // Hardware Control - moved to bottom
  addItem(new LabelControl(tr("■ Hardware Control"), ""));
  std::vector<QString> brown_panda_mode_texts{tr("OEM"), tr("INTERRUPT"), tr("OVERRIDE")};
  ButtonParamControl* brown_panda_mode_setting = new ButtonParamControl("np_brown_panda_mode", tr("  └ BrownPanda Mode"),
                                          tr("OEM - BrownPanda uses OEM settings (0).\nINTERRUPT - BrownPanda interrupts the stock system (1).\nOVERRIDE - BrownPanda overrides all control (2)."),
                                          "",
                                          brown_panda_mode_texts);
  addItem(brown_panda_mode_setting);

  auto resetBtn = new ButtonControl(tr("Reset nagaspilot settings"), tr("RESET"));
  connect(resetBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset all settings?"), tr("Reset"), this)) {
      params.putBool("np_device_reset_conf", true);
    }
  });
  addItem(resetBtn);

  fs_watch = new ParamWatcher(this);
  QObject::connect(fs_watch, &ParamWatcher::paramChanged, [=](const QString &param_name, const QString &param_value) {
    updateStates();
  });

  connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    is_onroad = !offroad;
    updateStates();
  });

  updateStates();
}

void NPPanel::showEvent(QShowEvent *event) {
  updateStates();
}

void NPPanel::updateStates() {
  // Watch all critical parameters
  fs_watch->addParam("np_dcp_mode");
  fs_watch->addParam("np_dcp_personality");
  fs_watch->addParam("np_dlp_mode");
  fs_watch->addParam("np_vtsc_enabled");
  fs_watch->addParam("np_mtsc_enabled");
  fs_watch->addParam("np_pda_enabled");
  fs_watch->addParam("np_vrc_enabled");
  fs_watch->addParam("np_soc_enabled");
  fs_watch->addParam("np_master_safety_enabled");
  fs_watch->addParam("np_lat_lca_speed");
  fs_watch->addParam("np_lat_hand_off_enable");
  fs_watch->addParam("np_osm_region");

  if (!isVisible()) {
    return;
  }

  // Foundation system state logic
  int dcp_mode = std::atoi(params.get("np_dcp_mode").c_str());
  int dlp_mode = std::atoi(params.get("np_dlp_mode").c_str());
  
  bool dcp_active = (dcp_mode > 0);
  bool dlp_active = (dlp_mode > 0);
  bool foundation_active = dcp_active || dlp_active;

  // Speed control systems depend on DCP being active
  bool speed_controls_available = dcp_active;
  if (toggles.count("np_vtsc_enabled")) {
    toggles["np_vtsc_enabled"]->setEnabled(speed_controls_available && !is_onroad);
  }
  if (toggles.count("np_mtsc_enabled")) {
    toggles["np_mtsc_enabled"]->setEnabled(speed_controls_available && !is_onroad);
  }
  if (toggles.count("np_pda_enabled")) {
    toggles["np_pda_enabled"]->setEnabled(speed_controls_available && !is_onroad);
  }

  // Safety systems are always available but enhanced when foundation is active
  if (toggles.count("np_vrc_enabled")) {
    toggles["np_vrc_enabled"]->setEnabled(!is_onroad);
  }
  if (toggles.count("np_soc_enabled")) {
    toggles["np_soc_enabled"]->setEnabled(!is_onroad);
  }

  // System coordination depends on any NagasPilot system being active
  bool coordination_needed = foundation_active || 
                            params.getBool("np_vtsc_enabled") || 
                            params.getBool("np_mtsc_enabled") || 
                            params.getBool("np_pda_enabled");
  
  if (toggles.count("np_master_safety_enabled")) {
    toggles["np_master_safety_enabled"]->setEnabled(!is_onroad);
  }
  if (toggles.count("np_system_health_monitoring")) {
    toggles["np_system_health_monitoring"]->setEnabled(!is_onroad);
  }

  // DLP advanced controls - Enable advanced DLP controls only for Laneless (2) and DLP (3) modes
  bool dlp_advanced = (dlp_mode >= 2);
  if (toggles.count("np_dlp_vision_curve")) {
    toggles["np_dlp_vision_curve"]->setEnabled(dlp_advanced && !is_onroad);
  }

  // Hand-off enable toggle - always available
  if (toggles.count("np_lat_hand_off_enable")) {
    toggles["np_lat_hand_off_enable"]->setEnabled(!is_onroad);
  }
  

  // DCP Safety fallback
  if (toggles.count("np_dcp_safety_fallback")) {
    toggles["np_dcp_safety_fallback"]->setEnabled(dcp_active && !is_onroad);
  }

  // Emergency fallback controls
  if (toggles.count("np_emergency_fallback_enabled")) {
    toggles["np_emergency_fallback_enabled"]->setEnabled(!is_onroad);
  }

  // Update OSM region display - GPS-based curvature calculation status
  if (osm_region_display) {
    bool osm_enabled = params.getBool("np_osm_enabled");
    if (osm_enabled) {
      std::string osm_region = params.get("np_osm_region");
      if (osm_region.empty()) {
        osm_region_display->setText("GPS curvature ready - No region");
      } else {
        // Convert region code to readable name with GPS status
        QString display_text;
        if (osm_region == "thailand") {
          display_text = "GPS curvature - Thailand";
        } else if (osm_region == "vietnam") {
          display_text = "GPS curvature - Vietnam";
        } else if (osm_region == "malaysia") {
          display_text = "GPS curvature - Malaysia";
        } else if (osm_region == "thailand_central") {
          display_text = "GPS curvature - Thailand Central";
        } else if (osm_region == "bangkok") {
          display_text = "GPS curvature - Bangkok";
        } else {
          display_text = "GPS curvature - " + QString::fromStdString(osm_region);
        }
        osm_region_display->setText(display_text);
      }
    } else {
      osm_region_display->setText("GPS curvature disabled");
    }
  }

  // Disable all configuration changes while driving for safety
  for (auto& [param_name, toggle] : toggles) {
    if (is_onroad) {
      toggle->setEnabled(false);
    }
  }
}

void NPPanel::expandToggleDescription(const QString &param) {
  toggles[param.toStdString()]->showDescription();
}
