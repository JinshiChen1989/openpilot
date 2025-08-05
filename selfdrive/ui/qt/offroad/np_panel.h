#pragma once

#include "selfdrive/ui/qt/offroad/settings.h"

class NPPanel : public ListWidget {
  Q_OBJECT
public:
  explicit NPPanel(SettingsWindow *parent);

public slots:
  void expandToggleDescription(const QString &param);

private:
  Params params;
  ParamWatcher *fs_watch;
  std::map<std::string, ParamControl*> toggles;
  QString brand;
  bool is_onroad = false;
  bool vehicle_has_long_ctrl;
  bool vehicle_has_radar_unavailable;

  void add_lateral_toggles();
  void add_longitudinal_toggles();
  void add_monitoring_toggles();
  void add_ui_toggles();
  void add_device_toggles();
  void add_safety_control_systems();
  void add_system_coordination();
  void updateStates();
  void showEvent(QShowEvent *event) override;

  ParamDoubleSpinBoxControl* lca_sec_toggle;
  LabelControl* osm_region_display;
  // Offset controls removed - using fixed engineering values (0.00) only
};
