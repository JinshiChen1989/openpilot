#include "selfdrive/ui/qt/offroad/trip_panel.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include <QTimer>
#include <QShowEvent>
#include <QHideEvent>
#include <string>
#include <stdexcept>

TripPanel::TripPanel(SettingsWindow *parent) : ListWidget(parent) {
  setupUI();
  loadTripData();
  updateTripDisplays();
  
  // Create timer but don't start it yet - only start when panel becomes visible
  update_timer = new QTimer(this);
  connect(update_timer, &QTimer::timeout, this, &TripPanel::updateTripDisplays);
}

void TripPanel::setupUI() {
  // Lifetime stats section (top row)
  addItem(new LabelControl(tr("■ Lifetime Statistics"), ""));
  
  lifetime_distance_label = new LabelControl(tr("Total Distance"), "0 km");
  addItem(lifetime_distance_label);
  
  lifetime_time_label = new LabelControl(tr("Total Driving Time"), "0h 0m");
  addItem(lifetime_time_label);
  
  lifetime_interventions_label = new LabelControl(tr("Total Interventions"), "0");
  addItem(lifetime_interventions_label);
  
  // Current session stats section (bottom row)  
  addItem(new LabelControl(tr("■ Current Session Statistics"), ""));
  
  current_distance_label = new LabelControl(tr("Session Distance"), "0 km");
  addItem(current_distance_label);
  
  current_time_label = new LabelControl(tr("Session Driving Time"), "0h 0m");
  addItem(current_time_label);
  
  current_interventions_label = new LabelControl(tr("Session Interventions"), "0");
  addItem(current_interventions_label);
}

void TripPanel::loadTripData() {
  // Initialize persistent lifetime parameters if they don't exist
  if (params.get("np_trip_lifetime_distance").empty()) params.put("np_trip_lifetime_distance", "0.0");
  if (params.get("np_trip_lifetime_time").empty()) params.put("np_trip_lifetime_time", "0");
  if (params.get("np_trip_lifetime_interventions").empty()) params.put("np_trip_lifetime_interventions", "0");
  
  // Current session parameters are reset by manager on boot (CLEAR_ON_MANAGER_START)
  // Initialize them to 0 if they don't exist (safe fallback)
  if (params.get("np_trip_current_distance").empty()) params.put("np_trip_current_distance", "0.0");
  if (params.get("np_trip_current_time").empty()) params.put("np_trip_current_time", "0");
  if (params.get("np_trip_current_interventions").empty()) params.put("np_trip_current_interventions", "0");
}

void TripPanel::updateTripDisplays() {
  try {
    // Get current values from parameters with safe conversion
    std::string lifetime_distance_str = params.get("np_trip_lifetime_distance");
    std::string lifetime_time_str = params.get("np_trip_lifetime_time");
    std::string lifetime_interventions_str = params.get("np_trip_lifetime_interventions");
    
    std::string current_distance_str = params.get("np_trip_current_distance");
    std::string current_time_str = params.get("np_trip_current_time");
    std::string current_interventions_str = params.get("np_trip_current_interventions");
    
    // Safe conversion with fallback to 0
    double lifetime_distance = lifetime_distance_str.empty() ? 0.0 : std::stod(lifetime_distance_str);
    int lifetime_time = lifetime_time_str.empty() ? 0 : std::stoi(lifetime_time_str);
    int lifetime_interventions = lifetime_interventions_str.empty() ? 0 : std::stoi(lifetime_interventions_str);
    
    double current_distance = current_distance_str.empty() ? 0.0 : std::stod(current_distance_str);
    int current_time = current_time_str.empty() ? 0 : std::stoi(current_time_str);
    int current_interventions = current_interventions_str.empty() ? 0 : std::stoi(current_interventions_str);
    
    // Update lifetime displays
    lifetime_distance_label->setValue(formatDistance(lifetime_distance));
    lifetime_time_label->setValue(formatTime(lifetime_time));
    lifetime_interventions_label->setValue(formatInterventions(lifetime_interventions));
    
    // Update current session displays
    current_distance_label->setValue(formatDistance(current_distance));
    current_time_label->setValue(formatTime(current_time));
    current_interventions_label->setValue(formatInterventions(current_interventions));
  } catch (const std::exception& e) {
    // Handle conversion errors gracefully - display default values
    lifetime_distance_label->setValue("0 km");
    lifetime_time_label->setValue("0h 0m");
    lifetime_interventions_label->setValue("0");
    
    current_distance_label->setValue("0 km");
    current_time_label->setValue("0h 0m");
    current_interventions_label->setValue("0");
  }
}


QString TripPanel::formatDistance(double meters) {
  if (meters >= 1000.0) {
    return QString("%1 km").arg(meters / 1000.0, 0, 'f', 1);
  } else {
    return QString("%1 m").arg((int)meters);
  }
}

QString TripPanel::formatTime(int seconds) {
  int hours = seconds / 3600;
  int minutes = (seconds % 3600) / 60;
  
  if (hours > 0) {
    return QString("%1h %2m").arg(hours).arg(minutes);
  } else {
    return QString("%1m").arg(minutes);
  }
}

QString TripPanel::formatInterventions(int count) {
  return QString("%1").arg(count);
}

void TripPanel::showEvent(QShowEvent *event) {
  updateTripDisplays();
  // Start timer when panel becomes visible
  update_timer->start(5000);  // Update every 5 seconds
}

void TripPanel::hideEvent(QHideEvent *event) {
  // Stop timer when panel is hidden to save resources
  update_timer->stop();
}