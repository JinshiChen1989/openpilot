#pragma once

#include "selfdrive/ui/qt/offroad/settings.h"

class TripPanel : public ListWidget {
  Q_OBJECT
public:
  explicit TripPanel(SettingsWindow *parent);
  
public slots:
  void updateTripDisplays();
  
private:
  Params params;
  QTimer *update_timer;
  
  // Display elements for lifetime stats (top row)
  LabelControl *lifetime_distance_label;
  LabelControl *lifetime_time_label; 
  LabelControl *lifetime_interventions_label;
  
  // Display elements for current session stats (bottom row)
  LabelControl *current_distance_label;
  LabelControl *current_time_label;
  LabelControl *current_interventions_label;
  
  void loadTripData();
  void setupUI();
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  
  // Helper functions for formatting
  QString formatDistance(double meters);
  QString formatTime(int seconds);
  QString formatInterventions(int count);
};