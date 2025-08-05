# Trip Panel Migration Plan

## Overview
Replace developer panel with a clean trip statistics panel showing driving data with 6 number displays in two rows. SSH functionality will be handled in background per SSH removal plan.

## Design Specification

### Panel Layout
**Top Row (Lifetime Statistics):**
- Total Distance (driven by openpilot)
- Total Time (driven by openpilot) 
- Total Interventions (driver interruptions)

**Bottom Row (Current Session Statistics):**
- Session Distance (since current boot)
- Session Time (since current boot)
- Session Interventions (since current boot)

### Features
- **Read-only display**: No buttons, scrolling, or interactive elements
- **Auto-updating**: Updates every 5 seconds when panel is visible
- **Clean formatting**: Distance in km/m, time in hours/minutes, interventions as count
- **Persistent data**: Lifetime stats survive reboots, session stats reset on boot

## Implementation

### Files Created
1. `/porting/trip/trip_panel.h` - Header file defining TripPanel class
2. `/porting/trip/trip_panel.cc` - Implementation with 6 LabelControl displays
3. `/porting/trip/trip_tracker.py` - Background daemon to track driving statistics

### Data Parameters
- `TripLifetimeDistance` - Total meters driven (persistent)
- `TripLifetimeTime` - Total seconds driven (persistent) 
- `TripLifetimeInterventions` - Total driver interruptions (persistent)
- `TripCurrentDistance` - Session meters (reset on boot)
- `TripCurrentTime` - Session seconds (reset on boot)  
- `TripCurrentInterventions` - Session interruptions (reset on boot)

### Integration Points

#### 1. Replace Developer Panel in Settings Menu
In `selfdrive/ui/qt/offroad/settings.cc`:
```cpp
#include "porting/trip/trip_panel.h"
// Replace DeveloperPanel with TripPanel
panel_widgets.insert({"Trip", new TripPanel(this)});
// Remove or comment out:
// panel_widgets.insert({"Developer", new DeveloperPanel(this)});
```

#### 2. Add Trip Tracker to Manager
Add to `system/manager/process_config.py`:
```python
PythonProcess("trip_tracker", "porting.trip.trip_tracker", enabled=True, onroad=False, offroad=True),
```

#### 3. Build System Integration
Add to `SConscript` for UI compilation:
```python
qt_widgets += ["porting/trip/trip_panel.cc"]
```

## Data Collection Method

### Trip Tracker Process
- Runs in background during offroad state
- Subscribes to `carState`, `controlsState`, `liveLocationKalman` 
- Updates parameters every second when openpilot is engaged
- Detects interventions on engagement state changes
- Calculates distance from velocity and time deltas
- Persists data to Params system

### Tracking Logic
- **Distance**: Integrated from velocity when engaged and moving (>0.5 m/s)
- **Time**: Accumulated when openpilot is engaged
- **Interventions**: Counted when engagement state changes from enabled to disabled

## SSH Handling
SSH functionality will be handled in background as per SSH removal plan:
- Developer panel UI removed
- SSH keys managed automatically in background
- Hardcoded GitHub username via parameters
- No user interaction needed

## Migration Benefits
- **Cleaner interface**: Pure information display without SSH complexity
- **Useful statistics**: Actual driving metrics for users
- **Minimal footprint**: Simple read-only design
- **Background SSH**: Automatic SSH management without UI clutter
- **Performance**: Lightweight background tracking

## Next Steps
1. Replace developer panel with trip panel in settings
2. Add trip_tracker to manager process list  
3. Implement background SSH management per SSH removal plan
4. Test data collection accuracy
5. Validate UI display formatting