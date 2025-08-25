import time
import threading
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from cereal import messaging

# ========================================================================
# CONSTANTS & CONFIGURATION
# ========================================================================

# Tracking Parameters
UPDATE_RATE_HZ = 1.0           # Update frequency (1 Hz for efficiency)
MIN_SPEED_THRESHOLD = 0.1      # Minimum speed to track (m/s)
MAX_REASONABLE_SPEED = 100.0   # Maximum reasonable speed (m/s = 360 km/h)
MAX_TIME_DELTA = 3600.0        # Maximum reasonable time delta (1 hour)

# Data Limits for Safety
MAX_LIFETIME_DISTANCE = 1000000000.0   # 1M km in meters
MAX_SESSION_DISTANCE = 10000000.0      # 10k km in meters
MAX_LIFETIME_TIME = 315000000          # 10 years in seconds
MAX_SESSION_TIME = 86400               # 24 hours in seconds
MAX_LIFETIME_INTERVENTIONS = 100000    # Maximum intervention count
MAX_SESSION_INTERVENTIONS = 1000       # Maximum interventions per session

# ========================================================================
# TRIP TRACKER IMPLEMENTATION
# ========================================================================

class TripTracker:
  # ========================================================================
  # INITIALIZATION & CONFIGURATION
  # ========================================================================

  def __init__(self):
    """Initialize trip tracker with messaging and state management"""
    self.params = Params()
    self.sm = messaging.SubMaster(['carState', 'controlsState', 'liveLocationKalman'])

    # Session timing
    self.session_start_time = time.time()
    self.last_update_time = time.time()
    self.last_position = None

    # State tracking with atomic operations
    self.is_engaged = False
    self.last_engaged_state = False

    # Thread control
    self.running = True

  # ========================================================================
  # THREAD MANAGEMENT
  # ========================================================================

  def start(self):
    """Start the trip tracking thread as daemon"""
    self.thread = threading.Thread(target=self.run, daemon=True)
    self.thread.start()

  def stop(self):
    """Stop the trip tracking thread gracefully"""
    self.running = False
    if hasattr(self, 'thread'):
      self.thread.join()

  def run(self):
    """Main tracking loop with rate limiting"""
    rk = Ratekeeper(UPDATE_RATE_HZ)  # 1 Hz update rate

    while self.running:
      self.sm.update()
      current_time = time.time()

      if self.sm.updated['carState'] and self.sm.updated['controlsState']:
        self.update_trip_data(current_time)

      rk.keep_time()

  # ========================================================================
  # DATA PROCESSING
  # ========================================================================

  def update_trip_data(self, current_time):
    """Update trip statistics"""
    try:
      car_state = self.sm['carState']
      controls_state = self.sm['controlsState']

      # Atomic state transition check
      new_engaged = controls_state.enabled
      prev_engaged = self.last_engaged_state

      # Track interventions (when driver takes control)
      if prev_engaged and not new_engaged:
        # Disengagement detected - count as intervention
        self.increment_interventions()

      # Update state atomically
      self.is_engaged = new_engaged
      self.last_engaged_state = new_engaged

      # Only track distance and time when openpilot is engaged and moving
      if self.is_engaged and car_state.vEgo > MIN_SPEED_THRESHOLD:
        time_delta = current_time - self.last_update_time

        # Update driving metrics
        self.update_driving_time(time_delta)
        self.update_distance(time_delta, car_state.vEgo)

      self.last_update_time = current_time

    except Exception as e:
      print(f"TripTracker error: {e}")

  # ========================================================================
  # METRIC UPDATES
  # ========================================================================

  def update_distance(self, time_delta, velocity):
    """Update distance traveled with validation and safety limits"""
    # Input validation
    if time_delta <= 0 or velocity < 0 or velocity > MAX_REASONABLE_SPEED:
      return

    distance_delta = velocity * time_delta  # meters

    try:
      # Update lifetime distance with safety limits
      lifetime_distance = self.params.get_float("np_trip_lifetime_distance", 0.0)
      lifetime_distance += distance_delta
      lifetime_distance = min(lifetime_distance, MAX_LIFETIME_DISTANCE)
      self.params.put_float("np_trip_lifetime_distance", lifetime_distance)

      # Update current session distance with safety limits
      current_distance = self.params.get_float("np_trip_current_distance", 0.0)
      current_distance += distance_delta
      current_distance = min(current_distance, MAX_SESSION_DISTANCE)
      self.params.put_float("np_trip_current_distance", current_distance)
    except (ValueError, TypeError):
      # Handle corrupted parameters by resetting
      self.params.put_float("np_trip_lifetime_distance", 0.0)
      self.params.put_float("np_trip_current_distance", 0.0)

  def update_driving_time(self, time_delta):
    """Update driving time with validation and safety limits"""
    # Input validation
    if time_delta <= 0 or time_delta > MAX_TIME_DELTA:
      return

    try:
      # Update lifetime time with safety limits
      lifetime_time = self.params.get_int("np_trip_lifetime_time", 0)
      lifetime_time += int(time_delta)
      lifetime_time = min(lifetime_time, MAX_LIFETIME_TIME)
      self.params.put_int("np_trip_lifetime_time", lifetime_time)

      # Update current session time with safety limits
      current_time = self.params.get_int("np_trip_current_time", 0)
      current_time += int(time_delta)
      current_time = min(current_time, MAX_SESSION_TIME)
      self.params.put_int("np_trip_current_time", current_time)
    except (ValueError, TypeError):
      # Handle corrupted parameters by resetting
      self.params.put_int("np_trip_lifetime_time", 0)
      self.params.put_int("np_trip_current_time", 0)

  def increment_interventions(self):
    """Increment intervention counters with safety limits"""
    try:
      # Update lifetime interventions with safety limits
      lifetime_interventions = self.params.get_int("np_trip_lifetime_interventions", 0)
      lifetime_interventions += 1
      lifetime_interventions = min(lifetime_interventions, MAX_LIFETIME_INTERVENTIONS)
      self.params.put_int("np_trip_lifetime_interventions", lifetime_interventions)

      # Update current session interventions with safety limits
      current_interventions = self.params.get_int("np_trip_current_interventions", 0)
      current_interventions += 1
      current_interventions = min(current_interventions, MAX_SESSION_INTERVENTIONS)
      self.params.put_int("np_trip_current_interventions", current_interventions)
    except (ValueError, TypeError):
      # Handle corrupted parameters by resetting
      self.params.put_int("np_trip_lifetime_interventions", 0)
      self.params.put_int("np_trip_current_interventions", 0)

  # ========================================================================
  # SESSION MANAGEMENT
  # ========================================================================

  def reset_current_session(self):
    """Reset current session statistics (called on system restart)"""
    self.params.put_float("np_trip_current_distance", 0.0)
    self.params.put_int("np_trip_current_time", 0)
    self.params.put_int("np_trip_current_interventions", 0)

# ========================================================================
# STANDALONE OPERATION
# ========================================================================


def main():
    """Main function for standalone operation"""
    tracker = TripTracker()

    # No need to reset session - parameters handle lifecycle automatically
    # Session data now persists and is managed by the parameter system

    try:
        tracker.start()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        tracker.stop()


if __name__ == "__main__":
    main()