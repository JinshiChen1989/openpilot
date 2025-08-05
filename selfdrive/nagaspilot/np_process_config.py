#!/usr/bin/env python3
"""
NagasPilot Process Configuration

Defines process configurations for NagasPilot-specific components
that can be integrated into the main process manager.
"""

from openpilot.system.manager.process_config import PythonProcess

# NagasPilot process definitions
np_processes = {
  # Audio notification daemon using NagasPilot parameters  
  "npbeepd": PythonProcess(
    "npbeepd", 
    "nagaspilot.selfdrive.ui.beepd",
    enabled=True,
    offroad=False,
  ),
  
  # Note: npmonitoringd removed - architecture follows OpenPilot standard with single hardcoded dmonitoringd
}

def get_np_process_config():
  """Return NagasPilot process configuration dictionary"""
  return np_processes

def integrate_with_main_config(main_config):
  """
  Integrate NagasPilot processes with main process configuration
  
  Args:
    main_config: Main process configuration dictionary
    
  Returns:
    Updated configuration with NagasPilot processes
  """
  # Merge NagasPilot processes with main config
  integrated_config = main_config.copy()
  integrated_config.update(np_processes)
  
  return integrated_config

# Integration example for main process manager
"""
Usage in main process_config.py:

# from nagaspilot.selfdrive.manager.np_process_config import integrate_with_main_config

# After defining main processes
managed_processes = {
  # ... existing processes ...
}

# Integrate NagasPilot processes
if os.path.exists("/data/params/d/np_device_enabled"):
  managed_processes = integrate_with_main_config(managed_processes)
"""