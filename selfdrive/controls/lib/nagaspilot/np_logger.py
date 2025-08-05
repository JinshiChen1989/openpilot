#!/usr/bin/env python3
"""
====================================================================
NAGASPILOT CENTRALIZED LOGGER - UNIFIED DEBUGGING INFRASTRUCTURE
====================================================================

OVERVIEW:
NpLogger provides a centralized, performance-optimized logging system
for all NagasPilot modules, enabling consistent debug output formatting,
module-specific log control, and production-ready performance optimization.

CORE FUNCTIONALITY:
- Module-specific logger instances with standardized prefixes
- Individual module log level control via parameters
- Performance optimization for production environments
- Consistent log formatting across all NagasPilot components
- Integration with OpenPilot's cloudlog infrastructure

LOGGING ARCHITECTURE:
┌─────────────────────────────────────────────────────────────────┐
│                    NpLogger Factory                             │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐│
│  │ NP_DLP      │ │ NP_MTSC     │ │ NP_VTSC     │ │ NP_SOC      ││
│  │ Logger      │ │ Logger      │ │ Logger      │ │ Logger      ││
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘│
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
                  ┌─────────────────┐
                  │ OpenPilot       │
                  │ cloudlog        │
                  └─────────────────┘

USAGE PATTERN:
    from openpilot.selfdrive.controls.lib.nagaspilot.np_logger import NpLogger
    
    np_logger = NpLogger('dlp')  # Initialize for DLP module
    
    # Standard log levels with automatic prefixing
    np_logger.debug("State transition")   # → [NP_DLP] State transition
    np_logger.info("Mode change")         # → [NP_DLP] Mode change  
    np_logger.warning("Parameter issue")  # → [NP_DLP] Parameter issue
    np_logger.error("Critical failure")   # → [NP_DLP] Critical failure

PERFORMANCE FEATURES:
- Conditional logging based on module enable flags
- Optimized for real-time control loop performance
- Minimal overhead when logging is disabled
- Efficient string formatting and processing

SUPPORTED MODULES:
- Core: DLP, DCP (foundation systems)
- Speed Control: MTSC, VTSC, VCSC, APSL, BPSL (filter layers)
- Safety: HOD, SSD, SOC (monitoring systems)
- Navigation: PDA, OSM, RED (positioning and avoidance)
- Override: LCA, APOM (driver assist and intervention)
- Each module has individual logging control via parameters
- Consistent formatting and behavior across all modules
    
    # NP-prefixed methods (aliases for consistency)
    np_logger.np_debug("Debug message")  # → [NP_LCA] Debug message
    np_logger.np_warning("Warning message")  # → [NP_LCA] Warning message
    
    # PlotJuggler plotting methods
    np_logger.plot_value("speed", 25.3, "m/s")  # → [NP_LCA]_PLOT np_speed=25.300000m/s
    np_logger.np_plot_value("speed", 25.3, "m/s")  # Same as above
    np_logger.plot_event("active", True)  # → [NP_LCA]_EVENT np_active=1.0
    np_logger.np_plot_event("active", True)  # Same as above

Features:
- Per-module logging enable/disable via parameters (1=enable, 0=disable)
- Consistent logging format across all NagasPilot modules
- Easy to disable all debug logs while keeping warnings/errors
- Backward compatible with existing cloudlog usage
- Works with PlotJuggler via Ethernet bridge for real-time debugging
- Saves logs locally in /data/log/ for offline analysis (same directory as OpenPilot)
- Log rotation: 60-second intervals OR 256KB files, 100 files retention (same as OpenPilot)

Note: OpenPilot uses cereal messaging, not ROS. For ROS bag compatibility, 
you would need a separate bridge application to convert cereal messages to ROS topics.
"""

import os
import time
import glob
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

# Note: OpenPilot uses cereal for messaging, not ROS
# True ROS bag support would require additional bridge software

# Local file logging configuration - same directory as OpenPilot swaglog
# Use /tmp for simulation mode to avoid permission issues
if os.getenv('SIMULATION') == '1':
    LOG_DIR = "/tmp/dragonpilot_logs"  # Simulation mode: use writable tmp directory
else:
    LOG_DIR = "/data/log"  # Production mode: same as OpenPilot's swaglog directory
os.makedirs(LOG_DIR, exist_ok=True)

# Log rotation settings - matching OpenPilot's swaglog system
LOG_ROTATION_INTERVAL = 60   # 60 seconds (same as OpenPilot)
LOG_MAX_BYTES = 1024 * 256   # 256KB per file (same as OpenPilot)
LOG_BACKUP_COUNT = 100       # Keep 100 files per module (~6.4MB total per module)

class NpLogger:
    """Centralized logger for NagasPilot modules with per-module enable/disable control"""
    
    # Module name to parameter mapping
    MODULE_PARAMS = {
        'lca': 'np_log_lca_enabled',           # Lane Change Assist
        'hod': 'np_log_hod_enabled',           # Hand Off Duration
        'ssd': 'np_log_ssd_enabled',           # Stand Still Duration
        'mtsc': 'np_log_mtsc_enabled',         # Map Turn Speed Control
        'vtsc': 'np_log_vtsc_enabled',         # Vision Turn Speed Control
        'vcsc': 'np_log_vcsc_enabled',         # Vertical Comfort Speed Control
        'soc': 'np_log_soc_enabled',           # Smart Offset Controller
        'pda': 'np_log_pda_enabled',           # Predictive Dynamic Acceleration
        'apsl': 'np_log_apsl_enabled',         # Accelerator Pedal Speed Learning
        'bpsl': 'np_log_bpsl_enabled',         # Brake Pedal Speed Learning
        'dcp': 'np_log_dcp_enabled',           # Dynamic Cruise Profiles
        'dlp': 'np_log_dlp_enabled',           # Dynamic Lane Profiling
        'osm': 'np_log_osm_enabled',           # OpenStreetMap Backend
        'red': 'np_log_red_enabled',           # Road Edge Detection
        'gcf': 'np_log_gcf_enabled',           # Gradient Compensation Factor
    }
    
    def __init__(self, module_name):
        """
        Initialize logger for specific module
        
        Args:
            module_name (str): Module name (lca, hod, ssd, mtsc, vtsc, vcsc, soc, pda, apsl, bpsl, dcp, dlp, osm, red, gcf)
        """
        self.module_name = module_name.lower()
        self.params = Params()
        
        if self.module_name not in self.MODULE_PARAMS:
            raise ValueError(f"Unknown module: {module_name}. Valid modules: {list(self.MODULE_PARAMS.keys())}")
            
        self.param_key = self.MODULE_PARAMS[self.module_name]
        self.prefix = f"[{self.module_name.upper()}]"
        
        # Local file logging setup with rotation (matching OpenPilot swaglog format)
        self.log_file_base = os.path.join(LOG_DIR, f"np_{self.module_name}")  # np_lca, np_hod, etc.
        self.current_log_file = None
        self.log_start_time = 0
        self.current_file_size = 0
        self.file_index = self._get_last_file_index()
        self.local_logging_enabled = True  # Can be controlled via parameter if needed
        
    
    def _is_logging_enabled(self):
        """Check if logging is enabled for this module (1=enabled, 0=disabled)"""
        return self.params.get_int(self.param_key, 0) == 1
    
    def _get_last_file_index(self):
        """Get the last file index for this module (like OpenPilot's swaglog)"""
        try:
            pattern = f"{self.log_file_base}.*"
            log_files = glob.glob(pattern)
            if not log_files:
                return 0
                
            # Extract indices from existing files
            indices = []
            for log_file in log_files:
                try:
                    index_str = log_file.split('.')[-1]
                    if index_str.isdigit():
                        indices.append(int(index_str))
                except:
                    # ⚠️ BROAD: Bare except could mask important file system errors
                    # TODO: Catch specific exceptions (IndexError, ValueError) for better error handling
                    continue
            
            return max(indices) if indices else 0
        except:
            # ⚠️ BROAD: Bare except could mask important file system errors
            # TODO: Catch specific exceptions (OSError, PermissionError) for better error handling
            return 0
    
    def _rotate_log_if_needed(self):
        """Rotate log file based on time and size (matching OpenPilot's swaglog system)"""
        current_time = time.time()
        
        # Check if we need to rotate (60 seconds OR 256KB size limit)
        time_exceeded = (self.current_log_file is None or 
                        current_time - self.log_start_time >= LOG_ROTATION_INTERVAL)
        size_exceeded = self.current_file_size >= LOG_MAX_BYTES
        
        if time_exceeded or size_exceeded:
            # Generate new log filename with index (like OpenPilot's swaglog.0000000001)
            self.file_index += 1
            self.current_log_file = f"{self.log_file_base}.{self.file_index:010d}"
            self.log_start_time = current_time
            self.current_file_size = 0
            
            # Clean up old log files (keep only last 100 files per module)
            self._cleanup_old_logs()
    
    def _cleanup_old_logs(self):
        """Remove old log files (keep only last 100 files like OpenPilot)"""
        try:
            pattern = f"{self.log_file_base}.*"
            log_files = glob.glob(pattern)
            
            # Sort by file index and keep only the last LOG_BACKUP_COUNT files
            log_files_with_index = []
            for log_file in log_files:
                try:
                    # Extract index from filename like "np_lca.0000000123"
                    index_str = log_file.split('.')[-1]
                    if index_str.isdigit():
                        log_files_with_index.append((int(index_str), log_file))
                except:
                    continue
            
            # Sort by index and remove old files
            log_files_with_index.sort()
            if len(log_files_with_index) > LOG_BACKUP_COUNT:
                files_to_remove = log_files_with_index[:-LOG_BACKUP_COUNT]
                for _, log_file in files_to_remove:
                    os.remove(log_file)
                    
        except Exception:
            # ⚠️ SILENT: Exception suppression could hide important disk space or permission issues
            # TODO: Log cleanup errors to help diagnose storage issues
            pass
    
    
    def _write_to_local_file(self, level, message):
        """Write log message to local file with timestamp and rotation (matching OpenPilot format)"""
        if not self.local_logging_enabled:
            return
            
        try:
            # Rotate log if needed (based on time and size like OpenPilot)
            self._rotate_log_if_needed()
            
            # Timestamp with seconds precision
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            log_entry = f"{timestamp} [{level.upper()}] {self.prefix} {message}\n"
            
            # Write to file and track size
            with open(self.current_log_file, 'a', encoding='utf-8') as f:
                f.write(log_entry)
                self.current_file_size += len(log_entry.encode('utf-8'))
                
        except Exception as e:
            # ⚠️ SILENT: Exception suppression could hide important file system issues
            # TODO: Consider logging to cloudlog when local file logging fails
            pass
    
    def debug(self, message):
        """Log debug message if module logging is enabled"""
        if self._is_logging_enabled():
            # Send to cloudlog for PlotJuggler/Ethernet bridge compatibility
            cloudlog.debug(f"{self.prefix} {message}")
            # Save locally with rotation
            self._write_to_local_file('debug', message)
    
    def info(self, message):
        """Log info message if module logging is enabled"""
        if self._is_logging_enabled():
            # Send to cloudlog for PlotJuggler/Ethernet bridge compatibility
            cloudlog.info(f"{self.prefix} {message}")
            # Save locally with rotation
            self._write_to_local_file('info', message)
    
    def warning(self, message):
        """Log warning message (always enabled for safety)"""
        # Send to cloudlog for PlotJuggler/Ethernet bridge compatibility
        cloudlog.warning(f"{self.prefix} {message}")
        # Save locally with rotation
        self._write_to_local_file('warning', message)
    
    def error(self, message):
        """Log error message (always enabled for safety)"""
        # Send to cloudlog for PlotJuggler/Ethernet bridge compatibility
        cloudlog.error(f"{self.prefix} {message}")
        # Save locally with rotation
        self._write_to_local_file('error', message)
    
    def plot_value(self, name, value, unit=""):
        """Log a numeric value optimized for PlotJuggler plotting"""
        if self._is_logging_enabled():
            # Format specifically for PlotJuggler numeric extraction with np_ prefix
            plot_msg = f"{self.prefix}_PLOT np_{name}={value:.6f}{unit}"
            cloudlog.info(plot_msg)
            self._write_to_local_file('plot', f"np_{name}={value:.6f}{unit}")
    
    def plot_event(self, event_name, state=True):
        """Log an event state optimized for PlotJuggler plotting"""
        if self._is_logging_enabled():
            value = 1.0 if state else 0.0
            plot_msg = f"{self.prefix}_EVENT np_{event_name}={value:.1f}"
            cloudlog.info(plot_msg)
            self._write_to_local_file('event', f"np_{event_name}={value:.1f}")
    
    # Convenience methods with np_ prefix for consistency
    def np_debug(self, message):
        """Log debug message with np_ prefix (alias for debug)"""
        self.debug(message)
    
    def np_info(self, message):
        """Log info message with np_ prefix (alias for info)"""
        self.info(message)
    
    def np_warning(self, message):
        """Log warning message with np_ prefix (alias for warning)"""
        self.warning(message)
    
    def np_error(self, message):
        """Log error message with np_ prefix (alias for error)"""
        self.error(message)
    
    def np_plot_value(self, name, value, unit=""):
        """Log numeric value with np_ prefix (alias for plot_value)"""
        self.plot_value(name, value, unit)
    
    def np_plot_event(self, event_name, state=True):
        """Log event state with np_ prefix (alias for plot_event)"""
        self.plot_event(event_name, state)

# Convenience functions for quick usage without instantiating class
def log_debug(module_name, message):
    """Quick debug logging function"""
    # ⚠️ PERFORMANCE: Creates new NpLogger instance for each call - inefficient
    # TODO: Consider caching NpLogger instances for better performance
    logger = NpLogger(module_name)
    logger.debug(message)

def log_info(module_name, message):
    """Quick info logging function"""
    logger = NpLogger(module_name)
    logger.info(message)

def log_warning(module_name, message):
    """Quick warning logging function"""
    logger = NpLogger(module_name)
    logger.warning(message)

def log_error(module_name, message):
    """Quick error logging function"""
    logger = NpLogger(module_name)
    logger.error(message)