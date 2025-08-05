"""
====================================================================
NAGASPILOT INTEGRATION MODULE - CORE INITIALIZATION
====================================================================

OVERVIEW:
This module provides the foundational integration layer between NagasPilot
and OpenPilot systems, handling model generation detection and core
initialization patterns that follow OpenPilot organizational standards.

CORE FUNCTIONALITY:
- Model generation detection for DLP capability assessment
- Parameter validation and system compatibility checking  
- Integration point for NagasPilot-specific functionality
- Follows OpenPilot organizational patterns for consistency

KEY FUNCTIONS:
- get_model_generation(): Determines DLP compatibility and model generation
- SIMULATION detection for testing environments
- Parameter validation with secure defaults

INTEGRATION POINTS:
- OpenPilot Parameters system for configuration management
- Model generation detection for feature capability
- Simulation environment detection for testing
- DLP mode validation and enablement logic

CODE REVIEW NOTES:
- Model generation logic: get_model_generation() (line ~13)
- DLP enablement: Based on np_dlp_mode parameter (modes 2-3)
- Simulation handling: Automatic DLP disable in simulation
- Parameter defaults: Safe fallbacks for all configurations
"""

import os

SIMULATION = "SIMULATION" in os.environ


def get_model_generation(params):
    """
    Get model generation for nagaspilot DLP.
    
    Similar to sunnypilot's implementation but uses nagaspilot-specific parameters:
    - np_dlp_mode: Unified lateral control mode (2=Laneless, 3=DLP enables DLP functionality)  
    - np_dlp_model_gen: Model generation (defaults to 1 for DLP-capable)
    
    Args:
        params (Params): Parameters instance for reading config
        
    Returns:
        tuple: (dlp_enabled, generation)
            - dlp_enabled (bool): True if DLP is enabled
            - generation (int): Model generation (1 = DLP-capable)
    """
    try:
        # Check if nagaspilot DLP is enabled based on unified mode (2=Laneless, 3=DLP)
        dlp_mode = int(params.get("np_dlp_mode", "0"))
        dlp_enabled = (dlp_mode >= 2) and not SIMULATION
        
        # Get model generation (nagaspilot always uses generation 1 for DLP)
        generation = int(params.get("np_dlp_model_gen", encoding="utf8")) if params.get("np_dlp_model_gen") else 1
        
        return dlp_enabled, generation
    except Exception:
        # Fallback if parameter access fails
        return False, 1