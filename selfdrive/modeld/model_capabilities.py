from enum import IntFlag, auto


class ModelCapabilities(IntFlag):
  """
  Model capabilities for nagaspilot DLP implementation.
  
  Simplified to focus only on DLP (Dynamic Lane Profile) capability.
  """
  Default = auto()
  """Default capability for standard nagaspilot operation."""
  
  LateralPlannerSolution = auto()
  """DLP capability - enables Dynamic Lane Profile lateral planning."""
  
  @staticmethod
  def get_by_gen(gen):
    """
    Get model capabilities by generation number.
    
    For nagaspilot DLP:
    - Generation 1: Has DLP capability (LateralPlannerSolution)
    - All others: Default only (no DLP)
    
    Args:
      gen (int): Model generation number
      
    Returns:
      ModelCapabilities: Combined capabilities for the generation
    """
    if gen == 1:
      # Generation 1 = DLP enabled
      return ModelCapabilities.Default | ModelCapabilities.LateralPlannerSolution
    else:
      # All other generations = standard nagaspilot (no DLP)
      return ModelCapabilities.Default