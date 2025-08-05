import os
from openpilot.common.params_pyx import Params as _Params, ParamKeyType, UnknownKeyName

# Create wrapper class to handle type issues
class Params(_Params):
    def put(self, key, value):
        # Handle different parameter types
        if isinstance(value, str):
            # Check if this should be an integer parameter (like PIDs)
            if key.endswith('Pid') or value.isdigit():
                try:
                    return self.put_int(key, int(value))
                except:
                    pass
            # Check if this should be a boolean parameter
            elif value in ('0', '1', 'true', 'false'):
                try:
                    return self.put_bool(key, value in ('1', 'true'))
                except:
                    pass
            # Fallback to string parameter
            try:
                return super().put(key, value.encode('utf-8'))
            except:
                # If all else fails, silently skip in simulation mode
                if os.getenv("SIMULATION"):
                    return
                raise
        return super().put(key, value)
    
    def get(self, key, default=None, encoding=None):
        # Handle encoding parameter for compatibility
        try:
            result = super().get(key, default)
            if result is not None and encoding:
                if isinstance(result, bytes):
                    return result.decode(encoding)
            return result
        except UnknownKeyName:
            # Return default for unknown keys in simulation mode
            if os.getenv("SIMULATION"):
                return default
            raise
    
    def get_bool(self, key, default=False):
        # Handle boolean parameter access with fallback
        try:
            return super().get_bool(key)
        except UnknownKeyName:
            # Return default for unknown keys in simulation mode
            if os.getenv("SIMULATION"):
                return default
            raise

# Add missing ParamKeyType values for compatibility
if not hasattr(ParamKeyType, 'CLEAR_ON_MANAGER_START'):
    ParamKeyType.CLEAR_ON_MANAGER_START = 0x04
if not hasattr(ParamKeyType, 'CLEAR_ON_ONROAD_TRANSITION'):
    ParamKeyType.CLEAR_ON_ONROAD_TRANSITION = 0x08
if not hasattr(ParamKeyType, 'CLEAR_ON_OFFROAD_TRANSITION'):
    ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION = 0x10
if not hasattr(ParamKeyType, 'DONT_LOG'):
    ParamKeyType.DONT_LOG = 0x20

assert Params
assert ParamKeyType
assert UnknownKeyName

if __name__ == "__main__":
  import sys

  params = Params()
  key = sys.argv[1]
  assert params.check_key(key), f"unknown param: {key}"

  if len(sys.argv) == 3:
    val = sys.argv[2]
    print(f"SET: {key} = {val}")
    params.put(key, val)
  elif len(sys.argv) == 2:
    print(f"GET: {key} = {params.get(key)}")
