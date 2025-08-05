# PDA (Parallel Drive Avoidance) - Implementation Status
**Simple Anchor Car Overtaking with TTC Safety**

---

## **Executive Summary**

**STATUS**: ✅ **IMPLEMENTED** - PDA operates as simple overtaking controller with TTC safety monitoring
**ARCHITECTURE**: Clean state machine with anchor car detection and intelligent boost logic
**COORDINATION**: Independent operation with SOC safety coordination via status communication

**CORE FEATURES**:
- **Anchor Car Detection**: Monitors adjacent lane vehicles for overtaking opportunities
- **Simple State Machine**: MONITORING → DETECTING → BOOSTING → COOLDOWN
- **TTC Safety**: Continuous Time-To-Collision monitoring during boost phases
- **Conservative Logic**: Only boost when front car is safe distance with adequate speed
- **SOC Coordination**: Communicates status to prevent dangerous boost+steering combinations

---

## **Implementation Details**

### **Architecture**
```
PDA Controller → Anchor Car Detection → Safety Checks → Speed Boost → DCP
               ↗ TTC Monitoring (front car)
               ↗ Gap Distance Validation
               ↗ SOC Status Communication
               ↗ Conservative Speed Limits
```

### **State Machine**
```python
class PDAState(IntEnum):
    DISABLED = 0      # PDA disabled or dependency not met  
    MONITORING = 1    # Watching for anchor cars
    DETECTING = 2     # Anchor car detected, analyzing lead vehicle safety
    SMART_BOOSTING = 3 # Intelligent acceleration based on lead vehicle analysis
    COOLDOWN = 4      # Gap time after boost (prevent continuous boosting)
```

### **Key Safety Features**
1. **TTC Safety Monitoring**: Continuous Time-To-Collision validation before/during boost
2. **Gap Distance Validation**: Minimum 30m front car distance with 15m safety margin
3. **Conservative Boost**: 5-10 km/h speed increase, never exceed target cruise speed
4. **Anchor Duration**: Requires 5-second minimum anchor car detection
5. **Cooldown Period**: 10-second gap between boosts prevents continuous operation

---

## **Safety Features**

### **TTC (Time To Collision) Monitoring** ✅ **IMPLEMENTED**
```python
def calculate_ttc(self, distance: float, ego_speed_ms: float, lead_speed_ms: float) -> float:
    """Calculate Time To Collision with lead vehicle"""
    relative_speed = ego_speed_ms - lead_speed_ms
    
    if distance > 0.1 and relative_speed > 0.3:
        return max(0.0, distance / relative_speed)
    
    return float('inf')  # No collision course

def check_ttc_safety(self, driving_context: Dict[str, Any], ego_speed_ms: float) -> Dict:
    """Three-tier TTC safety validation"""
    # Critical: 2.5s, Continue: 4.0s, Safe: 6.0s
```

### **Conservative Overtaking Logic** ✅ **IMPLEMENTED**
```python
def check_safe_overtaking(self, v_ego: float, target_speed: float, anchor_speed: float) -> Optional[float]:
    """Check all conditions for safe overtaking"""
    # 1. Anchor car slower than target cruise speed
    # 2. Front car enough safety distance (30m min)
    # 3. Won't need brake after passing
    # 4. Never over target cruise speed
```

---

## **Integration Points**

### **File Structure** ✅ **IMPLEMENTED**
```
/selfdrive/controls/lib/nagaspilot/np_pda_controller.py  (Single file implementation)
```

### **Dependencies** ✅ **VALIDATED**
- **DCP Foundation**: Requires Dynamic Control Profile active
- **ModelV2 LeadsV3**: Uses lead vehicle detection for anchor car analysis
- **TTC Integration**: Uses same TTC calculation as AEM for consistency
- **Parameter System**: All parameters use np_ prefix

### **SOC Coordination** ✅ **WORKING**
```python
# PDA communicates status for SOC safety coordination
driving_context['pda_status'] = {
    'active': is_boost_active,
    'reason': 'boost' if is_boost_active else 'monitoring'
}
```

---

## **Parameters**

### **Core PDA Parameters**
```python
("np_pda_enabled", "1"),                     # Enable PDA system
("np_pda_min_boost_kmh", "5.0"),             # Minimum boost (5 km/h)
("np_pda_max_boost_kmh", "10.0"),            # Maximum boost (10 km/h)
```

### **Safety Parameters**
```python
# TTC Safety Thresholds
self.ttc_safe_boost = 6.0            # Minimum TTC to start boost
self.ttc_continue_boost = 4.0        # Minimum TTC to continue boost  
self.ttc_critical = 2.5              # Critical TTC - abort immediately

# Distance & Timing
self.min_safe_distance = 30.0        # Minimum safe distance to front car
self.safety_margin = 15.0            # Extra margin after overtaking
self.anchor_min_duration = 5.0       # Anchor must exist for 5+ seconds
self.boost_duration = 8.0            # Maximum boost duration
self.cooldown_duration = 10.0        # Gap time between boosts
```

### **Dependency Parameters**
```python
("np_dcp_mode", ">0"),               # Requires DCP foundation active
("np_opom_enabled", "False"),        # Must not be in OPOM mode
```

---

## **Current Status**

### **✅ COMPLETED FEATURES**
- Simple state machine with anchor car detection
- TTC safety monitoring for front vehicle collision avoidance
- Conservative overtaking logic with multiple safety checks
- Gap distance validation and post-overtaking safety margins
- SOC coordination via status communication in driving context
- Proper timing controls (5s anchor detection, 8s max boost, 10s cooldown)
- Integration with DCP foundation and parameter system

### **🏗️ ARCHITECTURE DECISIONS**
- **Simple Independent Operation**: PDA manages overtaking logic independently
- **Conservative Safety**: Multiple safety layers prevent dangerous maneuvers  
- **Clean State Machine**: Clear state transitions with proper timing controls
- **SOC Communication**: Simple status sharing without complex coordination
- **TTC Integration**: Uses same calculation as AEM for consistency

---

## **Safety Validation**

### **Test Scenarios Passed** ✅
1. **TTC Monitoring**: PDA correctly validates TTC before/during boost phases
2. **Gap Distance Check**: Ensures minimum 30m distance to front vehicle
3. **Conservative Boost**: Speed increases stay within 5-10 km/h safe range
4. **State Machine Flow**: Proper transitions through all states with timing
5. **SOC Coordination**: Status communication prevents boost+steering conflicts
6. **Cooldown Behavior**: Proper gaps between boost cycles prevent aggressive behavior

### **Safety Margins** ✅
- **Conservative Speed Boost**: Maximum 10 km/h, never exceed cruise target
- **TTC Thresholds**: 6s safe start, 4s continue, 2.5s critical abort
- **Distance Requirements**: 30m minimum with 15m post-overtaking margin
- **Timing Controls**: 5s anchor detection, 8s max boost, 10s cooldown

---

## **Architecture Benefits**

### **Simple & Maintainable** ✅
- Single file implementation with clear responsibilities
- State machine provides predictable behavior
- Conservative safety thresholds prevent edge cases
- Clean integration with existing DCP foundation

### **Safety-First Design** ✅
- Multiple independent safety checks
- TTC monitoring prevents collision scenarios
- Conservative boost amounts and timing
- Graceful fallback on error conditions

### **Independent Operation** ✅
- Works standalone without complex dependencies
- SOC coordination via simple status communication
- Clear separation of concerns (PDA=speed, SOC=steering)
- Easy to enable/disable without affecting other systems

---

## **Conclusion**

**PDA IMPLEMENTATION STATUS**: ✅ **COMPLETE AND PRODUCTION READY**

**Key Achievements**:
- ✅ Simple, reliable anchor car overtaking logic
- ✅ Comprehensive TTC safety monitoring
- ✅ Conservative boost parameters and timing controls
- ✅ Clean SOC coordination without complex protocols
- ✅ Maintainable single-file architecture

**No further major development needed** - PDA operates safely and effectively as designed.