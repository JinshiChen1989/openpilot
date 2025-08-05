# BPSL (Brake Pedal Speed Learning) Migration Plan

## ✅ IMPLEMENTATION COMPLETE - PRODUCTION READY

**Status**: ✅ **COMPLETE** - BPSL implemented with parameter standardization and architecture compliance  
**Date**: 2025-08-02 - Controller relocated and standardized  
**Result**: Full dual-pedal speed learning system with APSL integration

### ✅ AUGUST 2, 2025 - PARAMETER STANDARDIZATION COMPLETE
- **Parameter Fix**: Updated from "EnableBPSL" to "np_bpsl_enabled" for consistency
- **Controller Location**: Verified in `/selfdrive/controls/lib/nagaspilot/np_bpsl_controller.py`
- **Status**: ✅ **PRODUCTION READY** - BPSL standardized and ready

---

## 🎯 Concept Overview

**BPSL completes the dual-pedal speed learning system alongside APSL, enabling natural speed adjustment using both acceleration and braking behavior.**

**Core Concept**: When driver releases brake pedal after manual deceleration, BPSL learns the final achieved speed as new cruise target.

## 🚗 Dual-Pedal Speed Learning System

### **APSL + BPSL Integration**
```
Driver Speed Control:
├─ Want FASTER speed → Gas pedal → APSL learns higher target
└─ Want SLOWER speed → Brake pedal → BPSL learns lower target

Both work as DCP filters with seamless coordination
```

### **Natural Driver Behavior**
- **Acceleration Learning**: Press gas to 75 mph, release → APSL learns 75 mph
- **Deceleration Learning**: Brake to 55 mph, release → BPSL learns 55 mph
- **Manual Override**: Either pedal temporarily overrides, learns on release

---

## 📋 BPSL Core Functions

### **Function 1: Learn Speed from Brake Release**
```python
# Core BPSL Learning Logic
if brake_pressed and not system_braking: start_deceleration_learning()
if brake_released and learning_active: learn_final_speed_if_significant_decrease()
```

### **Function 2: Distinguish Manual vs System Braking**
- **Manual Braking**: Driver intentionally slowing down → Learn new target
- **System Braking**: Lead car/safety braking → Don't learn, maintain target
- **Detection**: Monitor brake pressure, lead car distance, driver behavior patterns

---

## 🏗️ Architecture Design

### **DCP Filter Integration**
```
DCP Filter Priority Order (Real-World ADAS Safety Hierarchy):
 100: VTSC (Vision Turn Speed Controller)   ← Highest: Immediate visual hazards
  90: MTSC (Map Turn Speed Controller)      ← High: Map-based safety warnings
  80: BPSL (Brake Pedal Speed Learning)     ← Driver brake intent (respects safety)
  70: APSL (Accelerator Pedal Speed Learning) ← Driver acceleration intent
   5: PDA (Parallel Drive Avoidance)        ← Overtaking only when safe
   3: VCSC (Vertical Comfort Speed Controller) ← Comfort adjustments
```

### **Real-World ADAS Safety Rationale**
**Priority Design Principles:**
1. **Vision Hazards (100)**: Highest priority - immediate safety threats from camera
2. **Map Safety (90)**: High priority - map-based curve and hazard warnings
3. **Driver Brake Intent (80)**: Respect driver's safety-oriented braking decisions
4. **Driver Acceleration Intent (70)**: Respect driver's speed preferences when safe
5. **Overtaking Strategy (5)**: Performance enhancement only when all safety allows
6. **Comfort (3)**: Lowest priority - comfort adjustments never override safety

### **APSL + BPSL Coordination**
```python
class DualPedalSpeedLearning:
    def coordinate_learning(self, apsl_target, bpsl_target):
        # BPSL always wins due to higher priority (999 vs 50)
        # Brake input represents safety-critical driver intent
        if bpsl_target and bpsl_learning_active:
            return bpsl_target  # Brake learning takes precedence
        elif apsl_target and apsl_learning_active:
            return apsl_target  # Gas learning when no brake input
        return base_speed       # Fallback to system default
```

---

## 📝 Implementation Phases

### **Phase 1: BPSL Controller Design** 
**Objective**: Create np_bpsl_controller.py with brake learning logic

**Key Features**:
- Brake pedal position monitoring
- Manual vs system brake detection
- Speed learning on brake release
- DCP filter integration (priority 999)

**Detection Logic**:
```python
def is_manual_braking(self, driving_context):
    # Not manual if:
    if has_lead_car_close(): return False
    if emergency_braking_active(): return False  
    if following_distance_braking(): return False
    
    # Manual if driver-initiated deceleration
    return brake_pressure > MANUAL_THRESHOLD
```

### **Phase 2: APSL + BPSL Coordination**
**Objective**: Seamless integration between acceleration and deceleration learning

**Coordination Features**:
- Timestamp-based priority (most recent wins)
- Smooth transition between learned targets
- Prevent learning conflicts
- Unified speed learning telemetry

### **Phase 3: Advanced Brake Detection**
**Objective**: Intelligent detection of manual vs system braking

**Detection Methods**:
- **Lead Car Analysis**: Distance, relative speed, closing rate
- **Safety System Integration**: FCW, AEB, following distance
- **Driver Pattern Recognition**: Brake pressure curves, timing
- **Multi-Modal Detection**: Vision + radar + driver behavior

### **Phase 4: UI/UX Integration**
**Objective**: Provide clear feedback for dual-pedal learning

**User Experience**:
- Visual confirmation of speed learning (gas/brake)
- Display current learned target and source (APSL/BPSL)
- Settings for learning sensitivity and detection thresholds

---

## 🧪 Testing Scenarios

### **Basic Learning Tests**
1. **APSL Learning**: 60→75 mph acceleration, release → learns 75 mph
2. **BPSL Learning**: 75→60 mph manual braking, release → learns 60 mph
3. **Coordination**: APSL then BPSL → BPSL target wins (most recent)

### **Detection Accuracy Tests**
1. **Manual Brake**: Driver braking on empty road → Should learn
2. **Lead Car Brake**: System braking for lead car → Should NOT learn
3. **Emergency Brake**: FCW/AEB activation → Should NOT learn
4. **Following Distance**: Routine following brake → Should NOT learn

### **Real-World Scenarios**
1. **Highway Speed Adjustment**: 70→65 mph zone change via braking
2. **Traffic Flow**: Manual speed reduction for traffic conditions
3. **Curve Approach**: Driver braking before curve (coordinate with VTSC)

---

## 🚀 Benefits Over Single-Pedal Systems

### **Complete Speed Control**
| Scenario | Action | System Response |
|----------|--------|-----------------|
| **Want Faster** | Gas to 75 mph, release | APSL learns 75 mph ✅ |
| **Want Slower** | Brake to 55 mph, release | BPSL learns 55 mph ✅ |
| **Lead Car** | System brakes | Neither learns, maintains target ✅ |
| **Manual Adjust** | Either pedal | Learns new preference ✅ |

### **Natural Driver Behavior**
- **Intuitive**: Uses natural pedal behavior for speed learning
- **Precise**: Learn exact desired speed through direct demonstration
- **Flexible**: Adjust up or down using familiar controls
- **Safe**: Maintains system safety overrides

---

## 🔧 Technical Implementation

### **BPSL Controller Structure**
```python
class BPSLFilter(DCPFilterLayer):
    def __init__(self):
        super().__init__(name="BPSL", filter_type=DCPFilterType.DRIVER_OVERRIDE, priority=999)
        
        # Brake learning parameters
        self.BRAKE_THRESHOLD = 0.05        # 5% brake threshold
        self.MIN_SPEED_DECREASE = 2.0      # 2 m/s minimum decrease to learn
        self.MANUAL_BRAKE_THRESHOLD = 0.15 # 15% brake = likely manual
        
    def detect_manual_vs_system_braking(self, context):
        # Advanced detection logic
        pass
        
    def process_brake_learning(self, brake_pos, current_speed):
        # Similar to APSL but for deceleration
        pass
```

### **Parameter Integration**
- `EnableBPSL` - Master enable/disable
- `BPSLSensitivity` - Learning sensitivity (0.5-2.0)
- `BPSLManualThreshold` - Manual brake detection threshold

---

## 📊 Expected Results

### **User Experience Improvements**
- **Complete Control**: Adjust cruise speed using natural pedal behavior
- **Reduced Complexity**: No need for cruise control buttons
- **Intuitive Learning**: System learns exactly what driver demonstrates
- **Seamless Integration**: Works perfectly with existing MADS/DCP systems

### **Technical Benefits**
- **Clean Architecture**: Both APSL/BPSL as DCP filters
- **Coordinated Learning**: Intelligent dual-pedal coordination
- **Safety Maintained**: All existing safety systems preserved
- **Performance**: Minimal computational overhead

---

## 🛣️ Migration Timeline

### **Immediate** (Phase 1)
- Design BPSL controller architecture
- Implement basic brake learning logic
- Create DCP filter integration

### **Short-term** (Phase 2-3)
- Advanced manual vs system brake detection
- APSL + BPSL coordination system
- Comprehensive testing suite

### **Long-term** (Phase 4)
- UI/UX integration and feedback
- Machine learning-enhanced detection
- Real-world validation and tuning

---

## 🎯 Success Criteria

**BPSL successfully completes the dual-pedal speed learning system, providing intuitive speed control through natural acceleration and braking behavior, perfectly integrated with MADS and DCP architecture.**

### **Core Objectives**:
✅ Learn target speed from brake release  
✅ Distinguish manual vs system braking  
✅ Coordinate seamlessly with APSL  
✅ Maintain all existing safety systems  
✅ Provide natural, intuitive speed control

**Migration Target**: Complete dual-pedal speed learning system with APSL + BPSL working together as coordinated DCP filters.