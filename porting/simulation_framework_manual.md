# nagaspilot Simulation Framework - User Manual

## Overview

The nagaspilot Simulation Framework is a comprehensive Python-based toolkit for testing and validating the openpilot autonomous driving stack in safe, controlled virtual environments. Built on MetaDrive's physics simulation engine, it provides realistic vehicle dynamics, multi-camera sensor simulation, and comprehensive safety testing capabilities for autonomous driving algorithms.

**🎮 System Types: SIMULATION + TESTING + DEVELOPMENT**
- **✅ Runtime Simulation**: Real-time physics simulation with MetaDrive integration
- **✅ Testing Environment**: Safe validation of openpilot stack without real vehicles
- **✅ Development**: Interactive debugging and parameter tuning
- **✅ Multi-Modal**: Keyboard, joystick, and automated control interfaces

## Table of Contents
1. [System Architecture](#system-architecture)
2. [MetaDrive Integration](#metadrive-integration)
3. [Installation & Setup](#installation--setup)
4. [Bridge System Architecture](#bridge-system-architecture)
5. [Control Interfaces](#control-interfaces)
6. [Vehicle Dynamics & Physics](#vehicle-dynamics--physics)
7. [Camera Simulation & Multi-Camera Setup](#camera-simulation--multi-camera-setup)
8. [Safe Testing Environment](#safe-testing-environment)
9. [Integration with openpilot Stack](#integration-with-openpilot-stack)
10. [Performance Optimization](#performance-optimization)
11. [Troubleshooting](#troubleshooting)
12. [API Reference](#api-reference)

## System Architecture

### Core Components

**Bridge System**
- **run_bridge.py** - Main simulation orchestration and process management
- **bridge/common.py** - Abstract bridge interface and shared utilities
- **bridge/metadrive/metadrive_bridge.py** - MetaDrive-specific simulation bridge
- **bridge/metadrive/metadrive_world.py** - World state management and vehicle control
- **bridge/metadrive/metadrive_process.py** - Isolated MetaDrive process execution

**Vehicle Simulation**
- **lib/simulated_car.py** - Honda Civic 2022 CAN message simulation
- **lib/simulated_sensors.py** - IMU, GPS, camera, and peripheral state simulation
- **lib/camerad.py** - Camera daemon simulation with VisionIPC integration

**Control Systems**
- **lib/keyboard_ctrl.py** - Real-time keyboard input handling
- **lib/manual_ctrl.py** - Joystick/wheel integration with force feedback
- **lib/common.py** - Shared data structures and simulation state management

**Launch System**
- **launch_openpilot.sh** - openpilot initialization script for simulation mode

### Data Flow Architecture

#### Simulation Loop
```
MetaDrive Physics → Vehicle State → CAN Messages → openpilot Stack →
Control Commands → MetaDrive Actuators → Physics Update
```

#### Multi-Process Architecture
```
Main Process (Bridge) ↔ MetaDrive Process (Physics) ↔ openpilot Processes
         ↓                        ↓                           ↓
    Control Queue ←→ Camera Arrays ←→ Message Queues (ZMQ/MSGQ)
```

#### Sensor Data Flow
```
MetaDrive Sensors → RGB Images → OpenCL Conversion → YUV420 → 
VisionIPC → openpilot Vision Pipeline
```

#### Message Flow
```
Simulated CAN → CANPacker → Cereal Messages → pandaStates →
openpilot controlsd → carControl → Bridge → MetaDrive Vehicle
```

## MetaDrive Integration

### MetaDrive Engine Overview

MetaDrive is a comprehensive driving simulation platform that provides:
- **Realistic Physics**: Vehicle dynamics, tire models, collision detection
- **Procedural Maps**: Configurable road networks and traffic scenarios
- **Multi-Modal Sensors**: RGB cameras, LiDAR, IMU, GPS simulation
- **Traffic Simulation**: Dynamic traffic agents and environmental conditions

### Key Integration Components

**Camera Systems**
```python
# Road Camera (Primary vision input)
class RGBCameraRoad(CopyRamRGBCamera):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        lens = self.get_lens()
        lens.setFov(40)  # Field of view matching real C3 camera
        lens.setNear(0.1)
```

**Wide Camera (Enhanced perception)**
```python
class RGBCameraWide(CopyRamRGBCamera):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        lens = self.get_lens()
        lens.setFov(120)  # Wide-angle for better peripheral vision
        lens.setNear(0.1)
```

### Map Generation

**Track Configuration**
```python
def create_map(track_size=60):
    curve_len = track_size * 2
    return dict(
        type=MapGenerateMethod.PG_MAP_FILE,
        lane_num=2,
        lane_width=4.5,  # meters
        config=[
            None,
            straight_block(track_size),
            curve_block(curve_len, 90),  # 90-degree turns
            # ... creates rectangular loop track
        ]
    )
```

**MetaDrive Configuration**
```python
config = dict(
    use_render=self.should_render,
    vehicle_config=dict(
        enable_reverse=False,
        render_vehicle=False,
        image_source="rgb_road",
    ),
    sensors=sensors,
    image_on_cuda=_cuda_enable,  # GPU acceleration when available
    physics_world_step_size=self.TICKS_PER_FRAME/100,  # 20Hz physics
    traffic_density=0.0,  # No traffic for stability
    decision_repeat=1,
    preload_models=False,  # Faster startup
    anisotropic_filtering=False
)
```

## Installation & Setup

### Prerequisites

**System Requirements**
```bash
# Operating System
Ubuntu 20.04+ or compatible Linux distribution
Python 3.8+ with development headers
NVIDIA GPU recommended for CUDA acceleration

# Hardware Requirements
- Minimum 8GB RAM (16GB recommended)
- 2GB free disk space
- USB port for joystick/wheel (optional)
```

**Core Dependencies**
```bash
# Install MetaDrive simulation engine
pip install metadrive-simulator

# OpenGL dependencies for rendering
sudo apt-get install freeglut3-dev libgl1-mesa-dev libglu1-mesa-dev

# OpenCL for GPU-accelerated image processing
sudo apt-get install ocl-icd-libopencl1 opencl-headers clinfo
pip install pyopencl

# Controller support (optional)
sudo apt-get install joystick jstest-gtk
pip install evdev
```

**Build nagaspilot with Simulation Support**
```bash
# Clone and build nagaspilot
cd nagaspilot
./tools/sim/setup.sh

# Verify installation
python3 -c "import metadrive; print('MetaDrive installed successfully')"
python3 -c "import pyopencl as cl; print('OpenCL available')"
```

### Environment Setup

**Simulation Environment Variables**
```bash
# Set in launch_openpilot.sh
export PASSIVE="0"          # Active mode for controls
export NOBOARD="1"          # No hardware required
export SIMULATION="1"       # Enable simulation mode
export SKIP_FW_QUERY="1"    # Skip firmware checks
export FINGERPRINT="HONDA_CIVIC_2022"  # Vehicle type

# Block unnecessary processes for simulation
export BLOCK="camerad,loggerd,encoderd,micd,logmessaged"
```

**GPU Configuration**
```bash
# Check CUDA availability
nvidia-smi
python3 -c "import cupy; print('CUDA available')" 2>/dev/null || echo "CPU mode"

# Test OpenCL
clinfo  # List available OpenCL devices
```

### Quick Start Guide

**1. Launch openpilot in Simulation Mode**
```bash
cd nagaspilot
./tools/sim/launch_openpilot.sh
```

**2. Start the Simulation Bridge (New Terminal)**
```bash
cd nagaspilot
./tools/sim/run_bridge.py
```

**3. Control the Vehicle**
```
Keyboard Controls:
1 - Cruise Resume/Accel
2 - Cruise Set/Decel  
3 - Cruise Cancel
r - Reset Simulation
i - Toggle Ignition
q - Exit
WASD - Manual Control
```

## Bridge System Architecture

### SimulatorBridge Base Class

**Core Bridge Interface**
```python
class SimulatorBridge(ABC):
    TICKS_PER_FRAME = 5  # Physics updates per frame
    
    def __init__(self, dual_camera, high_quality):
        set_params_enabled()  # Configure openpilot for simulation
        self.params = Params()
        self.params.put_bool("AlphaLongitudinalEnabled", True)
        
        self.rk = Ratekeeper(100, None)  # 100Hz main loop
        self.simulator_state = SimulatorState()
```

**Bridge Process Management**
```python
def run(self, queue, retries=-1):
    bridge_p = Process(name="bridge", target=self.bridge_keep_alive, 
                      args=(queue, retries))
    bridge_p.start()
    return bridge_p

def bridge_keep_alive(self, q: Queue, retries: int):
    try:
        self._run(q)  # Main simulation loop
    finally:
        self.close("bridge terminated")
```

### MetaDrive Bridge Implementation

**Bridge Initialization**
```python
class MetaDriveBridge(SimulatorBridge):
    def __init__(self, dual_camera, high_quality, test_duration=math.inf, test_run=False):
        super().__init__(dual_camera, high_quality)
        self.should_render = False  # Headless by default
        self.test_duration = test_duration if test_run else math.inf
```

**World Spawning**
```python
def spawn_world(self, queue: Queue):
    sensors = {"rgb_road": (RGBCameraRoad, W, H)}
    if self.dual_camera:
        sensors["rgb_wide"] = (RGBCameraWide, W, H)
    
    config = dict(
        use_render=self.should_render,
        vehicle_config=dict(
            enable_reverse=False,
            render_vehicle=False,
            image_source="rgb_road",
        ),
        sensors=sensors,
        image_on_cuda=_cuda_enable,
        physics_world_step_size=self.TICKS_PER_FRAME/100,
        map_config=create_map(),
    )
    
    return MetaDriveWorld(queue, config, self.test_duration, 
                         self.test_run, self.dual_camera)
```

### Message Queue System

**Control Command Processing**
```python
QueueMessage = namedtuple("QueueMessage", ["type", "info"], defaults=[None])

class QueueMessageType(Enum):
    START_STATUS = 0
    CONTROL_COMMAND = 1
    TERMINATION_INFO = 2
    CLOSE_STATUS = 3

def control_cmd_gen(cmd: str):
    return QueueMessage(QueueMessageType.CONTROL_COMMAND, cmd)
```

**Command Parsing**
```python
# Control command format: "action_value"
if message.type == QueueMessageType.CONTROL_COMMAND:
    m = message.info.split('_')
    if m[0] == "steer":
        steer_manual = float(m[1])
    elif m[0] == "throttle":
        throttle_manual = float(m[1])
    elif m[0] == "brake":
        brake_manual = float(m[1])
    elif m[0] == "cruise":
        if m[1] == "down":
            self.simulator_state.cruise_button = CruiseButtons.DECEL_SET
        # ... handle other cruise commands
```

## Control Interfaces

### Keyboard Control System

**Real-Time Input Processing**
```python
def keyboard_poll_thread(q: 'Queue[QueueMessage]'):
    while True:
        c = getch()  # Non-blocking character input
        if c == '1':
            q.put(control_cmd_gen("cruise_up"))
        elif c == '2':
            q.put(control_cmd_gen("cruise_down"))
        elif c == 'w':
            q.put(control_cmd_gen(f"throttle_{1.0}"))
        elif c == 's':
            q.put(control_cmd_gen(f"brake_{1.0}"))
        # ... handle other keys
```

**Keyboard Commands**
```
Control Commands:
| Key  | Function              | Value Range |
|------|-----------------------|-------------|
| 1    | Cruise Resume/Accel   | Binary      |
| 2    | Cruise Set/Decel      | Binary      |
| 3    | Cruise Cancel         | Binary      |
| w    | Throttle              | 0.0 - 1.0   |
| s    | Brake                 | 0.0 - 1.0   |
| a/d  | Steer Left/Right      | -0.15/+0.15 |
| z/x  | Left/Right Blinker    | Binary      |
| i    | Toggle Ignition       | Binary      |
| r    | Reset Simulation      | Binary      |
| q    | Quit                  | Exit        |
```

### Joystick/Wheel Control System

**Hardware Detection**
```python
# Iterate over available joystick devices
print('Available devices:')
for fn in os.listdir('/dev/input'):
    if fn.startswith('js'):
        print(f'  /dev/input/{fn}')

# Open joystick device
fn = '/dev/input/js0'
jsdev = open(fn, 'rb')
```

**Axis Mapping**
```python
axis_names = {
    0x00: 'x',        # Steering wheel
    0x02: 'z',        # Gas pedal
    0x05: 'rz',       # Brake pedal
    0x08: 'wheel',    # Steering wheel
    0x09: 'gas',      # Gas pedal
    0x0a: 'brake',    # Brake pedal
    # ... additional axes
}
```

**Force Feedback Support**
```python
# Enable force feedback for steering wheels
import evdev
from evdev import ecodes, InputDevice
device = evdev.list_devices()[0]
evtdev = InputDevice(device)
val = 24000  # Force feedback strength
evtdev.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, val)
```

**Control Value Processing**
```python
def wheel_poll_thread(q: 'Queue[str]'):
    while True:
        evbuf = jsdev.read(8)
        value, mtype, number = struct.unpack('4xhBB', evbuf)
        
        if mtype & 0x02:  # Axis movement
            axis = axis_name_list[number]
            
            if axis == "z":  # Gas pedal
                fvalue = value / 32767.0
                normalized = (1 - fvalue) * 50
                q.put(control_cmd_gen(f"throttle_{normalized:f}"))
                
            elif axis == "x":  # Steering wheel
                fvalue = value / 32767.0
                q.put(control_cmd_gen(f"steer_{fvalue:f}"))
```

### Manual Override System

**Engagement Control**
```python
# openpilot engagement logic
throttle_out = throttle_op if self.simulator_state.is_engaged else throttle_manual
brake_out = brake_op if self.simulator_state.is_engaged else brake_manual
steer_out = steer_op if self.simulator_state.is_engaged else steer_manual

# Apply controls to simulation
self.world.apply_controls(steer_out, throttle_out, brake_out)
```

**Safety Override**
```python
# User brake always disengages openpilot
self.simulator_state.user_brake = brake_manual
if brake_manual > 0:
    self.simulator_state.is_engaged = False
```

## Vehicle Dynamics & Physics

### Simulated Vehicle Configuration

**Honda Civic 2022 Simulation**
```python
class SimulatedCar:
    """Simulates a Honda Civic 2022 (panda state + CAN messages)"""
    packer = CANPacker("honda_civic_ex_2022_can_generated")
    
    def __init__(self):
        self.pm = messaging.PubMaster(['can', 'pandaStates'])
        self.sm = messaging.SubMaster(['carControl', 'controlsState', 
                                      'carParams', 'selfdriveState'])
```

**Vehicle Parameters**
```python
# Vehicle configuration in MetaDrive
vehicle_config=dict(
    enable_reverse=False,      # Forward only for safety
    render_vehicle=False,      # Invisible vehicle for clean camera view
    image_source="rgb_road",   # Primary camera input
    max_speed_km_h=1000,      # No speed limit in simulation
)

# Physics parameters
physics_world_step_size=self.TICKS_PER_FRAME/100,  # 50ms physics steps
decision_repeat=1,                                  # Immediate response
```

### CAN Message Simulation

**Powertrain Bus Messages**
```python
def send_can_messages(self, simulator_state: SimulatorState):
    msg = []
    speed = simulator_state.speed * 3.6  # m/s to km/h
    
    # Speed and wheel speed messages
    msg.append(self.packer.make_can_msg("ENGINE_DATA", 0, 
                                       {"XMISSION_SPEED": speed}))
    msg.append(self.packer.make_can_msg("WHEEL_SPEEDS", 0, {
        "WHEEL_SPEED_FL": speed,
        "WHEEL_SPEED_FR": speed,
        "WHEEL_SPEED_RL": speed,
        "WHEEL_SPEED_RR": speed
    }))
    
    # Control inputs
    msg.append(self.packer.make_can_msg("SCM_BUTTONS", 0, 
                                       {"CRUISE_BUTTONS": simulator_state.cruise_button}))
    msg.append(self.packer.make_can_msg("STEER_STATUS", 0, 
                                       {"STEER_TORQUE_SENSOR": simulator_state.user_torque}))
```

**Safety and State Messages**
```python
# Safety-critical messages
msg.append(self.packer.make_can_msg("SEATBELT_STATUS", 0, 
                                   {"SEATBELT_DRIVER_LATCHED": 1}))
msg.append(self.packer.make_can_msg("GEARBOX", 0, 
                                   {"GEAR": 4, "GEAR_SHIFTER": 8}))  # Drive gear
msg.append(self.packer.make_can_msg("STANDSTILL", 0, 
                                   {"WHEELS_MOVING": 1 if simulator_state.speed >= 1.0 else 0}))

# Engagement state
msg.append(self.packer.make_can_msg("POWERTRAIN_DATA", 0, {
    "ACC_STATUS": int(simulator_state.is_engaged),
    "PEDAL_GAS": simulator_state.user_gas,
    "BRAKE_PRESSED": simulator_state.user_brake > 0
}))
```

### Physics Integration

**Vehicle State Mapping**
```python
# MetaDrive to openpilot state conversion
metadrive_vehicle_state = namedtuple("metadrive_vehicle_state", 
                                   ["velocity", "position", "bearing", "steering_angle"])

vehicle_state = metadrive_vehicle_state(
    velocity=vec3(x=float(env.vehicle.velocity[0]), 
                 y=float(env.vehicle.velocity[1]), z=0),
    position=env.vehicle.position,
    bearing=float(math.degrees(env.vehicle.heading_theta)),
    steering_angle=env.vehicle.steering * env.vehicle.MAX_STEERING
)
```

**Control Application**
```python
def apply_controls(self, steer_angle, throttle_out, brake_out):
    if (time.monotonic() - self.reset_time) > 2:  # Delay after reset
        self.vc[0] = steer_angle
        
        if throttle_out:
            self.vc[1] = throttle_out
        else:
            self.vc[1] = -brake_out  # Negative for braking
    else:
        self.vc[0] = 0  # No control during reset
        self.vc[1] = 0
        
    self.controls_send.send([*self.vc, self.should_reset])
```

## Camera Simulation & Multi-Camera Setup

### Camera Architecture

**VisionIPC Integration**
```python
class Camerad:
    """Simulates the camerad daemon"""
    def __init__(self, dual_camera):
        self.pm = messaging.PubMaster(['roadCameraState', 'wideRoadCameraState'])
        self.vipc_server = VisionIpcServer("camerad")
        
        # Create vision buffers
        self.vipc_server.create_buffers(VisionStreamType.VISION_STREAM_ROAD, 5, W, H)
        if dual_camera:
            self.vipc_server.create_buffers(VisionStreamType.VISION_STREAM_WIDE_ROAD, 5, W, H)
```

**Camera Parameters**
```python
# Image dimensions (matching comma three)
W, H = 1928, 1208

# Road Camera (Primary)
class RGBCameraRoad(CopyRamRGBCamera):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        lens = self.get_lens()
        lens.setFov(40)    # Narrow field of view
        lens.setNear(0.1)  # Near clipping plane

# Wide Camera (Peripheral)
class RGBCameraWide(CopyRamRGBCamera):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        lens = self.get_lens()
        lens.setFov(120)   # Wide field of view
        lens.setNear(0.1)
```

### Image Processing Pipeline

**RGB to YUV Conversion**
```python
def rgb_to_yuv(self, rgb):
    assert rgb.shape == (H, W, 3), f"{rgb.shape}"
    assert rgb.dtype == np.uint8
    
    # OpenCL GPU acceleration
    rgb_cl = cl_array.to_device(self.queue, rgb)
    yuv_cl = cl_array.empty_like(rgb_cl)
    self.krnl(self.queue, (self.Wdiv4, self.Hdiv4), None, 
              rgb_cl.data, yuv_cl.data).wait()
    yuv = np.resize(yuv_cl.get(), rgb.size // 2)
    return yuv.data.tobytes()
```

**OpenCL Kernel Configuration**
```python
# Set up OpenCL for RGB to YUV conversion
self.ctx = cl.create_some_context()
self.queue = cl.CommandQueue(self.ctx)
cl_arg = (f" -DHEIGHT={H} -DWIDTH={W} -DRGB_STRIDE={W * 3} "
          f"-DUV_WIDTH={W // 2} -DUV_HEIGHT={H // 2} "
          f"-DRGB_SIZE={W * H} -DCL_DEBUG ")

kernel_fn = os.path.join(BASEDIR, "tools/sim/rgb_to_nv12.cl")
with open(kernel_fn) as f:
    prg = cl.Program(self.ctx, f.read()).build(cl_arg)
    self.krnl = prg.rgb_to_nv12
```

### Multi-Camera Configuration

**Dual Camera Setup**
```bash
# Enable dual camera mode
./tools/sim/run_bridge.py --dual_camera
```

**Camera Positioning**
```python
# Camera position relative to vehicle (in MetaDrive coordinates)
C3_POSITION = Vec3(0.0, 0, 1.22)  # Height of comma three device
C3_HPR = Vec3(0, 0, 0)             # Heading, pitch, roll

def get_cam_as_rgb(cam):
    cam = env.engine.sensors[cam]
    cam.get_cam().reparentTo(env.vehicle.origin)
    cam.get_cam().setPos(C3_POSITION)
    cam.get_cam().setHpr(C3_HPR)
    img = cam.perceive(to_float=False)
    if not isinstance(img, np.ndarray):
        img = img.get()  # Convert cupy array to numpy
    return img
```

**Camera Data Flow**
```python
def send_camera_images(self, world: 'World'):
    world.image_lock.acquire()
    
    # Process road camera
    yuv = self.camerad.rgb_to_yuv(world.road_image)
    self.camerad.cam_send_yuv_road(yuv)
    
    # Process wide camera if available
    if world.dual_camera:
        yuv = self.camerad.rgb_to_yuv(world.wide_road_image)
        self.camerad.cam_send_yuv_wide_road(yuv)
```

### High Quality Mode

**Enhanced Rendering**
```bash
# Enable high quality rendering
./tools/sim/run_bridge.py --high_quality
```

**Quality Parameters**
```python
config = dict(
    anisotropic_filtering=high_quality,
    use_render=high_quality,      # Enable visual rendering
    show_logo=False,              # Clean display
    preload_models=not high_quality,  # Trade startup time for quality
)
```

## Safe Testing Environment

### Safety Systems

**Collision Detection**
```python
config = dict(
    out_of_route_done=False,     # Don't stop for route deviations
    on_continuous_line_done=False, # Allow line crossing
    crash_vehicle_done=False,    # Don't stop for vehicle crashes
    crash_object_done=False,     # Don't stop for object crashes
    traffic_density=0.0,         # No traffic for safety
)
```

**Reset and Recovery**
```python
def reset(self):
    env.reset()
    env.vehicle.config["max_speed_km_h"] = 1000  # Remove speed limits
    
    simulation_state = metadrive_simulation_state(
        running=True,
        done=False,
        done_info=None,
    )
    simulation_state_send.send(simulation_state)
```

**Emergency Stops**
```python
# Immediate brake override
if brake_manual > 0:
    self.simulator_state.user_brake = brake_manual
    self.simulator_state.is_engaged = False  # Disengage openpilot

# Reset on 'r' key
elif m[0] == "reset":
    self.world.reset()
```

### Test Environment Controls

**Controlled Track Layout**
```python
def create_map(track_size=60):
    """Create a simple rectangular track for testing"""
    curve_len = track_size * 2
    return dict(
        type=MapGenerateMethod.PG_MAP_FILE,
        lane_num=2,
        lane_width=4.5,
        config=[
            None,
            straight_block(track_size),    # 60m straight
            curve_block(curve_len, 90),    # 90-degree turn
            straight_block(track_size),    # 60m straight
            curve_block(curve_len, 90),    # 90-degree turn
            straight_block(track_size),    # 60m straight
            curve_block(curve_len, 90),    # 90-degree turn
            straight_block(track_size),    # 60m straight
            curve_block(curve_len, 90),    # 90-degree turn (complete loop)
        ]
    )
```

**Deterministic Environment**
```python
# Disable random elements for reproducible testing
config = dict(
    traffic_density=0.0,          # No random traffic
    random_agent_model=False,     # Consistent vehicle model
    random_lane_width=False,      # Fixed lane dimensions
    random_lane_num=False,        # Fixed number of lanes
)
```

### Test Automation

**Automated Test Scenarios**
```python
class MetaDriveBridge(SimulatorBridge):
    def __init__(self, dual_camera, high_quality, test_duration=math.inf, test_run=False):
        super().__init__(dual_camera, high_quality)
        self.test_run = test_run
        self.test_duration = test_duration if test_run else math.inf

# Test execution
if __name__ == "__main__":
    # 30-second automated test
    bridge = MetaDriveBridge(False, False, test_duration=30, test_run=True)
```

**Movement Validation**
```python
def read_sensors(self, state: SimulatorState):
    # Check for vehicle movement after engagement
    after_engaged_check = (is_engaged and 
                          time.monotonic() - self.first_engage >= 5 and 
                          self.test_run)
    
    # Calculate distance moved
    x_dist = abs(curr_pos[0] - self.vehicle_last_pos[0])
    y_dist = abs(curr_pos[1] - self.vehicle_last_pos[1])
    if x_dist >= 1 or y_dist >= 1:  # 1m threshold
        self.distance_moved += x_dist + y_dist
    
    # Fail test if vehicle doesn't move when engaged
    if after_engaged_check and self.distance_moved == 0:
        self.status_q.put(QueueMessage(QueueMessageType.TERMINATION_INFO, 
                                      {"vehicle_not_moving": True}))
        self.exit_event.set()
```

## Integration with openpilot Stack

### Process Configuration

**Simulation Mode Setup**
```bash
# Environment variables for simulation
export PASSIVE="0"                    # Active mode (sends controls)
export NOBOARD="1"                   # No Panda hardware required
export SIMULATION="1"                # Enable simulation mode
export SKIP_FW_QUERY="1"            # Skip firmware version checks
export FINGERPRINT="HONDA_CIVIC_2022" # Vehicle fingerprint

# Block unnecessary processes
export BLOCK="camerad,loggerd,encoderd,micd,logmessaged"
```

**Parameter Initialization**
```python
def set_params_enabled():
    """Configure openpilot parameters for simulation"""
    params = Params()
    
    # Enable longitudinal control
    params.put_bool("AlphaLongitudinalEnabled", True)
    
    # Disable driver monitoring in simulation
    params.put_bool("DriverMonitoring", False)
    
    # Set simulation-specific parameters
    params.put_bool("OpenpilotEnabledToggle", True)
```

### Message Integration

**CAN Message Publishing**
```python
def update(self, simulator_state: SimulatorState):
    try:
        self.send_can_messages(simulator_state)
        
        if self.idx % 50 == 0:  # 2Hz for panda states
            self.send_panda_state(simulator_state)
            
        self.idx += 1
    except Exception:
        traceback.print_exc()
        raise
```

**Sensor Message Simulation**
```python
class SimulatedSensors:
    """Simulates C3 sensors (IMU, GPS, cameras, peripherals)"""
    
    def __init__(self, dual_camera=False):
        self.pm = messaging.PubMaster([
            'accelerometer', 'gyroscope', 'gpsLocationExternal',
            'driverStateV2', 'driverMonitoringState', 'peripheralState'
        ])
        self.camerad = Camerad(dual_camera=dual_camera)
```

**IMU Simulation**
```python
def send_imu_message(self, simulator_state: 'SimulatorState'):
    for _ in range(5):  # 5 messages per update cycle
        # Accelerometer
        dat = messaging.new_message('accelerometer', valid=True)
        dat.accelerometer.sensor = 4
        dat.accelerometer.type = 0x10
        dat.accelerometer.timestamp = dat.logMonoTime
        dat.accelerometer.acceleration.v = [
            simulator_state.imu.accelerometer.x,
            simulator_state.imu.accelerometer.y, 
            simulator_state.imu.accelerometer.z
        ]
        self.pm.send('accelerometer', dat)
        
        # Gyroscope
        dat = messaging.new_message('gyroscope', valid=True)
        dat.gyroscope.gyroUncalibrated.v = [
            simulator_state.imu.gyroscope.x,
            simulator_state.imu.gyroscope.y,
            simulator_state.imu.gyroscope.z
        ]
        self.pm.send('gyroscope', dat)
```

### Control Loop Integration

**openpilot Control Reception**
```python
def _run(self, q: Queue):
    # Initialize simulated components
    self.simulated_car = SimulatedCar()
    self.simulated_sensors = SimulatedSensors(self.dual_camera)
    
    # Start worker threads
    self.simulated_car_thread = threading.Thread(
        target=rk_loop, 
        args=(functools.partial(self.simulated_car.update, self.simulator_state),
              100, self._exit_event)
    )
    
    self.simulated_camera_thread = threading.Thread(
        target=rk_loop,
        args=(functools.partial(self.simulated_sensors.send_camera_images, self.world),
              20, self._exit_event)
    )
```

**Control Application**
```python
# Read openpilot control outputs
self.simulated_car.sm.update(0)
self.simulator_state.is_engaged = self.simulated_car.sm['selfdriveState'].active

if self.simulator_state.is_engaged:
    # Apply openpilot controls
    throttle_op = np.clip(self.simulated_car.sm['carControl'].actuators.accel / 1.6, 0.0, 1.0)
    brake_op = np.clip(-self.simulated_car.sm['carControl'].actuators.accel / 4.0, 0.0, 1.0)
    steer_op = self.simulated_car.sm['carControl'].actuators.steeringAngleDeg
else:
    # Use manual controls
    throttle_op = brake_op = steer_op = 0.0

# Determine final control values
throttle_out = throttle_op if self.simulator_state.is_engaged else throttle_manual
brake_out = brake_op if self.simulator_state.is_engaged else brake_manual
steer_out = steer_op if self.simulator_state.is_engaged else steer_manual

# Send to simulation
self.world.apply_controls(steer_out, throttle_out, brake_out)
```

## Performance Optimization

### GPU Acceleration

**CUDA Support**
```python
# Enable CUDA for MetaDrive cameras when available
from metadrive.component.sensors.base_camera import _cuda_enable

config = dict(
    image_on_cuda=_cuda_enable,  # Use GPU for image processing
    sensors=sensors,
)

# Check CUDA availability
try:
    import cupy
    print("CUDA acceleration available")
    cuda_available = True
except ImportError:
    print("Using CPU mode")
    cuda_available = False
```

**OpenCL Optimization**
```python
# OpenCL setup for RGB to YUV conversion
self.ctx = cl.create_some_context()
self.queue = cl.CommandQueue(self.ctx)

# Optimized work group sizes
self.Wdiv4 = W // 4 if (W % 4 == 0) else (W + (4 - W % 4)) // 4
self.Hdiv4 = H // 4 if (H % 4 == 0) else (H + (4 - H % 4)) // 4

# GPU kernel execution
self.krnl(self.queue, (self.Wdiv4, self.Hdiv4), None, 
          rgb_cl.data, yuv_cl.data).wait()
```

### Process Optimization

**Multi-Process Architecture**
```python
# Separate process for MetaDrive physics
self.metadrive_process = multiprocessing.Process(
    name="metadrive process",
    target=functools.partial(metadrive_process, dual_camera, config,
                           self.camera_array, self.wide_camera_array, self.image_lock,
                           self.controls_recv, self.simulation_state_send,
                           self.vehicle_state_send, self.exit_event, 
                           self.op_engaged, test_duration, self.test_run)
)
```

**Shared Memory for Images**
```python
# Shared camera arrays for zero-copy image transfer
self.camera_array = Array(ctypes.c_uint8, W*H*3)
self.road_image = np.frombuffer(self.camera_array.get_obj(), 
                               dtype=np.uint8).reshape((H, W, 3))

if dual_camera:
    self.wide_camera_array = Array(ctypes.c_uint8, W*H*3)
    self.wide_road_image = np.frombuffer(self.wide_camera_array.get_obj(), 
                                        dtype=np.uint8).reshape((H, W, 3))
```

### Performance Configuration

**Physics Optimization**
```python
config = dict(
    physics_world_step_size=self.TICKS_PER_FRAME/100,  # 50ms steps (20Hz)
    decision_repeat=1,                                  # Immediate decisions
    preload_models=False,                              # Faster startup
    anisotropic_filtering=False,                       # Faster rendering
    show_logo=False,                                   # Skip splash screen
)
```

**Frame Rate Management**
```python
# Main loop: 100Hz
self.rk = Ratekeeper(100, None)

# Physics updates: 20Hz (every 5th frame)
if self.rk.frame % self.TICKS_PER_FRAME == 0:
    self.world.tick()
    self.world.read_cameras()

# Camera updates: 20Hz
self.simulated_camera_thread = threading.Thread(
    target=rk_loop,
    args=(functools.partial(self.simulated_sensors.send_camera_images, self.world),
          20, self._exit_event)
)

# Vehicle state: 100Hz
self.simulated_car_thread = threading.Thread(
    target=rk_loop,
    args=(functools.partial(self.simulated_car.update, self.simulator_state),
          100, self._exit_event)
)
```

### Memory Management

**Image Buffer Management**
```python
class CopyRamRGBCamera(RGBCamera):
    """Camera which copies content into RAM for faster image grabbing"""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cpu_texture = Texture()
        self.buffer.addRenderTexture(self.cpu_texture, GraphicsOutput.RTMCopyRam)
    
    def get_rgb_array_cpu(self):
        origin_img = self.cpu_texture
        img = np.frombuffer(origin_img.getRamImage().getData(), dtype=np.uint8)
        img = img.reshape((origin_img.getYSize(), origin_img.getXSize(), -1))
        img = img[:,:,:3]  # RGBA to RGB
        img = img[::-1]    # Flip vertical axis
        return img
```

**Process Communication Optimization**
```python
# Use Pipe for low-latency communication
self.controls_send, self.controls_recv = Pipe()
self.simulation_state_send, self.simulation_state_recv = Pipe()
self.vehicle_state_send, self.vehicle_state_recv = Pipe()

# Non-blocking reads
while self.simulation_state_recv.poll(0):
    md_state = self.simulation_state_recv.recv()
    # Process state...
```

## Troubleshooting

### Common Issues

**MetaDrive Installation Problems**
```bash
# Problem: MetaDrive fails to install
# Solution: Install system dependencies
sudo apt-get update
sudo apt-get install freeglut3-dev libgl1-mesa-dev libglu1-mesa-dev
pip install metadrive-simulator

# Problem: CUDA/GPU issues
# Solution: Check NVIDIA drivers and CUDA installation
nvidia-smi
python3 -c "import cupy; print('CUDA OK')" || echo "CUDA not available, using CPU"
```

**OpenCL Issues**
```bash
# Problem: OpenCL not found
# Solution: Install OpenCL runtime
sudo apt-get install ocl-icd-libopencl1 opencl-headers clinfo
clinfo  # Verify OpenCL devices

# Problem: PyOpenCL compilation errors
# Solution: Install development headers
sudo apt-get install python3-dev
pip install --upgrade pyopencl
```

**Performance Issues**
```bash
# Problem: Low frame rate
# Check CPU usage
htop
# Check GPU usage (if NVIDIA)
nvidia-smi
# Reduce quality settings
./tools/sim/run_bridge.py  # (without --high_quality)
```

### Debugging Techniques

**Enable Debug Logging**
```python
# Add debug prints to bridge
def print_status(self):
    print(f"""
State:
Ignition: {self.simulator_state.ignition}
Engaged: {self.simulator_state.is_engaged} 
Speed: {self.simulator_state.speed:.1f} m/s
Steering: {self.simulator_state.steering_angle:.1f} deg
""")
```

**Process Monitoring**
```bash
# Check running processes
ps aux | grep -E "(bridge|metadrive|openpilot)"

# Monitor message queues
cd nagaspilot
python3 -c "
import cereal.messaging as messaging
sm = messaging.SubMaster(['carState', 'carControl'])
while True:
    sm.update(1000)
    print(f'Speed: {sm[\"carState\"].vEgo:.1f}, Controls: {sm[\"carControl\"].enabled}')
"
```

**Camera Debug**
```python
# Check camera image flow
def send_camera_images(self, world: 'World'):
    world.image_lock.acquire()
    print(f"Camera shape: {world.road_image.shape}")
    print(f"Image range: {world.road_image.min()}-{world.road_image.max()}")
    
    yuv = self.camerad.rgb_to_yuv(world.road_image)
    print(f"YUV size: {len(yuv)} bytes")
    self.camerad.cam_send_yuv_road(yuv)
```

### Error Resolution

**Simulation Crashes**
```python
# Problem: MetaDrive process crashes
# Solution: Check error logs and restart
try:
    self._run(q)
except Exception as e:
    print(f"Bridge error: {e}")
    traceback.print_exc()
finally:
    self.close("bridge terminated")
```

**Control Issues**
```python
# Problem: Vehicle doesn't respond to controls
# Debug control application
def apply_controls(self, steer_angle, throttle_out, brake_out):
    print(f"Applying: steer={steer_angle:.2f}, throttle={throttle_out:.2f}, brake={brake_out:.2f}")
    
    if (time.monotonic() - self.reset_time) > 2:
        self.vc[0] = steer_angle
        self.vc[1] = throttle_out if throttle_out else -brake_out
    else:
        print("Controls blocked - recently reset")
        self.vc[0] = 0
        self.vc[1] = 0
```

**Engagement Problems**
```python
# Problem: openpilot won't engage
# Debug engagement conditions
def update(self, simulator_state: SimulatorState):
    print(f"Ignition: {simulator_state.ignition}")
    print(f"Speed: {simulator_state.speed:.1f} m/s")
    print(f"Engageable: {self.sm['selfdriveState'].engageable}")
    print(f"Engaged: {self.sm['selfdriveState'].active}")
```

### Performance Tuning

**CPU Optimization**
```bash
# Check CPU governor
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# Set performance mode
sudo cpufreq-set -g performance

# Check CPU affinity
taskset -c 0-3 ./tools/sim/run_bridge.py  # Use first 4 cores
```

**Memory Optimization**
```bash
# Check memory usage
free -h
# Check swap usage  
swapon --show

# Increase swap if needed
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

**Graphics Optimization**
```bash
# For integrated graphics
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330

# For NVIDIA
export __GL_SYNC_TO_VBLANK=0
export __GL_SYNC_DISPLAY_DEVICE=1
```

## API Reference

### Core Classes

#### SimulatorBridge
```python
class SimulatorBridge(ABC):
    """Abstract base class for simulation bridges"""
    
    TICKS_PER_FRAME = 5  # Physics updates per frame
    
    def __init__(self, dual_camera: bool, high_quality: bool):
        """Initialize bridge with camera and quality settings"""
        
    @abstractmethod
    def spawn_world(self, q: Queue) -> World:
        """Create and return simulation world"""
        pass
        
    def run(self, queue: Queue, retries: int = -1) -> Process:
        """Start bridge process"""
        
    def shutdown(self):
        """Gracefully shutdown bridge"""
```

#### MetaDriveBridge
```python
class MetaDriveBridge(SimulatorBridge):
    """MetaDrive-specific simulation bridge"""
    
    def __init__(self, dual_camera: bool, high_quality: bool, 
                 test_duration: float = math.inf, test_run: bool = False):
        """Initialize MetaDrive bridge"""
        
    def spawn_world(self, queue: Queue) -> MetaDriveWorld:
        """Create MetaDrive world with configured sensors"""
```

#### SimulatedCar
```python
class SimulatedCar:
    """Simulates Honda Civic 2022 CAN messages and panda state"""
    
    packer = CANPacker("honda_civic_ex_2022_can_generated")
    
    def __init__(self):
        """Initialize CAN message simulation"""
        
    def send_can_messages(self, simulator_state: SimulatorState):
        """Generate and send CAN messages based on simulator state"""
        
    def send_panda_state(self, simulator_state: SimulatorState):
        """Send panda hardware state messages"""
        
    def update(self, simulator_state: SimulatorState):
        """Update all vehicle messages"""
```

#### SimulatedSensors
```python
class SimulatedSensors:
    """Simulates C3 sensors (IMU, GPS, cameras, peripherals)"""
    
    def __init__(self, dual_camera: bool = False):
        """Initialize sensor simulation"""
        
    def send_imu_message(self, simulator_state: SimulatorState):
        """Send accelerometer and gyroscope data"""
        
    def send_gps_message(self, simulator_state: SimulatorState):
        """Send GPS location and velocity data"""
        
    def send_camera_images(self, world: World):
        """Process and send camera images"""
        
    def update(self, simulator_state: SimulatorState, world: World):
        """Update all sensor data"""
```

### Data Structures

#### SimulatorState
```python
class SimulatorState:
    """Complete simulation state container"""
    
    def __init__(self):
        self.valid: bool = False
        self.is_engaged: bool = False  
        self.ignition: bool = True
        
        # Vehicle dynamics
        self.velocity: vec3 = None
        self.bearing: float = 0
        self.steering_angle: float = 0
        self.speed: float = 0  # Computed property
        
        # User inputs
        self.user_gas: float = 0
        self.user_brake: float = 0  
        self.user_torque: float = 0
        self.cruise_button: int = 0
        
        # Indicators
        self.left_blinker: bool = False
        self.right_blinker: bool = False
        
        # Sensor data
        self.gps: GPSState = GPSState()
        self.imu: IMUState = IMUState()
```

#### QueueMessage
```python
QueueMessage = namedtuple("QueueMessage", ["type", "info"], defaults=[None])

class QueueMessageType(Enum):
    START_STATUS = 0      # Bridge startup status
    CONTROL_COMMAND = 1   # User control input
    TERMINATION_INFO = 2  # Simulation termination
    CLOSE_STATUS = 3      # Bridge shutdown status

def control_cmd_gen(cmd: str) -> QueueMessage:
    """Generate control command message"""
    return QueueMessage(QueueMessageType.CONTROL_COMMAND, cmd)
```

### Control Functions

#### Keyboard Control
```python
def keyboard_poll_thread(q: Queue[QueueMessage]):
    """Main keyboard input polling loop"""
    
def getch() -> str:
    """Get single character input without echo"""
    
def print_keyboard_help():
    """Display keyboard command reference"""
```

#### Joystick Control  
```python
def wheel_poll_thread(q: Queue[str]) -> NoReturn:
    """Main joystick/wheel input polling loop"""
    
# Control mappings
axis_names: dict[int, str]    # Joystick axis mappings
button_names: dict[int, str]  # Joystick button mappings
```

### Camera System

#### Camerad
```python
class Camerad:
    """Simulates the camerad daemon"""
    
    def __init__(self, dual_camera: bool):
        """Initialize camera simulation with VisionIPC"""
        
    def rgb_to_yuv(self, rgb: np.ndarray) -> bytes:
        """Convert RGB image to YUV420 format using OpenCL"""
        
    def cam_send_yuv_road(self, yuv: bytes):
        """Send road camera YUV data via VisionIPC"""
        
    def cam_send_yuv_wide_road(self, yuv: bytes):
        """Send wide road camera YUV data via VisionIPC"""
```

#### Camera Classes
```python
class RGBCameraRoad(CopyRamRGBCamera):
    """Primary road camera (40° FOV)"""
    
class RGBCameraWide(CopyRamRGBCamera): 
    """Wide angle camera (120° FOV)"""
    
class CopyRamRGBCamera(RGBCamera):
    """Base camera class with RAM copying for fast image access"""
    
    def get_rgb_array_cpu(self) -> np.ndarray:
        """Get RGB image data as numpy array"""
```

### Utility Functions

#### Map Generation
```python
def straight_block(length: float) -> dict:
    """Create straight road segment configuration"""
    
def curve_block(length: float, angle: float = 45, direction: int = 0) -> dict:
    """Create curved road segment configuration"""
    
def create_map(track_size: int = 60) -> dict:
    """Generate rectangular track map configuration"""
```

#### Bridge Utilities
```python
def rk_loop(function: callable, hz: int, exit_event: threading.Event):
    """Rate-keeping loop for threaded operations"""
    
def apply_metadrive_patches(arrive_dest_done: bool = True):
    """Apply necessary patches to MetaDrive for openpilot integration"""
```

### Constants and Configuration

```python
# Image dimensions
W, H = 1928, 1208  # Camera resolution

# Vehicle position in MetaDrive
C3_POSITION = Vec3(0.0, 0, 1.22)  # comma three height
C3_HPR = Vec3(0, 0, 0)             # Heading, pitch, roll

# Simulation timing
TICKS_PER_FRAME = 5  # Physics updates per frame (20Hz physics at 100Hz loop)

# Default track size
DEFAULT_TRACK_SIZE = 60  # meters per straight segment
```

### Command Line Interface

```bash
# Main bridge command
./tools/sim/run_bridge.py [OPTIONS]

OPTIONS:
  --joystick      Enable joystick/wheel control instead of keyboard
  --high_quality  Enable high-quality rendering and enhanced features  
  --dual_camera   Enable both road and wide-angle cameras
  -h, --help      Show help message and exit

# openpilot launch command
./tools/sim/launch_openpilot.sh

# Environment variables (set in launch_openpilot.sh)
export PASSIVE="0"                    # Active mode
export NOBOARD="1"                   # No hardware required
export SIMULATION="1"                # Simulation mode
export SKIP_FW_QUERY="1"            # Skip firmware checks
export FINGERPRINT="HONDA_CIVIC_2022" # Vehicle type
export BLOCK="camerad,loggerd,encoderd,micd,logmessaged"  # Blocked processes
```

---

## Getting Started Checklist

1. **✅ Install Dependencies**
   - MetaDrive simulation engine
   - OpenCL runtime and development headers
   - Python dependencies (pyopencl, numpy)

2. **✅ Test Installation**
   - Verify MetaDrive import
   - Check OpenCL device availability
   - Test joystick hardware (optional)

3. **✅ Launch Simulation**
   - Start openpilot in simulation mode
   - Launch simulation bridge
   - Verify camera feed and controls

4. **✅ Test Controls**
   - Try keyboard controls (WASD, 1/2/3)
   - Test cruise control engagement  
   - Verify emergency brake (S key)

5. **✅ Monitor Performance**
   - Check frame rates and CPU usage
   - Verify GPU acceleration (if available)
   - Test reset functionality (R key)

**Quick Start Commands:**
```bash
# Terminal 1: Launch openpilot
./tools/sim/launch_openpilot.sh

# Terminal 2: Start simulation  
./tools/sim/run_bridge.py

# Controls: 2 to engage, 1 to accelerate, S to brake, Q to quit
```

This manual provides comprehensive coverage of the nagaspilot Simulation Framework, from basic usage to advanced development and debugging. The simulation environment offers a safe, controlled way to develop, test, and validate autonomous driving algorithms without requiring real vehicles or risking safety.