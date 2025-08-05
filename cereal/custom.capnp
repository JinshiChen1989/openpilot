using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

struct NpControlsState @0x81c2f05a394cf4af {
  alkaActive @0 :Bool;                    # EXISTING

  # DCP Foundation @1-@15 (Longitudinal Control)
  npDcpMode @1 :UInt8;                    # Core DCP mode control (0=Off/Fallback, 1=Highway, 2=Urban, 3=DCP)
  npDcpStatus @2 :Bool;                   # DCP operational status
  npDcpPersonality @3 :UInt8;             # DCP personality setting (0=Relaxed, 1=Standard, 2=Aggressive)
  npDcpSafetyFallback @4 :Bool;           # DCP safety state
  npDcpFilterLayersActive @5 :Bool;       # Filter layer architecture active
  npDcpActiveFiltersCount @6 :UInt8;      # Number of currently active filters
  npDcpBaseSpeed @7 :Float32;             # Base speed from DCP foundation
  npDcpFinalSpeed @8 :Float32;            # Final speed after filter processing
  npDcpHighwayBias @9 :Float32;           # Highway mode bias (0.0-1.0)
  npDcpUrbanBias @10 :Float32;            # Urban mode bias (0.0-1.0)
  npDcpFoundationReady @11 :Bool;         # DCP foundation system ready
  npDcpFallbackActive @12 :Bool;          # Independent fallback control active
  npDcpMpcMode @13 :Text;                 # Current MPC mode (acc/blended/null)
  npDcpErrorCount @14 :UInt8;             # Consecutive error count
  npDcpReserved15 @15 :Void;              # Reserved placeholder

  # DLP Foundation @16-@25 (Lateral Control)
  npDlpMode @16 :UInt8;                   # DLP mode control (0=Off/Fallback, 1=Laneless, 2=Auto)
  npDlpStatus @17 :Bool;                  # DLP operational status
  npDlpVisionCurve @18 :Bool;             # Vision curve detection active
  npDlpLaneConfidence @19 :Float32;       # Current lane confidence level
  npDlpPathOffset @20 :Float32;           # Current DLP path offset
  npDlpModeAuto @21 :Bool;                # Auto mode decision active
  npDlpFoundationReady @22 :Bool;         # DLP foundation system ready
  npDlpEnhancementActive @23 :Bool;       # Enhancement systems (SOC/VRC) active
  npDlpFallbackActive @24 :Bool;          # Independent DLP fallback active
  npDlpReserved25 @25 :Void;              # Reserved placeholder
  
  # Speed Controllers @26-@40 (Phase 2)
  npVtscEnabled @26 :Bool;                # VTSC toggle state
  npVtscActive @27 :Bool;                 # VTSC currently limiting speed
  npVtscTargetSpeed @28 :Float32;         # VTSC calculated speed limit
  npVtscCurrentCurvature @29 :Float32;    # Current detected curvature (1/m)
  npVtscDistanceToCurve @30 :Float32;     # Distance to curve (m)
  npVtscState @31 :UInt8;                 # VTSC state machine state (0=Disabled, 1=Monitoring, 2=Entering, 3=Turning, 4=Leaving)
  
  # VCSC (Vertical Comfort Speed Controller) @32-@37
  npVcscEnabled @32 :Bool;                # VCSC toggle state
  npVcscActive @33 :Bool;                 # VCSC currently limiting speed
  npVcscTargetSpeed @34 :Float32;         # VCSC calculated speed limit
  npVcscComfortScore @35 :Float32;        # Current road comfort score
  npVcscConfidence @36 :Float32;          # Kalman filter confidence level
  npVcscSpeedReduction @37 :Float32;      # Current speed reduction (m/s)
  
  # MTSC (Map Turn Speed Controller) @38-@40
  npMtscEnabled @38 :Bool;                # MTSC toggle state
  npMtscActive @39 :Bool;                 # MTSC currently limiting speed
  npMtscTargetSpeed @40 :Float32;         # MTSC calculated speed limit
  
  # PDA (Parallel Drive Avoidance) @41-@43
  npPdaEnabled @41 :Bool;                 # PDA toggle state
  npPdaActive @42 :Bool;                  # PDA currently boosting speed for overtaking
  npPdaTargetSpeed @43 :Float32;          # PDA target speed with boost
  npPdaReserved44 @44 :Void;              # Reserved placeholder
  npPdaReserved45 @45 :Void;              # Reserved placeholder
  npPdaReserved46 @46 :Void;              # Reserved placeholder
  
  # GCF (Gradient Compensation Factor) @47-@50
  npGcfEnabled @47 :Bool;                 # GCF toggle state
  npGcfActive @48 :Bool;                  # GCF currently reducing speed
  npGcfCurrentSlope @49 :Float32;         # Current detected slope (degrees)
  npGcfSpeedFactor @50 :Float32;          # Applied speed reduction factor
  
  # Monitoring System Enhancements @51-@56 (Phase 4)
  npSsdEnabled @51 :Bool;                 # SSD toggle
  npSsdActive @52 :Bool;                  # SSD currently managing standstill
  npSsdTimeRemaining @53 :Float32;        # SSD time remaining in standstill
  npSsdDurationLevel @54 :UInt8;          # SSD duration setting (0-3)
  npHodEnabled @55 :Bool;                 # HOD toggle
  npHodActive @56 :Bool;                  # HOD currently bypassing DM
  
  # SOC (Smart Offset Controller) @57-@59 (Phase 5 Safety Layer)
  npSocEnabled @57 :Bool;                 # SOC toggle state
  npSocActive @58 :Bool;                  # SOC currently applying lateral offset
  npSocOffset @59 :Float32;               # Current lateral offset (+ = left, - = right)
  
  # APSL (Accelerator Pedal Speed Learning) @60-@63 (Dual-Pedal Speed Learning System)
  npApslEnabled @60 :Bool;                # APSL toggle state
  npApslLearningMode @61 :Bool;           # Currently learning from gas pedal input
  npApslPedalPressed @62 :Bool;           # Gas pedal above threshold
  npApslLearnedSpeed @63 :Float32;        # Learned target speed (m/s)
  
  # BPSL (Brake Pedal Speed Learning) @64-@68 (Dual-Pedal Speed Learning System)
  npBpslEnabled @64 :Bool;                # BPSL toggle state
  npBpslLearningMode @65 :Bool;           # Currently learning from brake input
  npBpslBrakePressed @66 :Bool;           # Brake above threshold
  npBpslLearnedSpeed @67 :Float32;        # Learned target speed (m/s)
  npBpslSystemBrakeActive @68 :Bool;      # System vs manual brake detection
}

struct ModelExt @0xaedffd8f31e7b55d {
  leftEdgeDetected @0 :Bool;
  rightEdgeDetected @1 :Bool;
}

struct CustomReserved2 @0xf35cc4560bbf6ec2 {
}

struct CustomReserved3 @0xda96579883444c35 {
}

struct CustomReserved4 @0x80ae746ee2596b11 {
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}
