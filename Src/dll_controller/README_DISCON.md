# UNICO Controller - DISCON Input File Documentation

## Overview

This document describes the configuration and usage of the DISCON controller input file for wind turbine control in QBlade. The controller supports both Horizontal Axis Wind Turbines (HAWT) and Vertical Axis Wind Turbines (VAWT), including H-type and Eggbeater-type Darrieus configurations.


### File Placement
The DISCON input file **must be placed in the same directory** as the QBlade project file (`.qpr`).

### File Integrity Rules
**WARNING: Strict compliance with these rules is mandatory for the controller to function correctly.**

1. **DO NOT** add or remove any input parameters
2. **DO NOT** change the order of the inputs
3. **DO NOT** add or remove lines from the file
4. **DO NOT** remove or modify section headers
5. **DO NOT** rename the input file

---

## Input Configuration Guidelines

1. Always Provide Values for All Parameters

2. Even if a parameter is marked as "Not used" for your specific configuration, **you must always assign a value to every parameter**. The controller reads all inputs sequentially, and missing values will cause parsing errors or unexpected behavior.

**Example:**
```
127.72359           ! dRgn2K                    - (Not used in mode 0)    [Nm/(rad/s)^2]
```
Even though `dRgn2K` is not used when `dRgn2Mode = 0`, a numerical value must still be provided.

### Matrix and Array Dimensions

For gain-scheduled parameters and lookup tables, **the array size must be declared before the data**. The controller uses this dimension to correctly parse the subsequent values.

**Example - Torque Controller Gains:**
```
3                                           ! nKpBr - Number of points [-]
0.16  -618761084                            ! GS [rad/s]  KpBr [Nms/rad]
0.26  -618761084                            ! GS [rad/s]  KpBr [Nms/rad]
0.26  -618761084                            ! GS [rad/s]  KpBr [Nms/rad]
```
Here, `3` specifies that three data points follow.

**Example - Cp Performance Table:**
```
3  17                                       ! Cp_nPitch  Cp_nTSR - Matrix dimensions [-] [-]
```
This declares a 3×17 matrix (3 pitch angles, 17 TSR values).

---

## QBlade Integration

This controller is designed for **QBlade 2.0.8 Enterprise**.

### Wind Speed Input (SWAP Data)

The controller retrieves the undisturbed wind velocity from the SWAP array. The following line at the end of the input file specifies this:

```
SWAP DATA
26 "Abs Wind Vel. at Hub [m/s]"
```

- **Index 26** corresponds to the absolute wind velocity at hub height in QBlade 2.0.8 Enterprise
- This allows the controller to access the freestream wind speed for control calculations

> **Note:** SWAP indices may vary between QBlade versions. Verify compatibility if using a different version.

---

## Control Modes - Detailed Description

### Region 2 Control (`dRgn2Mode`)

Region 2 is the partial load region where the controller maximizes power capture.

#### Mode 0: Optimal TSR Tracking


```
dRgn2Mode = 0
```
The controller uses a PI controller to track the optimal tip-speed ratio (TSR). The reference generator speed is calculated as:

```
ω_ref = min(max(ω_min, ω_opt), ω_rated)

```

where `ω_opt = v × TSR_opt / R`

This mode provides smooth control across all sub-regions using the PI controller output directly.

#### Mode 1: K-ω² Torque Law (Switching Control)
```
dRgn2Mode = 1
```
This mode uses the classical K-ω² torque control law with switching between sub-regions:

**K Coefficient Calculation:**

If `dKSwitch = 1`: Uses user-defined value `dRgn2K`
If `dKSwitch = 0`: Automatically calculated based on architecture:
- **HAWT:**           `K = 0.5 × ρ × π × R⁵ × Cp_max / TSR_opt³`
- **VAWT H-type:**    `K = ρ × R⁴ × H × Cp_max / TSR_opt³`
- **VAWT Eggbeater:** `K = 0.5 × ρ × A × R³ × Cp_max / TSR_opt³`

**Sub-regions:**

| Sub-region |          Condition         |                     Control Law                       |
|------------|----------------------------|-------------------------------------------------------|
| Region 1.5 | ω < ω_minSwitch            | Linear interpolation / PI tracking to maintain ω_min  |
| Region 2   | ω_minSwitch ≤ ω < ω_switch | Direct K-ω² law: `T = K × ω²`                         |
| Region 2.5 | ω_switch ≤ ω < ω_rated     | Linear interpolation between K-ω² and PI output       |

The switching logic uses linear interpolation to ensure smooth transitions between sub-regions.

---

### Region 3 Control (`dRgn3Mode`)

Region 3 is the full load region where wind speed exceeds rated wind speed.

#### Mode 0: Rated Power Tracking
```
dRgn3Mode = 0
```
The controller calculates the required Cp to maintain rated power:

```
Cp_target = (P_rated × η) / (0.5 × ρ × A × v³)
```

The Cp is limited between 0 and Cp_max. The target TSR is then obtained by interpolating the 1D Cp/TSR curve (derived from the 2D Cp table), and the reference speed is:

```
ω_ref = v × TSR_R3 / R
```

#### Mode 1: Linear Speed Reduction
```
dRgn3Mode = 1
```
This mode reduces the rotor speed linearly with increasing wind speed:

- If `v < V_lin`: `ω_ref = ω_rated`
- If `v ≥ V_lin`: `ω_ref = ω_rated + m × (v - V_lin)`

Where:
- `V_lin` (`dVLin`): Wind speed at which linear reduction begins [m/s]
- `m` (`dMRgn3Lin`): Angular coefficient [(rad/s)/(m/s)], typically negative

The reference speed is limited to: `ω_ref = max(ω_ref, ω_min)`

**Example:** With `m = -0.02` and `V_lin = 25 m/s`, for every 1 m/s increase above 25 m/s, the reference speed decreases by 0.02 rad/s.

---

## Lookup Tables (LUTs) and Gain Scheduling

All lookup tables are processed in the source file `discon_main.c` using linear interpolation. The interpolation variable depends on the specific LUT.

### LUT Summary

|                   LUT                        |     Interpolation Variable         |                  Purpose                 |
|----------------------------------------------|------------------------------------|------------------------------------------|
| Torque Gains (KpBr, KiBr, KpAr, KiAr)        | Generator Speed          [rad/s]   | Gain scheduling for torque PI controller |
| Pitch PI Gains (PC_GS_Kp, PC_GS_Ki)          | Measured Pitch Angle     [rad]     | Gain scheduling for pitch PI controller  |
| Peak Shaving (PS_BldPitchMin)                | Wind Speed               [m/s]     | Minimum pitch angle schedule             |
| EKF System Poles (WE_FOPoles)                | Estimated Wind Speed     [m/s]     | Observer pole placement                  |
| Cp Performance Table                         | Pitch [deg] and TSR      [-]       | 2D interpolation for TBE/EKF             |

### Torque Controller Gains
```c
dKpBr = interpolateLUT(GS_KpBr, KpBr, nKpBr, GeneratorSpeed);
dKiBr = interpolateLUT(GS_KiBr, KiBr, nKiBr, GeneratorSpeed);
dKpAr = interpolateLUT(GS_KpAr, KpAr, nKpAr, GeneratorSpeed);
dKiAr = interpolateLUT(GS_KiAr, KiAr, nKiAr, GeneratorSpeed);
```

- **KpBr, KiBr**: Below-rated gains (Region 2)
- **KpAr, KiAr**: Above-rated gains (Region 3)

### Pitch PI Gains
```c
PC_Kp = interpolateLUT(PC_GS_angles, PC_GS_Kp, nPC_GS, MeasuredPitch);
PC_Ki = interpolateLUT(PC_GS_angles, PC_GS_Ki, nPC_GS, MeasuredPitch);
```

Gains are scheduled on the measured pitch angle to compensate for the nonlinear aerodynamic response.

### Peak Shaving
```c
pitch_min = interpolateLUT(PS_WindSpeeds, PS_BldPitchMin, nPS, WindSpeed);
```

Provides a minimum pitch angle as a function of wind speed to limit rotor thrust loads.

---

## Cp Performance Table

The Cp table is a 2D lookup table that maps pitch angle and tip-speed ratio to power coefficient.

### Table Structure
```
3  17                                       ! Cp_nPitch  Cp_nTSR
-1.0 0.0 1.0                                ! Cp_v_pitch [deg]
0.0 0.25 0.5 ... 4.0                        ! Cp_v_TSR   [-]
<Cp values: nTSR rows × nPitch columns>
```

### Usage Modes

The 2D table is used directly by the Wind Speed Estimator (TBE/EKF). Additionally, a 1D Cp/TSR curve is derived for Region 3 control based on the `dFPitch` parameter:

#### Variable Pitch Mode (`dFPitch = -100`)
```
dFPitch = -100    ! Sentinel value for variable pitch
```
The controller extracts the **maximum Cp envelope**: for each TSR value, it selects the maximum Cp across all pitch angles. This represents the best achievable performance for a pitch-controlled turbine.

#### Fixed Pitch Mode (`dFPitch ≠ -100`)
```
dFPitch = 0       ! Fixed pitch at 0 rad (0°)
```
The controller extracts the Cp/TSR curve at the specified fixed pitch angle. The value is in radians and is converted to degrees internally. This is used for stall-controlled turbines or for analysis at a specific pitch setting.

### 2D Interpolation (for TBE/EKF)
```c
Cp = interpolate2D(Cp_v_pitch, Cp_nPitch, Cp_v_TSR, Cp_nTSR, Cp_matrix, pitch_deg, TSR);
```

### 1D Interpolation (for Region 3)
```c
TSR_R3 = interpolateLUT(Cp_lut, TSR_lut, nCpTSR, Cp_target);
```

---

## Parameter Reference

### Architecture Selection

| Parameter               | Value |           Description               |
|-------------------------|-------|-------------------------------------|
| `Architecture_Selector` |   1   | HAWT (Horizontal Axis Wind Turbine) |
| `Architecture_Selector` |   2   | VAWT (Vertical Axis Wind Turbine)   |
| `Darrieus_Selector`     |   1   |         H-type VAWT                 |
| `Darrieus_Selector`     |   2   |     Eggbeater-type VAWT             |

### Control Modes Summary

| Parameter   | Value |             Description              |
|-------------|-------|--------------------------------------|
| `dRgn2Mode` |   0   | Optimal TSR tracking (PI controller) |
| `dRgn2Mode` |   1   |     K-ω² torque law with switching   |
| `dRgn3Mode` |   0   |        Rated power tracking          |
| `dRgn3Mode` |   1   |        Linear speed reduction        |

### Filter Switches

All filter switches follow the same convention:
- `0` = Filter OFF
- Any other value = Filter ON

### Wind Speed Estimator Modes

| `WSE_Mode` |             Description             |
|------------|-------------------------------------|
|     0      | Raw wind speed (direct measurement) |
|     1      |   Torque Balance Estimator (TBE)    |
|     2      |   Extended Kalman Filter (EKF)      |

---

## File Structure Summary

```
#UNICO CONTROLLER INPUT FILE
├── Echo settings
├── #GEOMETRY DATA
├── #WIND TURBINE SPECS
├── #GENERATOR SPECS
├── #CONTROL MODE
├── #DATA FILTERS and STATUS FLAGS
├── #SET POINT SMOOTHER FILTERS and STATUS FLAGS
├── #PITCH CONTROL INPUTS
├── #WINDSPEED ESTIMATOR INPUTS
├── #TORQUE CONTROLLER GAINS (GS on generator speed)
├── #PITCH PI GAINS (GS on pitch angle)
├── #PEAK SHAVING
├── #EKF SYSTEM POLES
├── #CP PERFORMANCE TABLE
├── #IMPORTANT NOTES
└── SWAP DATA
```

## Version History

| Version | QBlade Compatibility |              Notes              |
|---------|----------------------|---------------------------------|
| Current | 2.0.8 Enterprise     | SWAP index 26 for wind velocity |


