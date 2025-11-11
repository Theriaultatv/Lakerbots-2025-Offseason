# Pumpkin Subsystem Implementation - Completed

## Summary
Successfully added a new "Pumpkin" subsystem with a motor on CAN ID 13 that outputs 2 volts for 4 seconds when the Y button is pressed.

## Files Created/Modified

### ✅ Created Files:
1. **src/main/java/frc/robot/subsystems/PumpkinSubsystem.java**
   - TalonFX motor controller on CAN ID 13
   - VoltageOut control request for voltage-based control
   - Methods to set voltage and stop motor
   - Telemetry output to SmartDashboard (voltage and current)
   - Motor configured in Coast mode

2. **src/main/java/frc/robot/commands/RunPumpkinCommand.java**
   - Command to run motor at specified voltage for specified duration
   - Uses default values from Constants (2V, 4 seconds)
   - Properly stops motor when command ends or is interrupted
   - Includes console logging for debugging

### ✅ Modified Files:
3. **src/main/java/frc/robot/Constants.java**
   - Added `Pumpkin` inner class with:
     - `kMotorCANId = 13`
     - `kVoltageOutput = 2.0` volts
     - `kRunDuration = 4.0` seconds

4. **src/main/java/frc/robot/RobotContainer.java**
   - Instantiated PumpkinSubsystem
   - Bound Y button to RunPumpkinCommand with 4-second timeout
   - Added getter method for PumpkinSubsystem

## Features Implemented

### Motor Control:
- ✅ Motor on CAN ID 13
- ✅ Outputs 2 volts when commanded
- ✅ Runs for exactly 4 seconds
- ✅ Triggered by Y button press
- ✅ Automatically stops after timeout

### Safety Features:
- ✅ Motor stops when command ends
- ✅ Motor stops if command is interrupted
- ✅ Coast mode for smooth deceleration
- ✅ Timeout prevents indefinite running

### Telemetry:
- ✅ Motor voltage displayed on SmartDashboard
- ✅ Motor current displayed on SmartDashboard
- ✅ Console logging for command start/stop

## Testing Checklist

### Before Testing on Robot:
- [x] Code compiles successfully (BUILD SUCCESSFUL)
- [ ] Verify CAN ID 13 is not used by another device
- [ ] Check motor wiring and connections
- [ ] Ensure motor can safely spin (no obstructions)

### During Testing:
- [ ] Press Y button and verify motor spins
- [ ] Verify motor runs for 4 seconds
- [ ] Verify motor stops automatically after 4 seconds
- [ ] Check SmartDashboard for voltage reading (~2V)
- [ ] Check SmartDashboard for current reading
- [ ] Test interrupting command (press Y again during run)
- [ ] Verify motor stops when robot is disabled

### Optional Enhancements:
- [ ] Add current limiting if needed
- [ ] Add different voltage levels for different buttons
- [ ] Add variable duration control
- [ ] Add emergency stop button
- [ ] Add motor temperature monitoring

## Usage

**To run the Pumpkin motor:**
1. Enable the robot (teleop mode)
2. Press the **Y button** on the Xbox controller
3. Motor will spin at 2V for 4 seconds
4. Motor automatically stops after 4 seconds

**To stop early:**
- Press Y button again to interrupt the command
- Disable the robot

## Technical Details

**Motor Controller:** CTRE TalonFX (Phoenix 6)
**Control Mode:** VoltageOut
**CAN Bus:** CANivore ("canivore")
**CAN ID:** 13
**Voltage:** 2.0 volts
**Duration:** 4.0 seconds
**Neutral Mode:** Coast

## Reference
Based on Phoenix6 example: https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/ControlRequestLimits/src/main/java/frc/robot/Robot.java
