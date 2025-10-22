# Vision System Independence Fix + Auto-Align with Offset Feature - Completed ✅

## ✅ All Changes Completed:

### 1. **Constants.java** - Camera names match PhotonVision
   - ✅ Camera names set to "CAM_FL" and "CAM_FR"
   - ✅ Matches actual PhotonVision camera configuration

### 2. **VisionSubsystem.java** - Enhanced with comprehensive metrics
   - ✅ Added import for `frc.robot.Constants`
   - ✅ Updated camera initialization to use `Constants.Vision.kCameraNameFL` and `Constants.Vision.kCameraNameFR`
   - ✅ Maintained independent processing for each camera
   - ✅ Each camera uses `getAllUnreadResults()` for real-time updates
   - ✅ **Detects and displays ANY AprilTag ID, not just ID 14**
   - ✅ **Added comprehensive metrics for each camera**
   - ✅ **Flat naming structure (no folders) for easy viewing in SmartDashboard**
   - ✅ Separate SmartDashboard outputs for FL and FR cameras

### 3. **Robot.java** - Removed duplicate logic, fixed bug, added controller outputs
   - ✅ Removed duplicate PhotonCamera instances (cameraFL, cameraFR)
   - ✅ Removed duplicate vision processing in `teleopPeriodic()`
   - ✅ Fixed bug where FR camera was checking `result.hasTargets()` instead of `result2.hasTargets()`
   - ✅ Now uses VisionSubsystem methods to get camera data
   - ✅ Removed unused PhotonCamera import
   - ✅ Added separate auto-turn commands for FL (A button) and FR (B button) cameras
   - ✅ **Added complete Xbox controller button and axis outputs to SmartDashboard**

### 4. **RobotContainer.java** - Updated button bindings
   - ✅ Added VisionSubsystem instance
   - ✅ Added `getVisionSubsystem()` getter method
   - ✅ VisionSubsystem now properly integrated into robot architecture
   - ✅ **NEW: Start button resets field-centric heading**
   - ✅ **NEW: Left bumper aligns 1 foot LEFT of AprilTag**
   - ✅ **NEW: Right bumper aligns 1 foot RIGHT of AprilTag**
   - ✅ Removed SysId routines from Start button (now only on Back button)

### 5. **AlignToAprilTagCommand.java** - Enhanced Auto-Align with Lateral Offset
   - ✅ **Created new command for automatic alignment to AprilTag**
   - ✅ **Drives robot to target distance from detected tag**
   - ✅ **Centers robot and faces the tag**
   - ✅ **NEW: Supports lateral offset positioning (left/right of tag)**
   - ✅ **Uses both FL and FR cameras (tries FL first, falls back to FR)**
   - ✅ **PID-like control for smooth alignment including strafing**
   - ✅ **Real-time SmartDashboard feedback during alignment**
   - ✅ **Fixed: Uses `setControl()` for proper drivetrain control**
   - ✅ **Stops robot completely when within tolerance (no drift)**
   - ✅ **Two constructors: one for center alignment, one for offset alignment**

## 🎯 Result:

Both cameras (FL and FR) now work **completely independently** with **comprehensive metrics** and **advanced auto-align capability with lateral offset**:
- ✅ Each camera processes its own results in real-time
- ✅ Detects and identifies ANY AprilTag (not limited to ID 14)
- ✅ No requirement for both cameras to see targets
- ✅ Real-time updates via `getAllUnreadResults()`
- ✅ No duplicate processing logic
- ✅ Bug fixed where FR camera was checking wrong result
- ✅ **Build successful** - Code compiles without errors or warnings
- ✅ **Flat naming** - All metrics appear at top level in SmartDashboard
- ✅ **Complete controller monitoring** - All Xbox controller inputs visible
- ✅ **Auto-align with offset** - Left/Right bumpers align to left/right of tag
- ✅ **Robot stops when aligned** - No drift when within tolerance
- ✅ **Lateral positioning** - Can position robot to the side of tags

## 🎮 Updated Button Configuration:

### **Xbox Controller Buttons:**
- **A Button**: Brake (hold wheels in X pattern)
- **B Button**: Point wheels toward joystick direction
- **X Button**: SysId Reverse (when holding Back button)
- **Y Button**: SysId Forward (when holding Back button)
- **Left Bumper**: 🆕 **Align 1 foot to the LEFT of AprilTag**
- **Right Bumper**: 🆕 **Align 1 foot to the RIGHT of AprilTag**
- **Back Button**: Used with X/Y for SysId routines
- **Start Button**: 🆕 **Reset field-centric heading (gyro reset)**
- **Left Stick**: Drive forward/backward and strafe left/right
- **Right Stick**: Rotate robot

### **Auto-Align Behavior:**
- **Left Bumper (Hold)**: Robot aligns to be 1 foot (0.3048m) to the LEFT of the detected AprilTag
- **Right Bumper (Hold)**: Robot aligns to be 1 foot (0.3048m) to the RIGHT of the detected AprilTag
- Both maintain 6 meters distance from tag and face toward it
- Robot will strafe left/right to achieve the lateral offset position

## 📊 SmartDashboard Outputs:

### **Front-Left Camera (FL) Metrics:**
- `FL Target Visible` - Whether any target is detected
- `FL Detected Tag ID` - The AprilTag ID currently detected (-1 if none)
- `FL Target Yaw (deg)` - Horizontal angle to target
- `FL Target Pitch (deg)` - Vertical angle to target
- `FL Target Area (%)` - Percentage of camera view occupied by target
- `FL Approx Distance` - Approximate distance to target (rough estimation)
- `FL Total Targets` - Number of AprilTags visible in frame
- `FL Camera Connected` - Camera connection status

### **Front-Right Camera (FR) Metrics:**
- `FR Target Visible` - Whether any target is detected
- `FR Detected Tag ID` - The AprilTag ID currently detected (-1 if none)
- `FR Target Yaw (deg)` - Horizontal angle to target
- `FR Target Pitch (deg)` - Vertical angle to target
- `FR Target Area (%)` - Percentage of camera view occupied by target
- `FR Approx Distance` - Approximate distance to target (rough estimation)
- `FR Total Targets` - Number of AprilTags visible in frame
- `FR Camera Connected` - Camera connection status

### **Auto-Align Status (when Left/Right Bumper pressed):**
- `Auto Align Active` - Whether auto-align is currently running (true/false)
- `Auto Align Status` - Current status ("Aligning...", "ALIGNED", "No Target Visible", "Complete", "Interrupted")
- `Auto Align Yaw Error` - Current yaw error in degrees
- `Auto Align Distance Error` - Current distance error in meters
- `Auto Align Lateral Error` - 🆕 Current lateral position error in meters
- `Auto Align Lateral Offset` - 🆕 Target lateral offset (0.3048 for left, -0.3048 for right)
- `Auto Align Target Distance` - Current distance to target
- `Auto Align Detected Tag` - Which AprilTag ID is being tracked
- `Auto Align Forward Speed` - Current forward/backward speed command (0 when aligned)
- `Auto Align Strafe Speed` - 🆕 Current left/right strafe speed command (0 when aligned)
- `Auto Align Rotation Speed` - Current rotation speed command (0 when aligned)

### **Xbox Controller Buttons:**
- `Controller A Button` - A button state (true/false)
- `Controller B Button` - B button state (true/false)
- `Controller X Button` - X button state (true/false)
- `Controller Y Button` - Y button state (true/false)
- `Controller Left Bumper` - Left bumper state (true/false)
- `Controller Right Bumper` - Right bumper state (true/false)
- `Controller Back Button` - Back button state (true/false)
- `Controller Start Button` - Start button state (true/false)
- `Controller Left Stick Button` - Left stick button state (true/false)
- `Controller Right Stick Button` - Right stick button state (true/false)

### **Xbox Controller Axes:**
- `Controller Left Stick X` - Left stick horizontal (-1.0 to 1.0)
- `Controller Left Stick Y` - Left stick vertical (-1.0 to 1.0)
- `Controller Right Stick X` - Right stick horizontal (-1.0 to 1.0)
- `Controller Right Stick Y` - Right stick vertical (-1.0 to 1.0)
- `Controller Left Trigger` - Left trigger (0.0 to 1.0)
- `Controller Right Trigger` - Right trigger (0.0 to 1.0)
- `Controller POV` - D-Pad angle in degrees (-1 if not pressed, 0=up, 90=right, 180=down, 270=left)

### **Legacy Vision Outputs (for compatibility):**
- `AprilTag 14 Yaw FL` - Yaw angle from front-left camera
- `Vision Target Visible FL` - Whether FL camera sees target
- `AprilTag 14 Yaw FR` - Yaw angle from front-right camera
- `Vision Target Visible FR` - Whether FR camera sees target

### **Auto-Turn Commands:**
- `Auto Turn Command FL` - Turn command when A button held and FL sees target
- `Auto Turn Command FR` - Turn command when B button held and FR sees target

## 🎮 Key Features:

1. **Multi-Tag Detection**: Each camera will detect ANY AprilTag, not just ID 14
2. **Priority System**: If multiple tags are visible, it prioritizes Tag ID 14, otherwise uses the best (closest/largest) target
3. **Comprehensive Metrics**: Distance estimation, target area, pitch/yaw angles, connection status, and more
4. **Independent Operation**: Each camera works completely independently
5. **Real-time Updates**: Uses `getAllUnreadResults()` for minimal latency
6. **Tag Identification**: Shows which specific AprilTag ID each camera is seeing
7. **Flat Structure**: All metrics appear at the top level in SmartDashboard (no folders)
8. **Complete Controller Monitoring**: All Xbox controller buttons, axes, triggers, and D-Pad visible in real-time
9. **🆕 Auto-Align with Lateral Offset**: Left/Right bumpers position robot to the side of AprilTags
10. **🆕 Stops When Aligned**: Robot completely stops when within tolerance (no drift or oscillation)
11. **🆕 Strafing Control**: Robot can move sideways to achieve lateral positioning

## 🎯 Auto-Align Feature Details:

### **How to Use:**

**Left Bumper (Align Left of Tag):**
1. Point robot toward an AprilTag (either camera can see it)
2. **Hold Left Bumper** on Xbox controller
3. Robot will automatically:
   - Rotate to face the tag
   - Drive forward/backward to reach 6 meters distance
   - **Strafe left to position 1 foot to the LEFT of the tag**
   - **Stop completely when aligned** (within tolerance)
4. Release Left Bumper to stop auto-align

**Right Bumper (Align Right of Tag):**
1. Point robot toward an AprilTag (either camera can see it)
2. **Hold Right Bumper** on Xbox controller
3. Robot will automatically:
   - Rotate to face the tag
   - Drive forward/backward to reach 6 meters distance
   - **Strafe right to position 1 foot to the RIGHT of the tag**
   - **Stop completely when aligned** (within tolerance)
4. Release Right Bumper to stop auto-align

**Start Button (Reset Heading):**
- Press Start button to reset the field-centric heading (gyro reset)

### **Auto-Align Behavior:**
- **Camera Fallback**: Tries FL camera first, automatically switches to FR if FL doesn't see target
- **Smooth Control**: Uses proportional control (PID-like) for smooth movements including strafing
- **Speed Limits**: Max 1.0 m/s forward/backward and strafe, 1.0 rad/s rotation
- **Tolerance**: Stops when within 5° yaw, 1.0m distance, and 0.1m lateral position
- **Complete Stop**: Robot velocity set to 0 when aligned (no drift)
- **Safety**: Stops immediately if no target visible
- **Real-time Feedback**: All alignment data visible in SmartDashboard including strafe speed

### **Current Tunable Constants (in AlignToAprilTagCommand.java):**
- `YAW_KP = 0.05` - Rotation speed gain (increase for faster rotation)
- `DISTANCE_KP = 1.5` - Forward/backward speed gain (increase for faster approach)
- `STRAFE_KP = 0.05` - 🆕 Left/right strafe speed gain (increase for faster lateral movement)
- `TARGET_DISTANCE = 6.0` - Target distance in meters (currently set to 6 meters)
- `YAW_TOLERANCE = 5.0` - Acceptable yaw error in degrees (±5°)
- `DISTANCE_TOLERANCE = 1.0` - Acceptable distance error in meters (±1m)
- `LATERAL_TOLERANCE = 0.1` - 🆕 Acceptable lateral position error in meters (±0.1m)
- `MAX_SPEED = 1.0` - Maximum forward/backward and strafe speed (m/s)
- `MAX_ROTATION_SPEED = 1.0` - Maximum rotation speed (rad/s)

### **Lateral Offset Explanation:**
- **Left Bumper**: `lateralOffset = +0.3048m` (1 foot) → Robot positions to the LEFT of tag
- **Right Bumper**: `lateralOffset = -0.3048m` (1 foot) → Robot positions to the RIGHT of tag
- Robot uses strafing (sideways movement) to achieve the lateral offset
- When aligned, robot will be facing the tag but offset to the side by 1 foot

### **Alignment Tolerance Explanation:**
With current settings:
- Robot stops when distance is between **5.0m and 7.0m** from the tag
- Robot stops when yaw is within **±5°** of center
- Robot stops when lateral position is within **±0.1m** of target offset
- When all three conditions are met, robot **completely stops** (velocity = 0)
- SmartDashboard will show "ALIGNED" status
- Forward Speed, Strafe Speed, and Rotation Speed will all show 0

## 🔧 Usage:

All metrics will now appear directly in your SmartDashboard:
- All FL camera metrics starting with "FL "
- All FR camera metrics starting with "FR "
- All controller inputs starting with "Controller "
- All auto-align status starting with "Auto Align "
- Easy to identify which camera is seeing which AprilTag
- Real-time distance and angle information for both cameras
- Monitor all controller inputs for debugging and testing
- **Hold Left Bumper to align 1 foot LEFT of any visible AprilTag**
- **Hold Right Bumper to align 1 foot RIGHT of any visible AprilTag**
- **Press Start to reset field-centric heading**
- **Watch "Auto Align Status" change to "ALIGNED" when robot stops**
- **Monitor "Auto Align Strafe Speed" to see lateral movement**

## ✅ Build Status:
- **Build: SUCCESSFUL** ✅
- **Warnings: NONE** ✅
- **Errors: NONE** ✅
- Ready for deployment to robot

## 🚀 Next Steps:
1. Deploy code to robot
2. Test auto-align feature with different AprilTags
3. Test Left Bumper (align left of tag) and Right Bumper (align right of tag)
4. Verify robot stops completely when within tolerance
5. Verify robot strafes correctly to achieve lateral offset
6. Tune PID constants if needed for smoother/faster alignment
7. Adjust TARGET_DISTANCE if 6 meters isn't optimal
8. Adjust lateral offset distance (currently 1 foot = 0.3048m) if needed
9. Test Start button for field-centric heading reset
