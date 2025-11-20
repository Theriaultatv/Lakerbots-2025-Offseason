# TODO: Align to Tag Button Press with Pumpkin Motor

## Tasks to Complete:

- [x] Update Constants.java - Change pumpkin duration from 4.0 to 3.0 seconds
- [x] Update AlignToAprilTagCommand.java - Make command finish when aligned
- [x] Update RobotContainer.java - Change left bumper to button press with pumpkin sequence
- [x] Fix timeout issue - Added 2-second timeout if no AprilTag detected
- [x] Update target distance - Changed from 7.0m to 1.0m (directly in front of tag)
- [ ] Test alignment on button press
- [ ] Verify pumpkin motor runs for 3 seconds after alignment

## Progress:
✓ All code changes completed successfully!
✓ Fixed issue where command would hang if no AprilTag was visible
✓ Updated alignment target to 1 meter directly in front of tag

## Changes Made:

1. **Constants.java**: Updated `kRunDuration` from 4.0 to 3.0 seconds

2. **AlignToAprilTagCommand.java**: 
   - Modified `isFinished()` to return `isAligned` instead of `false`
   - Added timeout mechanism (2 seconds) if no AprilTag is detected
   - Added debug logging to track command execution
   - **Updated TARGET_DISTANCE from 7.0m to 1.0m** - Robot now aligns 1 meter directly in front of tag with 0° heading
   - Command now properly finishes in these cases:
     * When robot is aligned to the tag
     * When no tag is found after 2 seconds

3. **RobotContainer.java**: 
   - Changed left bumper binding from `.whileTrue()` to `.onTrue()`
   - Added command sequence: Align to center of tag (1m away) → Run pumpkin motor for 3 seconds
   - Removed right bumper binding (no longer needed)

## Alignment Behavior:
- **Distance**: 1 meter from AprilTag
- **Heading**: 0 degrees (facing directly at the tag)
- **Position**: Centered on the tag (no lateral offset)

## Debug Messages:
The command now prints console messages to help diagnose issues:
- "Starting alignment" - When button is pressed
- "No target visible - waiting..." - When searching for AprilTag
- "Alignment complete!" - When aligned successfully
- "No target found after 2 seconds - canceling" - If timeout occurs

## Testing Required:
- Test that pressing left bumper starts alignment
- Verify robot aligns 1 meter directly in front of AprilTag with 0° heading
- Confirm pumpkin motor automatically runs for 3 seconds after alignment
- Ensure sequence completes without manual intervention
- Check console output for debug messages
