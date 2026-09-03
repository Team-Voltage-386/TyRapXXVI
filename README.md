# Change update: turret yaw hard-stop protection

This branch fixes the turret yaw limit handling so the voltage command is actually clamped whenever a left or right hard-stop limit is active. Previously, the clamp result was discarded and the original controller output could still be sent to the yaw motor.

The positive impact is a real software safety boundary at the mechanical limits. Auto-aim can move the turret away from an active stop but cannot continue driving into it, reducing stalled-motor current, protecting the geartrain, and preserving yaw calibration for the rest of the match.

## TyRap XXVI robot code

FRC robot software for Team Voltage 386.
