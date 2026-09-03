# Change update: calibrated shot gating

This branch prevents the auto-aim system from commanding a shot when the calculated target distance is outside the calibrated shooter data. `Turret` now reads the `LaunchingParameters.isValid()` result before it applies a hood setpoint, spins the flywheel, or enables the spindexer. Invalid shots hold the hood at its safe maximum angle, stop the flywheel, keep the spindexer off, and provide controller rumble while a shot is requested.

The scoring and passing distance limits were also narrowed to ranges for which every required table has calibration data. This avoids silently using a boundary value from an incomplete hood, RPM, or time-of-flight table. The positive impact is more predictable shot behavior: the robot will only feed a game piece when it has a calibrated solution, while drivers receive immediate feedback when repositioning is required.

## TyRap XXVI robot code

FRC robot software for Team Voltage 386.
