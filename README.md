# Change update: intake retract command ownership

This branch makes intake retraction an exclusive command by declaring `IntakeSubsystem` as a requirement of `RetractIntake`. It also changes `IntakeSubsystem.retractCommand()` to return the same retract profile used by the operator control rather than calling an IO method whose target value was not consumed by the closed-loop update.

The positive impact is a reliable stow action from every public command path. Deploy, manual, and retract actions can no longer run concurrently and fight for the intake mechanism, reducing unexpected motion during teleop and mode transitions.

## TyRap XXVI robot code

FRC robot software for Team Voltage 386.
