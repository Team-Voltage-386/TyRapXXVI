# Turret startup and travel

Yaw is measured from the intake direction (0 degrees). The permitted interval is
**+20 through +270 degrees**, inclusive. The forbidden arc is +270 through +360/0
to +20 degrees. Positive encoder yaw and positive commanded yaw voltage are
expected to move toward +270, matching the existing controller's convention.

## Startup and calibration

- Physically position the turret at the marked **+20-degree** position before
  starting or restarting robot code. The relative encoder cannot discover its
  absolute position at startup.
- Startup assigns +20 degrees to the encoder and resets the motion profile there;
  calibration does not command movement.
- Holding driver **Start for two seconds** recalibrates yaw at +20 degrees. Only
  use this with the turret stationary at that physical mark. This replaces the
  previous intake-facing zero action.
- If setting the encoder fails, yaw motor output remains disabled until a
  successful calibration.

## Travel protection

All position targets use the same heading conversion in auto aim, direct angle
commands, and simulation. Forbidden targets select the nearest endpoint by angular
distance (ties select +20). For example, -90 becomes +270, +300 selects +270, and
+350 selects +20. The controller uses non-wrapping scalar rotations, so a +270 to
+20 move travels through the permitted interval rather than across zero.

Automatic output and manual voltage are checked every cycle against encoder
position and the physical limit input. At either endpoint, motion farther outward
is blocked while inward motion is permitted. Spark forward/reverse encoder soft
limits are also enabled. All yaw output is capped after feedforward is added.

## Automatic homing

Manipulator **Back** stops yaw and reports that automatic homing is disabled.
The former routine assumed a +20.5-degree switch and then commanded 0 degrees;
neither assumption is valid for this configuration. Before restoring homing,
measure the actual switch position and direction with the shorter chain and
define a calibration routine that respects the permitted interval. Use physical
+20 positioning and driver Start calibration in the meantime.

## Robot verification before normal operation

1. Start at the +20 mark; verify encoder telemetry reports approximately 0.055556
   rotations and reported yaw is +20 degrees.
2. At low jog voltage, verify positive output increases encoder yaw and negative
   output decreases it. Verify the physical angle convention matches this sign.
3. Verify outward manual/automatic output is blocked at both endpoints and inward
   motion is allowed. Check physical switch handling and Spark soft limits too.
4. Check +20 to +270 and +270 to +20 moves, including travel through +180.
5. Request headings 0, 10, 20, 200, 270, 300, 350, and -90; verify their endpoint
   handling and that auto aim does not shoot toward an unreachable target.
6. Check driver Start calibration, the disabled homing notification, and a restart
   at the physical +20 mark. Reposition before restarting if needed.

These are software travel endpoints. If they coincide with mechanical collision
points, measure stopping distance and adjust the limits inward before normal-speed
operation; software output cutoffs do not remove momentum or encoder error.
