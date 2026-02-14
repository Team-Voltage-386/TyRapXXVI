// Imports
package frc.robot.util;

import edu.wpi.first.math.MathUtil;

// This Command calculates velocities for limelight for one value
public class TrapezoidController {

  // Create variables
  private double threshold;
  private double desiredVel;
  private double minVel;
  private double maxVel;
  private double maxAccPerStep;
  private double maxDccPerStep;
  private double commandedVel;
  private double decelKp;

  // Constructor
  public TrapezoidController(
      double startVel,
      double threshold,
      double minVel,
      double maxVel,
      double maxAcc,
      double maxDcc,
      double decelKp) {
    this.commandedVel = startVel; // Initialize to current speed
    this.threshold = threshold;
    this.minVel = minVel;
    this.maxVel = maxVel;
    this.decelKp = decelKp;
    this.maxAccPerStep = maxAcc / (1 / 0.02);
    this.maxDccPerStep = maxDcc / (1 / 0.02);
  }

  public void setMaxVel(double maxVel) {
    this.maxVel = maxVel;
  }

  public void setMaxAccel(double maxAccel) {
    this.maxAccPerStep = maxAccel / (1 / 0.02);
  }

  public void setDecelKp(double decelProp) {
    this.decelKp = decelProp;
  }

  // Start Calculations
  public void reinit(double currentSpeed) {
    this.commandedVel = MathUtil.clamp(currentSpeed, -this.maxVel, this.maxVel);
  }

  public double calculate(double currentDistance, double currentVel) {
    // currentVel input not used at this time
    desiredVel = currentDistance * decelKp;

    // Desired Velocities
    // If the error is less than the threshold, meaning we have driven to the correct place,
    //    stop the robot by setting speed to 0
    if (Math.abs(currentDistance) < threshold) {
      desiredVel = 0;
      // If not, meaning we still have to drive further, set the calculated velocity
      //    to be between a min and max, while using the correct sign (+ or -)
    } else {
      desiredVel = Math.copySign(MathUtil.clamp(Math.abs(desiredVel), minVel, maxVel), desiredVel);
    }

    // Commanded Velocity ramped
    // If the desired velocity is greater than the commanded velocity (in the same direction), we
    // accelerate
    double deltaVel = 0.0;
    if ((Math.abs(desiredVel) - Math.abs(commandedVel)) > 0) {
      deltaVel = maxAccPerStep;
      // If not, we decelerate
      // We have to switch because there are different values for both
    } else {
      deltaVel = maxDccPerStep;
    }
    // Set commanded velocity based on acceleration or deceleration
    // Acceleration
    if (desiredVel > commandedVel) {
      commandedVel = Math.min(desiredVel, commandedVel + deltaVel);
      // Deceleration
    } else if (desiredVel < commandedVel) {
      commandedVel = Math.max(desiredVel, commandedVel - deltaVel);
    }

    // Return final velocity
    return commandedVel;
  }
}
