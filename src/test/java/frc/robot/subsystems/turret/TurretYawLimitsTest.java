package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.jr.TurretConstants;
import org.junit.jupiter.api.Test;

class TurretYawLimitsTest {
  private static final double EPSILON = 1e-9;

  @Test
  void startupIsAtTheIncludedLowerEndpoint() {
    assertEquals(20.0, TurretConstants.turretStartupAngleDeg, EPSILON);
    assertEquals(20.0 / 360.0, TurretConstants.turretMinAngleRot, EPSILON);
    assertEquals(270.0 / 360.0, TurretConstants.turretMaxAngleRot, EPSILON);
    assertEquals(20.0, TurretYawLimits.limitDegrees(20.0), EPSILON);
    assertEquals(270.0, TurretYawLimits.limitDegrees(270.0), EPSILON);
  }

  @Test
  void headingsInsideTheTravelRangeAreUnchanged() {
    for (double angle : new double[] {20.1, 45.0, 90.0, 180.0, 200.0, 269.9}) {
      assertEquals(angle, TurretYawLimits.limitDegrees(angle), EPSILON);
    }
  }

  @Test
  void forbiddenHeadingsChooseTheNearestEndOfTheArcAcrossZero() {
    for (double angle : new double[] {0.0, 10.0, 19.9, 325.0, 350.0, 360.0}) {
      assertEquals(20.0, TurretYawLimits.limitDegrees(angle), EPSILON);
    }
    for (double angle : new double[] {270.1, 280.0, 300.0, 324.9}) {
      assertEquals(270.0, TurretYawLimits.limitDegrees(angle), EPSILON);
    }
  }

  @Test
  void negativeAndMultiTurnHeadingsResolveToTheSamePhysicalHeading() {
    assertEquals(270.0, TurretYawLimits.limitDegrees(-90.0), EPSILON);
    assertEquals(200.0, TurretYawLimits.limitDegrees(-160.0), EPSILON);
    assertEquals(20.0, TurretYawLimits.limitDegrees(-10.0), EPSILON);
    assertEquals(270.0, TurretYawLimits.limitDegrees(-1080.0 - 90.0), EPSILON);
    assertEquals(90.0, TurretYawLimits.limitDegrees(1080.0 + 90.0), EPSILON);
    // Rotation2d arithmetic can supply the equivalent signed heading to the IO layer.
    assertEquals(
        270.0,
        TurretYawLimits.limitDegrees(
            Rotation2d.fromDegrees(180).plus(Rotation2d.fromDegrees(90)).getDegrees()),
        EPSILON);
  }

  @Test
  void lowerBoundaryBlocksOnlyMotionFartherDown() {
    for (double angle : new double[] {0.0, 19.0, 20.0}) {
      assertEquals(0.0, voltage(angle, -2.5, false, false), EPSILON);
      assertEquals(2.5, voltage(angle, 2.5, false, false), EPSILON);
    }
  }

  @Test
  void upperBoundaryBlocksOnlyMotionFartherUp() {
    for (double angle : new double[] {270.0, 271.0, 360.0}) {
      assertEquals(0.0, voltage(angle, 2.5, false, false), EPSILON);
      assertEquals(-2.5, voltage(angle, -2.5, false, false), EPSILON);
    }
  }

  @Test
  void physicalSwitchesBlockOutputEvenBeforeSoftwareEndpoints() {
    assertEquals(0.0, voltage(25.0, -2.5, true, false), EPSILON);
    assertEquals(2.5, voltage(25.0, 2.5, true, false), EPSILON);
    assertEquals(0.0, voltage(265.0, 2.5, false, true), EPSILON);
    assertEquals(-2.5, voltage(265.0, -2.5, false, true), EPSILON);
  }

  @Test
  void allOutputIncludingFeedforwardAndManualVoltageIsCapped() {
    assertEquals(TurretConstants.maxYawVoltage, voltage(90, 12.0, false, false), EPSILON);
    assertEquals(-TurretConstants.maxYawVoltage, voltage(90, -12.0, false, false), EPSILON);
    assertEquals(0.0, voltage(20, 0.0, true, false), EPSILON);
  }

  @Test
  void invalidMeasurementsAndVoltagesStopTheMotor() {
    assertEquals(0.0, voltage(Double.NaN, 2.5, false, false), EPSILON);
    assertEquals(0.0, voltage(90.0, Double.NaN, false, false), EPSILON);
    assertEquals(0.0, voltage(Double.POSITIVE_INFINITY, 2.5, false, false), EPSILON);
    assertEquals(0.0, voltage(90.0, Double.NEGATIVE_INFINITY, false, false), EPSILON);
  }

  @Test
  void profileReturnsFrom270To20ThroughTheAllowedArc() {
    var controller =
        new ProfiledPIDController(
            TurretConstants.turretYawKp,
            0.0,
            TurretConstants.turretYawKd,
            new TrapezoidProfile.Constraints(
                TurretConstants.turretMaxRotationSpeedRotPerSec,
                TurretConstants.turretMaxAccelRotPerSec2));
    double current = TurretConstants.turretMaxAngleRot;
    double target = TurretYawLimits.limitDegrees(20.0) / 360.0;
    controller.reset(current);
    for (int tick = 0; tick < 200; tick++) {
      controller.calculate(current, target);
      double next = controller.getSetpoint().position;
      assertTrue(next <= current + EPSILON);
      assertTrue(next >= TurretConstants.turretMinAngleRot - EPSILON);
      current = next;
    }
    assertEquals(target, current, EPSILON);
  }

  private static double voltage(double degrees, double volts, boolean lower, boolean upper) {
    return TurretYawLimits.limitVoltage(degrees / 360.0, volts, lower, upper);
  }
}
