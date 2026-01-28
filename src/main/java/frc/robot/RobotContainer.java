package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.constants.jr.DriveConstants;
import frc.robot.constants.sim.VisionConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Vision vis;
  private Flywheel flywheel;
  private Turret turret;
  // private Elevator elevator;

  public SimContainer sim;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:

        // Real robot, instantiate hardware IO implementations
        if (DriveConstants.isReefscape) {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSparkFlexCancoder(0),
                  new ModuleIOSparkFlexCancoder(1),
                  new ModuleIOSparkFlexCancoder(2),
                  new ModuleIOSparkFlexCancoder(3));
        } else {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSparkMaxCancoder(0),
                  new ModuleIOSparkMaxCancoder(1),
                  new ModuleIOSparkMaxCancoder(2),
                  new ModuleIOSparkMaxCancoder(3));
        }

        vis =
            new Vision(
                drive::addVisionMeasurement,
                Stream.of(VisionConstants.cameraConfigs)
                    .map(VisionIOPhotonVision::new)
                    .toArray(VisionIOPhotonVision[]::new));

        flywheel = null;
        break;

      case SIM:
        sim = new SimContainer();

        SwerveDriveSimulation driveSim = sim.getDriveSim();
        SwerveModuleSimulation[] mods = driveSim.getModules();

        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSim.getGyroSimulation()),
                new ModuleIOSim(mods[0]),
                new ModuleIOSim(mods[1]),
                new ModuleIOSim(mods[2]),
                new ModuleIOSim(mods[3]));
        vis =
            new Vision(
                drive::addVisionMeasurement,
                Stream.of(VisionConstants.cameraConfigs)
                    .map(
                        cam ->
                            new VisionIOPhotonVisionSim(cam, driveSim::getSimulatedDriveTrainPose))
                    .toArray(VisionIOPhotonVision[]::new));

        TurretIOSim turretIo =
            new TurretIOSim(
                driveSim::getSimulatedDriveTrainPose,
                driveSim::getDriveTrainSimulatedChassisSpeedsFieldRelative);
        sim.registerSimulator(turretIo);

        turret = new Turret(turretIo, drive::getPose);

        flywheel =
            new Flywheel(
                new FlywheelIOSim(turretIo::setFlywheelSpeed, turretIo::setFlywheelShooting));

        // ElevatorIOSim elevatorSim = new ElevatorIOSim();
        // simContainer.registerSimulator(elevatorSim);
        // elevator = new Elevator(elevatorSim);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vis = new Vision(drive::addVisionMeasurement);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("Nothing", Commands.none());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // TODO: extract this into a constant
    Transform2d robotScoreOffsetRight = new Transform2d(0, 0.1, Rotation2d.fromDegrees(0));
    Transform2d robotScoreOffsetLeft = new Transform2d(0, -0.1, Rotation2d.fromDegrees(0));

    Pose2d reefEscape = DriveToPose.tagPose(11, new Transform2d(1, 0, Rotation2d.kCW_90deg));
    Pose2d playerStation =
        DriveToPose.tagPose(1, new Transform2d(Translation2d.kZero, Rotation2d.k180deg));

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            reefEscape, new Pose2d(playerStation.getTranslation(), Rotation2d.kZero));

    // PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The
    // constraints for this path.
    PathConstraints constraints =
        PathConstraints.unlimitedConstraints(
            12.0); // You can also use unlimited constraints, only limited by motor torque and
    // nominal battery voltage

    // Create the path using the waypoints created above
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null,
            // The ideal starting state, this is only relevant for pre-planned paths, so can be null
            // for on-the-fly paths.
            new GoalEndState(0.0, playerStation.getRotation())
            // Goal end state. You can set a holonomic rotation here. If using a differential
            // drivetrain, the rotation will have no effect.
            );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;
    autoChooser.addOption(
        "11L to CS to 6R",
        new SequentialCommandGroup(
            new DriveToPose(drive, DriveToPose.tagPose(11, robotScoreOffsetLeft)),
            AutoBuilder.followPath(path),
            Commands.waitSeconds(3),
            new DriveToPose(drive, DriveToPose.tagPose(6, robotScoreOffsetRight))));

    Pose2d[] testPoses =
        new Pose2d[] {
          new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(90)),
          new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(0)),
          new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90)),
        };
    List<Waypoint> pptestWaypoints1 = PathPlannerPath.waypointsFromPoses(testPoses);

    // Create the path using the waypoints created above
    PathPlannerPath pptestpath1 =
        new PathPlannerPath(
            pptestWaypoints1,
            constraints,
            null,
            // The ideal starting state, this is only relevant for pre-planned paths, so can be null
            // for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(90))
            // Goal end state. You can set a holonomic rotation here. If using a differential
            // drivetrain, the rotation will have no effect.
            );

    pptestpath1.preventFlipping = true;
    Collections.reverse(Arrays.asList(testPoses));
    List<Waypoint> pptestWaypoints2 = PathPlannerPath.waypointsFromPoses(testPoses);

    // Create the path using the waypoints created above
    PathPlannerPath pptestpath2 =
        new PathPlannerPath(
            pptestWaypoints2,
            constraints,
            null,
            // The ideal starting state, this is only relevant for pre-planned paths, so can be null
            // for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(0))
            // Goal end state. You can set a holonomic rotation here. If using a differential
            // drivetrain, the rotation will have no effect.
            );
    pptestpath2.preventFlipping = true;
    autoChooser.addOption(
        "PP Path Test",
        Commands.sequence(
            new DriveToPose(drive, testPoses[0]),
            Commands.repeatingSequence(
                AutoBuilder.followPath(pptestpath1),
                AutoBuilder.followPath(pptestpath2),
                Commands.waitSeconds(4))));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                Rotation2d::new));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    if (flywheel != null && turret != null) {
      controller.leftTrigger().whileTrue(flywheel.shootCommand());

      turret.setDefaultCommand(
          turret.aimAtCommand(
              flywheel::getShotSpeed,
              new Pose3d(new Translation3d(11.9, 4.1, 1.5), Rotation3d.kZero)));

      controller
          .povUp()
          .whileTrue(new RepeatCommand(turret.addPitchCommand(Rotation2d.fromDegrees(5))));
      controller
          .povDown()
          .whileTrue(new RepeatCommand(turret.addPitchCommand(Rotation2d.fromDegrees(-5))));
      controller
          .povLeft()
          .whileTrue(new RepeatCommand(turret.addYawCommand(Rotation2d.fromDegrees(-5))));
      controller
          .povRight()
          .whileTrue(new RepeatCommand(turret.addYawCommand(Rotation2d.fromDegrees(5))));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void simulationInit() {
    sim.simulationInit(drive::setPose);
  }

  public void simulationPeriodic() {
    sim.simulationPeriodic();
  }
}
