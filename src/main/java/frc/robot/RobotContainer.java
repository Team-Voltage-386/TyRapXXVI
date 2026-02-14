package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveDistance2;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.HubActivity;
import frc.robot.commands.RotateToAngle;
import frc.robot.constants.jr.DriveConstants;
import frc.robot.constants.jr.VisionConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.turret.ShotCalculation;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.TuningUtil;
import java.io.IOException;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // LED and Rumble
  private final LightSubsystem m_lightSubsystem = new LightSubsystem();

  // Subsystems
  private final Drive drive;
  private final Vision vis;
  private Flywheel flywheel;
  private Turret turret;
  private final IntakeSubsystem intake;
  private final ClimbSubsystem climb;
  private final SpindexerSubsystem spindexer;
  private ShotCalculation shotCalculation;

  TuningUtil runVolts = new TuningUtil("/Tuning/Turret/TestRunVolts", 0.5);
  TuningUtil setRPM = new TuningUtil("/Tuning/Flywheel/TestSetRPM", 100);
  TuningUtil setDegrees = new TuningUtil("/Tuning/Turret/TestSetDegrees", 100);

  public SimContainer sim;

  // Controller
  private final CommandXboxController kDriveController = new CommandXboxController(0);
  private final CommandXboxController kManipController = new CommandXboxController(1);

  // LEDs and Rumble
  private final HubActivity hubActivity = new HubActivity(m_lightSubsystem, kDriveController);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    m_lightSubsystem.setToColor(5, 200, 0, 0);
    m_lightSubsystem.setToColor(6, 0, 0, 200);
    m_lightSubsystem.setToColor(9, 0, 200, 0);

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

        flywheel = new Flywheel(new FlywheelIOSparkMax());

        shotCalculation = new ShotCalculation(drive);
        turret = new Turret(new TurretIOSparkMax(), drive::getPose, flywheel, shotCalculation);

        vis =
            new Vision(
                drive::addVisionMeasurement,
                Stream.of(VisionConstants.cameraConfigs)
                    .map(VisionIOPhotonVision::new)
                    .toArray(VisionIOPhotonVision[]::new));

        intake = new IntakeSubsystem(new IntakeIOSparkMax());
        climb = new ClimbSubsystem();
        spindexer = new SpindexerSubsystem();
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
        // disable vision simulation for performance reasons
        vis = new Vision(drive::addVisionMeasurement);
        spindexer = new SpindexerSubsystem();

        flywheel = new Flywheel(new FlywheelIOSim());
        IntakeIOSim intakeIOSim = new IntakeIOSim(driveSim);

        TurretIOSim turretIo =
            new TurretIOSim(
                driveSim::getSimulatedDriveTrainPose,
                driveSim::getDriveTrainSimulatedChassisSpeedsFieldRelative,
                spindexer,
                flywheel,
                intakeIOSim);
        sim.registerSimulator(turretIo);
        shotCalculation = new ShotCalculation(drive);
        turret = new Turret(turretIo, drive::getPose, flywheel, shotCalculation);

        intake = new IntakeSubsystem(intakeIOSim);
        climb = new ClimbSubsystem();

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
        intake = new IntakeSubsystem(null);
        climb = new ClimbSubsystem();
        spindexer = new SpindexerSubsystem();
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
    autoChooser.addOption("Drive square", DriveCommands.driveSquare(drive));
    autoChooser.addOption("SquareStraight", DriveCommands.SquareStraight(drive));
    autoChooser.addOption("SmallSquare", DriveCommands.SmallSquare(drive));
    autoChooser.addOption("BlueSquare", DriveCommands.BlueSquare(drive));
    autoChooser.addOption("LeftNeutralZone", buildLeftNeutralZoneAuto());
    autoChooser.addOption("LeftNeutralDepot", buildNeutralCollectDepot());

    // TODO: extract this into a constant
    Transform2d robotScoreOffsetRight = new Transform2d(0, 0.1, Rotation2d.fromDegrees(0));
    Transform2d robotScoreOffsetLeft = new Transform2d(0, -0.1, Rotation2d.fromDegrees(0));

    Pose2d reefEscape = DriveToPose.tagPose(11, new Transform2d(1, 0, Rotation2d.kCW_90deg));
    Pose2d playerStation =
        DriveToPose.tagPose(1, new Transform2d(Translation2d.kZero, Rotation2d.k180deg));

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            reefEscape, new Pose2d(playerStation.getTranslation(), Rotation2d.kZero));

    // PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 *
    // Math.PI); // The
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
            // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null
            // for on-the-fly paths.
            new GoalEndState(0.0, playerStation.getRotation())
            // Goal end state. You can set a holonomic rotation here. If using a
            // differential
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
            // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null
            // for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(90))
            // Goal end state. You can set a holonomic rotation here. If using a
            // differential
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
            // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null
            // for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(0))
            // Goal end state. You can set a holonomic rotation here. If using a
            // differential
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
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -kDriveController.getLeftY(),
            () -> -kDriveController.getLeftX(),
            () -> -kDriveController.getRightX()));

    // Lock to nearest 45° when A button is held
    kDriveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -kDriveController.getLeftY(),
                () -> -kDriveController.getLeftX(),
                () -> getAngleForRamp()));

    // Switch to X pattern when X button is pressed
    kDriveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    kDriveController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    if (turret != null) {
      System.out.println("running at " + runVolts.getValue());
      kDriveController
          .povRight()
          .whileTrue(
              new FunctionalCommand(
                  () -> {},
                  () -> {
                    System.out.println("running at " + runVolts.getValue());
                    ((TurretIOSparkMax) turret.io).testHoodVoltage(runVolts.getValue());
                  },
                  (c) -> {
                    ((TurretIOSparkMax) turret.io).testHoodVoltage(0);
                  },
                  () -> false,
                  flywheel));

      kDriveController
          .povLeft()
          .whileTrue(
              new FunctionalCommand(
                  () -> {},
                  () -> {
                    System.out.println("running at " + -runVolts.getValue());
                    ((TurretIOSparkMax) turret.io).testHoodVoltage(-runVolts.getValue());
                  },
                  (c) -> {
                    ((TurretIOSparkMax) turret.io).testHoodVoltage(0);
                  },
                  () -> false,
                  flywheel));

      kDriveController
          .povUp()
          .onTrue(turret.runOnce(() -> turret.io.setTurretYaw(Rotation2d.k180deg)));

      kDriveController
          .rightTrigger()
          .whileTrue(
              turret
                  .aimAtCommand(() -> getHubPose3d())
                  .alongWith(spindexer.feederOnCommand())
                  .alongWith(spindexer.spindexerOnCommand()));

      kDriveController
          .rightTrigger()
          .onFalse(
              new InstantCommand(() -> flywheel.setFlywheelSpeed(0))
                  .alongWith(spindexer.feederOffCommand())
                  .alongWith(spindexer.spindexerOffCommand()));
      kDriveController.leftTrigger().whileTrue(turret.adjustPitch(() -> setDegrees.getValue()));

      kDriveController
          .start()
          .onTrue(turret.runOnce(() -> ((TurretIOSparkMax) turret.io).setHoodZero()));

      kDriveController
          .rightBumper()
          .onTrue(new InstantCommand(() -> pathfindToPath("AlignTowerFromBottom"), drive));

      kDriveController
          .leftBumper()
          .onTrue(new InstantCommand(() -> pathfindToPath("AlignTowerFromTop"), drive));

      kManipController
          .povRight()
          .whileTrue(
              new FunctionalCommand(
                  () -> {},
                  () -> {
                    System.out.println("running at " + runVolts.getValue());
                    ((FlywheelIOSparkMax) flywheel.io).testFlywheelVoltage(runVolts.getValue());
                  },
                  (c) -> {
                    ((FlywheelIOSparkMax) flywheel.io).testFlywheelVoltage(0);
                  },
                  () -> false,
                  flywheel));

      kManipController
          .povLeft()
          .whileTrue(
              new FunctionalCommand(
                  () -> {},
                  () -> {
                    System.out.println("running at " + runVolts.getValue());
                    ((FlywheelIOSparkMax) flywheel.io).testFlywheelVoltage(-runVolts.getValue());
                  },
                  (c) -> {
                    ((FlywheelIOSparkMax) flywheel.io).testFlywheelVoltage(0);
                  },
                  () -> false,
                  flywheel));
      kManipController.rightBumper().whileTrue(flywheel.shootCommand(() -> setRPM.getValue()));
      // Manipulator controller bindings
      kManipController.a().onTrue(intake.deployCommand());
      kManipController.b().onTrue(intake.retractCommand());
      kManipController.x().onTrue(intake.takeInCommand());
      kManipController.y().onTrue(intake.stopMotorCommand());

      kManipController.rightTrigger().onTrue(spindexer.spindexerOnCommand());
      kManipController.rightTrigger().onFalse(spindexer.spindexerOffCommand());
      kManipController.leftTrigger().onTrue(spindexer.feederOnCommand());
      kManipController.leftTrigger().onFalse(spindexer.feederOffCommand());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String selectedAutoName = autoChooser.getSendableChooser().getSelected();
    if (selectedAutoName == null) {
      return Commands.none();
    }
    switch (selectedAutoName) {
      case "Drive square":
        setPoseFromPathStart("Square");
        break;
      case "SquareStraight":
        setPoseFromPathStart("SquareStraight");
        break;
      case "SmallSquare":
        setPoseFromPathStart("SmallSquare");
        break;
      case "BlueSquare":
        setPoseFromPathStart("BlueSquare");
        break;
      case "LeftNeutralZone":
        setPoseFromPathStart("StartCollectNeutralTopQtr");
        break;
      case "LeftNeutralDepot":
        setPoseFromPathStart("CollectNeutralTopToDepot");
        break;
    }
    return autoChooser.get();
  }

  public void simulationInit() {
    sim.simulationInit(drive::setPose);
  }

  public void simulationPeriodic() {
    sim.simulationPeriodic(drive::addVisionMeasurement);
  }

  public Rotation2d getAngletoHub() {
    Rotation2d angleToHub = Rotation2d.fromDegrees(0);
    Pose2d robotPosition = drive.getPose();
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    if (currentAlliance.isPresent()) {
      angleToHub =
          switch (currentAlliance.get()) {
            case Red -> {
              yield Constants.redHubPose
                  .toTranslation2d()
                  .minus(robotPosition.getTranslation())
                  .getAngle();
            }
            case Blue -> {
              yield Constants.blueHubPose
                  .toTranslation2d()
                  .minus(robotPosition.getTranslation())
                  .getAngle();
            }
          };
    }
    Logger.recordOutput("/Drive/AngleToHub", angleToHub.getDegrees());
    return angleToHub;
  }

  public Translation3d getHubPose() {
    Translation3d hubPose = new Translation3d();
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    if (currentAlliance.isPresent()) {
      switch (currentAlliance.get()) {
        case Red:
          hubPose = Constants.redHubPose;
          break;
        case Blue:
          hubPose = Constants.blueHubPose;
          break;
        default:
          hubPose = Constants.blueHubPose;
          break;
      }
      ;
    }
    return hubPose;
  }

  public Rotation2d getLadderAngle() {
    Rotation2d ladderAngle = new Rotation2d();
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    if (currentAlliance.isPresent()) {
      switch (currentAlliance.get()) {
        case Red:
          ladderAngle = Rotation2d.fromDegrees(180);
          break;
        case Blue:
          ladderAngle = Rotation2d.fromDegrees(0);
          break;
        default:
          ladderAngle = Rotation2d.fromDegrees(0);
          break;
      }
      ;
    }
    return ladderAngle;
  }

  public Pose3d getHubPose3d() {
    return new Pose3d(getHubPose(), Rotation3d.kZero);
  }

  public Pose3d getPose3d(Translation3d pose) {
    return new Pose3d(pose, Rotation3d.kZero);
  }

  public void pathfindToPosition(double xPosition, double yPosition) {
    // Since we are using a holonomic drivetrain, the rotation component of this
    // pose
    // represents the goal holonomic rotation
    Pose2d targetPose = new Pose2d(xPosition, yPosition, Rotation2d.fromDegrees(180));

    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand =
        AutoBuilder.pathfindToPose(
            targetPose, constraints, 0.0); // , // Goal end velocity in meters/sec
    // 0.0 // Rotation delay distance in meters. This is how far the robot should
    // travel before
    // attempting to rotate.
    CommandScheduler.getInstance().schedule(pathfindingCommand);
  }

  public void pathfindToPath(String pathName) {
    // Load the path we want to pathfind to and follow
    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile(pathName);
    } catch (FileVersionException e) {
      System.err.println("Path is wrong file version!");
      e.printStackTrace();
      return;
    } catch (IOException e) {
      System.err.println("Path file not found!");
      e.printStackTrace();
      return;
    } catch (ParseException e) {
      System.err.println("Path file can't be parsed!");
      e.printStackTrace();
      return;
    }

    // Create the constraints to use while pathfinding. The constraints defined in
    // the path will
    // only be used for the path.
    PathConstraints constraints =
        new PathConstraints(2.5, 2.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
    CommandScheduler.getInstance().schedule(pathfindingCommand);
  }

  public boolean areLightsOn() {
    return m_lightSubsystem.areLightsOn();
  }

  public boolean isHubActive() {
    return hubActivity.hubIsActive();
  }

  public HubActivity getHubActivityCommand() {
    return hubActivity;
  }

  public void setIsAheadHub(boolean setTo) {
    getHubActivityCommand().setIsAhead(setTo);
  }

  public Rotation2d getAngleForRamp() {
    double degrees = drive.getPose().getRotation().getDegrees();
    double rot45 = Math.ceil(degrees / 90) * 90 - 45;
    Rotation2d rotation = new Rotation2d(rot45 * Math.PI / 180); // Rotation degrees to radians
    return rotation;
  }

  protected void setPoseFromPathStart(String pathName) {
    try {
      System.out.println("Setting pose from path start: " + pathName);
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

      List<Waypoint> waypoints;
      Optional<Pose2d> pose = path.getStartingHolonomicPose();
      Optional<Alliance> ally = DriverStation.getAlliance();
      waypoints = path.getWaypoints();
      Waypoint first = waypoints.get(0);
      Rotation2d startRotation = Rotation2d.kZero;
      if (pose.isPresent()) {
        startRotation = pose.get().getRotation();
      }
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          System.out.println("Flipping start location for red");
          first = first.flip();
          if (pose.isPresent()) {
            startRotation = pose.get().getRotation().plus(Rotation2d.fromDegrees(180));
          }
        }
      }
      if (pose.isPresent()) {
        drive.setPose(new Pose2d(first.anchor(), startRotation));
        System.out.println(first.toString());
        System.out.println("First.anchor(): " + first.anchor().toString());
        System.out.println("startRotation: " + startRotation.toString());
      } else {
        System.out.println("Error getting PathPlanner pose");
      }
      if (Robot.isSimulation()) {
        if (sim != null) {
          System.out.println("Setting sim pose to " + drive.getPose());
          sim.getDriveSim().setSimulationWorldPose(drive.getPose());
        }
      }
    } catch (IOException e) {
      System.err.println("Path file not found!");
      e.printStackTrace();
    } catch (ParseException e) {
      System.err.println("Path file can't be parsed!");
      e.printStackTrace();
    }
  }

  public Command buildLeftNeutralZoneAuto() {
    Command auto =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                turret.enableAutoAimCommand(() -> getHubPose3d()), intake.deployCommand()),
            DriveCommands.buildFollowPath("StartCollectNeutralTopQtr"),
            spindexer.spindexerOnCommand().alongWith(spindexer.feederOnCommand()),
            new WaitCommand(3),
            spindexer.spindexerOffCommand().alongWith(spindexer.feederOffCommand()),
            DriveCommands.buildFollowPath("CollectDepot"),
            spindexer.spindexerOnCommand().alongWith(spindexer.feederOnCommand()),
            new WaitCommand(3),
            spindexer.spindexerOffCommand().alongWith(spindexer.feederOffCommand()),
            turret
                .disableAutoAimCommand()
                .alongWith(intake.retractCommand(), climb.deployCommand()),
            DriveCommands.buildFollowPath("AlignTowerFromDepot"),
            new RotateToAngle(drive, () -> Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(2)),
            climb.retractCommand());
    return auto;
  }

  public Command buildNeutralCollectDepot() {
    Command auto =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                turret.enableAutoAimCommand(() -> getHubPose3d()), intake.deployCommand()),
            DriveCommands.buildFollowPath("CollectNeutralTopToDepot"),
            spindexer.spindexerOnCommand().alongWith(spindexer.feederOnCommand()),
            DriveCommands.buildFollowPath("DepotSlowCollect"),
            new WaitCommand(5),
            spindexer.spindexerOffCommand().alongWith(spindexer.feederOffCommand()),
            turret
                .disableAutoAimCommand()
                .alongWith(intake.retractCommand(), climb.deployCommand()),
            DriveCommands.buildFollowPath("AlignTowerFromDepot"),
            new RotateToAngle(drive, () -> getLadderAngle(), Rotation2d.fromDegrees(2)),
            new DriveDistance2(drive, () -> 0.3, -90),
            climb.retractCommand());
    return auto;
  }
}
