package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
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
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HubActivity;
import frc.robot.constants.jr.DriveConstants;
import frc.robot.constants.jr.IntakeConstants;
import frc.robot.constants.jr.VisionConstants;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkFlex;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.turret.ShotCalculation;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOSparkMax2;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIONull;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.AutoWrapper;
import frc.robot.util.TuningUtil;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
  private final SpindexerSubsystem spindexer;
  private ShotCalculation shotCalculation;

  TuningUtil runVolts = new TuningUtil("/Tuning/Turret/TestRunVolts", 1.8);
  TuningUtil setDegrees = new TuningUtil("/Tuning/Turret/TestSetDegrees", 0.0);

  public SimContainer sim;

  // Controller
  private final CommandXboxController kDriveController = new CommandXboxController(0);
  private final CommandXboxController kManipController = new CommandXboxController(1);

  // LEDs and Rumble
  private final HubActivity hubActivity = new HubActivity(m_lightSubsystem, kDriveController);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Autos
  private final Map<String, AutoWrapper> autos = new HashMap<>();

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
        if (!DriveConstants.isReefscape) {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSparkFlexCancoder(0, true, false),
                  new ModuleIOSparkFlexCancoder(1, true, false),
                  new ModuleIOSparkFlexCancoder(2, true, false),
                  new ModuleIOSparkFlexCancoder(3, true, false));
        } else {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSparkMaxCancoder(0),
                  new ModuleIOSparkMaxCancoder(1),
                  new ModuleIOSparkMaxCancoder(2),
                  new ModuleIOSparkMaxCancoder(3));
        }

        flywheel = new Flywheel(new FlywheelIOSparkFlex());

        shotCalculation = new ShotCalculation(drive);
        spindexer = new SpindexerSubsystem();
        turret =
            new Turret(
                new TurretIOSparkMax2(),
                drive::getPose,
                flywheel,
                shotCalculation,
                spindexer::isFeederOn,
                this::isInAllianceArea,
                this::verticalHalfOfField,
                this::getShotTriggerValue);

        vis =
            new Vision(
                drive::addVisionMeasurement,
                Stream.of(VisionConstants.cameraConfigs)
                    .map(VisionIOPhotonVision::new)
                    .toArray(VisionIOPhotonVision[]::new));

        intake = new IntakeSubsystem(new IntakeIOSparkMax());

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
        vis =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIONull(
                    VisionConstants.cameraConfigs[0], driveSim::getSimulatedDriveTrainPose));
        ;
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
        turret =
            new Turret(
                turretIo,
                drive::getPose,
                flywheel,
                shotCalculation,
                spindexer::isFeederOn,
                this::isInAllianceArea,
                this::verticalHalfOfField,
                this::getShotTriggerValue);

        intake = new IntakeSubsystem(intakeIOSim);

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
        spindexer = new SpindexerSubsystem();
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("Nothing", Commands.none());

    // Configure the button bindings
    configureButtonBindings();

    initializeAutos();
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
      /*kDriveController
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
                  turret));

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
                  turret)); */

      kManipController
          .povUp()
          .whileTrue(
              new FunctionalCommand(
                  () -> {},
                  () -> {
                    System.out.println("running at " + IntakeConstants.DEPLOY_MANUAL_VOLTAGE);
                    intake.testDeployVoltage(IntakeConstants.DEPLOY_MANUAL_VOLTAGE);
                  },
                  (c) -> {
                    intake.testDeployVoltage(0);
                  },
                  () -> false,
                  intake));

      kManipController
          .povDown()
          .whileTrue(
              new FunctionalCommand(
                  () -> {},
                  () -> {
                    System.out.println("running at " + -IntakeConstants.DEPLOY_MANUAL_VOLTAGE);
                    intake.testDeployVoltage(-IntakeConstants.DEPLOY_MANUAL_VOLTAGE);
                  },
                  (c) -> {
                    intake.testDeployVoltage(0);
                  },
                  () -> false,
                  intake));

      kDriveController.povUp().onTrue(turret.manualIncrimentPitch(Rotation2d.fromDegrees(.5)));
      kDriveController.povDown().onTrue(turret.manualIncrimentPitch(Rotation2d.fromDegrees(-.5)));

      kDriveController.rightTrigger().onTrue(intake.takeInCommand());
      kDriveController.rightTrigger().onFalse(intake.stopMotorCommand());
      /*kDriveController
          .rightTrigger()
          .whileTrue(
              new ConditionalCommand(
                  turret.aimAtCommand(() -> getHubPose3d()),
                  flywheel.shootCommand(),
                  () -> turret.isAutoAimEnabled()));

      kDriveController
          .rightTrigger()
          .onFalse(
              new InstantCommand(() -> flywheel.setFlywheelSpeed(0))
                  .alongWith(
                      turret.runOnce(() -> turret.io.setTurretPitch(Rotation2d.fromDegrees(62))))
                  .alongWith(spindexer.spindexerOffCommand())
                  .andThen(new WaitCommand(.5))
                  .andThen(spindexer.feederOffCommand())); */
      kDriveController.leftTrigger().whileTrue(turret.adjustPitch(() -> setDegrees.getValue()));

      kDriveController
          .back()
          .onTrue(turret.runOnce(() -> ((TurretIOSparkMax2) turret.io).setHoodZero()));

      // kDriveController.start().onTrue(turret.toggleAutoAimCommand());
      kManipController
          .back()
          .onTrue(turret.runOnce(() -> ((TurretIOSparkMax2) turret.io).setYawZero()));
      kDriveController.rightBumper().onTrue(intake.reverseCommand());
      kDriveController.rightBumper().onFalse(intake.stopMotorCommand());

      kManipController
          .povLeft()
          .whileTrue(
              new FunctionalCommand(
                  () -> {},
                  () -> {
                    System.out.println("running at " + runVolts.getValue());
                    ((TurretIOSparkMax2) turret.io).testTurretVoltage(runVolts.getValue());
                  },
                  (c) -> {
                    ((TurretIOSparkMax2) turret.io).testTurretVoltage(0);
                  },
                  () -> false,
                  turret));
      kManipController
          .povRight()
          .whileTrue(
              new FunctionalCommand(
                  () -> {},
                  () -> {
                    System.out.println("running at " + -1 * runVolts.getValue());
                    ((TurretIOSparkMax2) turret.io).testTurretVoltage(-1 * runVolts.getValue());
                  },
                  (c) -> {
                    ((TurretIOSparkMax2) turret.io).testTurretVoltage(0);
                  },
                  () -> false,
                  turret));

      kManipController.rightBumper().onTrue(turret.adjustYaw(setDegrees::getValue));
      kManipController
          .start()
          .onTrue(turret.toggleAutoAimCommand()); // .alongWith(vis.toggleHubTags()));
      // Manipulator controller bindings
      kManipController.a().onTrue(spindexer.feederReverseCommand());
      kManipController
          .a()
          .onFalse(spindexer.feederOffCommand().alongWith(spindexer.spindexerOffCommand()));
      kManipController.b().onTrue(intake.retractCommand());
      kManipController.x().onTrue(intake.reverseCommand());
      kManipController.x().onFalse(intake.stopMotorCommand());
      kManipController.y().onTrue(intake.deployCommand());

      kManipController
          .rightTrigger()
          .onTrue(spindexer.spindexerOnCommand().andThen(spindexer.feederOnCommand()));
      kManipController
          .rightTrigger()
          .onFalse(
              spindexer
                  .spindexerOffCommand()
                  .andThen(new WaitCommand(0.25))
                  .andThen(spindexer.feederOffCommand()));
      kManipController.leftTrigger().onTrue(spindexer.feederOnCommand());
      kManipController.leftTrigger().onFalse(spindexer.feederOffCommand());
    }
  }

  public double getShotTriggerValue() {
    return kManipController.getHID().getRightTriggerAxis();
  }

  protected void registerAuto(AutoWrapper auto) {
    autos.put(auto.getName(), auto);
    autoChooser.addOption(auto.getName(), auto.getAutoCommand());
  }

  public void initializeAutos() {
    try {
      registerAuto(new AutoWrapper("SimpleLeftDepot", "Spacing", true, buildSimpleLeftAuto()));
      registerAuto(new AutoWrapper("BackUpAndShoot", "Spacing", true, buildSimpleRightAuto()));
      registerAuto(
          new AutoWrapper(
              "RightTwoCollect",
              "CollectNeutralBottomShoot",
              true,
              buildRightNeutralZoneTwoCollect()));
      registerAuto(
          new AutoWrapper(
              "LeftTwoCollect", "CollectNeutralTopShoot", true, buildLeftNeutralZoneTwoCollect()));
    } catch (FileVersionException | IOException | ParseException e) {
      System.err.println("Error loading auto: " + e.getMessage());
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
    if (!autos.containsKey(selectedAutoName)) {
      System.err.println("Selected auto not found: " + selectedAutoName);
      return Commands.none();
    }
    if (autos.get(selectedAutoName).isResetNav()) {
      setPoseFromPathStart(autos.get(selectedAutoName).getInitialPath());
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

  public Rotation2d getLeftLadderAngle() {
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

  // added Right
  public Rotation2d getRightLadderAngle() {
    Rotation2d ladderAngle = new Rotation2d();
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    if (currentAlliance.isPresent()) {
      switch (currentAlliance.get()) {
        case Red:
          ladderAngle = Rotation2d.fromDegrees(0);
          break;
        case Blue:
          ladderAngle = Rotation2d.fromDegrees(180);
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

  public Command pathfindToPosition(double xPosition, double yPosition) {
    // Since we are using a holonomic drivetrain, the rotation component of this
    // pose
    // represents the goal holonomic rotation
    Pose2d targetPose = new Pose2d(xPosition, yPosition, Rotation2d.fromDegrees(0));

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
    return pathfindingCommand;
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

  public boolean isInAllianceArea() {

    double currentXPos = drive.getPose().getX();

    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    if (alliance == DriverStation.Alliance.Blue) {
      return currentXPos < 4.625594;
    }
    return currentXPos > 11.915394;
  }

  // true = standing at blue looking toward red -> right
  //       standing at red looking toward blue -> left
  //       (between y of 0 and 4.034536 meters is false)
  public boolean verticalHalfOfField() {
    double currentYPos = drive.getPose().getY();
    if (currentYPos < 4.034536) {
      return false;
    }
    return true;
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

  protected void setPoseFromPathStart(PathPlannerPath path) {
    System.out.println("Setting pose from path start: " + path.name);

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
  }

  public void runTeleopStart() {
    // CommandScheduler.getInstance()
    //     .schedule(
    //         new SequentialCommandGroup(
    //              new WaitCommand(2), Commands.none()));
    // Ensure the intake is deployed. This is mainly for simulation testing
    // since the intake should normally be deployed at the start of auto
    // CommandScheduler.getInstance().schedule(intake.deployCommand());

    // Stop all motors when we enter teleop to clean up from auto
    CommandScheduler.getInstance()
        .schedule(
            intake
                .stopMotorCommand()
                .alongWith(spindexer.spindexerOffCommand())
                .alongWith(spindexer.feederOffCommand()));
  }

  public Command buildLeftNeutralZoneAuto() {
    Command auto =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                turret.enableAutoAimCommand(() -> getHubPose3d()), intake.deployCommand()),
            new WaitCommand(0.5),
            intake.takeInCommand(),
            DriveCommands.buildFollowPath("StartCollectNeutralTopQtr"),
            intake.stopMotorCommand(),
            spindexer.spindexerOnCommand().alongWith(spindexer.feederOnCommand()),
            new WaitCommand(4),
            spindexer.spindexerOffCommand().alongWith(spindexer.feederOffCommand()),
            intake.takeInCommand(),
            DriveCommands.buildFollowPath("CollectDepot"),
            spindexer.spindexerOnCommand().alongWith(spindexer.feederOnCommand()),
            intake.stopMotorCommand());
    return auto;
  }

  public Command buildSimpleLeftAuto() {
    Command auto =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                turret.enableAutoAimCommand(() -> getHubPose3d()), intake.deployCommand()),
            DriveCommands.buildFollowPath("Spacing"),
            new WaitCommand(.5),
            spindexer.spindexerOnCommand().alongWith(spindexer.feederOnCommand()),
            new WaitCommand(3),
            spindexer.spindexerOffCommand().alongWith(spindexer.feederOffCommand()),
            intake.takeInCommand(),
            DriveCommands.buildFollowPath("DepotFromCenter"),
            spindexer.spindexerOnCommand().alongWith(spindexer.feederOnCommand()),
            DriveCommands.buildFollowPath("DepotSlowCollect"),
            new WaitCommand(4),
            intake.stopMotorCommand());
    return auto;
  }

  public Command buildSimpleRightAuto() {
    Command auto =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                turret.enableAutoAimCommand(() -> getHubPose3d()), intake.deployCommand()),
            DriveCommands.buildFollowPath("Spacing"),
            new WaitCommand(1),
            spindexer.spindexerOnCommand().alongWith(spindexer.feederOnCommand()),
            new WaitCommand(5),
            spindexer.spindexerOffCommand().alongWith(spindexer.feederOffCommand()));
    return auto;
  }

  public Command buildRightNeutralZoneTwoCollect() {
    Command auto =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                turret.enableAutoAimCommand(() -> getHubPose3d()), intake.deployCommand()),
            new WaitCommand(0.5),
            intake.takeInCommand(),
            DriveCommands.buildFollowPath("CollectNeutralBottomShoot"),
            spindexer
                .spindexerOnCommand()
                .alongWith(spindexer.feederOnCommand())
                .alongWith(intake.stopMotorCommand()),
            new WaitCommand(6),
            spindexer
                .spindexerOffCommand()
                .alongWith(spindexer.feederOffCommand())
                .alongWith(intake.takeInCommand()),
            DriveCommands.buildFollowPath("RightSecondCollect"),
            spindexer
                .spindexerOnCommand()
                .alongWith(spindexer.feederOnCommand())
                .alongWith(intake.stopMotorCommand()));
    return auto;
  }

  public Command buildLeftNeutralZoneTwoCollect() {
    Command auto =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                turret.enableAutoAimCommand(() -> getHubPose3d()), intake.deployCommand()),
            new WaitCommand(0.5),
            intake.takeInCommand(),
            DriveCommands.buildFollowPath("CollectNeutralTopShoot"),
            new WaitCommand(1.0),
            spindexer
                .spindexerOnCommand()
                .alongWith(spindexer.feederOnCommand())
                .alongWith(intake.stopMotorCommand()),
            new WaitCommand(6),
            spindexer
                .spindexerOffCommand()
                .alongWith(spindexer.feederOffCommand().alongWith(intake.takeInCommand())),
            DriveCommands.buildFollowPath("LeftSecondCollect"),
            spindexer
                .spindexerOnCommand()
                .alongWith(spindexer.feederOnCommand())
                .alongWith(intake.stopMotorCommand()));
    return auto;
  }
}
