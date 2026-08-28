// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.constants.jr.DriveConstants;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.COTS.WHEELS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.littletonrobotics.junction.Logger;

/**
 * The other robots on the field, driven from outside the JVM.
 *
 * <p>maple-sim will hold as many drivetrains as you give it, but it has no opinion about what they
 * do: its own answer for opponents is a PathPlanner replay or a second gamepad, so there is no
 * decision-making in either. sparky-sim has decision-making and no physics. This class is the seam
 * between them -- it owns the bodies, and something outside says where they should go.
 *
 * <p><b>Nothing here decides anything.</b> That is deliberate and it is the whole design. Every
 * behaviour an opponent has -- where it drives, when it intakes, when it dumps -- arrives over
 * NetworkTables from the process that is already running a strategy layer. Putting even a simple
 * "drive at the nearest ball" rule in here would be a second AI to keep in agreement with the first,
 * and this bridge has already paid once for having two models of the same thing.
 *
 * <h2>The wire</h2>
 *
 * Inbound, under {@value #TABLE}:
 *
 * <ul>
 *   <li>{@code Roster} -- {@code double[]}, flattened (x, y, theta) triples in metres and radians,
 *       blue-origin. Read <b>once</b>: the first non-empty value creates that many drivetrains at
 *       those poses and the roster is never consulted again. A match's cast does not change
 *       mid-match, and re-reading it would make "move a robot" and "add a robot" the same message.
 *   <li>{@code Tick} -- {@code int}, a heartbeat. See below.
 *   <li>{@code <i>/Speeds} -- {@code double[3]}, field-relative vx, vy (m/s) and omega (rad/s).
 *   <li>{@code <i>/Intake} -- {@code boolean}, run that robot's intake.
 *   <li>{@code <i>/Release} -- {@code boolean}, spit held pieces back onto the floor.
 * </ul>
 *
 * Outbound, as AdvantageKit outputs rather than a second bespoke table, so that AdvantageScope and
 * the replay see them for free: {@code FieldSimulation/BridgeRobots} (poses), {@code
 * FieldSimulation/BridgeRobotsSpeeds} (flattened field-relative vx, vy, omega) and {@code
 * FieldSimulation/BridgeRobotsHeld} (piece counts). What arrived on the inbound side is logged too,
 * under {@code Bridge/Robots/*} -- a replay that cannot see the commands cannot explain the motion.
 *
 * <h2>The heartbeat, and why it is not a timestamp</h2>
 *
 * <p>If the far side stops talking -- the match ended, the harness crashed, somebody hit ctrl-C --
 * the last speed it sent must not become a permanent instruction. Five robots driving into a wall
 * for the rest of a session is not a scenario, it is a mess to be cleaned up before the next one.
 *
 * <p>So the far side increments {@code Tick} every time it commands, and {@link #periodic()} counts
 * how many cycles have passed since that number last changed. Past {@value #STALE_CYCLES} cycles
 * every robot is commanded to zero and its intake stops. A counter rather than a timestamp because
 * the two processes do not share a clock, and the version of this that compared NT timestamps to the
 * local one is exactly the kind of thing that works until the day it silently does not.
 *
 * <p>Inert until spoken to: with no roster published this class creates nothing, costs four
 * subscriptions, and a normal simulation session is unchanged.
 */
public class BridgeRobots {

  /** Where the far side publishes. Plain NT, not AdvantageKit: these are inputs. */
  public static final String TABLE = "/Bridge/Robots";

  /**
   * Six robots on a field, minus the one that is the actual robot code. A cap rather than a
   * capability -- the arena will take more, but a roster longer than this is a mistake on the other
   * side of the wire and is better refused than simulated.
   */
  public static final int MAX_ROBOTS = 5;

  /** Cycles of silence before the field goes still. 25 at 50 Hz is half a second. */
  public static final int STALE_CYCLES = 25;

  /**
   * Cycles between two pieces of a dump.
   *
   * <p>A hopper holds forty. Returning forty balls to the floor in one tick puts forty circles at
   * one point and asks dyn4j to sort it out, which it does by flinging them. Spacing the dump means
   * they land where the robot actually was, which is both calmer and closer to what dumping looks
   * like.
   */
  private static final int RELEASE_PERIOD = 10;

  /** How far behind the robot a released piece lands, in metres. Clear of its own intake. */
  private static final double RELEASE_BEHIND_M = 0.55;

  /**
   * REBUILT's piece. The one game-specific name in this file.
   *
   * <p>An opponent that can pick up and put down is game-agnostic machinery; the string it collects
   * is not, and neither is {@link RebuiltFuelOnField}. Both are one-line changes for next season,
   * which is the right amount of coupling for a class whose reason to exist is the season after
   * this one.
   */
  private static final String PIECE_TYPE = "Fuel";

  private static final double[] ZERO3 = new double[] {0.0, 0.0, 0.0};

  /**
   * Same drivetrain as ours.
   *
   * <p>Opponents that are faster or slower than the robot under test would make every contested
   * scenario a statement about the difference rather than about the robot code, so they get the
   * config the real one has. If a campaign ever wants a deliberately quicker opponent, that belongs
   * on the wire as a per-robot parameter and not as a different constant here.
   */
  private static final DriveTrainSimulationConfig CONFIG =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Kilograms.of(DriveConstants.robotMassKg))
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              COTS.ofMark4i(
                  DriveConstants.driveGearbox,
                  DriveConstants.turnGearbox,
                  WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                  3))
          .withCustomModuleTranslations(DriveConstants.moduleTranslations)
          .withBumperSize(Inches.of(30), Inches.of(30));

  /** One commanded robot: a body, a hand, and the three topics that drive them. */
  private static final class Entry {
    final SelfControlledSwerveDriveSimulation drive;
    final IntakeSimulation intake;
    final DoubleArraySubscriber speeds;
    final BooleanSubscriber wantIntake;
    final BooleanSubscriber wantRelease;
    int releaseCountdown = 0;

    Entry(
        SelfControlledSwerveDriveSimulation drive,
        IntakeSimulation intake,
        DoubleArraySubscriber speeds,
        BooleanSubscriber wantIntake,
        BooleanSubscriber wantRelease) {
      this.drive = drive;
      this.intake = intake;
      this.speeds = speeds;
      this.wantIntake = wantIntake;
      this.wantRelease = wantRelease;
    }
  }

  private final SimulatedArena arena;
  private final NetworkTableInstance nt;
  private final DoubleArraySubscriber roster;
  private final IntegerSubscriber heartbeat;
  private final List<Entry> robots = new ArrayList<>();

  private long lastBeat = Long.MIN_VALUE;
  private int cyclesSinceBeat = 0;

  public BridgeRobots(SimulatedArena arena) {
    this.arena = arena;
    this.nt = NetworkTableInstance.getDefault();
    // keepDuplicates so that a repeated identical command still counts as
    // traffic. Not needed for the heartbeat, which changes every time by
    // construction, but the speeds legitimately repeat when a robot is asked
    // to hold still and those must not read as silence.
    this.roster =
        nt.getDoubleArrayTopic(TABLE + "/Roster")
            .subscribe(new double[0], PubSubOption.keepDuplicates(true));
    this.heartbeat = nt.getIntegerTopic(TABLE + "/Tick").subscribe(0);
  }

  /** How many robots this is driving. Zero until a roster arrives. */
  public int count() {
    return robots.size();
  }

  /**
   * Command the extra robots, then say where they ended up.
   *
   * <p>Call before {@link SimulatedArena#simulationPeriodic()}: these are setpoints for the step
   * that is about to happen, and commanding after it means every opponent acts on the world as it
   * was one tick ago.
   */
  public void periodic() {
    if (robots.isEmpty()) {
      createFromRoster();
      if (robots.isEmpty()) {
        return;
      }
    }

    final long beat = heartbeat.get();
    if (beat != lastBeat) {
      lastBeat = beat;
      cyclesSinceBeat = 0;
    } else {
      cyclesSinceBeat++;
    }
    final boolean live = cyclesSinceBeat < STALE_CYCLES;

    final Pose2d[] poses = new Pose2d[robots.size()];
    final long[] held = new long[robots.size()];
    final double[] measured = new double[robots.size() * 3];
    final double[] commanded = new double[robots.size() * 3];

    for (int i = 0; i < robots.size(); i++) {
      final Entry entry = robots.get(i);
      final double[] raw = live ? entry.speeds.get(ZERO3) : ZERO3;
      final double[] command = raw.length == 3 ? raw : ZERO3;
      apply(entry, command, live);

      poses[i] = entry.drive.getActualPoseInSimulationWorld();
      held[i] = entry.intake.getGamePiecesAmount();

      // Measured and published rather than left for the far side to work
      // out by differencing poses. The strategy layer reads a robot's speed
      // to tell one that is waiting from one that is being held, and to lead
      // a moving target; a difference quotient off a 20 ms pose stream is
      // noisy exactly where those decisions are made. The number is right
      // here, so send it.
      final ChassisSpeeds speeds = entry.drive.getActualSpeedsFieldRelative();
      measured[i * 3] = speeds.vxMetersPerSecond;
      measured[i * 3 + 1] = speeds.vyMetersPerSecond;
      measured[i * 3 + 2] = speeds.omegaRadiansPerSecond;

      System.arraycopy(command, 0, commanded, i * 3, 3);
    }

    Logger.recordOutput("FieldSimulation/BridgeRobots", poses);
    Logger.recordOutput("FieldSimulation/BridgeRobotsHeld", held);
    Logger.recordOutput("FieldSimulation/BridgeRobotsSpeeds", measured);
    Logger.recordOutput("Bridge/Robots/Commanded", commanded);
    Logger.recordOutput("Bridge/Robots/Live", live);
  }

  /** Drive one robot, and work its intake. */
  private void apply(Entry entry, double[] command, boolean live) {
    // Rotated here rather than by asking runChassisSpeeds for a field-centric
    // drive, because that route rotates by the *odometry* pose -- which is only
    // right if somebody has been calling periodic() to update the estimator and
    // reset it to the true pose at the start. An opponent has no odometry worth
    // the name, so use the pose the physics actually has.
    final Rotation2d heading = entry.drive.getActualPoseInSimulationWorld().getRotation();
    entry.drive.runChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(command[0], command[1], command[2], heading),
        Translation2d.kZero,
        false,
        true);

    final boolean wantIntake = live && entry.wantIntake.get();
    if (wantIntake != entry.intake.isRunning()) {
      if (wantIntake) {
        entry.intake.startIntake();
      } else {
        entry.intake.stopIntake();
      }
    }

    if (live && entry.wantRelease.get()) {
      if (entry.releaseCountdown <= 0) {
        release(entry);
        entry.releaseCountdown = RELEASE_PERIOD;
      } else {
        entry.releaseCountdown--;
      }
    } else {
      entry.releaseCountdown = 0;
    }
  }

  /** Put one held piece back on the floor, behind the robot. */
  private void release(Entry entry) {
    if (!entry.intake.obtainGamePieceFromIntake()) {
      return;
    }
    final Pose2d pose = entry.drive.getActualPoseInSimulationWorld();
    final Translation2d where =
        pose.getTranslation()
            .plus(new Translation2d(-RELEASE_BEHIND_M, 0.0).rotateBy(pose.getRotation()));
    arena.addGamePiece(new RebuiltFuelOnField(where));
  }

  /**
   * Put an extra robot on its own battery, by cancelling it out of ours.
   *
   * <p><b>maple-sim has one battery for the entire arena.</b> {@link SimulatedBattery} is static:
   * every {@link SwerveModuleSimulation} constructor registers its drive motor's supply current on
   * it, every {@code MapleMotorSim} registers its steer motor, and {@code simulationSubTick} sums
   * the lot, drops the voltage accordingly, and hands the result to {@code
   * RoboRioSim.setVInVoltage} -- which is the rail voltage the robot code under test reads.
   *
   * <p>So five extra drivetrains draw from the battery of the robot being tested. Six robots
   * driving hard is several hundred amps, the voltage clamps at the brownout threshold, and the
   * first contested campaign produced hundreds of {@code BrownOut Detected} lines in every match.
   * That is not cosmetic: {@code SimulatedBattery.clamp} then limits every motor's applied
   * voltage, so the robot under test is genuinely slowed down by the presence of opponents. Every
   * finding from such a run would be contaminated by an artefact of the test rig, and the
   * contamination looks exactly like a robot that browns out under load.
   *
   * <p>There is no unregister, so this registers a <b>negative</b> appliance instead: a supplier
   * returning minus this robot's own draw, which the sum cancels exactly. Both terms read the same
   * instantaneous voltage, so the cancellation is exact rather than approximate.
   *
   * <p>Physically this is the correct model and not a workaround for one. Every robot in a real
   * match carries its own battery, and an opponent accelerating has no effect whatsoever on our
   * rail voltage. The shared battery is the artefact; this removes it.
   *
   * <p>What it does <em>not</em> do is give each extra robot a battery that can sag under its own
   * load -- they all run at whatever our rail is doing. That is the right trade: the point of the
   * extras is bodies that move with intent, and modelling their brownouts would mean simulating
   * five more electrical systems to make five opponents slightly slower.
   */
  private static void takeOffOurBattery(SwerveDriveSimulation body) {
    SimulatedBattery.addElectricalAppliances(
        () -> {
          double amps = 0.0;
          for (SwerveModuleSimulation module : body.getModules()) {
            amps += module.getDriveMotorSupplyCurrent().in(Amps);
            amps += module.getSteerMotorSupplyCurrent().in(Amps);
          }
          return Amps.of(-amps);
        });
  }

  /**
   * Build the cast, once, from the first roster that arrives.
   *
   * <p>A malformed roster is dropped rather than partly honoured. Half a cast is a scenario nobody
   * asked for, and the far side is in a better position to notice that {@code count()} never became
   * what it expected than to debug why match 40 had two opponents instead of three.
   */
  private void createFromRoster() {
    final double[] raw = roster.get(new double[0]);
    if (raw.length == 0) {
      return;
    }
    if (raw.length % 3 != 0 || raw.length / 3 > MAX_ROBOTS) {
      System.err.println(
          "BridgeRobots: ignoring a roster of "
              + raw.length
              + " doubles -- expected (x, y, theta) triples, at most "
              + MAX_ROBOTS
              + " robots");
      return;
    }

    for (int i = 0; i < raw.length / 3; i++) {
      final Pose2d start = new Pose2d(raw[i * 3], raw[i * 3 + 1], new Rotation2d(raw[i * 3 + 2]));
      final SwerveDriveSimulation body = new SwerveDriveSimulation(CONFIG, start);
      arena.addDriveTrainSimulation(body);
      takeOffOurBattery(body);

      // Matches IntakeIOSim's geometry, for the same reason the drivetrain
      // config does: an opponent whose reach differs from ours turns every
      // contested piece into a statement about the difference.
      final IntakeSimulation intake =
          IntakeSimulation.OverTheBumperIntake(
              PIECE_TYPE,
              body,
              Inches.of(29),
              Inches.of(12),
              IntakeSimulation.IntakeSide.FRONT,
              40);

      final String prefix = TABLE + "/" + i;
      robots.add(
          new Entry(
              new SelfControlledSwerveDriveSimulation(body),
              intake,
              nt.getDoubleArrayTopic(prefix + "/Speeds")
                  .subscribe(ZERO3, PubSubOption.keepDuplicates(true)),
              nt.getBooleanTopic(prefix + "/Intake").subscribe(false),
              nt.getBooleanTopic(prefix + "/Release").subscribe(false)));
    }
    Logger.recordOutput("Bridge/Robots/Count", robots.size());
    System.out.println("BridgeRobots: driving " + robots.size() + " extra robot(s)");
  }
}
