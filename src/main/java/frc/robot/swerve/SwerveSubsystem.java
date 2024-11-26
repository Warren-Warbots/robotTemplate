// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import dev.doglog.DogLog;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.util.ControllerHelpers;
import frc.robot.util.FmsUtil;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SweveSubsystem. */
  private final SwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric drive_no_snap;
  private final SwerveRequest.ApplyRobotSpeeds drive_robot_rel;
  private final SwerveRequest.FieldCentricFacingAngle drive_snap;
  private SwerveState state = SwerveState.NO_SNAP;
  private double currTopSpeed = SwerveConstants.maxSpeed;
  private double currTopRotationSpeed = SwerveConstants.maxRotSpeed;
  private final ChassisSpeeds driverDesiredSpeeds = new ChassisSpeeds();
  private CommandXboxController driverXboxController;
  private SwerveDriveState swerveDriveState = new SwerveDriveState();
  private double lastAddedVisionTimestamp = 0;
  private PoseEstimate estimatePose;
  private Matrix<N3, N1> stdDevs;

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  private Telemetry telem = new Telemetry(SwerveConstants.maxSpeed);

  private Rotation2d snapAngle = new Rotation2d();

  private Pose2d targetPose = new Pose2d();
  
  /*
   * TODO
   * current limits
   * open/closed loop ramps
   * 
   * 
   * 
   */

  public SwerveSubsystem(CommandXboxController driverXboxController) {
    drivetrain = new SwerveDrivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft,
        TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
    drive_no_snap = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    drive_snap = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    drive_robot_rel = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);
    drive_snap.HeadingController = SwerveConstants.snapController;
    drive_snap.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    drive_snap.HeadingController.setTolerance(SwerveConstants.snapTolerance); // maybe add velocity tolerance
    this.driverXboxController = driverXboxController;
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
    drivetrain.registerTelemetry(telem::telemeterize);
  }

  public ChassisSpeeds getTeleopDriveSpeeds() {
    double forward = ControllerHelpers.getExponent(
        ControllerHelpers.getDeadbanded(driverXboxController.getLeftY(), SwerveConstants.leftYDeadband),
        SwerveConstants.leftYExponent);
    double strafe = ControllerHelpers.getExponent(
        ControllerHelpers.getDeadbanded(driverXboxController.getLeftX(), SwerveConstants.leftXDeadband),
        SwerveConstants.leftXExponent);
    double rotate = -1.0 * ControllerHelpers.getExponent(
        ControllerHelpers.getDeadbanded(driverXboxController.getRightX(), SwerveConstants.rightXDeadband),
        SwerveConstants.rightXExponent);

    if (!FmsUtil.isRedAlliance()) {
      strafe *= -1.0;
      forward *= -1.0;
    }
    driverDesiredSpeeds.vxMetersPerSecond = forward;
    driverDesiredSpeeds.vyMetersPerSecond = strafe;
    driverDesiredSpeeds.omegaRadiansPerSecond = rotate;

    return driverDesiredSpeeds;

  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> drivetrain.getState().Pose, // Supplier of current robot pose //TODO figure out if calling getState
                                            // twice in a row, for pose and speeds is bad
          drivetrain::resetPose, // Consumer for seeding pose against auto
          () -> drivetrain.getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          this::setRobotRelativeSpeedsWithFeedForwards,
          new PPHolonomicDriveController(
              // PID constants for translation
              SwerveConstants.translationPIDConstantsAuto,
              // PID constants for rotation
              SwerveConstants.rotationPIDConstantsAuto),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the
          // case
          FmsUtil::isRedAlliance,
          this // Subsystem for requirements
      );
    } catch (Exception ex) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  public void setState(SwerveState newState) {
    state = newState;
  }

  public SwerveState getState() {
    return state;
  }

  public void setRobotRelativeSpeeds(ChassisSpeeds robotRelativeChassisSpeeds) {
    // only to be used in auto
    drivetrain.setControl(drive_robot_rel.withSpeeds(robotRelativeChassisSpeeds));
  }

  public void setRobotRelativeSpeedsWithFeedForwards(ChassisSpeeds robotRelativeChassisSpeeds,
      DriveFeedforwards feedforwards) {
    // only to be used in auto
    drivetrain.setControl(drive_robot_rel.withSpeeds(robotRelativeChassisSpeeds)
        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
  }

  public Command enableSnap() {
    return Commands.runOnce(() -> setState(SwerveState.SNAP));
  }

  public Command disableSnap() {
    return Commands.runOnce(() -> setState(SwerveState.NO_SNAP));
  }

  public void setSnapAngle(Rotation2d newSnapAngle) {
    this.snapAngle = newSnapAngle;
  }

  public Rotation2d getSnapAngle() {
    return this.snapAngle;
  }

  public Pose2d getPose() {
    return swerveDriveState.Pose;
  }

  public ChassisSpeeds getRobotSpeeds() {
    return swerveDriveState.Speeds;
  }

  public double getRobotRotationSpeed() {
    return currTopRotationSpeed;
  }

  public double getRobotTopSpeed() {
    return currTopSpeed;
  }

  public void setRobotTopSpeeds(double newTopSpeed, double newTopRotationSpeed) {
    this.currTopSpeed = newTopSpeed;
    this.currTopRotationSpeed = newTopRotationSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (SwerveConstants.useLimelight) {
      LimelightHelpers.SetRobotOrientation("", swerveDriveState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      addVisionPosesToPoseEstimator(true);
    }
    swerveDriveState = drivetrain.getState(); // not sure if this should be before or after vision pose est

    DogLog.log("Swerve/ModuleStates", swerveDriveState.ModuleStates);

    // logging here, dont log anything that ends up in the hoot log files (ie status
    // signals)

    DogLog.log("Swerve/EstimatedPose", swerveDriveState.Pose);
    DogLog.log("Swerve/TopSpeed", currTopSpeed);
    DogLog.log("Swerve/TopRotationSpeed", currTopRotationSpeed);
    DogLog.log("Swerve/State", state);

    if (DriverStation.isTeleop()) {

      getTeleopDriveSpeeds();
      // if driver is trying to rotate during snap, cancel snap
      if (state != SwerveState.NO_SNAP
          && Math.abs(driverDesiredSpeeds.omegaRadiansPerSecond) > SwerveConstants.snapEndThreshold) {
        state = SwerveState.NO_SNAP;
      }

      switch (state) {
        case NO_SNAP:
          // if we arent trying to rotate, use a PIDController to try and keep rotational
          // velocity at zero, so the robot doesnt stray
          if (Math.abs(driverDesiredSpeeds.omegaRadiansPerSecond) < SwerveConstants.rightXDeadband) {
            driverDesiredSpeeds.omegaRadiansPerSecond = SwerveConstants.noSnapController
                .calculate(-1 * swerveDriveState.Speeds.omegaRadiansPerSecond, 0.0, 1.0);
          }
          drivetrain
              .setControl(drive_no_snap.withVelocityX(driverDesiredSpeeds.vxMetersPerSecond * getRobotTopSpeed())
                  .withVelocityY(driverDesiredSpeeds.vyMetersPerSecond * getRobotTopSpeed())
                  .withRotationalRate(driverDesiredSpeeds.omegaRadiansPerSecond * getRobotRotationSpeed()));
          break;
        case SNAP:
          drivetrain.setControl(drive_snap
              .withVelocityX(driverDesiredSpeeds.vxMetersPerSecond * getRobotTopSpeed())
              .withVelocityY(driverDesiredSpeeds.vyMetersPerSecond * getRobotTopSpeed())
              .withTargetDirection(this.snapAngle));
          break;
        

      }

    } else {
      // only do something when we are in snap mode, this should only happen when we
      // are not driving,
      // ie for aiming at things
      // we need to be careful about enabling snap during a path
      if (state == SwerveState.SNAP) {
        drivetrain.setControl(drive_snap
            .withVelocityX(0)
            .withVelocityY(0)
            .withTargetDirection(this.snapAngle));
      }
    }

  }

  public void addVisionPosesToPoseEstimator(boolean useMegaTag2) {

    boolean isDisabled = false;
    if (useMegaTag2) {

      estimatePose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

    } else {
      estimatePose = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
      isDisabled = DriverStation.isDisabled();

    }
    if (estimatePose == null) {
      return;
    }

    // This prevents pose estimator from having crazy poses if the Limelight loses
    // power
    if (estimatePose.tagCount == 0 || estimatePose.pose.getX() == 0.0 && estimatePose.pose.getY() == 0.0) {
      return;
    }
    if (estimatePose.timestampSeconds == lastAddedVisionTimestamp) {
      return;
    }

    if (!useMegaTag2 && (estimatePose.tagCount < 2 || estimatePose.avgTagArea < (isDisabled ? 0.05 : 0.4))) {
      return;
    }
    stdDevs = useMegaTag2 ? SwerveConstants.megaTag2stdDev : SwerveConstants.megaTag2stdDev;
    if (!useMegaTag2 && isDisabled) {
      stdDevs = SwerveConstants.megaTag1DisabledstdDev;
    }

    drivetrain.addVisionMeasurement(estimatePose.pose, Utils.fpgaToCurrentTime(estimatePose.timestampSeconds), stdDevs);
    lastAddedVisionTimestamp = estimatePose.timestampSeconds;

  }


  

  public Command calibrateWheelRadius() {

    double[] distances = new double[4];
    for (int i = 0; i < 4; i++) {

      distances[i] = drivetrain.getModuleLocations()[i].getNorm(); // distance in meters to center of robot
    }
    SwerveRequest.Idle idle = new SwerveRequest.Idle();
    SwerveRequest.ApplyRobotSpeeds calibrationPosition = drive_robot_rel.withSpeeds(new ChassisSpeeds(0, 0, 0.35));
    // first rotate a bit so that the wheels get in the rotation positions (ie with
    // axles pointing towars the center of the robot)
    return run(() -> drivetrain.setControl(calibrationPosition)).withTimeout(0.5)
        // next stop for a bit
        .andThen(run(() -> drivetrain.setControl(idle)).withTimeout(0.25))
        // then zero the gyros and encoders
        .andThen(
            runOnce(() -> {
              drivetrain.getPigeon2().setYaw(0);
              for (int i = 0; i < 4; i++) {

                drivetrain.getModule(i).getDriveMotor().setPosition(0);
              }

            }))
        .andThen(
            // now drive slowly in a circle measuring how far the wheels have rotated vs how
            // far the gyro has rotated
            // since we know where the modules are on the robot, we can figure out what the
            // wheel cirumfrence must be
            run(
                () -> {
                  // refresh all of the signals so that the gyro value was taken at approx the
                  // same time as the encoder value
                  // in sim this moves us from an error of ~ 0.005" to < 0.001"
                  BaseStatusSignal.waitForAll(0.0, drivetrain.getPigeon2().getYaw(),
                      drivetrain.getModule(0).getDriveMotor().getRotorPosition(),
                      drivetrain.getModule(1).getDriveMotor().getRotorPosition(),
                      drivetrain.getModule(2).getDriveMotor().getRotorPosition(),
                      drivetrain.getModule(3).getDriveMotor().getRotorPosition());
                  double degrees_traveled = drivetrain.getPigeon2().getYaw(false).getValueAsDouble();
                  double avg_radius = 0;
                  for (Integer i = 0; i < 4; i++) {
                    double arc_len_traveled = distances[i] * Units.degreesToRadians(degrees_traveled);

                    double rotations = drivetrain.getModule(i).getDriveMotor().getRotorPosition(false)
                        .getValueAsDouble() / TunerConstants.kDriveGearRatio; // latency comp in the future? or just
                                                                              // make sure gyro value is pulled same as
                                                                              // drive module
                    double est_circum = arc_len_traveled / rotations;
                    double est_radius = Units.metersToInches(Math.abs(est_circum / (Math.PI * 2)));
                    avg_radius += est_radius;
                    DogLog.log("Swerve/Module" + i.toString() + "radius", est_radius);
                  }
                  avg_radius /= 4.0;
                  DogLog.log("Swerve/AvgRadius", avg_radius);
                  drivetrain.setControl(calibrationPosition);

                }).until(() -> drivetrain.getPigeon2().getYaw().getValueAsDouble() > 360 * 5)
        // keep going for 5 full rotations, since the robot is driving slow this will
        // take a while
        );

  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      drivetrain.updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

}
