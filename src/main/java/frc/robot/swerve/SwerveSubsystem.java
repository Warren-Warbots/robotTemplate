// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import java.lang.reflect.Field;

import com.ctre.phoenix6.Utils;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.util.ControllerHelpers;
import frc.robot.util.FieldUtil;
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
  private boolean snapEnabled = false;


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
    double forward =  ControllerHelpers.getExponent(
        ControllerHelpers.getDeadbanded(driverXboxController.getLeftY(),SwerveConstants.leftYDeadband),SwerveConstants.leftYExponent);
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
  public Command enableSnap(){
    return Commands.runOnce(()->setState(SwerveState.SNAP));
  }

  public Command disableSnap(){
    return Commands.runOnce(()->setState(SwerveState.NO_SNAP));
  }

  public void setSnapAngle(Rotation2d newSnapAngle){
    this.snapAngle = newSnapAngle;
  }

  public Rotation2d getSnapAngle(){
    return this.snapAngle;
  }
  public Pose2d getPose() {
    return swerveDriveState.Pose;
  }

  public ChassisSpeeds getRobotSpeeds() {
    return swerveDriveState.Speeds;
  }

  public double getRobotRotationSpeed(){
    return currTopRotationSpeed;
  }

  public double getRobotTopSpeed(){
    return currTopSpeed;
  }

  public void setRobotTopSpeeds(double newTopSpeed, double newTopRotationSpeed){
    this.currTopSpeed = newTopSpeed;
    this.currTopRotationSpeed = newTopRotationSpeed;
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    swerveDriveState = drivetrain.getState();
    if (SwerveConstants.useLimelight) {
      LimelightHelpers.SetRobotOrientation("", swerveDriveState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      addVisionPosesToPoseEstimator(true);
    }

    // logging here, dont log anything that ends up in the hoot log files (ie status
    // signals)
    DogLog.log("Swerve/EstimatedPose", swerveDriveState.Pose);
    DogLog.log("Swerve/TopSpeed",currTopSpeed);
    DogLog.log("Swerve/TopRotationSpeed",currTopRotationSpeed);
    DogLog.log("Swerve/State",state);

    if (DriverStation.isTeleop()) {

      getTeleopDriveSpeeds();
      // if driver is trying to rotate during snap, cancel snap
      if (state != SwerveState.NO_SNAP
          && Math.abs(driverDesiredSpeeds.omegaRadiansPerSecond) > SwerveConstants.snapEndThreshold) {
        state = SwerveState.NO_SNAP;
      }

      switch (state) {
        case NO_SNAP:
          //if we arent trying to rotate, use a PIDController to try and keep rotational velocity at zero, so the robot doesnt stray
          if (Math.abs(driverDesiredSpeeds.omegaRadiansPerSecond)<SwerveConstants.rightXDeadband){
            driverDesiredSpeeds.omegaRadiansPerSecond = SwerveConstants.noSnapController.calculate(-1*swerveDriveState.Speeds.omegaRadiansPerSecond, 0.0, 1.0);
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
      // do nothing because auto will send commands to drivetrain seperately
      if (state == SwerveState.SNAP) {
        drivetrain.setControl(drive_snap
            .withVelocityX(0)
            .withVelocityY(0)
            .withTargetDirection(this.snapAngle));
      }
    }

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
