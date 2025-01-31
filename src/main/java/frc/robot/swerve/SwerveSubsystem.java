// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import java.lang.reflect.Field;
import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.ControllerHelpers;
import frc.robot.util.FieldUtil;
import frc.robot.util.FmsUtil;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.ReefUtil;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SweveSubsystem. */
  public SwerveDrivetrain drivetrain;
  private SwerveRequest.FieldCentric drive_no_snap;
  private SwerveRequest.ApplyRobotSpeeds drive_robot_rel;
  private SwerveRequest.FieldCentricFacingAngle drive_snap;
  private SwerveRequest.FieldCentricFacingAngle driveMaintainHeading;
  private SwerveState state = SwerveState.NO_SNAP;
  private double currTopSpeedPercent = 1.0; // 0 to 1.0 -> percent of robotSpeedAt12V to drive at
  private double currTopRotationSpeedPercent = 1.0; // also a percent
  private ChassisSpeeds driverDesiredSpeeds = new ChassisSpeeds();
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
  private Optional<Rotation2d> lastMaintainHeadingAngle = Optional.empty();
  private double rotationJoystickLastTouched= -1;
  private double highSpeedLastTime= -1;
  private Pose2d targetPose = new Pose2d();
  
  /*
   * TODO
   * drive to pose command
   * make robust zero drivetrain command 
   * try out poinhole model localization
   * dont collect vision poses while moving
   * move swerve requests to constants?
   * 
   */

  public SwerveSubsystem(CommandXboxController driverXboxController) {
    drivetrain = new SwerveDrivetrain(SwerveConstants.swerveDrivetrainConstants,
    SwerveConstants.FrontLeft,
    SwerveConstants.FrontRight,
    SwerveConstants.BackLeft,
    SwerveConstants.BackRight);

    drive_robot_rel = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);
    drive_no_snap = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity)
        .withDeadband(0.05 * SwerveConstants.maxSpeed)
        .withRotationalDeadband(0.01 * SwerveConstants.maxRotSpeed);
    drive_snap = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity)
        .withDeadband(0.05 * SwerveConstants.maxSpeed)
        .withRotationalDeadband(0.05 * SwerveConstants.maxRotSpeed);
    drive_snap.HeadingController = SwerveConstants.snapController;
    drive_snap.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    drive_snap.HeadingController.setTolerance(SwerveConstants.snapTolerance); // maybe add velocity tolerance
    

    driveMaintainHeading = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity)
    .withDeadband(0.05 * SwerveConstants.maxSpeed)
    .withRotationalDeadband(0.05 * SwerveConstants.maxRotSpeed);
    driveMaintainHeading.HeadingController = SwerveConstants.maintainHeadingController;
    driveMaintainHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveMaintainHeading.HeadingController.setTolerance(SwerveConstants.maintainHeadingTolerance);
     // maybe add velocity tolerance
    
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
    return currTopRotationSpeedPercent * SwerveConstants.maxRotSpeed;
  }

  public double getRobotTopSpeed() {
    return currTopSpeedPercent * SwerveConstants.maxSpeed;
  }

  public void setRobotTopSpeeds(double newTopSpeed, double newTopRotationSpeed) {
    this.currTopSpeedPercent = newTopSpeed;
    this.currTopRotationSpeedPercent = newTopRotationSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (SwerveConstants.useLimelight) {
      LimelightHelpers.SetRobotOrientation("", swerveDriveState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0); //TODO pass more data
      addVisionPosesToPoseEstimator(true);
    }
    swerveDriveState = drivetrain.getState(); // not sure if this should be before or after vision pose est
    double currentTime = Timer.getFPGATimestamp();
    double robotSpeed = new Translation2d(swerveDriveState.Speeds.vxMetersPerSecond,swerveDriveState.Speeds.vyMetersPerSecond).getNorm();
    DogLog.log("Swerve/ModuleStates", swerveDriveState.ModuleStates);

    // logging here, dont log anything that ends up in the hoot log files (ie status
    // signals)

    DogLog.log("Swerve/EstimatedPose", swerveDriveState.Pose);
    DogLog.log("Swerve/TopSpeedPercent", currTopSpeedPercent);
    DogLog.log("Swerve/TopRotationSpeedPercent", currTopRotationSpeedPercent);
    DogLog.log("Swerve/State", state);

    DogLog.log("Swerve/Speeds", swerveDriveState.Speeds);

    if (DriverStation.isTeleop()) {
      
      getTeleopDriveSpeeds();
      // if driver is trying to rotate during snap, cancel snap
      // if we add more swervestates that snap in the future, need to add these to the
      // check here
      
      if (state == SwerveState.SNAP
          && Math.abs(driverDesiredSpeeds.omegaRadiansPerSecond) > SwerveConstants.rightXDeadband) {
        state = SwerveState.NO_SNAP;
      }
      if (Math.abs(driverDesiredSpeeds.omegaRadiansPerSecond) > SwerveConstants.rightXDeadband
      ) {
            rotationJoystickLastTouched = currentTime;
        }

        if (robotSpeed>1
        ) {
              highSpeedLastTime = currentTime;
          }

      DogLog.log("Swerve/TeleopDesiredSpeeds", driverDesiredSpeeds);
      switch (state) {
        case NO_SNAP:
          if (Math.abs(driverDesiredSpeeds.omegaRadiansPerSecond) > SwerveConstants.rightXDeadband 
          || lastMaintainHeadingAngle.isEmpty()
          || ((currentTime-highSpeedLastTime)>0.1)  
          || ( (currentTime-rotationJoystickLastTouched < 0.2)) ){
            drivetrain
            .setControl(drive_no_snap.withVelocityX(driverDesiredSpeeds.vxMetersPerSecond * getRobotTopSpeed())
                .withVelocityY(driverDesiredSpeeds.vyMetersPerSecond * getRobotTopSpeed())
                .withRotationalRate(
                    driverDesiredSpeeds.omegaRadiansPerSecond * getRobotRotationSpeed()));
            lastMaintainHeadingAngle = Optional.of(swerveDriveState.Pose.getRotation());
            
          } else {
            drivetrain.setControl(driveMaintainHeading
              .withVelocityX(driverDesiredSpeeds.vxMetersPerSecond * getRobotTopSpeed())
              .withVelocityY(driverDesiredSpeeds.vyMetersPerSecond * getRobotTopSpeed())
              .withTargetDirection(lastMaintainHeadingAngle.get()));

          }
            
          break;
        case SNAP:
          drivetrain.setControl(drive_snap
              .withVelocityX(driverDesiredSpeeds.vxMetersPerSecond * getRobotTopSpeed())
              .withVelocityY(driverDesiredSpeeds.vyMetersPerSecond * getRobotTopSpeed())
              .withTargetDirection(this.snapAngle));
          break;

        
        case CALIBRATION:
          Pose2d selectedReefFace = FieldUtil.getReef().A().getLeft();
          Pose2d desiredPose = FieldUtil.addRobotOffset(selectedReefFace);
          Translation2d driverTranslation = new Translation2d(driverDesiredSpeeds.vxMetersPerSecond, driverDesiredSpeeds.vyMetersPerSecond);
  
          Translation2d[] reefRelativeVectors = {(new Translation2d(1,0)).rotateBy(selectedReefFace.getRotation()),
          (new Translation2d(-1,0)).rotateBy(selectedReefFace.getRotation()),
          (new Translation2d(0,1)).rotateBy(selectedReefFace.getRotation()),
           (new Translation2d(0,-1)).rotateBy(selectedReefFace.getRotation())};
          

          //this code goes through each direction (forward,backward,left,right) from the perspective of somone looking at the reef
          // for each of these directions, whichever one is closest to the drivers chosen input is the direction we choose to move in
          double largestDotProduct =-10;
          int directionIndex = -1;
          for (int i =0;i<4;i++){
            double currDotProd = reefRelativeVectors[i].toVector().dot(driverTranslation.toVector());
            if (largestDotProduct<currDotProd){
              largestDotProduct=currDotProd;
              directionIndex=i;
            }
          }
          Translation2d scaledDesiredMovementVector = reefRelativeVectors[directionIndex].times(largestDotProduct);


          drivetrain.setControl(drive_snap
              .withVelocityX( scaledDesiredMovementVector.getX()* getRobotTopSpeed())
              .withVelocityY(scaledDesiredMovementVector.getY() * getRobotTopSpeed())
              .withTargetDirection(selectedReefFace.getRotation()));
          // do nothing
          break;

      }

    } else {
      // only do something when we are in snap mode, this should only happen when we
      // are not driving,
      // ie for aiming at things in auto while static
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

  public Command testDriveGains(double vel) {
    return Commands.runOnce(() -> state = SwerveState.CALIBRATION).andThen(
        Commands.run(() -> {
          DogLog.log("Swerve/setVel", vel);
          drivetrain.setControl(drive_robot_rel.withSpeeds(new ChassisSpeeds(vel, 0, 0)));
        })).finallyDo(() -> {
          state = SwerveState.NO_SNAP;
          drivetrain.setControl(new SwerveRequest.Idle());
          DogLog.log("Swerve/setVel", 0);
        });
  }

  public Command calibrateVolts(double volts) {
    SwerveRequest.SysIdSwerveTranslation kvRequest = new SwerveRequest.SysIdSwerveTranslation();
    return

    Commands.runOnce(() -> state = SwerveState.CALIBRATION).andThen(Commands.run(() -> {
      BaseStatusSignal.waitForAll(0.0,
          drivetrain.getModule(0).getDriveMotor().getVelocity(),
          drivetrain.getModule(1).getDriveMotor().getVelocity(),
          drivetrain.getModule(2).getDriveMotor().getVelocity(),
          drivetrain.getModule(3).getDriveMotor().getVelocity());
      double avgvel = (drivetrain.getModule(0).getDriveMotor().getVelocity().getValueAsDouble() +
          drivetrain.getModule(1).getDriveMotor().getVelocity().getValueAsDouble() +
          drivetrain.getModule(2).getDriveMotor().getVelocity().getValueAsDouble() +
          drivetrain.getModule(3).getDriveMotor().getVelocity().getValueAsDouble()) / 4.0;
      DogLog.log("Swerve/volts", volts);
      DogLog.log("Swerve/vel", avgvel);
      if (avgvel > 0.01) {
        DogLog.log("Swerve/kvEst", volts / avgvel);
      }
      drivetrain.setControl(kvRequest.withVolts(volts));
    })).finallyDo(() -> {
      state = SwerveState.NO_SNAP;
      DogLog.log("Swerve/kvEst", 0);
    });
  }

  public Command calibrateWheelRadius() {

    double[] distances = new double[4];
    for (int i = 0; i < 4; i++) {

      distances[i] = drivetrain.getModuleLocations()[i].getNorm(); // distance in meters to center of robot
    }
    SwerveRequest.Idle idle = new SwerveRequest.Idle();
    // SwerveRequest.SysIdSwerveRotation calRotation = new
    // SwerveRequest.SysIdSwerveRotation();
    SwerveRequest.ApplyRobotSpeeds calRotation = new SwerveRequest.ApplyRobotSpeeds()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // first rotate a bit so that the wheels get in the rotation positions (ie with
    // axles pointing towars the center of the robot)
    return run(() -> {
      state = SwerveState.CALIBRATION;
      drivetrain.setControl(calRotation.withSpeeds(new ChassisSpeeds(0, 0, 4.0)));
      // drivetrain.setControl(calRotation.withRotationalRate(1.0));
    }).withTimeout(0.15)
        // // next stop for a bit
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
                  state = SwerveState.CALIBRATION;
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
                        .getValueAsDouble() / SwerveConstants.driveGearRatio; // latency comp in the future? or just
                                                                              // make sure gyro value is pulled same as
                                                                              // drive module
                    double est_circum = arc_len_traveled / rotations;
                    double est_radius = Units.metersToInches(Math.abs(est_circum / (Math.PI * 2)));
                    avg_radius += est_radius;
                    DogLog.log("Swerve/Module" + i.toString() + "radius", est_radius);
                  }
                  avg_radius /= 4.0;
                  DogLog.log("Swerve/AvgRadius", avg_radius);
                  drivetrain.setControl(calRotation.withSpeeds(new ChassisSpeeds(0, 0, 4.0)));

                  // drivetrain.setControl(calRotation.withRotationalRate(1.0));

                }).until(() -> drivetrain.getPigeon2().getYaw().getValueAsDouble() > 360 * 15)
                .andThen(run(() -> drivetrain.setControl(idle)).withTimeout(0.25))
                .finallyDo(()->state = SwerveState.NO_SNAP)
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
