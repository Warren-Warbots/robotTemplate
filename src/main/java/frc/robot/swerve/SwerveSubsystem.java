// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import java.util.HashMap;
import java.util.Optional;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import dev.doglog.DogLog;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.swerve.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.ControllerHelpers;
import frc.robot.util.FieldUtil;
import frc.robot.util.FmsUtil;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class SwerveSubsystem {
    public TunerSwerveDrivetrain drivetrain;
    private SwerveRequest.FieldCentric drive_field_rel;
    private SwerveRequest.ApplyRobotSpeeds drive_robot_rel;
    private SwerveRequest.FieldCentricFacingAngle drive_snap;
    private SwerveRequest.FieldCentricFacingAngle driveMaintainHeading;
    private SwerveRequest.FieldCentricFacingAngle auto_point;
    private SwerveRequest.RobotCentric drive_robot_centric;
    public WantedState wantedState = WantedState.TELEOP_DRIVE;
    private SystemState systemState = SystemState.TELEOP_DRIVE;
    private double currTopSpeedPercent = 1.0;
    private double currTopRotationSpeedPercent = 1.0;
    private ChassisSpeeds driverDesiredSpeeds = new ChassisSpeeds();
    private double currentTime = 0.0;
    private double xVelocity;
    private double yVelocity;
    private double rVelocity;
    private Pose2d targetPose = new Pose2d();
    private double tranlationMag;
    private double maxVelocity;
    private double maxRVelocity;
    private boolean isContinuous;
    private double atGoalTolerance;
    private Rotation2d diffRotation;
    private SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(8);
    private SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(8);
    private Translation2d swerveCOR = new Translation2d(0, 0);

    public SwerveDriveState swerveDriveState = new SwerveDriveState();
    public SwerveDriveState lastSwerveDriveState = new SwerveDriveState();

    private Matrix<N3, N1> stdDevs;
    private static final double kSimLoopPeriod = 0.005;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Telemetry telem = new Telemetry(SwerveConstants.maxSpeed);
    private XboxController driverXboxController;
    private Optional<Rotation2d> lastMaintainHeadingAngle = Optional.empty();
    private double rotationJoystickLastTouched = -1;
    private double highSpeedLastTime = -1;

    double robotSpeed;
    private Pose2d driveToPoseTargetPose = new Pose2d();
    private double driveToPoseMaxSpeed;
    private double driveToPoseMaxAngularSpeed;
    private double driveToPoseTranslationToleranceMeters;
    private double driveToPoseRotationToleranceDegrees;

    private Rotation2d snapAngle = new Rotation2d();
    private Translation2d snapPoint = new Translation2d();
    private Pose2d startingPose = new Pose2d();

    private boolean atGoal = false;
    private boolean timerHasBeenEnabled = false;

    Rotation2d foo = Rotation2d.fromDegrees(67);
    Translation2d fooo = new Translation2d();

    //this is the variable for the automatic driving task
    Pose2d foooo = new Pose2d();



    Rotation2d dPadSnap = Rotation2d.fromDegrees(67);

    private String[] limelightNames = { "limelight" };

    private HashMap<String, Double> lastAddedVisionTimestampMap = new HashMap<String, Double>();

    public SwerveSubsystem(XboxController driverXboxController) {

        drivetrain = new TunerSwerveDrivetrain(SwerveConstants.swerveDrivetrainConstants,
                SwerveConstants.FrontLeft,
                SwerveConstants.FrontRight,
                SwerveConstants.BackLeft,
                SwerveConstants.BackRight);

        drive_robot_rel = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);
        drive_field_rel = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity)
                .withDeadband(0.08)
                .withRotationalDeadband(0.06 * SwerveConstants.maxRotSpeed);
        drive_snap = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity)
                .withDeadband(0.08)
                .withRotationalDeadband(0.06 * SwerveConstants.maxRotSpeed);
        drive_snap.HeadingController = SwerveConstants.snapController;
        drive_snap.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        drive_snap.HeadingController.setTolerance(SwerveConstants.snapTolerance);
        auto_point = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity)
                .withDeadband(0.08)
                .withRotationalDeadband(0.06 * SwerveConstants.maxRotSpeed);
        drive_robot_centric = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity)
                .withDeadband(0.08)
                .withRotationalDeadband(0.06 * SwerveConstants.maxRotSpeed);
        
        this.driverXboxController = driverXboxController;

        if (Utils.isSimulation()) {
            startSimThread();
        }

        drivetrain.registerTelemetry(telem::telemeterize);
    }

    public enum WantedState {
        TELEOP_DRIVE,
        DRIVE_HALF_SPEED,
        CENTRIC_DRIVE,
        SNAP,
        AUTO_POINT;
        }

    private enum SystemState {
        TELEOP_DRIVE,
        DRIVE_HALF_SPEED,
        CENTRIC_DRIVE,
        SNAP,
        SNAP_POINT,
        AUTO_POINT;
        }

    // this handles simple, 1:1 transitions
    private SystemState handleStateTransitions() {
        return switch (wantedState) {
            case TELEOP_DRIVE -> SystemState.TELEOP_DRIVE;
            case DRIVE_HALF_SPEED -> SystemState.DRIVE_HALF_SPEED;
            case CENTRIC_DRIVE -> SystemState.CENTRIC_DRIVE;
            case SNAP -> SystemState.SNAP;
            case AUTO_POINT -> SystemState.AUTO_POINT;
         
        };
    }

    // this applies function based on systemState
    public void applyStates() {
        switch (systemState) {
            case TELEOP_DRIVE -> teleopDrive();
            case DRIVE_HALF_SPEED -> driveHalfSpeed();
            case CENTRIC_DRIVE -> centricDrive();
            case SNAP -> snap();
            case SNAP_POINT -> snapPoint();
            case AUTO_POINT -> autoPoint();

        }
    }

    public ChassisSpeeds getTeleopDriveSpeeds() {
        double xVariable = driverXboxController.getLeftX();
        double yVariable = driverXboxController.getLeftY();
        double mag = Math.hypot(xVariable, yVariable);
        double direction = Math.atan2(yVariable, xVariable);

        double translationScaled = ControllerHelpers.getExponent(
                ControllerHelpers.getDeadbanded(mag, SwerveConstants.leftYDeadband),
                SwerveConstants.leftYExponent);
        double rotate = -1.0 * ControllerHelpers.getExponent(
                ControllerHelpers.getDeadbanded(driverXboxController.getRightX(), SwerveConstants.rightXDeadband),
                SwerveConstants.rightXExponent);

        double forward = Math.sin(direction) * translationScaled;
        double strafe = Math.cos(direction) * translationScaled;

        if (!FmsUtil.isRedAlliance()) {
            strafe *= -1.0;
            forward *= -1.0;
        }

        driverDesiredSpeeds.vxMetersPerSecond = forward;
        driverDesiredSpeeds.vyMetersPerSecond = strafe;
        driverDesiredSpeeds.omegaRadiansPerSecond = rotate;

        return driverDesiredSpeeds;

    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public SystemState getState() {
        return systemState;
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

    /* need to add reset gyro method */

    public void setSnapPoint(Translation2d snapPoint) {
        // sets the point to look at while in snap
        this.snapPoint = snapPoint;
    }

    public void setSnapAngle(Rotation2d snapAngle) {
        this.snapAngle = snapAngle;

    }

    public void setDriveToPose(Pose2d targetPose, double translationToleranceMeters, double maxSpeed,
            double rotationToleranceDegrees, double maxAngularSpeed) {
        this.driveToPoseTargetPose = targetPose;
        this.driveToPoseTranslationToleranceMeters = translationToleranceMeters;
        this.driveToPoseMaxSpeed = maxSpeed;
        this.driveToPoseMaxAngularSpeed = maxAngularSpeed;
        this.driveToPoseRotationToleranceDegrees = rotationToleranceDegrees;
    }

    public void setDriveToFieldRelativeOffset(Transform2d vectorToMove, double translationToleranceMeters,
            double maxSpeed, double rotationToleranceDegrees, double maxAngularSpeed) {
        this.driveToPoseTargetPose = swerveDriveState.Pose.plus(vectorToMove);
        setDriveToPose(driveToPoseTargetPose, translationToleranceMeters, maxSpeed, rotationToleranceDegrees,
                maxAngularSpeed);
    }

    public void setDriveToRobotRelativeOffset(Transform2d vectorToMoveRobotFrame, double translationToleranceMeters,
            double maxSpeed, double rotationToleranceDegrees, double maxAngularSpeed) {
        this.driveToPoseTargetPose = swerveDriveState.Pose.plus(
                vectorToMoveRobotFrame.plus(new Transform2d(Translation2d.kZero, swerveDriveState.Pose.getRotation())));
        setDriveToPose(driveToPoseTargetPose, translationToleranceMeters, maxSpeed, rotationToleranceDegrees,
                maxAngularSpeed);
    }

    public void velocityDriveToPose(Pose2d target, double maxVelocity, double maxRVelocity,
            double atGoalTolerance, boolean isContinuous) {
        this.targetPose = target;
        this.maxVelocity = maxVelocity;
        this.maxRVelocity = maxRVelocity;
        this.isContinuous = isContinuous;
        this.atGoalTolerance = atGoalTolerance;
    }

    public boolean velocityAtGoal() {
        return atGoal;
    }

    // todo add distance to target

    public boolean isAtDriveToPoseSetpoint() {

        DogLog.log("Swerve/isAlignAtGoal/distance",
                getPose().getTranslation().getDistance(driveToPoseTargetPose.getTranslation()));
        DogLog.log("Swerve/isAlignAtGoal/angle",
                FieldUtil.angleBetweenRotation2ds(driveToPoseTargetPose.getRotation(), getPose().getRotation()));

        DogLog.log("Swerve/isAlignAtGoal/distanceBool",
                getPose().getTranslation()
                        .getDistance(driveToPoseTargetPose.getTranslation()) < driveToPoseTranslationToleranceMeters);
        DogLog.log("Swerve/isAlignAtGoal/angleBool",
                FieldUtil.angleBetweenRotation2ds(driveToPoseTargetPose.getRotation(),
                        getPose().getRotation()) < driveToPoseRotationToleranceDegrees);

        return getPose().getTranslation()
                .getDistance(driveToPoseTargetPose.getTranslation()) < driveToPoseTranslationToleranceMeters
                && FieldUtil.angleBetweenRotation2ds(driveToPoseTargetPose.getRotation(),
                        getPose().getRotation()) < driveToPoseRotationToleranceDegrees;

    }
    public void setSwerveRotationValue(double angle){
        foo = new Rotation2d(Units.degreesToRadians(angle));
    }
   public void setAutoDrivePose(Pose2d pose){
    foooo = pose;
   }

    public void resetPose(Pose2d pose) {
        startingPose = pose;
        drivetrain.resetPose(pose);
    }

    private void collectInputs() {
        if (SwerveConstants.useLimelight) {
            addVisionPosesToPoseEstimator();
        }
        robotSpeed = new Translation2d(swerveDriveState.Speeds.vxMetersPerSecond,
                swerveDriveState.Speeds.vyMetersPerSecond).getNorm();
        getTeleopDriveSpeeds();
        DogLog.log("Swerve/rotation",swerveDriveState.Pose.getRotation().getDegrees());
        DogLog.log("Swerve/swerveDriveState/ModuleStates", swerveDriveState.ModuleStates);
        DogLog.log("Swerve/swerveDriveState/EstimatedPose", swerveDriveState.Pose);
        DogLog.log("Swerve/swerveDriveState/Speeds", swerveDriveState.Speeds);
        DogLog.log("Swerve/TopSpeedPercent", currTopSpeedPercent);
        DogLog.log("Swerve/TopRotationSpeedPercent", currTopRotationSpeedPercent);
        DogLog.log("Swerve/TeleopDesiredSpeeds", driverDesiredSpeeds);
        DogLog.log("Swerve/SystemState", systemState.name());
        DogLog.log("Swerve/WantedState", wantedState.name());
        DogLog.log("Swerve/atGoal", atGoal);
        DogLog.log("Swerve/foooo", fooo.getAngle());


    }

    public void periodic() {
        collectInputs();
        systemState = handleStateTransitions();
        applyStates();

        swerveDriveState = drivetrain.getState();
        lastSwerveDriveState = swerveDriveState;
        swerveDriveState = drivetrain.getState();

        currentTime = Timer.getFPGATimestamp();

 

    }

    private void teleopDrive() {
        double joystickX = driverDesiredSpeeds.vxMetersPerSecond;
        double joystickY = driverDesiredSpeeds.vyMetersPerSecond;
        double joystickRot = driverDesiredSpeeds.omegaRadiansPerSecond;
            drivetrain
                    .setControl(drive_field_rel
                            .withVelocityX(joystickX * getRobotTopSpeed())
                            .withVelocityY(joystickY * getRobotTopSpeed())
                            .withRotationalRate(joystickRot * getRobotRotationSpeed()));


         
    }

    private void driveHalfSpeed() {
       double joystickX = driverDesiredSpeeds.vxMetersPerSecond;
       double joystickY = driverDesiredSpeeds.vyMetersPerSecond;
       double joystickRot = driverDesiredSpeeds.omegaRadiansPerSecond;
       drivetrain
                .setControl(drive_field_rel
                            .withVelocityX(joystickY * 0.5 * getRobotTopSpeed())
                            .withVelocityY(joystickX * 0.5 * getRobotTopSpeed())
                            .withRotationalRate(joystickRot * getRobotRotationSpeed()));


    }

    private void centricDrive() {
        double joystickX = driverDesiredSpeeds.vxMetersPerSecond;
        double joystickY = driverDesiredSpeeds.vyMetersPerSecond;
        double joystickRot = driverDesiredSpeeds.omegaRadiansPerSecond;
            drivetrain
                    .setControl(drive_robot_centric
                            .withVelocityX(joystickX * getRobotTopSpeed())
                            .withVelocityY(joystickY * getRobotTopSpeed())
                            .withRotationalRate(joystickRot * getRobotRotationSpeed()));

    }

    private void snap() {
        drivetrain.setControl(drive_snap
                .withVelocityY(driverDesiredSpeeds.vxMetersPerSecond * getRobotTopSpeed())
                .withVelocityX(driverDesiredSpeeds.vyMetersPerSecond * getRobotTopSpeed())
                .withTargetDirection(foo));
    }

    private void snapPoint() {
        drivetrain.setControl(drive_snap
                .withVelocityY(driverDesiredSpeeds.vxMetersPerSecond * getRobotTopSpeed())
                .withVelocityX(driverDesiredSpeeds.vyMetersPerSecond * getRobotTopSpeed())
                .withTargetDirection(dPadSnap));
    }

    private void autoPoint() {
         Pose2d currentPose = getPose();
         Translation2d error = foooo.getTranslation().minus(currentPose.getTranslation());
         //Pose2d currentPose = drivetrain.getState().Pose;
         if (error.getNorm() >= 1){
        drivetrain.setControl(auto_point
                .withVelocityY(error.getY())
                .withVelocityX(error.getX())
                .withTargetDirection(foooo.getRotation()));
                
                
         } else {
             drivetrain.setControl(auto_point
                .withVelocityY(driverDesiredSpeeds.vxMetersPerSecond * 0.0 * getRobotTopSpeed())
                .withVelocityX(driverDesiredSpeeds.vyMetersPerSecond * 0.0 * getRobotTopSpeed()));
         }

    };


    

    private void calibration() {
        // hi
        // how's it going?
    }

    public void addVisionPosesToPoseEstimator() {

        for (String limelightName : limelightNames) {

            LimelightHelpers.SetRobotOrientation(limelightName, swerveDriveState.Pose.getRotation().getDegrees(), 0,
                    0, 0,
                    0, 0);

            boolean isDisabled = DriverStation.isDisabled();
            if (isDisabled) {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 1);
                LimelightHelpers.SetThrottle(limelightName, 100);
            } else {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 0);
                LimelightHelpers.SetThrottle(limelightName, 0);
            }
            DogLog.log("Swerve/" + limelightName + "HeartBeat",
                    NetworkTableInstance.getDefault().getTable(limelightName).getEntry("hb").getDouble(0));
            DogLog.log("Swerve/" + limelightName + "TV", LimelightHelpers.getTV(limelightName));
            if (limelightName.equals("limelight-hp")) {

                continue;

            }
            PoseEstimate estimatePoseMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
            PoseEstimate estimatePoseMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
            PoseEstimate poseToAdd;

            if (estimatePoseMT1 == null || estimatePoseMT2 == null) {
                continue;
            }

            // This prevents pose estimator from having crazy poses if the Limelight loses
            // power
            if (estimatePoseMT2.tagCount == 0
                    || estimatePoseMT2.pose.getX() == 0.0 && estimatePoseMT2.pose.getY() == 0.0) {
                continue;
            }
            if (estimatePoseMT2.timestampSeconds == lastAddedVisionTimestampMap.getOrDefault(limelightName, 0.0)) {
                continue;
            }

            DogLog.log("Swerve/" + limelightName + "_avgTagArea", estimatePoseMT1.avgTagArea);
            if (estimatePoseMT1.avgTagArea >= (isDisabled ? 0.1 : 1.5) && !limelightName.equals("limelight-hp")) {

                poseToAdd = estimatePoseMT1;
                stdDevs = isDisabled ? SwerveConstants.megaTag1DisabledstdDev : SwerveConstants.megaTag1stdDev;

            } else {
                poseToAdd = estimatePoseMT2;
                stdDevs = SwerveConstants.megaTag2stdDev;
            }

            DogLog.log("Swerve/" + limelightName + "_PoseMT2", estimatePoseMT2.pose);
            DogLog.log("Swerve/" + limelightName + "_PoseMT1", estimatePoseMT1.pose);
            DogLog.log("Swerve/" + limelightName + "_lastAddedPose", poseToAdd.pose);

            drivetrain.addVisionMeasurement(poseToAdd.pose, Utils.fpgaToCurrentTime(poseToAdd.timestampSeconds),
                    stdDevs);
            lastAddedVisionTimestampMap.put(limelightName, Utils.fpgaToCurrentTime(poseToAdd.timestampSeconds));

        }

    }

    private void startSimThread() {
        System.out.println("Starting Sim thread");
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