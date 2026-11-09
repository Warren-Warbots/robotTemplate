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

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.driverstation.XboxController;
import frc.robot.swerve.generated.CompTunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.ControllerHelpers;
import frc.robot.util.FieldUtil;
import frc.robot.util.FmsUtil;

public class SwerveSubsystem {
    public TunerSwerveDrivetrain drivetrain;
    private SwerveRequest.FieldCentric drive_field_rel;
    private SwerveRequest.ApplyRobotVelocity drive_robot_rel;
    private SwerveRequest.FieldCentricFacingAngle drive_snap;
    private SwerveRequest.FieldCentricFacingAngle driveMaintainHeading;
    private SwerveRequest.RobotCentric drive_robot_centric;
    public WantedState wantedState = WantedState.TELEOP_DRIVE;
    private SystemState systemState = SystemState.TELEOP_DRIVE;
    private double currTopSpeedPercent = 1.0;
    private double currTopRotationSpeedPercent = 1.0;
    private ChassisVelocities driverDesiredSpeeds = new ChassisVelocities();
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

    public SwerveDriveState swerveDriveState = new SwerveDriveState(4);
    public SwerveDriveState lastSwerveDriveState = new SwerveDriveState(4);

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

    private String[] limelightNames = { "limelight" }; // add limelight code

    private HashMap<String, Double> lastAddedVisionTimestampMap = new HashMap<String, Double>();

    public SwerveSubsystem(XboxController driverXboxController) {

        drivetrain = new TunerSwerveDrivetrain(SwerveConstants.swerveDrivetrainConstants,
                SwerveConstants.FrontLeft,
                SwerveConstants.FrontRight,
                SwerveConstants.BackLeft,
                SwerveConstants.BackRight);

        drive_robot_rel = new SwerveRequest.ApplyRobotVelocity().withDriveRequestType(DriveRequestType.Velocity);
        drive_field_rel = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity)
                .withDeadband(0.08)
                .withRotationalDeadband(0.06 * SwerveConstants.maxRotSpeed);
        drive_snap = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity)
                .withDeadband(0.08)
                .withRotationalDeadband(0.06 * SwerveConstants.maxRotSpeed);
        drive_snap.HeadingController = SwerveConstants.snapController;
        drive_snap.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        drive_snap.HeadingController.setTolerance(SwerveConstants.snapTolerance);

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
        CALIBRATION,
        DRIVE_TO_POSE,
        DRIVE_WITH_VELOCITY,
        SNAP,
        SNAP_POINT;
    }

    private enum SystemState {
        TELEOP_DRIVE,
        CALIBRATION,
        DRIVE_TO_POSE,
        DRIVE_WITH_VELOCITY,
        SNAP,
        SNAP_POINT;
    }

    // this handles simple, 1:1 transitions
    private SystemState handleStateTransitions() {
        return switch (wantedState) {
            case TELEOP_DRIVE -> SystemState.TELEOP_DRIVE;
            case CALIBRATION -> SystemState.CALIBRATION;
            case DRIVE_TO_POSE -> SystemState.DRIVE_TO_POSE;
            case DRIVE_WITH_VELOCITY -> SystemState.DRIVE_WITH_VELOCITY;
            case SNAP -> SystemState.SNAP;
            case SNAP_POINT -> SystemState.SNAP_POINT;
        };
    }

    // this applies function based on systemState
    public void applyStates() {
        switch (systemState) {
            case TELEOP_DRIVE -> teleopDrive();
            case CALIBRATION -> calibration();
            case DRIVE_TO_POSE -> driveToPose();
            case DRIVE_WITH_VELOCITY -> driveWithVelocity();
            case SNAP -> snap();
            case SNAP_POINT -> snapPoint();
        }
    }

    public ChassisVelocities getTeleopDriveSpeeds() {
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

        driverDesiredSpeeds.vx = forward;
        driverDesiredSpeeds.vy = strafe;
        driverDesiredSpeeds.omega = rotate;

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

    public ChassisVelocities getRobotSpeeds() {
        return swerveDriveState.Velocity;
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
                vectorToMoveRobotFrame.plus(new Transform2d(Translation2d.ZERO, swerveDriveState.Pose.getRotation())));
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

    public void resetPose(Pose2d pose) {
        startingPose = pose;
        drivetrain.resetPose(pose);
    }

    private void collectInputs() {
        if (SwerveConstants.useLimelight) {
            addVisionPosesToPoseEstimator();
        }
        robotSpeed = new Translation2d(swerveDriveState.Velocity.vx,
                swerveDriveState.Velocity.vy).getNorm();
        getTeleopDriveSpeeds();
        DogLog.log("Swerve/swerveDriveState/ModuleStates", swerveDriveState.ModuleVelocities);
        DogLog.log("Swerve/swerveDriveState/EstimatedPose", swerveDriveState.Pose);
        DogLog.log("Swerve/swerveDriveState/Speeds", swerveDriveState.Velocity);
        DogLog.log("Swerve/TopSpeedPercent", currTopSpeedPercent);
        DogLog.log("Swerve/TopRotationSpeedPercent", currTopRotationSpeedPercent);
        DogLog.log("Swerve/TeleopDesiredSpeeds", driverDesiredSpeeds);
        DogLog.log("Swerve/SystemState", systemState.name());
        DogLog.log("Swerve/WantedState", wantedState.name());
        DogLog.log("Swerve/atGoal", atGoal);

    }

    public void periodic() {
        collectInputs();
        systemState = handleStateTransitions();
        applyStates();

        swerveDriveState = drivetrain.getState();
        lastSwerveDriveState = swerveDriveState;
        swerveDriveState = drivetrain.getState();

        currentTime = Timer.getTimestamp();

        if (Math.abs(driverDesiredSpeeds.omega) > SwerveConstants.rightXDeadband) {
            rotationJoystickLastTouched = currentTime;

            if (systemState == SystemState.SNAP || systemState == SystemState.SNAP_POINT) {
                setWantedState(WantedState.TELEOP_DRIVE);
            }

        }

        if (robotSpeed > 1) {
            highSpeedLastTime = currentTime;
        }
        if (systemState != SystemState.TELEOP_DRIVE) {
            lastMaintainHeadingAngle = Optional.empty();
        }

    }

    private void teleopDrive() {
        if (Math.abs(driverDesiredSpeeds.omega) > SwerveConstants.rightXDeadband
                || lastMaintainHeadingAngle.isEmpty()
                || ((currentTime - highSpeedLastTime) > 0.1)
                || ((currentTime - rotationJoystickLastTouched < 0.2))) {

            drivetrain
                    .setControl(drive_field_rel
                            .withVelocityX(driverDesiredSpeeds.vx * getRobotTopSpeed())
                            .withVelocityY(driverDesiredSpeeds.vy * getRobotTopSpeed())
                            .withRotationalRate(
                                    driverDesiredSpeeds.omega * getRobotRotationSpeed()));
            lastMaintainHeadingAngle = Optional.of(swerveDriveState.Pose.getRotation());

        } else {
            drivetrain.setControl(drive_snap
                    .withVelocityX(driverDesiredSpeeds.vx * getRobotTopSpeed())
                    .withVelocityY(driverDesiredSpeeds.vy * getRobotTopSpeed())
                    .withTargetDirection(lastMaintainHeadingAngle.get()));

        }

        timerHasBeenEnabled = false;
    }

    private void snap() {
        drivetrain.setControl(drive_snap
                .withVelocityX(driverDesiredSpeeds.vx * getRobotTopSpeed())
                .withVelocityY(driverDesiredSpeeds.vy * getRobotTopSpeed())
                .withTargetDirection(snapAngle));
    }

    private void snapPoint() {
        drivetrain.setControl(drive_snap
                .withVelocityX(driverDesiredSpeeds.vx * getRobotTopSpeed())
                .withVelocityY(driverDesiredSpeeds.vy * getRobotTopSpeed())
                .withTargetDirection(FieldUtil.getFieldRelativeAngleToPose(swerveDriveState.Pose, snapPoint)));
    }

    private void driveToPose() {
        Translation2d error = driveToPoseTargetPose.getTranslation()
                .minus(swerveDriveState.Pose.getTranslation());
        double distanceToGoal = error.getNorm();
        Rotation2d directionOfTravel = error.getAngle().orElse(Rotation2d.ZERO);
        double velocityOutput = 0.0;
        if (RobotState.isAutonomous()) {
            velocityOutput = Math.min(
                    Math.abs(SwerveConstants.autoDriveToPoseController.calculate(distanceToGoal, 0)),
                    driveToPoseMaxSpeed);
        } else {
            velocityOutput = Math.min(
                    Math.abs(SwerveConstants.teleopDriveToPoseController.calculate(distanceToGoal, 0)),
                    driveToPoseMaxSpeed);
        }
        double xComponent = velocityOutput * directionOfTravel.getCos();
        double yComponent = velocityOutput * directionOfTravel.getSin();

        DogLog.log("Swerve/DriveToPoint/xVelocitySetpoint", xComponent);
        DogLog.log("Swerve/DriveToPoint/yVelocitySetpoint", yComponent);
        DogLog.log("Swerve/DriveToPoint/velocityOutput", velocityOutput);
        DogLog.log("Swerve/DriveToPoint/linearDistance", distanceToGoal);
        DogLog.log("Swerve/DriveToPoint/directionOfTravel", directionOfTravel);
        DogLog.log("Swerve/DriveToPoint/desiredPoint", driveToPoseTargetPose);

        drivetrain.setControl(drive_snap
                .withVelocityX(xComponent)
                .withVelocityY(yComponent)
                .withTargetDirection(driveToPoseTargetPose.getRotation())
                .withMaxAbsRotationalRate(driveToPoseMaxAngularSpeed));
    }

    private void driveWithVelocity() {
        Pose2d currentPose = getPose();
        Translation2d difference = targetPose.getTranslation().minus(currentPose.getTranslation());
        if (isContinuous) {
            tranlationMag = maxVelocity;
        } else {
            tranlationMag = Math
                    .abs(SwerveConstants.autoDriveToPoseController.calculate(difference.getNorm(), 0));

        }
        tranlationMag = Math.min(tranlationMag, maxVelocity);

        diffRotation = difference.getAngle().orElse(Rotation2d.ZERO);
        xVelocity = tranlationMag * diffRotation.getCos();
        yVelocity = tranlationMag * diffRotation.getSin();
        double xSlew = xVelocity;
        double ySlew = yVelocity;

        drivetrain.setControl(drive_snap
                .withVelocityX(xSlew)
                .withVelocityY(ySlew)
                .withCenterOfRotation(swerveCOR)
                .withTargetDirection(targetPose.getRotation())
                .withMaxAbsRotationalRate(maxRVelocity));
        atGoal = (targetPose.minus(getPose())).getTranslation().getNorm() < atGoalTolerance; // dont double
                                                                                             // calc
        // This function is used for when following a path it maintains our swerve
        // velocity thoughout the path
        // isContinous is what set to know which points to stop at and which to maintian
        // out velocity
    }

    private void calibration() {
        // hi
        // how's it going?
        // very well hbu?
    }

    public void addVisionPosesToPoseEstimator() {
        // add vision code

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