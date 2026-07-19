// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.example_pivator_subsystem.PivatorSubsystem;
import frc.robot.autos.ArcFollower;
import frc.robot.autos.PathFollower;
import frc.robot.example_intake_subsystem.IntakeSubsystem;
import frc.robot.lights_subsystem.LightsSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.FieldUtil;

public class RobotManager {
  public WantedRobotState wantedState = WantedRobotState.STOW;
  public CurrentRobotState currentState = CurrentRobotState.STOW;

  public SwerveSubsystem swerve;
  public LightsSubsystem lights;
  public PivatorSubsystem pivot;
  public IntakeSubsystem intake;
  private double timestampAtSetState = Timer.getFPGATimestamp();

  public boolean hasGP = false;

  // Managed Path Following
  private PathFollower currentPathFollower = null;
  private ArcFollower currentArcFollower = null;

  private Pose2d[] lastWaypoints = null;

  public RobotManager(SwerveSubsystem swerve, LightsSubsystem lights, PivatorSubsystem pivot, IntakeSubsystem intake) {
    this.swerve = swerve;
    this.lights = lights;
    this.pivot = pivot;
    this.intake = intake;

  }

  public void setWantedRobotState(WantedRobotState state) {
    DogLog.log("Robot/wantedState", state);
    timestampAtSetState = Timer.getFPGATimestamp();
    this.wantedState = state;

  }

  /**
   * Universal path driving method for Auto routines.
   * Automatically handles PathFollower instantiation and waypoint tracking.
   * Returns true when the entire array of points has been reached.
   */
  public boolean drivePath(Pose2d[] waypoints, double maxVel, double maxRotVel, double tolerance, boolean continuous,
      boolean mirror) {
    // If we've started a new path, reset the follower
    if (waypoints != lastWaypoints) {
      currentPathFollower = new PathFollower(this, waypoints)
          .withMaxVelocity(maxVel)
          .withMaxRotateVelocity(maxRotVel)
          .withTolerance(tolerance)
          .continuous(continuous)
          .withMirror(mirror);
      lastWaypoints = waypoints;
    }

    if (currentPathFollower == null)
      return true;
    return currentPathFollower.run();
  }

  public boolean driveArc(Pose2d[] waypoints, double maxVel, double maxRotVel, double tolerance, double addTurnDegrees,
      int nPoints, boolean continuous,
      boolean turnClockwise,
      boolean mirror) {
    // If we've started a new path, reset the follower
    if (waypoints != lastWaypoints) {
      currentArcFollower = new ArcFollower(this, waypoints)
          .withMaxVelocity(maxVel)
          .withMaxRotateVelocity(maxRotVel)
          .withTolerance(tolerance)
          .addTurnDegrees(addTurnDegrees)
          .addTurnPoints(nPoints)
          .continuous(continuous)
          .turnClockwise(turnClockwise)
          .withMirror(mirror);
      lastWaypoints = waypoints;
    }

    if (currentArcFollower == null)
      return true;
    return currentArcFollower.run();
  }

  /** Concise version for single-point driving in Auto */
  public boolean drivePoint(Pose2d point, double maxVel, double tolerance, boolean mirror) {
    return drivePath(new Pose2d[] { point }, maxVel, 3.5, tolerance, false, mirror);
  }

  public void startDriveToPose(Pose2d desiredPose, double translationToleranceMeters, double maxSpeed,
      double rotationToleranceDegrees, double maxAngularSpeed) {
    swerve.setDriveToPose(desiredPose, translationToleranceMeters, maxSpeed, rotationToleranceDegrees, maxAngularSpeed);
    swerve.setWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POSE);
  }

  public void startVelocityDrivetoPose(Pose2d targetPose, double maxVelocity, double maxRotateVelo,
      double atGoalTolerance,
      boolean isContinuous) {
    setWantedRobotState(WantedRobotState.DRIVE_WITH_VELOCITY);
    swerve.velocityDriveToPose(targetPose, maxVelocity, maxRotateVelo, atGoalTolerance, isContinuous);
  }

  public void periodic() {
    double timeInState = Timer.getFPGATimestamp() - timestampAtSetState;
    lights.setLightState(currentState);
    collectInputs();
    currentState = handleStateTransitions();
    applyStates();
    DogLog.log("Robot/currentState", currentState.name());
    DogLog.log("Robot/wantedstate", wantedState.name());

  }

  public void collectInputs() {
    DogLog.log("Robot/hasGP", intake.getSensor());
    hasGP = intake.getSensor();

  }

  private CurrentRobotState handleStateTransitions() {
    return switch (wantedState) {
      case STOW: {
        yield CurrentRobotState.STOW; // always go to stow when wanted state is stow.
      }
      case INTAKE: {
        yield CurrentRobotState.INTAKE;
      }
      case AUTO_SCORE_L4: {
        // this is where specific conditions/logic and/or safety code lives
        if (pivot.atPosition() && hasGP && swerve.isAtDriveToPoseSetpoint()) {
          yield CurrentRobotState.SCORE_L4;
        } else {
          yield CurrentRobotState.PREPARE_SCORE_L4;
        }
      }
      case DRIVE_WITH_VELOCITY: {
        yield CurrentRobotState.DRIVE_WITH_VELOCITY;
      }
    };
  }

  // this applies Function based on RobotState
  private void applyStates() {
    switch (currentState) {
      case STOW -> stow();
      case INTAKE -> intake();
      case PREPARE_SCORE_L4 -> prepareScoreL4();
      case SCORE_L4 -> scoreL4();
      case DRIVE_WITH_VELOCITY -> driveWithVelocity();
    }
    ;
  }

  private void stow() {
    intake.setWantedState(IntakeSubsystem.WantedState.STOP);
    pivot.setWantedState(PivatorSubsystem.WantedState.STOW);
    swerve.setWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE);

  }

  private void intake() {
    intake.setWantedState(IntakeSubsystem.WantedState.INTAKE);
    pivot.setWantedState(PivatorSubsystem.WantedState.STOW);

  }

  private void prepareScoreL4() {
    intake.setWantedState(IntakeSubsystem.WantedState.STOP);
    pivot.setWantedState(PivatorSubsystem.WantedState.LVL4);
    startDriveToPose(FieldUtil.getExamplePose(), 0.05, 3.0, 1, 2.0);
    // transition to actively scoring is handled in handleStateTransitions()
  }

  private void scoreL4() {
    intake.setWantedState(IntakeSubsystem.WantedState.OUTTAKE);
    pivot.setWantedState(PivatorSubsystem.WantedState.LVL4);
    if (!hasGP) {
      setWantedRobotState(WantedRobotState.STOW);
    }
  }

  private void driveWithVelocity() {
    swerve.setWantedState(SwerveSubsystem.WantedState.DRIVE_WITH_VELOCITY);
  }

}
