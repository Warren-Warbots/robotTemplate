// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.example_pivator_subsystem.PivatorSubsystem;
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

  public boolean hasGP = false;

  private Timer scoreTimer = new Timer();

  public RobotManager(SwerveSubsystem swerve, LightsSubsystem lights, PivatorSubsystem pivot, IntakeSubsystem intake) {
    this.swerve = swerve;
    this.lights = lights;
    this.pivot = pivot;
    this.intake = intake;

  }

  public void setWantedRobotState(WantedRobotState state) {
    DogLog.log("Robot/wantedState", state.name());
    this.wantedState = state;

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
    collectInputs();
    currentState = handleStateTransitions();
    applyStates();
    DogLog.log("Robot/currentState", currentState.name());
    DogLog.log("Robot/wantedState", wantedState.name());

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
      // if you add a state and forget a case here, this makes it scream in the
      // Driver Station instead of silently doing nothing
      default -> DriverStation.reportError("RobotManager has no behavior for state " + currentState, false);
    }
  }

  private void stow() {
    intake.setWantedState(IntakeSubsystem.WantedState.STOP);
    pivot.setWantedState(PivatorSubsystem.WantedState.STOW);
    swerve.setWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE);
    lights.setWantedState(LightsSubsystem.WantedState.IDLE);

  }

  private void intake() {
    intake.setWantedState(IntakeSubsystem.WantedState.INTAKE);
    pivot.setWantedState(PivatorSubsystem.WantedState.STOW);
    lights.setWantedState(LightsSubsystem.WantedState.INTAKING);

  }

  private void prepareScoreL4() {
    intake.setWantedState(IntakeSubsystem.WantedState.STOP);
    pivot.setWantedState(PivatorSubsystem.WantedState.LVL4);
    lights.setWantedState(LightsSubsystem.WantedState.PREPARING);
    startDriveToPose(FieldUtil.getExamplePose(), 0.05, 3.0, 1, 2.0);
    // restart the timer every loop while preparing - the moment we switch to
    // SCORE_L4 this stops running, so the timer measures how long we've been
    // scoring. this is how you time things in state logic.
    scoreTimer.restart();
    // transition to actively scoring is handled in handleStateTransitions()
  }

  private void scoreL4() {
    intake.setWantedState(IntakeSubsystem.WantedState.OUTTAKE);
    pivot.setWantedState(PivatorSubsystem.WantedState.LVL4);
    lights.setWantedState(LightsSubsystem.WantedState.SCORING);
    // score for at least half a second before stowing, even if the sensor says
    // the game piece already left (sensors can flicker)
    if (!hasGP && scoreTimer.hasElapsed(0.5)) {
      setWantedRobotState(WantedRobotState.STOW);
    }
  }

  private void driveWithVelocity() {
    swerve.setWantedState(SwerveSubsystem.WantedState.DRIVE_WITH_VELOCITY);
    lights.setWantedState(LightsSubsystem.WantedState.AUTO_DRIVING);
  }

}
