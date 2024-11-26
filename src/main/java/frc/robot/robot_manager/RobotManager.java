// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lights.BlinkPattern;
import frc.robot.lights.LightsSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.FieldUtil;

public class RobotManager extends SubsystemBase {
    public RobotState state = RobotState.STOW_HAS_GP;
    public RobotState lastState = RobotState.STOW_NO_GP;
    public SwerveSubsystem swerve;
    private LightsSubsystem lights;

    public RobotManager(SwerveSubsystem swerve, LightsSubsystem lights) {
        this.swerve = swerve;
        this.lights = lights;
        
    }

    public void setState(RobotState state) {
        this.state = state;
    }
    public Command setModeCommand(RobotState state){
      return setModeCommand(state, false);
    }
    public Command setModeCommand(RobotState state,boolean isAuto) {
      // in auto, we want the snap command to end as soon as we are in a non snap state
      // in teleop, we want it to stick around (aka dont run disable snap) so that the snap doesnt cancel itself if the driver doesnt tell it to do something else
      if (state.isSnapState()){
        return Commands.runOnce(()-> setState(state)).andThen(swerve.enableSnap());
      } else if (isAuto) {
        return Commands.runOnce(() -> setState(state)).andThen(swerve.disableSnap());
      } else {
        return Commands.runOnce(() -> setState(state));
      }
    }

   

    public Command waitForStateCommand(RobotState waitState) {
        return Commands.waitUntil(() -> this.state == waitState);
    }

  @Override
  public void periodic() {
    DogLog.log("Robot/state",state);
    
    switch (state){
      case SPEAKER_SHOOTING:
        swerve.setSnapAngle(FieldUtil.getFieldRelativeAngleToPose(swerve.getPose(), FieldUtil.getSpeakerPose()));
        break;
      case STOW_HAS_GP:
        break;
      case STOW_NO_GP:
        break;
      case AMP:
        swerve.setSnapAngle(FieldUtil.getAmpAngle());
        break;
      case INTAKING:
        break;
      default:
        break;
      
    }
    if (lastState!=state){
      lights.setColor(state.getLedColor(), state.getBlinkPattern());
      swerve.setRobotTopSpeeds(state.getRobotTopSpeed(),state.getRobotTopRotationalSpeed());
    }
    lastState = state;

  }
}
