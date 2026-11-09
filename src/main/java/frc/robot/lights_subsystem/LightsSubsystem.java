// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights_subsystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANdle;
import dev.doglog.DogLog;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.system.Timer;
import frc.robot.Constants;
import frc.robot.robot_manager.CurrentRobotState;

public class LightsSubsystem {
  private CurrentRobotState state;
  private double timestampAtSetState = Timer.getTimestamp();
  CANdle candle;

  public LightsSubsystem() {
    candle = new CANdle(Constants.lightsId, new CANBus(CANPort.CAN_D0));
    candle.getConfigurator().apply(LightsConstants.candleConfig);
  }

  public void setLightState(CurrentRobotState robotState) {
    state = robotState;
  }

  public void periodic() {
    // This is where your state machine lives
    double timeInState = Timer.getTimestamp() - timestampAtSetState;
    DogLog.log("LightsSubsystem/state", state.name());

    switch (state) {

      case STOW:
        candle.setControl(LightsConstants.blue);
        break;
      case INTAKE:
        candle.setControl(LightsConstants.pink);
        break;
      case PREPARE_SCORE_L4:
        candle.setControl(LightsConstants.rainbow);
        break;
      case SCORE_L4:
        candle.setControl(LightsConstants.white);
        break;

    }

  }

}
