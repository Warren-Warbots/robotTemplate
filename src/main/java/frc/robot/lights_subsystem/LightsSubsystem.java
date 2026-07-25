// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights_subsystem;

import com.ctre.phoenix6.hardware.CANdle;
import dev.doglog.DogLog;
import frc.robot.Constants;

public class LightsSubsystem {
  /**
   * Shows robot status on the LED strip. This is the simplest example of the
   * subsystem pattern: no sensors, no closed loop, one output device.
   */

  public WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;

  CANdle candle;

  public LightsSubsystem() {
    candle = new CANdle(Constants.lightsId);
    candle.getConfigurator().apply(LightsConstants.candleConfig);
  }

  // states are named for what the robot is doing, the color for each one is
  // picked in applyStates()
  public enum WantedState {
    IDLE,
    INTAKING,
    PREPARING,
    SCORING,
    AUTO_DRIVING;
  }

  private enum SystemState {
    IDLE,
    INTAKING,
    PREPARING,
    SCORING,
    AUTO_DRIVING;
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;

  }

  private void collectInputs() {
    // this is where logging goes
    DogLog.log("LightsSubsystem/wantedState", wantedState.name());
    DogLog.log("LightsSubsystem/systemState", systemState.name());
  }

  // this handles simple, 1:1 transitions (see robot manager for more complex
  // transitions)
  private SystemState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> SystemState.IDLE;
      case INTAKING -> SystemState.INTAKING;
      case PREPARING -> SystemState.PREPARING;
      case SCORING -> SystemState.SCORING;
      case AUTO_DRIVING -> SystemState.AUTO_DRIVING;
    };
  }

  // this applies light patterns based on systemState
  private void applyStates() {
    switch (systemState) {
      case IDLE -> candle.setControl(LightsConstants.blue);
      case INTAKING -> candle.setControl(LightsConstants.pink);
      case PREPARING -> candle.setControl(LightsConstants.rainbow);
      case SCORING -> candle.setControl(LightsConstants.white);
      case AUTO_DRIVING -> candle.setControl(LightsConstants.green);
    }
  }

  public void periodic() {
    collectInputs();
    systemState = handleStateTransitions();
    applyStates();
  }

}
