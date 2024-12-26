// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.example_subsystem;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldUtil;
import frc.robot.util.TalonFxUtils;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  ExampleState state = ExampleState.STOW;
  ExampleState lastState = ExampleState.STOW;
  private double timestampAtSetState = Timer.getFPGATimestamp();
  TalonFX intakeMotor;

  public ExampleSubsystem() {
    // initialize motors here
    // step 1 is make config object for each motor in subsystem constants folder
    // step 2 is to use configure talon function to apply config to that motor
    intakeMotor = new TalonFX(ExampleConstants.intakeMotorId);
    TalonFxUtils.configureTalon(intakeMotor, ExampleConstants.intakeMotorConfig);
  }

  @Override
  public void periodic() {
    // This is where your state machine lives
    double timeInState = Timer.getFPGATimestamp() - timestampAtSetState;

    if (lastState != state) {
      // this stuff runs once the first time you go into a state, if you need
      // something to update continuously, like aiming the drivebase based on
      // position, put it in the main state machine
      // if you need it to happen once, put it here
      switch (state) {
        case INTAKING:
          intakeMotor.setControl(ExampleConstants.intakeVoltageOut.withOutput(1.0));
          break;
        case OUTTAKING:
          intakeMotor.setControl(ExampleConstants.intakeVoltageOut.withOutput(-1.0));

        default:
          break;
      }
    }

    switch (state) {

      case STOW:
        intakeMotor.setControl(ExampleConstants.intakeVoltageOut.withOutput(0.0));
        break;
      case INTAKING:

        break;
      case OUTTAKING:

        if (timeInState > 10) {
          state = ExampleState.STOW;
        }
        break;
      default:
        break;

    }
    lastState = state;

  }

}
