// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.example_intake_subsystem;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.util.TalonFxUtils;

public class IntakeSubsystem {
  /** Creates a new IntakeSubsystem. */

  public WantedState wantedState = WantedState.STOP;
  private SystemState systemState = SystemState.STOPPED;

  private double timestampAtSetState = Timer.getFPGATimestamp();

  TalonFX intakeMotor;
  CANrange canrange = new CANrange(Constants.intakeCANrangeId);

  public IntakeSubsystem() {
    /*
     * initialize motors here
     * step 1 is make config object for each motor in subsystem constants folder
     * step 2 is to use configure talon function to apply config to that motor
     * intakeMotor = new TalonFX(Constants.intake_Motor_ID);
     */
    intakeMotor = new TalonFX(Constants.intakeMotorId);
    TalonFxUtils.configureTalon(intakeMotor, IntakeConstants.intakeMotorConfig);
  }

  public enum WantedState {
    INTAKE,
    OUTTAKE,
    STOP;
  }

  private enum SystemState {
    INTAKING,
    OUTTAKING,
    STOPPED;
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;

  }

  public boolean getSensor() {
    return canrange.getIsDetected().getValue();

  }

  private void collectInputs() {
    getSensor();
    // this is where logging goes
    DogLog.log("ExampleIntakeSubsystem/hasGP", getSensor());
    DogLog.log("ExampleIntakeSubsystem/wantedState", wantedState.name());
    DogLog.log("ExampleIntakeSubsystem/systemState", systemState.name());
  }

  // this handles simple, 1:1 transitions (see robot manager for more complex
  // transitions)
  private SystemState handleStateTransitions() {
    return switch (wantedState) {
      case INTAKE -> SystemState.INTAKING;
      case OUTTAKE -> SystemState.OUTTAKING;
      case STOP -> SystemState.STOPPED;
    };
  }

  // this applies motor controls based on systemState (simple version)
  private void applyStates() {
    switch (systemState) {
      case INTAKING -> intakeMotor.setControl(IntakeConstants.intakeVoltageOut.withOutput(4.0));
      case OUTTAKING -> intakeMotor.setControl(IntakeConstants.intakeVoltageOut.withOutput(-4.0));
      case STOPPED -> intakeMotor.setControl(IntakeConstants.intakeVoltageOut.withOutput(0.0));
    }
  }

  /*
   * add functions IF NEEDED, try not to add too many.
   * some examples could be:
   * public functions so that other parts of the robot can check things like:
   * at Goal
   */

  public void periodic() {
    collectInputs();
    systemState = handleStateTransitions();
    applyStates();
    double timeInState = Timer.getFPGATimestamp() - timestampAtSetState;
  }

}
