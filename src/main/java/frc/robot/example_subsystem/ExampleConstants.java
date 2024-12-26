// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.example_subsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants;

/** Add your docs here. */
public class ExampleConstants {

    public static int intakeMotorId = 19;


    //DO NOT JUST COPY PASTE THIS FOR A SUBSYSTEM, this is an example of how to build a config object. 
    // each subsystem needs its own config. Just copy "public static TalonFXConfiguration <name of motor> = new TalonFXConfiguration()" and add configs as needed
    public static TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration()
    .withCurrentLimits(
                new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(85)
                .withStatorCurrentLimit(45)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimitEnable(true)
        )
        .withMotorOutput(
                new MotorOutputConfigs()
                .withDutyCycleNeutralDeadband(0.05)
        )
        .withOpenLoopRamps(Constants.OPEN_LOOP_RAMP)
        .withClosedLoopRamps(Constants.CLOSED_LOOP_RAMP);

    public static VoltageOut intakeVoltageOut = new VoltageOut(0.0);
    

}
