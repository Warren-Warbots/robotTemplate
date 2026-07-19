// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.example_pivator_subsystem;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class PivatorConstants {

        /**
         * IDs go in global constants file, not here.
         * DO NOT JUST COPY PASTE THIS FOR A SUBSYSTEM, this is an example of how to
         * build a config object.
         * each subsystem needs its own config. Just copy "public static
         * TalonFXConfiguration <name of motor> = new TalonFXConfiguration()" and add
         * configs as needed
         */
        public static double minElevatorHeight = 0;
        public static double maxElevatorHeight = 31.375;
        public static double elevatorTolerance = 0.5;

        public static double minPivotRot = -0.1;
        public static double maxPivotRot = 0.7;
        public static double pivotTolerance = 1.0 / 360;

        public static TalonFXConfiguration elevatorFrontMotorConfig = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                        .withNeutralMode(NeutralModeValue.Brake)
                                        .withInverted(InvertedValue.CounterClockwise_Positive))
                        .withVoltage(new VoltageConfigs().withPeakForwardVoltage(12)
                                        .withPeakReverseVoltage(-12))
                        .withSlot0(new Slot0Configs()
                                        .withGravityType(GravityTypeValue.Elevator_Static)
                                        .withKP(1).withKI(0).withKD(1.0))
                        .withTorqueCurrent(new TorqueCurrentConfigs()
                                        .withPeakForwardTorqueCurrent(120)
                                        .withPeakReverseTorqueCurrent(-120))
                        .withMotionMagic(new MotionMagicConfigs()
                                        .withMotionMagicAcceleration(1000)
                                        .withMotionMagicCruiseVelocity(1000));

        public static TalonFXConfiguration elevatorBackMotorConfig = new TalonFXConfiguration()
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                        .withSupplyCurrentLimit(85)
                                        .withStatorCurrentLimit(45)
                                        .withSupplyCurrentLimitEnable(true)
                                        .withStatorCurrentLimitEnable(true))
                        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                                        .withInverted(InvertedValue.CounterClockwise_Positive));

        public static MotionMagicVoltage elevatorMotionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);

        public static TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration()
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                        .withSupplyCurrentLimit(85)
                                        .withStatorCurrentLimit(45)
                                        .withSupplyCurrentLimitEnable(true)
                                        .withStatorCurrentLimitEnable(true))
                        .withMotorOutput(new MotorOutputConfigs()
                                        .withNeutralMode(NeutralModeValue.Brake)
                                        .withInverted(InvertedValue.Clockwise_Positive))
                        .withVoltage(new VoltageConfigs().withPeakForwardVoltage(8)
                                        .withPeakReverseVoltage(-8))
                        .withSlot0(new Slot0Configs()
                                        .withGravityType(GravityTypeValue.Arm_Cosine)
                                        .withKP(0.45).withKI(0).withKD(0.8))
                        .withTorqueCurrent(new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(120)
                                        .withPeakReverseTorqueCurrent(-120))
                        .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true));

        public static PositionVoltage pivotPositionVoltage = new PositionVoltage(0.0).withSlot(0);

}
