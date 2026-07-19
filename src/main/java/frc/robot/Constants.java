package frc.robot;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;

public class Constants {
        public static boolean IS_AT_COMP = false;

        public static final String BETA_SERIAL_NUMBER = "-"; // 0329F366
        public static final String SERIAL_NUMBER = System.getenv("serialnum");

        public static final boolean IS_COMP_BOT = true;

        public static String CANBUS_NAME = "rio";

        public static final ClosedLoopRampsConfigs CLOSED_LOOP_RAMP = new ClosedLoopRampsConfigs()
                        .withDutyCycleClosedLoopRampPeriod(0.04)
                        .withTorqueClosedLoopRampPeriod(0.04)
                        .withVoltageClosedLoopRampPeriod(0.04);
        public static final OpenLoopRampsConfigs OPEN_LOOP_RAMP = new OpenLoopRampsConfigs()
                        .withDutyCycleOpenLoopRampPeriod(0.04)
                        .withTorqueOpenLoopRampPeriod(0.04)
                        .withVoltageOpenLoopRampPeriod(0.04);
        // SWERVE
        // Note: swerve IDs will generate in swerve CompTunerConstants, but should be
        // moved here for convienence
        public static final int kPigeonId = 1;
        public static final int kFrontLeftSteerMotorId = 2;
        public static final int kFrontLeftDriveMotorId = 3;
        public static final int kFrontLeftEncoderId = 4;
        public static final int kFrontRightSteerMotorId = 5;
        public static final int kFrontRightDriveMotorId = 6;
        public static final int kFrontRightEncoderId = 7;
        public static final int kBackLeftSteerMotorId = 8;
        public static final int kBackLeftDriveMotorId = 9;
        public static final int kBackLeftEncoderId = 10;
        public static final int kBackRightSteerMotorId = 11;
        public static final int kBackRightDriveMotorId = 12;
        public static final int kBackRightEncoderId = 13;

        // Intake
        public static final int intakeMotorId = 20;
        public static final int intakeCANrangeId = 21;

        // Pivator
        public static final int pivotMotorId = 30;
        public static final int pivotCANcoderId = 31;

        // Elevator
        public static final int elevatorFrontMotorId = 40;
        public static final int elevatorBackMotorId = 41;

        // Motor IDs here
        public static final int lightsId = 55;

}
