package frc.robot.constants;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;

public class Constants {

    public static boolean IS_COMP = false;

    public static final String BETA_SERIAL_NUMBER = "-";// 0329F366
    public static final String SERIAL_NUMBER = System.getenv("serialnum");

    public static String CANBUS_NAME = "rio";

    public static final ClosedLoopRampsConfigs CLOSED_LOOP_RAMP = new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(0.04)
            .withTorqueClosedLoopRampPeriod(0.04)
            .withVoltageClosedLoopRampPeriod(0.04);
    public static final OpenLoopRampsConfigs OPEN_LOOP_RAMP = new OpenLoopRampsConfigs()
            .withDutyCycleOpenLoopRampPeriod(0.04)
            .withTorqueOpenLoopRampPeriod(0.04)
            .withVoltageOpenLoopRampPeriod(0.04);

    // LIGHTS
    public static final int Lights_ID = 55;
}
