// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerHelpers {
    public static double getDeadbanded(double joystickVal, double threshold) {
        double newJoystickVal = joystickVal;
        if (Math.abs(joystickVal) < threshold) {
            newJoystickVal = 0;
        } else {
            newJoystickVal = getSign(joystickVal) * (1 / (1 - threshold)) * (Math.abs(joystickVal) - threshold);
        }
        return newJoystickVal;
    }

    private static double getSign(double num) {
        return num >= 0 ? 1 : -1;
    }

    public static double getExponent(double joystickVal, double exponent) {
        return getSign(joystickVal) * Math.abs(Math.pow(Math.abs(joystickVal), exponent));
    }

    public static SequentialCommandGroup getVibrateOnceCommand(CommandXboxController controller) {
        return Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 1))
                .andThen(Commands.waitSeconds(0.1))
                .andThen(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0))
                .andThen(Commands.waitSeconds(0.1));
    }

    public static Command getVibrateCommand(CommandXboxController controller) {

        return getVibrateOnceCommand(controller)
                .andThen(getVibrateOnceCommand(controller))
                .andThen(getVibrateOnceCommand(controller))
                .andThen(getVibrateOnceCommand(controller));
    }

}