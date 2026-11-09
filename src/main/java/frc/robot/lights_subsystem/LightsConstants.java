// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights_subsystem;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import org.wpilib.util.Color;

/** Add your docs here. */
public class LightsConstants {

        public static CANdleConfiguration candleConfig = new CANdleConfiguration().withLED(
                        new LEDConfigs().withStripType(StripTypeValue.GRBW)
                                        .withBrightnessScalar(1));
        private static int START_LED = 0;
        private static int END_LED = 136;

        public static SolidColor blue = new SolidColor(START_LED, END_LED)
                        .withColor(new RGBWColor(Color.MIDNIGHT_BLUE));
        public static SolidColor white = new SolidColor(START_LED, END_LED).withColor(new RGBWColor(Color.WHITE));
        public static SolidColor pink = new SolidColor(START_LED, END_LED).withColor(new RGBWColor(Color.DARK_RED));

        public static RainbowAnimation rainbow = new RainbowAnimation(START_LED, END_LED)
                        .withDirection(AnimationDirectionValue.Forward);

}
