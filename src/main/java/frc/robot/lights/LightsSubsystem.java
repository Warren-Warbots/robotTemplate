// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.controls.Follower;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class LightsSubsystem extends SubsystemBase {
    /**
     * The duration (in seconds) that lights should be on or off when in fast blink
     * mode.
     */
    private static final double FAST_BLINK_DURATION = 0.04;

    /**
     * The duration (in seconds) that lights should be on or off when in slow blink
     * mode.
     */
    private static final double SLOW_BLINK_DURATION = 0.25;

    private final CANdle candle;
    private final Timer blinkTimer = new Timer();
    private Color color = Color.kWhite;
    private BlinkPattern blinkPattern = BlinkPattern.SOLID;

    public LightsSubsystem(
            CANdle candle) {
        this.candle = candle;
        blinkTimer.start();
    }

    public void setColor(Color color, BlinkPattern blinkPattern) {
        this.color = color;
        this.blinkPattern = blinkPattern;
    }

    public void setColor(Color color) {
        this.color = color;
    }

    public Color getColor() {
        return color;
    }

    @Override
    public void periodic() {
        


        if (DriverStation.isDisabled()) {
            if ((DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)) {
                color = Color.kRed;
                blinkPattern = BlinkPattern.SOLID;
            } else {
                color = Color.kBlue;
                blinkPattern = BlinkPattern.SOLID;
            }
            // white= has note, red prepare shoot, blue shoot

        }

        Color8Bit color8Bit = new Color8Bit(color);

        if (blinkPattern == BlinkPattern.SOLID) {
            candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue);
        } else {
            double time = blinkTimer.get();
            double onDuration = 0;
            double offDuration = 0;

            if (blinkPattern == BlinkPattern.BLINK_FAST) {
                onDuration = FAST_BLINK_DURATION;
                offDuration = FAST_BLINK_DURATION * 2;
            } else if (blinkPattern == BlinkPattern.BLINK_SLOW) {
                onDuration = SLOW_BLINK_DURATION;
                offDuration = SLOW_BLINK_DURATION * 2;
            }

            if (time >= offDuration) {
                blinkTimer.reset();
                candle.setLEDs(0, 0, 0);
            } else if (time >= onDuration) {
                candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue);
            }
        }

        DogLog.log("Lights/Color", color.toString());
        DogLog.log("Lights/BlinkPattern", blinkPattern.toString());
    }
}