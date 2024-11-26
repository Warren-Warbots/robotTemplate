// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.lights.BlinkPattern;
import frc.robot.swerve.SwerveConstants;

/** Add your docs here. */
public enum RobotState {
    SPEAKER_SHOOTING(Color.kGreen,BlinkPattern.BLINK_FAST,true,4.0),
    AMP(Color.kBlue,BlinkPattern.SOLID,true,3.0),
    STOW_NO_GP(Color.kRed,BlinkPattern.SOLID),
    STOW_HAS_GP(Color.kOrange,BlinkPattern.BLINK_SLOW),
    INTAKING(Color.kAliceBlue,BlinkPattern.SOLID,false,3.0);


    private boolean snap;
    private double robotTopSpeed;
    private double robotTopRotationalSpeed;
    private Color ledColor;
    private BlinkPattern blinkPattern;

    RobotState(Color ledColor,BlinkPattern blinkPattern) {
        this.snap = false;
        this.ledColor = ledColor;
        this.blinkPattern = blinkPattern;
        this.robotTopSpeed = SwerveConstants.maxSpeed;
        this.robotTopRotationalSpeed = SwerveConstants.maxRotSpeed;
    }
    
    RobotState(Color ledColor,BlinkPattern blinkPattern,boolean snap) {
        this.snap = snap;
        this.ledColor = ledColor;
        this.blinkPattern = blinkPattern;
        this.robotTopSpeed = SwerveConstants.maxSpeed;
        this.robotTopRotationalSpeed = SwerveConstants.maxRotSpeed;
    }


    RobotState(Color ledColor,BlinkPattern blinkPattern,boolean snap, double robotTopSpeed) {
        this.snap = snap;
        this.ledColor = ledColor;
        this.blinkPattern = blinkPattern;
        this.robotTopSpeed = robotTopSpeed;
        this.robotTopRotationalSpeed = SwerveConstants.maxRotSpeed;
    }

    RobotState(Color ledColor,BlinkPattern blinkPattern,boolean snap,double robotTopSpeed,double robotTopRotationalSpeed) {
        this.snap = snap;
        this.ledColor = ledColor;
        this.blinkPattern = blinkPattern;
        this.robotTopSpeed = robotTopSpeed;
        this.robotTopRotationalSpeed = robotTopRotationalSpeed;
    }

    public boolean isSnapState() {
        return snap;
    }
    public double getRobotTopSpeed() {
        return robotTopSpeed;
    }

    public double getRobotTopRotationalSpeed() {
        return robotTopRotationalSpeed;
    }

    public Color getLedColor() {
        return ledColor;
    }

    public BlinkPattern getBlinkPattern() {
        return blinkPattern;
    }

}