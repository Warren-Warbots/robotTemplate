package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorButtonBox {
    int port;
    CommandJoystick joystick;

    public OperatorButtonBox(int port) {
        this.port = port;
        this.joystick = new CommandJoystick(port);
    }

    public Trigger L4_L_Button() {
        return joystick.button(1); // HANDLE
    }

    public Trigger L4_R_Button() {
        return joystick.button(2); // CRUISE
    }

    public Trigger L3_L_Button() {
        return joystick.button(3); // FLASH
    }

    public Trigger L3_R_Button() {
        return joystick.button(4); // AUDIO
    }

    public Trigger L2_L_Button() {
        return joystick.button(5); // WIPERS
    }

    public Trigger L2_R_Button() {
        return joystick.button(6); // MAP
    }

    public Trigger L1_L_Button() {
        return joystick.button(7); // LIGHT
    }

    public Trigger L1_R_Button() {
        return joystick.button(8); // TALK
    }

    public Trigger ENGINE_START() {
        return joystick.button(11); // ENGINE START
    }

    public Trigger ENTER() {
        return joystick.button(10); // ENTER
    }

    public Trigger MANUAL_INTAKE() {
        return joystick.button(13); // 1ST UP
    }

    public Trigger MANUAL_OUTTAKE() {
        return joystick.button(14); // 1ST DOWN
    }

    public Trigger FOURTH_UP() {
        return joystick.button(19); // 4TH UP
    }

    public Trigger FOURTH_DOWN() {
        return joystick.button(20); // 4TH DOWN
    }

    public Trigger ABS_DOWN() {
        return joystick.button(21); // ABS DOWN/LEFT
    }

    public Trigger ABS_UP() {
        return joystick.button(22); // ABS UP/RIGHT
    }

    public Trigger ABS_PRESS() {
        return joystick.button(22); // ABS PRESS
    }

    public Trigger TC_DOWN() {
        return joystick.button(23); // ABS DOWN/LEFT
    }

    public Trigger TC_UP() {
        return joystick.button(24); // ABS UP/RIGHT
    }

    public Trigger TC_PRESS() {
        return joystick.button(26); // ABS PRESS
    }

    public Trigger MANUAL_HANG_UP() {
        return joystick.pov(0); // JOYSTICK UP
    }

    public Trigger MANUAL_HANG_DOWN() {
        return joystick.pov(180); // JOYSTICK DOWN
    }

}