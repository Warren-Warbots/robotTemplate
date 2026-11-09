package frc.robot.util;

import org.wpilib.driverstation.Joystick;
import org.wpilib.driverstation.POVDirection;

public class OperatorButtonBox {
    int port;
    public Joystick joystick;

    public OperatorButtonBox(int port) {
        this.port = port;
        this.joystick = new Joystick(port);
    }

    public boolean L4_L_Button() {
        return joystick.getRawButton(0); // HANDLE
    }

    public boolean L4_R_Button() {
        return joystick.getRawButton(1); // CRUISE
    }

    public boolean L3_L_Button() {
        return joystick.getRawButton(2); // FLASH
    }

    public boolean L3_R_Button() {
        return joystick.getRawButton(3); // AUDIO
    }

    public boolean L2_L_Button() {
        return joystick.getRawButton(4); // WIPERS
    }

    public boolean L2_R_Button() {
        return joystick.getRawButton(5); // MAP
    }

    public boolean L1_L_Button() {
        return joystick.getRawButton(6); // LIGHT
    }

    public boolean L1_R_Button() {
        return joystick.getRawButton(7); // TALK
    }

    public boolean ENTER() {
        return joystick.getRawButton(9); // ENTER
    }

    public boolean ENGINE_START() {
        return joystick.getRawButton(10); // ENGINE START
    }

    public boolean REVERSE_SHOOTER_TOGGLE() {
        return joystick.getRawButton(12); // 1ST UP
    }

    public boolean MANUAL_OUTTAKE() {
        return joystick.getRawButton(13); // 1ST DOWN
    }

    public boolean FOURTH_UP() {
        return joystick.getRawButton(18); // 4TH UP
    }

    public boolean FOURTH_DOWN() {
        return joystick.getRawButton(19); // 4TH DOWN
    }

    public boolean ABS_DOWN() {
        return joystick.getRawButton(20); // ABS DOWN/LEFT
    }

    public boolean ABS_UP() {
        return joystick.getRawButton(21); // ABS UP/RIGHT
    }

    public boolean TC_DOWN() {
        return joystick.getRawButton(22); // TC DOWN/LEFT
    }

    public boolean TC_UP() {
        return joystick.getRawButton(23); // TC UP/RIGHT
    }

    public boolean ABS_PRESS() {
        return joystick.getRawButton(24); // ABS PRESS
    }

    public boolean TC_PRESS() {
        return joystick.getRawButton(25); // TC PRESS
    }

    public boolean MANUAL_HANG_UP() {
        return joystick.getPOV() == POVDirection.UP; // JOYSTICK UP
    }

    public boolean MANUAL_HANG_DOWN() {
        return joystick.getPOV() == POVDirection.DOWN; // JOYSTICK DOWN
    }

}