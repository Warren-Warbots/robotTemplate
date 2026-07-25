package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;

public class OperatorButtonBox {
    /**
     * Wraps the operator button box (a repurposed car dashboard panel) and gives
     * every button a human readable name, the same way XboxController does.
     *
     * getXxxButton() is true the whole time the button is held.
     * getXxxButtonPressed() is true only on the one loop where it first goes down.
     *
     * Poll these in Robot.teleopPeriodic, for example:
     * if (buttonBox.getCruiseButtonPressed()) {
     * manager.setWantedRobotState(WantedRobotState.STOW);
     * }
     */

    Joystick joystick;

    // button numbers, named for the physical label printed on the box
    private static final int HANDLE = 1;
    private static final int CRUISE = 2;
    private static final int FLASH = 3;
    private static final int AUDIO = 4;
    private static final int WIPERS = 5;
    private static final int MAP = 6;
    private static final int LIGHT = 7;
    private static final int TALK = 8;
    private static final int ENTER = 10;
    private static final int ENGINE_START = 11;
    private static final int FIRST_UP = 13;
    private static final int FIRST_DOWN = 14;
    private static final int FOURTH_UP = 19;
    private static final int FOURTH_DOWN = 20;
    private static final int ABS_DOWN = 21;
    private static final int ABS_UP = 22;
    private static final int ABS_PRESS = 25; // verify on the real box before relying on this
    private static final int TC_DOWN = 23;
    private static final int TC_UP = 24;
    private static final int TC_PRESS = 26;

    public OperatorButtonBox(int port) {
        this.joystick = new Joystick(port);
    }

    public boolean getHandleButton() {
        return joystick.getRawButton(HANDLE);
    }

    public boolean getHandleButtonPressed() {
        return joystick.getRawButtonPressed(HANDLE);
    }

    public boolean getCruiseButton() {
        return joystick.getRawButton(CRUISE);
    }

    public boolean getCruiseButtonPressed() {
        return joystick.getRawButtonPressed(CRUISE);
    }

    public boolean getFlashButton() {
        return joystick.getRawButton(FLASH);
    }

    public boolean getFlashButtonPressed() {
        return joystick.getRawButtonPressed(FLASH);
    }

    public boolean getAudioButton() {
        return joystick.getRawButton(AUDIO);
    }

    public boolean getAudioButtonPressed() {
        return joystick.getRawButtonPressed(AUDIO);
    }

    public boolean getWipersButton() {
        return joystick.getRawButton(WIPERS);
    }

    public boolean getWipersButtonPressed() {
        return joystick.getRawButtonPressed(WIPERS);
    }

    public boolean getMapButton() {
        return joystick.getRawButton(MAP);
    }

    public boolean getMapButtonPressed() {
        return joystick.getRawButtonPressed(MAP);
    }

    public boolean getLightButton() {
        return joystick.getRawButton(LIGHT);
    }

    public boolean getLightButtonPressed() {
        return joystick.getRawButtonPressed(LIGHT);
    }

    public boolean getTalkButton() {
        return joystick.getRawButton(TALK);
    }

    public boolean getTalkButtonPressed() {
        return joystick.getRawButtonPressed(TALK);
    }

    public boolean getEnterButton() {
        return joystick.getRawButton(ENTER);
    }

    public boolean getEnterButtonPressed() {
        return joystick.getRawButtonPressed(ENTER);
    }

    public boolean getEngineStartButton() {
        return joystick.getRawButton(ENGINE_START);
    }

    public boolean getEngineStartButtonPressed() {
        return joystick.getRawButtonPressed(ENGINE_START);
    }

    public boolean getFirstUpButton() {
        return joystick.getRawButton(FIRST_UP);
    }

    public boolean getFirstUpButtonPressed() {
        return joystick.getRawButtonPressed(FIRST_UP);
    }

    public boolean getFirstDownButton() {
        return joystick.getRawButton(FIRST_DOWN);
    }

    public boolean getFirstDownButtonPressed() {
        return joystick.getRawButtonPressed(FIRST_DOWN);
    }

    public boolean getFourthUpButton() {
        return joystick.getRawButton(FOURTH_UP);
    }

    public boolean getFourthUpButtonPressed() {
        return joystick.getRawButtonPressed(FOURTH_UP);
    }

    public boolean getFourthDownButton() {
        return joystick.getRawButton(FOURTH_DOWN);
    }

    public boolean getFourthDownButtonPressed() {
        return joystick.getRawButtonPressed(FOURTH_DOWN);
    }

    public boolean getAbsDownButton() {
        return joystick.getRawButton(ABS_DOWN);
    }

    public boolean getAbsDownButtonPressed() {
        return joystick.getRawButtonPressed(ABS_DOWN);
    }

    public boolean getAbsUpButton() {
        return joystick.getRawButton(ABS_UP);
    }

    public boolean getAbsUpButtonPressed() {
        return joystick.getRawButtonPressed(ABS_UP);
    }

    public boolean getAbsPressButton() {
        return joystick.getRawButton(ABS_PRESS);
    }

    public boolean getAbsPressButtonPressed() {
        return joystick.getRawButtonPressed(ABS_PRESS);
    }

    public boolean getTcDownButton() {
        return joystick.getRawButton(TC_DOWN);
    }

    public boolean getTcDownButtonPressed() {
        return joystick.getRawButtonPressed(TC_DOWN);
    }

    public boolean getTcUpButton() {
        return joystick.getRawButton(TC_UP);
    }

    public boolean getTcUpButtonPressed() {
        return joystick.getRawButtonPressed(TC_UP);
    }

    public boolean getTcPressButton() {
        return joystick.getRawButton(TC_PRESS);
    }

    public boolean getTcPressButtonPressed() {
        return joystick.getRawButtonPressed(TC_PRESS);
    }

    // the little joystick on the box reports as a POV hat, not as buttons
    public boolean getJoystickUp() {
        return joystick.getPOV() == 0;
    }

    public boolean getJoystickDown() {
        return joystick.getPOV() == 180;
    }

}
