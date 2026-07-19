package frc.robot.simulation;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Class to keep all the mechanism-specific objects together and out of the main
 * example
 */

public class SimMech {
        private final double HEIGHT = 100; // Controls the height of the mech2d SmartDashboard
        private final double WIDTH = 100; // Controls the height of the mech2d SmartDashboard

        public SimMech() {
        }

        Mechanism2d mech2d = new Mechanism2d(WIDTH, HEIGHT);

        MechanismLigament2d right_elevatorStick = mech2d
                        .getRoot("right_elevatorStick Root", 49, 0)
                        .append(new MechanismLigament2d("right_elevatorStick",
                                        10, 90, 6, new Color8Bit(Color.kCyan)));

        MechanismLigament2d tippy_top_elevatorStick = right_elevatorStick
                        .append(new MechanismLigament2d("tippy_top_elevatorStick", 9, 90, 6, new Color8Bit(
                                        Color.kGold)));

        MechanismLigament2d left_elevatorStick = mech2d
                        .getRoot("leftElevatorStick root", 40, 0)
                        .append(new MechanismLigament2d("leftElevatorStick",
                                        10, 90, 6, new Color8Bit(Color.kCyan)));

        MechanismLigament2d pivotArm = right_elevatorStick
                        .append(new MechanismLigament2d("notPivotArm", 1, 270, 6, new Color8Bit(Color.kGreen)))
                        .append(new MechanismLigament2d("pivotArm", 25, 0, 6, new Color8Bit(Color.kGreen)));

        MechanismLigament2d drivetrain = mech2d
                        .getRoot("drivebase", 30, 2)
                        .append(new MechanismLigament2d("drive", 34, 0, 12, new Color8Bit(Color.kDarkRed)));

        public double wrapAngle(double angle) {
                return ((angle + 180) % 360 + 360) % 360 - 180;

        }

        public double wrapTo0_2PI(double angle) {
                return (angle % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);

        }

        public void updatePivot(StatusSignal<Angle> angle, StatusSignal<Angle> height) {
                // pivotArm.setLength(25);
                pivotArm.setAngle(wrapAngle(angle.getValueAsDouble() * 360));

                right_elevatorStick.setLength(height.getValueAsDouble());
                left_elevatorStick.setLength(height.getValueAsDouble());

                SmartDashboard.putData("mech2d", mech2d); // Creates mech2d in SmartDashboard
        }
}
