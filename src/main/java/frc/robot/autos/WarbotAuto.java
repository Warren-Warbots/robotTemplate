package frc.robot.autos;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.Timer;
import frc.robot.robot_manager.RobotManager;

public abstract class WarbotAuto {

    public Timer autoIntakeTimer = new Timer();
    protected RobotManager manager;
    protected static boolean mirror = false;
    protected boolean isFinished = false;

    public abstract void init();

    public abstract void periodic();

    public boolean isFinished() {
        return isFinished;
    }

    public void setManager(RobotManager robotManager) {
        this.manager = robotManager;
    }

    protected static Pose2d p(double x, double y, double degrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
    }

    protected static Translation2d t(double x, double y) {
        return new Translation2d(x, y);
    }

    public void resetSwervePose(Pose2d startingPose) {
        manager.swerve.resetPose(startingPose);

    }

}