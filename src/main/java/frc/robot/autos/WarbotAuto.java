package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.robot_manager.RobotManager;

public abstract class WarbotAuto {

    public Timer autoIntakeTimer = new Timer();
    protected RobotManager manager;
    protected static boolean mirror = false;
    protected boolean isFinished = false;

    // every auto shares one of each follower across its states. give them a
    // path with setPath() inside the state that drives it, and reset() them
    // in your init() so re-running the auto starts over
    protected PathFollower pathFollower;
    protected ArcFollower arcFollower;

    public abstract void init();

    public abstract void periodic();

    public boolean isFinished() {
        return isFinished;
    }

    public void setManager(RobotManager robotManager) {
        this.manager = robotManager;
        if (pathFollower == null) {
            pathFollower = new PathFollower(robotManager);
            arcFollower = new ArcFollower(robotManager);
        }
    }

    protected static Pose2d p(double x, double y, double degrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
    }

    protected static Translation2d t(double x, double y) {
        return new Translation2d(x, y);
    }

    private static boolean poseResetUsed = false;

    /**
     * Called by Autos at the start of each autonomous run, so the next
     * resetSwervePose() call works again.
     */
    public static void allowPoseReset() {
        poseResetUsed = false;
    }

    /**
     * Sets where the robot thinks it is on the field. Only the FIRST call of
     * each autonomous run actually happens - so when autos run in a sequence,
     * every auto can safely call this in its starting state, and autos after
     * the first automatically keep the pose where the previous auto ended.
     */
    public void resetSwervePose(Pose2d startingPose) {
        if (poseResetUsed) {
            return;
        }
        poseResetUsed = true;
        manager.swerve.resetPose(startingPose);

    }

    /**
     * Drives toward a single point. Safe to call every loop - no follower
     * object needed. Handles mirroring/alliance flipping for you.
     * Use a loose tolerance + continuous true to pass through the point
     * without slowing down, or a tight tolerance + continuous false to
     * stop at it. Check atDriveTarget() to know when you've arrived.
     */
    protected void driveTo(Pose2d target, double maxVelocity, double tolerance, boolean continuous) {
        manager.startVelocityDrivetoPose(PathFollower.applyFlipping(target, mirror),
                maxVelocity, 3.5, tolerance, continuous);
    }

    protected boolean atDriveTarget() {
        return manager.swerve.velocityAtGoal();
    }

}