package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.FieldUtil;
import frc.robot.util.FmsUtil;

import java.util.Arrays;
import java.util.List;

public class PathFollower {
    private final RobotManager manager;
    private List<Pose2d> waypoints = List.of();
    private Pose2d[] lastSetPath = null;
    private double maxDriveVelocity = 2.5;
    private double maxRotateVelocity = 3.5;
    private double atGoalTolerance = 1.0;
    // every path gets a timeout so a stuck path can't hang the rest of auto.
    // use withTimeout() if a path needs more (or less) time than this.
    private double timeout_s = 10;
    private boolean isContinuous = false;
    private boolean isMirrored = false;

    private int currentWaypointIndex = 0;
    private boolean hasStartedCurrentWaypoint = false;
    private Timer timeoutTimer = new Timer();
    private boolean warnedNoPath = false;

    public PathFollower(RobotManager manager, Pose2d... waypoints) {
        this.manager = manager;
        if (waypoints.length > 0) {
            setPath(waypoints);
        }
    }

    /**
     * Gives the follower a path to drive. Safe to call every loop: handing it
     * the SAME array does nothing, handing it a DIFFERENT array starts the new
     * path from its first waypoint (with all settings back at their defaults,
     * so each state fully describes its own path).
     *
     * Because "new path" means "different array", do not drive the same array
     * in two back-to-back states - the follower can't tell that's a new run.
     */
    public PathFollower setPath(Pose2d... newWaypoints) {
        if (newWaypoints != lastSetPath) {
            reset();
            this.waypoints = Arrays.asList(newWaypoints);
            lastSetPath = newWaypoints;
            // back to defaults so settings from the previous path can't leak
            // into this one
            maxDriveVelocity = 2.5;
            maxRotateVelocity = 3.5;
            atGoalTolerance = 1.0;
            timeout_s = 10;
            isContinuous = false;
            isMirrored = false;
        }
        return this;
    }

    public PathFollower withMaxVelocity(double vel) {
        this.maxDriveVelocity = vel;
        return this;
    }

    public PathFollower withMaxRotateVelocity(double rotVel) {
        this.maxRotateVelocity = rotVel;
        return this;
    }

    public PathFollower withTolerance(double tol) {
        this.atGoalTolerance = tol;
        return this;
    }

    public PathFollower withTimeout(double timeout) {
        this.timeout_s = timeout;
        return this;
    }

    public PathFollower continuous(boolean isCont) {
        this.isContinuous = isCont;
        return this;
    }

    public PathFollower withMirror(boolean mirror) {
        this.isMirrored = mirror;
        return this;
    }

    public void reset() {
        currentWaypointIndex = 0;
        hasStartedCurrentWaypoint = false;
        timeoutTimer.stop();
        timeoutTimer.reset();
        // makes the next setPath() start fresh, even if it gets the same array
        lastSetPath = null;
    }

    /** True once the whole path has been driven (or the timeout gave up). */
    public boolean isDone() {
        return !waypoints.isEmpty() && currentWaypointIndex >= waypoints.size();
    }

    /**
     * Called every loop. Drives the path a little further each call - check
     * isDone() to see when the path is finished.
     */
    public void run() {
        if (waypoints.isEmpty()) {
            if (!warnedNoPath) {
                warnedNoPath = true;
                DriverStation.reportError("PathFollower has no path - call setPath() first", false);
            }
            return;
        }

        if (isDone()) {
            return;
        }

        // start() only does something the first time - the timer measures how
        // long this whole path has been running
        timeoutTimer.start();
        if (timeoutTimer.hasElapsed(timeout_s)) {
            DriverStation.reportError("PathFollower timed out after " + timeout_s + "s, skipping rest of path", false);
            currentWaypointIndex = waypoints.size();
            return;
        }

        Pose2d p = waypoints.get(currentWaypointIndex);
        p = applyFlipping(p, isMirrored);

        boolean cont = isContinuous || (currentWaypointIndex != waypoints.size() - 1);

        if (!hasStartedCurrentWaypoint) {
            manager.startVelocityDrivetoPose(p, maxDriveVelocity, maxRotateVelocity, atGoalTolerance, cont);
            hasStartedCurrentWaypoint = true;
        }

        if (manager.swerve.velocityAtGoal()) {
            currentWaypointIndex++;
            hasStartedCurrentWaypoint = false;
        }

        DogLog.log("Autos/PathFollower/waypointIndex", currentWaypointIndex);
        DogLog.log("Autos/PathFollower/targetPose", p);
        DogLog.log("Autos/PathFollower/timeRunning", timeoutTimer.get());
    }

    public static Pose2d applyFlipping(Pose2d pose, boolean isMirrored) {
        double x = pose.getX();
        double y = pose.getY();
        Rotation2d rot = pose.getRotation();

        if (isMirrored) {
            y = FieldUtil.FIELD_WIDTH - y;
            rot = Rotation2d.fromDegrees(-rot.getDegrees());
        }

        if (FmsUtil.isRedAlliance()) {
            x = FieldUtil.FIELD_LENGTH - x;
            rot = Rotation2d.fromDegrees(180 - rot.getDegrees());
        }

        return new Pose2d(x, y, rot);
    }
}
