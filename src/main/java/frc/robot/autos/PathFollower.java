package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.FmsUtil;

import java.util.Arrays;
import java.util.List;

public class PathFollower {
    private final RobotManager manager;
    private final List<Pose2d> waypoints;
    private double maxDriveVelocity = 2.5;
    private double maxRotateVelocity = 3.5;
    private double atGoalTolerance = 1.0;
    private double timeout_s = 300;
    private boolean isContinuous = false;
    private boolean isMirrored = false;

    private int currentWaypointIndex = 0;
    private boolean hasStartedCurrentWaypoint = false;

    // Field dimensions for flipping (2024/2026 standard)
    private static final double FIELD_LENGTH = 16.541;
    private static final double FIELD_WIDTH = 8.211;

    public PathFollower(RobotManager manager, Pose2d... waypoints) {
        this.manager = manager;
        this.waypoints = Arrays.asList(waypoints);
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
    }

    /**
     * Called continuously. Returns true when the entire path is complete.
     */
    public boolean run() {
        if (currentWaypointIndex >= waypoints.size()) {
            return true;
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

        return currentWaypointIndex >= waypoints.size();
    }

    public static Pose2d applyFlipping(Pose2d pose, boolean isMirrored) {
        double x = pose.getX();
        double y = pose.getY();
        Rotation2d rot = pose.getRotation();

        if (isMirrored) {
            y = FIELD_WIDTH - y;
            rot = Rotation2d.fromDegrees(-rot.getDegrees());
        }

        if (FmsUtil.isRedAlliance()) {
            x = FIELD_LENGTH - x;
            rot = Rotation2d.fromDegrees(180 - rot.getDegrees());
        }

        return new Pose2d(x, y, rot);
    }
}