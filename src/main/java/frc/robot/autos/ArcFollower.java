package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.Circle;
import frc.robot.util.FieldUtil;
import frc.robot.util.FmsUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import dev.doglog.DogLog;

public class ArcFollower {
    private final RobotManager manager;
    private List<Pose2d> waypoints = List.of();
    private Pose2d[] lastSetPath = null;
    private double maxDriveVelocity = 2.5;
    private double maxRotateVelocity = 3.5;
    private double atGoalTolerance = 0.25;
    private double addTurnDegrees = 0.0;
    private int addTurnPoints = 5;

    // every arc gets a timeout so a stuck arc can't hang the rest of auto.
    // use withTimeout() if an arc needs more (or less) time than this.
    private double timeout_s = 10;
    private boolean isContinuous = false;
    private boolean turnClockwise = false;
    private boolean isMirrored = false;

    private int currentWaypointIndex = 0;
    private boolean hasStartedCurrentWaypoint = false;
    private Timer timeoutTimer = new Timer();
    private boolean warnedNoPath = false;
    // the interpolated arc points get built on the first run() call, after all
    // the withX() settings have been applied
    private ArrayList<Pose2d> wayPointsInterpolated = new ArrayList<>();

    public ArcFollower(RobotManager manager, Pose2d... waypoints) {
        this.manager = manager;
        if (waypoints.length > 0) {
            setPath(waypoints);
        }
    }

    /**
     * Gives the follower an arc to drive - exactly 3 waypoints (start, middle,
     * end). Safe to call every loop: handing it the SAME array does nothing,
     * handing it a DIFFERENT array starts the new arc from its beginning (with
     * all settings back at their defaults, so each state fully describes its
     * own arc).
     *
     * Because "new arc" means "different array", do not drive the same array
     * in two back-to-back states - the follower can't tell that's a new run.
     */
    public ArcFollower setPath(Pose2d... newWaypoints) {
        if (newWaypoints != lastSetPath) {
            reset();
            this.waypoints = Arrays.asList(newWaypoints);
            lastSetPath = newWaypoints;
            if (newWaypoints.length < 3) {
                DriverStation.reportError(
                        "ArcFollower needs 3 waypoints (start, middle, end) but got " + newWaypoints.length
                                + " - this arc will be skipped",
                        false);
            }
            // back to defaults so settings from the previous arc can't leak
            // into this one
            maxDriveVelocity = 2.5;
            maxRotateVelocity = 3.5;
            atGoalTolerance = 0.25;
            addTurnDegrees = 0.0;
            addTurnPoints = 5;
            timeout_s = 10;
            isContinuous = false;
            turnClockwise = false;
            isMirrored = false;
        }
        return this;
    }

    public ArcFollower withMaxVelocity(double vel) {
        this.maxDriveVelocity = vel;
        return this;
    }

    public ArcFollower withMaxRotateVelocity(double rotVel) {
        this.maxRotateVelocity = rotVel;
        return this;
    }

    public ArcFollower withTolerance(double tol) {
        this.atGoalTolerance = tol;
        return this;
    }

    public ArcFollower withTimeout(double timeout) {
        this.timeout_s = timeout;
        return this;
    }

    public ArcFollower continuous(boolean isCont) {
        this.isContinuous = isCont;
        return this;
    }

    public ArcFollower turnClockwise(boolean turnClockwise) {
        this.turnClockwise = turnClockwise;
        return this;
    }

    public ArcFollower withMirror(boolean mirror) {
        this.isMirrored = mirror;
        return this;
    }

    public ArcFollower addTurnDegrees(double turnDegrees) {
        this.addTurnDegrees = turnDegrees;
        return this;
    }

    public ArcFollower addTurnPoints(int nPoints) {
        this.addTurnPoints = nPoints;
        return this;
    }

    public void reset() {
        currentWaypointIndex = 0;
        hasStartedCurrentWaypoint = false;
        timeoutTimer.stop();
        timeoutTimer.reset();
        wayPointsInterpolated.clear();
        // makes the next setPath() start fresh, even if it gets the same array
        lastSetPath = null;
    }

    /** True once the whole arc has been driven (or the timeout gave up). */
    public boolean isDone() {
        if (waypoints.isEmpty()) {
            return false;
        }
        // a broken arc (fewer than 3 waypoints) counts as done so the auto
        // moves on - setPath already reported the error
        if (waypoints.size() < 3) {
            return true;
        }
        return !wayPointsInterpolated.isEmpty() && currentWaypointIndex >= wayPointsInterpolated.size();
    }

    // builds the arc points in un-flipped (blue, un-mirrored) coordinates.
    // mirroring/alliance flipping happens later in run(), through the same
    // applyFlipping() that PathFollower uses, so the math here never needs to
    // know about it.
    private void buildInterpolatedPoints() {
        Rotation2d r = new Rotation2d();
        Translation2d startPoint = waypoints.get(0).getTranslation();
        Translation2d midPoint = waypoints.get(1).getTranslation();
        Translation2d endPoint = waypoints.get(2).getTranslation();
        Circle c = new Circle(startPoint,
                midPoint,
                endPoint);
        DogLog.log("Autos/Arc/center", c.getCenter());
        DogLog.log("Autos/Arc/arcRadius", c.getRadius());
        Rotation2d startAngle = startPoint.minus(c.getCenter()).getAngle();
        Rotation2d midAngle = midPoint.minus(c.getCenter()).getAngle();
        Rotation2d endAngle = endPoint.minus(c.getCenter()).getAngle();

        Rotation2d angleDelta = midAngle.minus(startAngle);

        Rotation2d current = startAngle;
        Rotation2d angleIncrement = angleDelta.div(addTurnPoints);
        for (int w = 0; w <= addTurnPoints; w++) {

            Rotation2d thetaCenterToRobot = current;
            Rotation2d thetaDesiredP90 = new Rotation2d(thetaCenterToRobot.getCos(), thetaCenterToRobot.getSin());
            if (turnClockwise) {
                r = thetaDesiredP90.plus(Rotation2d.fromDegrees(90));
            } else {
                r = thetaDesiredP90.minus(Rotation2d.fromDegrees(90));
            }

            wayPointsInterpolated.add(new Pose2d(c.getPoint(current), r));
            current = current.plus(angleIncrement);
        }
        angleDelta = endAngle.minus(current);
        angleIncrement = angleDelta.div(addTurnPoints);
        for (int w = 0; w <= addTurnPoints; w++) {

            Rotation2d thetaCenterToRobot = current;
            Rotation2d thetaDesiredP90 = new Rotation2d(thetaCenterToRobot.getCos(), thetaCenterToRobot.getSin());
            if (turnClockwise) {
                r = thetaDesiredP90.plus(Rotation2d.fromDegrees(90));
            } else {
                r = thetaDesiredP90.minus(Rotation2d.fromDegrees(90));
            }

            wayPointsInterpolated.add(new Pose2d(c.getPoint(current), r));
            current = current.plus(angleIncrement);
        }
    }

    /**
     * Called every loop. Drives the arc a little further each call - check
     * isDone() to see when the arc is finished.
     */
    public void run() {
        if (waypoints.isEmpty()) {
            if (!warnedNoPath) {
                warnedNoPath = true;
                DriverStation.reportError("ArcFollower has no arc - call setPath() first", false);
            }
            return;
        }

        // a broken arc finishes instantly instead of crashing - setPath
        // already reported the error
        if (waypoints.size() < 3) {
            return;
        }

        if (wayPointsInterpolated.isEmpty()) {
            buildInterpolatedPoints();
        }

        if (isDone()) {
            return;
        }

        // start() only does something the first time - the timer measures how
        // long this whole arc has been running
        timeoutTimer.start();
        if (timeoutTimer.hasElapsed(timeout_s)) {
            DriverStation.reportError("ArcFollower timed out after " + timeout_s + "s, skipping rest of arc", false);
            currentWaypointIndex = wayPointsInterpolated.size();
            return;
        }

        boolean cont = isContinuous || (currentWaypointIndex != wayPointsInterpolated.size() - 1);

        Pose2d p = applyFlipping(wayPointsInterpolated.get(currentWaypointIndex), isMirrored);
        DogLog.log("Autos/Arc/p", p);
        DogLog.log("Autos/Arc/waypointIndex", currentWaypointIndex);
        DogLog.log("Autos/Arc/timeRunning", timeoutTimer.get());

        if (!hasStartedCurrentWaypoint) {
            manager.startVelocityDrivetoPose(p, maxDriveVelocity, maxRotateVelocity, atGoalTolerance, cont);
            hasStartedCurrentWaypoint = true;
        }

        if (manager.swerve.velocityAtGoal()) {
            currentWaypointIndex++;
            hasStartedCurrentWaypoint = false;
        }
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
