package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.Circle;
import frc.robot.util.FmsUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import dev.doglog.DogLog;

public class ArcFollower {
    private final RobotManager manager;
    private final List<Pose2d> waypoints;
    private double maxDriveVelocity = 2.5;
    private double maxRotateVelocity = 3.5;
    private double atGoalTolerance = 0.25;
    private double addTurnDegrees = 0.0;
    private int addTurnPoints = 5;

    private double timeout_s = 300;
    private boolean isContinuous = false;
    private boolean turnClockwise = false;
    private boolean isMirrored = false;
    private Pose2d p;

    private int currentWaypointIndex = 0;
    private boolean hasStartedCurrentWaypoint = false;

    // Field dimensions for flipping (2024/2026 standard)
    private static final double FIELD_LENGTH = 16.541;
    private static final double FIELD_WIDTH = 8.211;

    public ArcFollower(RobotManager manager, Pose2d... waypoints) {
        this.manager = manager;
        this.waypoints = Arrays.asList(waypoints);
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
    }

    /**
     * Called continuously. Returns true when the entire path is complete.
     */
    public boolean run() {
        ArrayList<Pose2d> wayPointsInterpolated = new ArrayList<>();
        Circle c;
        Rotation2d r = new Rotation2d();
        Translation2d startPoint = waypoints.get(0).getTranslation();
        Translation2d midPoint = waypoints.get(1).getTranslation();
        Translation2d endPoint = waypoints.get(2).getTranslation();
        c = new Circle(startPoint,
                midPoint,
                endPoint);
        DogLog.log("Autos/Arc/center", c.getCenter());
        Rotation2d startAngle = startPoint.minus(c.getCenter()).getAngle();
        Rotation2d midAngle = midPoint.minus(c.getCenter()).getAngle();
        Rotation2d endAngle = endPoint.minus(c.getCenter()).getAngle();

        Rotation2d angleDelta = midAngle.minus(startAngle);

        Rotation2d current = startAngle;
        Rotation2d angleIncrement = angleDelta.div(addTurnPoints);
        for (int w = 0; w <= addTurnPoints; w++) {

            Rotation2d thetaCenterToRobot = current;
            Rotation2d thetaDesiredP90 = new Rotation2d(thetaCenterToRobot.getCos(), thetaCenterToRobot.getSin());
            Rotation2d thetaDesiredN90 = new Rotation2d(thetaCenterToRobot.getCos(), thetaCenterToRobot.getSin());
            if (turnClockwise) {
                r = !isMirrored ? thetaDesiredP90.plus(Rotation2d.fromDegrees(90))
                        : thetaDesiredP90.minus(Rotation2d.fromDegrees(90));
            } else {
                r = !isMirrored ? thetaDesiredP90.minus(Rotation2d.fromDegrees(90))
                        : thetaDesiredP90.plus(Rotation2d.fromDegrees(90));
            }

            wayPointsInterpolated.add(new Pose2d(c.getPoint(current), r));
            current = current.plus(angleIncrement);
            DogLog.log("Autos/Arc/arcRadius", c.getRadius());
        }
        angleDelta = endAngle.minus(current);
        angleIncrement = angleDelta.div(addTurnPoints);
        for (int w = 0; w <= addTurnPoints; w++) {

            Rotation2d thetaCenterToRobot = current;
            Rotation2d thetaDesiredP90 = new Rotation2d(thetaCenterToRobot.getCos(), thetaCenterToRobot.getSin());
            Rotation2d thetaDesiredN90 = new Rotation2d(thetaCenterToRobot.getCos(), thetaCenterToRobot.getSin());
            if (turnClockwise) {
                r = !isMirrored ? thetaDesiredP90.plus(Rotation2d.fromDegrees(90))
                        : thetaDesiredP90.minus(Rotation2d.fromDegrees(90));
            } else {
                r = !isMirrored ? thetaDesiredP90.minus(Rotation2d.fromDegrees(90))
                        : thetaDesiredP90.plus(Rotation2d.fromDegrees(90));
            }

            wayPointsInterpolated.add(new Pose2d(c.getPoint(current), r));
            current = current.plus(angleIncrement);
            DogLog.log("Autos/Arc/arcRadius", c.getRadius());
        }
        if (currentWaypointIndex >= wayPointsInterpolated.size()) {
            return true;
        }

        isContinuous = isContinuous || (currentWaypointIndex != waypoints.size() - 1);

        Pose2d p = wayPointsInterpolated.get(currentWaypointIndex);
        DogLog.log("Autos/Arc/p", p);

        if (!hasStartedCurrentWaypoint) {
            manager.startVelocityDrivetoPose(p, maxDriveVelocity, maxRotateVelocity, atGoalTolerance, isContinuous);
            hasStartedCurrentWaypoint = true;
        }

        if (manager.swerve.velocityAtGoal()) {
            currentWaypointIndex++;
            hasStartedCurrentWaypoint = false;
        }

        return currentWaypointIndex >= wayPointsInterpolated.size();
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