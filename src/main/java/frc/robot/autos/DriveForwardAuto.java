package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

public class DriveForwardAuto extends WarbotAuto {

    public enum State {
        START,
        UNDER_TRENCH,
        TEST_1,
        TEST_2,
        DRIVE_TO_FINISH,
        DONE;

        // The states are sequenced to the order they are running in
        private static final State[] vals = values();

        public State next() {
            return vals[Math.min(this.ordinal() + 1, vals.length - 1)];
        }
    }

    public State currentState = State.START;
    private Timer stateTimer = new Timer();

    private static final Pose2d[] UNDER_TRENCH_1 = { p(5.151, 0.637, 0.1), p(7.370, 0.680, 2.4) };
    private static final Pose2d[] TEST_1 = { p(7.370, 0.680, 0.1), p(7.714, 2.909, 89.1), p(5.990, 3.536, 175.5) };
    private static final Pose2d[] TEST_2 = { p(5.990, 3.536, 175.5), p(5.971, 0.718, 0.0) };
    private static final Pose2d FINISH_POINT = p(3.000, 1.000, 90.0);
    // Each path is created using a pose2d which can be edited by warPath instead of
    // finding each point individually, you can also add as many points to one path

    public DriveForwardAuto() {
    }

    @Override
    public void init() {
        currentState = State.START;
        isFinished = false;
        // start the followers over in case this auto is being re-run
        pathFollower.reset();
        arcFollower.reset();
    }

    @Override
    public void periodic() {
        DogLog.log("AutoState", currentState);
        switch (currentState) {
            case START:
                // In the starting state we set the starting point of our path,
                // which only needs to be set once
                resetSwervePose(PathFollower.applyFlipping(UNDER_TRENCH_1[0], mirror));
                currentState = currentState.next();
                break;

            case UNDER_TRENCH:
                // driving to a SINGLE point needs no follower - just call
                // driveTo every loop. loose tolerance (0.5m) + continuous true
                // means we pass through the point without slowing down, flowing
                // straight into the arc that starts there
                driveTo(UNDER_TRENCH_1[1], 2.0, 0.5, true);
                if (atDriveTarget()) {
                    currentState = currentState.next();
                }
                break;

            case TEST_1:
                // for a MULTI-point path, use a follower. setPath tells it
                // which waypoints to drive (a new path starts from its
                // beginning, with default settings), then run() drives it a
                // little further each loop
                arcFollower.setPath(TEST_1)
                        .withMaxVelocity(2.0)
                        .withMaxRotateVelocity(3.5)
                        .withTolerance(0.4)
                        .addTurnPoints(100)
                        .continuous(true)
                        .turnClockwise(true)
                        .withMirror(mirror);
                arcFollower.run();
                if (arcFollower.isDone()) {
                    currentState = currentState.next();
                }
                break;

            case TEST_2:
                pathFollower.setPath(TEST_2)
                        .withMaxVelocity(2.0)
                        .withMaxRotateVelocity(3.5)
                        .withTolerance(0.4)
                        .withMirror(mirror);
                pathFollower.run();
                if (pathFollower.isDone()) {
                    // start timing the next state (see DRIVE_TO_FINISH)
                    stateTimer.restart();
                    currentState = currentState.next();
                }
                break;

            case DRIVE_TO_FINISH:
                // another single point, but this time tight tolerance (0.1m) +
                // continuous false means actually stop there
                driveTo(FINISH_POINT, 2.0, 0.1, false);
                // followers time out on their own, but a bare driveTo doesn't -
                // the timer makes sure a stuck robot can't sit here forever
                if (atDriveTarget() || stateTimer.hasElapsed(3.0)) {
                    currentState = currentState.next();
                }
                break;

            case DONE:
                // This State is to end our auto
                isFinished = true;
                break;
        }
    }
}
