package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

public class DriveForwardAuto extends WarbotAuto {

    public enum State {
        START,
        TEST_1,
        TEST_2,
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
    // Each path is created using a pose2d which can be edited by warPath instead of
    // finding each point individually, you can also add as many points to one path

    public DriveForwardAuto() {
    }

    @Override
    public void init() {
        currentState = State.START;
        stateTimer.reset();
    }

    @Override
    public void periodic() {
        DogLog.log("AutoState", currentState);
        switch (currentState) {
            case START:
                resetSwervePose(PathFollower.applyFlipping(UNDER_TRENCH_1[0], mirror));
                stateTimer.restart();
                currentState = currentState.next();
                // In the starting state we set the starting point of our path,
                // which is only needed to be set once
                break;

            case TEST_1:
                DogLog.log("driveArc", manager.driveArc(TEST_1, 2.0, 3.5, 0.4, 90.0, 100, true, true, mirror));
                if (manager.driveArc(TEST_1, 2.0, 3.5, 0.4, 90.0, 100, true, true, mirror)) {
                    // After running the starting state you can follow each path using the drivePath
                    // function
                    if (manager.swerve.velocityAtGoal()) {
                        currentState = currentState.next();
                        // This if statement waits till the path is completed to go to the next state

                        // Use this if statement if you’re calling a system state like intake, but not
                        // if you’re setting the next state to a path.
                    }

                }
                break;

            case TEST_2:
                DogLog.log("drivePath", manager.drivePath(TEST_2, 2.0, 3.5, 0.4, false, mirror));

                if (manager.drivePath(TEST_2, 2.0, 3.5, 0.4, false, mirror)) {
                    if (manager.swerve.velocityAtGoal()) {
                        currentState = currentState.next();
                    }
                }
                break;

            case DONE:
                if (manager.swerve.velocityAtGoal()) {
                    isFinished = true;
                    // This State is to end out auto

                }
                break;
        }
    }
}