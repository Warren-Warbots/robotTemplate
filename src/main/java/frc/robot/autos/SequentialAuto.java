package frc.robot.autos;

import dev.doglog.DogLog;
import frc.robot.robot_manager.RobotManager;

public class SequentialAuto extends WarbotAuto {
    /**
     * Runs any number of autos one after another. When the current auto says
     * it is finished, the next one starts. This is itself just a little state
     * machine, where the state is which auto we're on.
     *
     * A DoNothingAuto in the list finishes instantly, so unused slots flow
     * straight through to the next one.
     */

    private final WarbotAuto[] autos;
    private int currentIndex = 0;

    public SequentialAuto(WarbotAuto... autos) {
        this.autos = autos;
    }

    @Override
    public void setManager(RobotManager robotManager) {
        super.setManager(robotManager);
        for (WarbotAuto auto : autos) {
            auto.setManager(robotManager);
        }
    }

    @Override
    public void init() {
        isFinished = false;
        currentIndex = 0;
        if (autos.length == 0) {
            isFinished = true;
            return;
        }
        autos[0].init();
    }

    @Override
    public void periodic() {
        if (isFinished) {
            return;
        }
        DogLog.log("Autos/sequenceIndex", currentIndex);

        WarbotAuto current = autos[currentIndex];
        current.periodic();

        if (current.isFinished()) {
            currentIndex++;
            if (currentIndex < autos.length) {
                autos[currentIndex].init();
            } else {
                isFinished = true;
            }
        }
    }
}
