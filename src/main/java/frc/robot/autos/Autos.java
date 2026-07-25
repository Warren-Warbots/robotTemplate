package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.robot_manager.RobotManager;

public class Autos {
    /**
     * Holds the auto choosers for the dashboard. Each slot picks one auto, and
     * the selected autos run one after another (see SequentialAuto). Leave
     * unused slots on DoNothing - they finish instantly.
     */

    // how many auto slots show up on the dashboard
    private static final int NUM_SLOTS = 3;

    private final RobotManager manager;
    private final List<SendableChooser<WarbotAuto>> choosers = new ArrayList<>();
    private final SendableChooser<Boolean> mirrorAuto = new SendableChooser<>();
    private SequentialAuto sequence = new SequentialAuto();

    public Autos(RobotManager robotManager) {
        this.manager = robotManager;
        sequence.setManager(manager);

        for (int i = 0; i < NUM_SLOTS; i++) {
            SendableChooser<WarbotAuto> chooser = new SendableChooser<>();
            chooser.setDefaultOption("DoNothing", new DoNothingAuto());

            // add every auto here - each slot needs its own copy, which is why
            // this is inside the loop
            chooser.addOption("DriveForwardAuto", new DriveForwardAuto());

            SmartDashboard.putData("chooser" + (i + 1), chooser);
            choosers.add(chooser);
        }

        mirrorAuto.setDefaultOption("(Right) Normal", false);
        mirrorAuto.addOption("(Left) Mirrored", true);
        SmartDashboard.putData("MirrorAuto", mirrorAuto);

    }

    public void preloadAuto() {
        // collect whatever is selected in each slot and line them up in order
        WarbotAuto[] selected = new WarbotAuto[choosers.size()];
        for (int i = 0; i < choosers.size(); i++) {
            WarbotAuto choice = choosers.get(i).getSelected();
            selected[i] = (choice != null) ? choice : new DoNothingAuto();
        }
        sequence = new SequentialAuto(selected);
        sequence.setManager(manager);
    }

    public void init() {
        // lets the first auto in the sequence set the starting pose (and only
        // the first - see WarbotAuto.resetSwervePose)
        WarbotAuto.allowPoseReset();
        sequence.init();
    }

    public void periodic() {
        DogLog.log("Autos/sequenceFinished", sequence.isFinished());
        sequence.periodic();
    }

    public void updateMirror() {
        WarbotAuto.mirror = mirrorAuto.getSelected();
    }

    public boolean getMirror() {
        return WarbotAuto.mirror;
    }
}
