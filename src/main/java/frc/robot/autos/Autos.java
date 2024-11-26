// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.Optional;
import java.util.jar.Attributes.Name;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.swerve.SwerveState;
import frc.robot.swerve.SwerveSubsystem;

// TODO configure pathplanner configs here and make localization subsystem

/** Add your docs here. */
public class Autos {
    private RobotManager manager;
    private final SendableChooser<AutoChoice> autoChooser = new SendableChooser<>();
    private PathPlannerAuto selectedAuto;
    private Optional<AutoChoice> lastSelectedAuto = Optional.empty();

    public Autos(RobotManager robotManager) {
        this.manager = robotManager;

        NamedCommands.registerCommand("Stow",
                this.manager.setModeCommand(RobotState.STOW_NO_GP, true));

        NamedCommands.registerCommand("aim_at_speaker",
                this.manager.setModeCommand(RobotState.SPEAKER_SHOOTING, true)
                        .andThen(Commands.waitSeconds(2.0))
                        .andThen(this.manager.setModeCommand(RobotState.STOW_NO_GP, true)));

        NamedCommands.registerCommand("intake",
                this.manager.setModeCommand(RobotState.INTAKING)
                        .alongWith(overrideY()));

        NamedCommands.registerCommand("clear",
                clear());

        autoChooser.setDefaultOption("DoNothing", AutoChoice.DO_NOTHING);
        autoChooser.addOption("Tests", AutoChoice.TESTS);

        SmartDashboard.putData(autoChooser);
        FollowPathCommand.warmupCommand().schedule();
        PathfindingCommand.warmupCommand().schedule();

        selectedAuto = new PathPlannerAuto(AutoChoice.DO_NOTHING.pathName);
    }

    public AutoChoice getAuto() {
        AutoChoice rawAuto = autoChooser.getSelected();
        if (rawAuto == null) {
            rawAuto = AutoChoice.DO_NOTHING;
        }

        return rawAuto;
    }

    public void preloadAuto() {
        AutoChoice currentlySelectedAuto = autoChooser.getSelected();
        if (lastSelectedAuto.isEmpty()) {
            if (currentlySelectedAuto != null) {

                lastSelectedAuto = Optional.of(currentlySelectedAuto);
            } else {
                lastSelectedAuto = Optional.of(AutoChoice.DO_NOTHING);
            }

        }
        if (lastSelectedAuto.get() != currentlySelectedAuto) {
            // this means that a change to the autochoice was just made, time to preload a
            // new auto
            lastSelectedAuto = Optional.of(currentlySelectedAuto);
            try {

                selectedAuto = new PathPlannerAuto(lastSelectedAuto.get().pathName);
            } catch (Exception e) {
                DogLog.log("AutoChooser/failedAutoName", currentlySelectedAuto);
            }

        }
    }

    public Command getAutoCommand() {
        return selectedAuto;
    }


    /*
     * replace the ()->0.0 with methods that return feedback values 
     * for example, if you were trying to grab a note on the centerline in 2024, you could do just a overrideY() feedback
     * since you only need to move left/right to adjust for the note centering
     * 
     * in your robot manager or maybe a subsystem that tracks the notes, you would compute the difference between the 
     * robots position and the notes position, multiply it by a kP and then return that value
     * 
     * ex: 
     * in RobotManager.java:
     * double kP = 3.0;
     * 
     * public double yErrorToNote(){
     *  return kP* (notePose.getTranslation().getY() - robotPose.getTranslation().getY())
     * }
     * 
     * in Autos.java
     * 
     * private Command overrideYToLineUpWithNote() {
        return Commands.runOnce(() -> PPHolonomicDriveController.overrideYFeedback(manager::yErrorToNote));
    }


        once you made that command, when you started intaking you would make a named command that runs the
        overrideYToLineUpWithNote command during the autopath 
     */
    private Command overrideX() {
        return Commands.runOnce(() -> PPHolonomicDriveController.overrideXFeedback(() -> 0.0));
    }
    private Command overrideY() {
        return Commands.runOnce(() -> PPHolonomicDriveController.overrideYFeedback(() -> 0.0));
    }
    private Command overrideXY() {
        return Commands.runOnce(() -> PPHolonomicDriveController.overrideXYFeedback(() -> 0.0,()->0.0));
    }
    private Command overrideRotation() {
        return Commands.runOnce(() -> PPHolonomicDriveController.overrideRotationFeedback(() -> 0.0));
    }

    private Command clear() {
        return Commands.runOnce(() -> PPHolonomicDriveController.clearFeedbackOverrides());
    }

}