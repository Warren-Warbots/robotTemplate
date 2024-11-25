// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.Optional;


import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import dev.doglog.DogLog;
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
        
        autoChooser.setDefaultOption("DoNothing", AutoChoice.DO_NOTHING);
        autoChooser.addOption("Tests", AutoChoice.TESTS);
        
        SmartDashboard.putData(autoChooser);
        FollowPathCommand.warmupCommand().schedule();
        PathfindingCommand.warmupCommand().schedule();
        


        NamedCommands.registerCommand("aim_at_speaker", 
        manager.setModeCommand(RobotState.SPEAKER_SHOOTING,true)
        .andThen(Commands.waitSeconds(2.0))
        .andThen(manager.setModeCommand(RobotState.STOW_NO_GP,true)));
        
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
        // return new PathPlannerAuto(getAuto().pathName);
        return selectedAuto;
    }


}