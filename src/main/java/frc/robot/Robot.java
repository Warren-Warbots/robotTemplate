// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.Autos;
import frc.robot.constants.Constants;
import frc.robot.lights.LightsSubsystem;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.swerve.SwerveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private CommandXboxController driverController = new CommandXboxController(0);
  private SwerveSubsystem swerve = new SwerveSubsystem(driverController);
  private final CANdle candle = new CANdle(Constants.Lights_ID,
      Constants.CANBUS_NAME);
  private final LightsSubsystem lights = new LightsSubsystem(candle);

  private final RobotManager manager = new RobotManager(swerve, lights);
  private Autos autos = new Autos(manager);

  public Robot() {

    DogLog.setOptions(
        new DogLogOptions().withCaptureNt(false)
            .withCaptureDs(true)
            .withNtPublish(!Constants.IS_COMP));
    configureButtonBindings();
  }

  public void configureButtonBindings() {
    driverController.a().onTrue(manager.setModeCommand(RobotState.SPEAKER_SHOOTING));
    driverController.b().onTrue(manager.setModeCommand(RobotState.STOW_HAS_GP));
    driverController.y().onTrue(manager.setModeCommand(RobotState.AMP));
    

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    autos.preloadAuto();
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autos.getAutoCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
