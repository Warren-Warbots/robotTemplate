// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.TimedRobot;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.driverstation.XboxController;
import frc.robot.autos.Autos;
import frc.robot.autos.DriveForwardAuto;
import frc.robot.autos.WarbotAuto;
import frc.robot.example_pivator_subsystem.PivatorSubsystem;
import frc.robot.example_intake_subsystem.IntakeSubsystem;
import frc.robot.lights_subsystem.LightsSubsystem;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.WantedRobotState;
import frc.robot.simulation.PhysicsSim;
import frc.robot.swerve.SwerveSubsystem;

public class Robot extends TimedRobot {
  private XboxController driverController = new XboxController(0);
  private SwerveSubsystem swerve = new SwerveSubsystem(driverController);
  private LightsSubsystem lights = new LightsSubsystem();
  private PivatorSubsystem pivator = new PivatorSubsystem();
  private IntakeSubsystem intake = new IntakeSubsystem();

  private final RobotManager manager = new RobotManager(swerve, lights, pivator, intake);
  private Autos autos = new Autos(manager);

  public Robot() {

    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureConsole(false)
            .withCaptureDs(true)
            .withNtTunables(!Constants.IS_AT_COMP)
            .withLogExtras(true));
  }

  @Override
  public void robotPeriodic() {
    manager.periodic();
    swerve.periodic();
    pivator.periodic();
    intake.periodic();
    lights.periodic();
  }

  @Override
  public void driverStationConnected() {
    DogLog.log("IsCompBot", Constants.IS_AT_COMP);

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    autos.updateMirror();
    autos.preloadAuto();
    autos.init();

  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    autos.updateMirror();
    autos.preloadAuto();
    autos.init();
  }

  @Override
  public void autonomousPeriodic() {
    autos.periodic();
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    DogLog.log("TimeLeft", MatchState.getMatchTime());
    // This is were we call all the driver and operator controllers for use during
    // teleop
    boolean leftTrigger = driverController.getLeftTrigger() > 0.5;
    boolean rightTrigger = driverController.getRightTrigger() > 0.5;
    boolean povLeft = driverController.getDpadLeftButtonPressed();
    boolean povRight = driverController.getDpadRightButtonPressed();
    boolean startPressed = driverController.getMenuButton();
    boolean rightBumper = driverController.getRightBumperButton();
    boolean leftBumper = driverController.getLeftBumperButton();

    if (leftTrigger) {
      manager.setWantedRobotState(WantedRobotState.AUTO_SCORE_L4);
    } else if (rightTrigger) {
      manager.setWantedRobotState(WantedRobotState.INTAKE);
    } else if (rightBumper) {
      manager.setWantedRobotState(WantedRobotState.STOW);
    } else if (leftBumper) {
      manager.setWantedRobotState(WantedRobotState.DRIVE_WITH_VELOCITY);
    }

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void utilityInit() {

  }

  @Override
  public void utilityPeriodic() {
  }

  @Override
  public void utilityExit() {
  }

  @Override
  public void simulationInit() {
    // this initializes the physics sim in simulation
    PhysicsSim.getInstance().addSimProfile(pivator.pivotSimProfile);
    PhysicsSim.getInstance().addSimProfile(pivator.frontElevatorSimProfile);

  }

  @Override
  public void simulationPeriodic() {
    // this continuously runs the physics sim in simulation
    PhysicsSim.getInstance().run();

  }
}
