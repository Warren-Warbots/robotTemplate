// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
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
        new DogLogOptions().withCaptureNt(false)
            .withCaptureDs(true)
            .withNtPublish(!Constants.IS_AT_COMP));
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
  public void robotInit() {
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
    SmartDashboard.putNumber("TimeLeft", DriverStation.getMatchTime());
    // This is were we call all the driver and operator controllers for use during
    // teleop
    boolean leftTrigger = driverController.getLeftTriggerAxis() > 0.5;
    boolean rightTrigger = driverController.getRightTriggerAxis() > 0.5;
    boolean povLeft = driverController.getPOV() == 270;
    boolean povRight = driverController.getPOV() == 90;
    boolean startPressed = driverController.getStartButton();
    boolean rightBumper = driverController.getRightBumper();
    boolean leftBumper = driverController.getLeftBumper();
    boolean bButton = driverController.getBButtonPressed();
    boolean yButton = driverController.getYButtonPressed();
    boolean aButton = driverController.getAButtonPressed();
    boolean povForward = driverController.getPOV() == 0;
    boolean povBackword = driverController.getPOV() == 180;
    boolean xButton = driverController.getXButton();
    

    if (leftTrigger) {
      manager.setWantedRobotState(WantedRobotState.AUTO_SCORE_L4);
    } else if (rightTrigger) {
      manager.setWantedRobotState(WantedRobotState.INTAKE);
    } else if (rightBumper) {
      manager.setWantedRobotState(WantedRobotState.STOW);
    } else if (bButton) {
      manager.setWantedRobotState(WantedRobotState.DRIVE_HALF_SPEED);
    } else if (yButton) {
      manager.setWantedRobotState(WantedRobotState.CENTRIC_DRIVE);
    } else if (aButton) {
      manager.setWantedRobotState(WantedRobotState.SNAP);
    } else if (povForward) {
      manager.setWantedSwerveAngle(0);
      manager.setWantedRobotState(WantedRobotState.SNAP);
    } else if (povRight){
      manager.setWantedSwerveAngle(90);
      manager.setWantedRobotState(WantedRobotState.SNAP);
    } else if (povLeft){
      manager.setWantedSwerveAngle(270);
      manager.setWantedRobotState(WantedRobotState.SNAP);
    } else if (povBackword){
      manager.setWantedSwerveAngle(180);
      manager.setWantedRobotState(WantedRobotState.SNAP);
    } else if (leftBumper){
      manager.setWantedRobotState(WantedRobotState.AUTO_POINT);
    }

    }

  
  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
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
