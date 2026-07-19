// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.swerve.generated.CompTunerConstants;

/** Add your docs here. */
public class SwerveConstants {

    public static SwerveDrivetrainConstants swerveDrivetrainConstants = CompTunerConstants.DrivetrainConstants;

    public static SwerveModuleConstants FrontLeft = CompTunerConstants.FrontLeft;
    public static SwerveModuleConstants FrontRight = CompTunerConstants.FrontRight;
    public static SwerveModuleConstants BackLeft = CompTunerConstants.BackLeft;
    public static SwerveModuleConstants BackRight = CompTunerConstants.BackRight;

    // physical constants, get by driving robot, or asking design
    public static final double maxSpeed = CompTunerConstants.kSpeedAt12Volts.baseUnitMagnitude();
    public static final double maxRotSpeed = 7.0; // rad/second, should probably tune this?

    public static final double driveGearRatio = CompTunerConstants.kDriveGearRatio;

    // driver configs - driver should tune these

    // deadband, below this number, controller doesnt do anything
    public static final double leftXDeadband = 0.05;
    public static final double rightXDeadband = 0.05;
    public static final double leftYDeadband = 0.05;

    public static final double leftXExponent = 3;
    public static final double leftYExponent = 3;
    public static final double rightXExponent = 2;

    // vision configs
    public static final boolean useLimelight = true;
    public static final Matrix<N3, N1> megaTag1DisabledstdDev = VecBuilder.fill(.5, .5, 0.5);
    public static final Matrix<N3, N1> megaTag1stdDev = VecBuilder.fill(.5, .5, 9999999);

    public static final Matrix<N3, N1> megaTag2stdDev = VecBuilder.fill(.7, .7, 9999999);

    // snap configs

    public static final PhoenixPIDController snapController = new PhoenixPIDController(16.383, 0.0, 1);

    public static final double snapTolerance = 0.5; // radians
    public static final double maintainHeadingTolerance = 0.5; // radians - this does nothing

    public static final PIDController teleopDriveToPoseController = new PIDController(1, 0, 0);
    public static final PIDController autoDriveToPoseController = new PIDController(1, 0, 0);

}
