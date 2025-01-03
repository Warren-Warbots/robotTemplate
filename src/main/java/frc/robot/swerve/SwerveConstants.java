// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.swerve.generated.BetaTunerConstants;
import frc.robot.swerve.generated.CompTunerConstants;

/** Add your docs here. */
public class SwerveConstants {


public static SwerveDrivetrainConstants swerveDrivetrainConstants = Constants.IS_COMP_BOT ? CompTunerConstants.DrivetrainConstants : BetaTunerConstants.DrivetrainConstants;

public static SwerveModuleConstants FrontLeft = Constants.IS_COMP_BOT ? CompTunerConstants.FrontLeft : BetaTunerConstants.FrontLeft;
public static SwerveModuleConstants FrontRight = Constants.IS_COMP_BOT ? CompTunerConstants.FrontRight : BetaTunerConstants.FrontRight;
public static SwerveModuleConstants BackLeft = Constants.IS_COMP_BOT ? CompTunerConstants.BackLeft: BetaTunerConstants.BackLeft;
public static SwerveModuleConstants BackRight = Constants.IS_COMP_BOT ? CompTunerConstants.BackRight: BetaTunerConstants.BackRight;

//physical constants, get by driving robot, or asking design
public static final double maxSpeed = Constants.IS_COMP_BOT ? CompTunerConstants.kSpeedAt12Volts.baseUnitMagnitude() : BetaTunerConstants.kSpeedAt12Volts.baseUnitMagnitude();  //tune this in the Tuner Constants file
public static final double maxRotSpeed = 7.0; // rad/second, should probably tune this?

public static final double driveGearRatio = Constants.IS_COMP_BOT ? CompTunerConstants.kDriveGearRatio : BetaTunerConstants.kDriveGearRatio;


//driver configs - driver should tune these

//deadband, below this number, controller doesnt do anything
public static final double leftXDeadband = 0.05; 
public static final double rightXDeadband = 0.05;
public static final double leftYDeadband = 0.05;

public static final double leftXExponent = 3;
public static final double leftYExponent = 3;
public static final double rightXExponent = 2;



//vision configs
public static final boolean useLimelight = true; 
public static final Matrix<N3,N1> megaTag1DisabledstdDev = VecBuilder.fill(.5,.5,0.5);
public static final Matrix<N3,N1> megaTag1stdDev = VecBuilder.fill(.5,.5,9999999);

public static final Matrix<N3,N1> megaTag2stdDev = VecBuilder.fill(.7,.7,9999999);

//snap configs
// this was sysid-ed but the hand tuned gains were like 15,0,1 so sysid is not worth
public static final PhoenixPIDController snapController = new PhoenixPIDController(16.383, 0.0, 1); 


public static final PhoenixPIDController maintainHeadingController = new PhoenixPIDController(15, 0.0, 1); 

public static final double snapTolerance = 0.5; //radians
public static final double maintainHeadingTolerance = 0.5; //radians - this does nothing



// auto configs
public static final PIDConstants translationPIDConstantsAuto = new PIDConstants(15.0, 0.0, 0.0);
public static final PIDConstants rotationPIDConstantsAuto = new PIDConstants(15, 0.0, 0);



}
