// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public class SwerveConstants {


//physical constants, get by driving robot
public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(); 
public static final double maxRotSpeed = 10.0; // rad/second, should probably tune this?

//driver configs - driver should tune these

//deadband, below this number, controller doesnt do anything
public static final double leftXDeadband = 0.05; 
public static final double rightXDeadband = 0.05;
public static final double leftYDeadband = 0.05;

public static final double leftXExponent = 3;
public static final double leftYExponent = 3;
public static final double rightXExponent = 3;



//vision configs
public static final boolean useLimelight = true; 
public static final Matrix<N3,N1> megaTag1DisabledstdDev = VecBuilder.fill(.5,.5,0.5);
public static final Matrix<N3,N1> megaTag1stdDev = VecBuilder.fill(.5,.5,9999999);

public static final Matrix<N3,N1> megaTag2stdDev = VecBuilder.fill(.7,.7,9999999);

//snap configs
// this was sysid-ed but the hand tuned gains were like 15,0,1 so sysid is not worth
public static final PhoenixPIDController snapController = new PhoenixPIDController(16.383, 0.0, 1.031); 
public static final double snapTolerance = 0.02; //radians

public static final double snapEndThreshold = 0.075;// if rotation input > this, cancel snap




// auto configs
public static final PIDConstants translationPIDConstantsAuto = new PIDConstants(15.0, 0.0, 0.0);
public static final PIDConstants rotationPIDConstantsAuto = new PIDConstants(15, 0.0, 0);



}
