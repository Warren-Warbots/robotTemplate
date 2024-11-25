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

/** Add your docs here. */
public class SwerveConstants {


//physical constants, get by driving robot
public static final double maxSpeed = 5.0; 
public static final double maxRotSpeed = 10.0; // rad/second

//driver configs - driver should tune these

//deadband, below this number, controller doesnt do anything
public static final double leftXDeadband = 0.05; 
public static final double rightXDeadband = 0.05;
public static final double leftYDeadband = 0.05;

public static final double leftXExponent = 1.5;
public static final double leftYExponent = 1.5;
public static final double rightXExponent = 2;



//vision configs
public static final boolean useLimelight = true; 
public static final Matrix<N3,N1> megaTag1DisabledstdDev = VecBuilder.fill(.5,.5,0.5);
public static final Matrix<N3,N1> megaTag1stdDev = VecBuilder.fill(.5,.5,9999999);

public static final Matrix<N3,N1> megaTag2stdDev = VecBuilder.fill(.7,.7,9999999);

//snap configs
public static final PhoenixPIDController snapController = new PhoenixPIDController(15.0, 0.0, 1.0);
public static final double snapTolerance = 0.02; //radians

public static final double snapEndThreshold = 0.075;// if rotation input > this, cancel snap
public static final PhoenixPIDController noSnapController = new PhoenixPIDController(10.0, 0.0, 0.0);




// auto configs
public static final PIDConstants translationPIDConstantsAuto = new PIDConstants(10.0, 0.0, 0.0);
public static final PIDConstants rotationPIDConstantsAuto = new PIDConstants(15.0, 0.0, 0.0);



}
