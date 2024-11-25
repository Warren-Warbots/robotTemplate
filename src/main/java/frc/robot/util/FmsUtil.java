// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FmsUtil {
    
    public static boolean isRedAlliance() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        return alliance == Alliance.Red;
    }


}