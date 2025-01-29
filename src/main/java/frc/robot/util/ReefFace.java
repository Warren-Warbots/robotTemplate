// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class ReefFace {
    Translation2d faceCenter;
    Translation2d leftBranch;
    Translation2d rightBranch;
    
    Rotation2d faceAngle;

    public ReefFace(Translation2d faceCenter,Translation2d leftBranch,Translation2d rightBranch,Rotation2d faceAngle){
        this.faceCenter=faceCenter;
        this.faceAngle=faceAngle;
        this.leftBranch=leftBranch;
        this.rightBranch=rightBranch;
    }

    public Pose2d getLeft(){
        return new Pose2d(leftBranch,faceAngle);
    }
    public Pose2d getRight(){
        return new Pose2d(rightBranch,faceAngle);
    }
    public Pose2d getCenter(){
        return new Pose2d(faceCenter,faceAngle);
    }
    public Rotation2d getAngle(){
        return faceAngle;
    }

    
}
