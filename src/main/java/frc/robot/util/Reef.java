// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Reef {
    public ReefFace[] reefFaces = new ReefFace[6];
    
    public Reef(Translation2d[] reefFaceCenters, Translation2d[] reefLeftBranches, Translation2d[] reefRightBranches, Rotation2d[] reefSnapAngles){
        for (int i=0;i<6;i++){
            reefFaces[i] = new ReefFace(reefFaceCenters[i], reefLeftBranches[i], reefRightBranches[i], reefSnapAngles[i]);
        }
    }

    public ReefFace A(){
        return reefFaces[0];
    }

    public ReefFace B(){
        return reefFaces[1];
    }
    public ReefFace C(){
        return reefFaces[2];
    }
    public ReefFace D(){
        return reefFaces[3];
    }
    public ReefFace E(){
        return reefFaces[4];
    }
    public ReefFace F(){
        return reefFaces[5];
    }


}
