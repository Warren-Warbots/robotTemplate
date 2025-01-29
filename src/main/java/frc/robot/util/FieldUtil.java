package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;


public class FieldUtil {

       public static Rotation2d getFieldRelativeAngleToPose(Pose2d current, Pose2d target) {
    return Rotation2d.fromRadians(Math.atan2(target.getY() - current.getY(), target.getX() - current.getX()));
  }

    static Translation2d robotToBranchOffset = new Translation2d(0.5,0.5);
    static double reefWidth = 1.66;
    static double branchSpacing=0.33;
    static Translation2d blueReefCenter = new Translation2d(4.5,4);

    static Translation2d blueACenter = new Translation2d(reefWidth/2,0);
    static Translation2d blueALeft = blueACenter.plus(new Translation2d(0,-1*branchSpacing/2));
    static Translation2d blueARight = blueACenter.plus(new Translation2d(0,branchSpacing/2));

    static Rotation2d blueBRotation= Rotation2d.fromDegrees(-60);
    static Translation2d blueBCenter = blueACenter.rotateBy(blueBRotation).plus(blueReefCenter); 
    static Translation2d blueBLeft = blueALeft.rotateBy(blueBRotation).plus(blueReefCenter);
    static Translation2d blueBRight = blueARight.rotateBy(blueBRotation).plus(blueReefCenter);


    static Rotation2d blueCRotation= Rotation2d.fromDegrees(-120);
    static Translation2d blueCCenter = blueACenter.rotateBy(blueCRotation).plus(blueReefCenter); 
    static Translation2d blueCLeft = blueALeft.rotateBy(blueCRotation).plus(blueReefCenter);
    static Translation2d blueCRight = blueARight.rotateBy(blueCRotation).plus(blueReefCenter);

    static Rotation2d blueDRotation= Rotation2d.fromDegrees(-180);
    static Translation2d blueDCenter = blueACenter.rotateBy(blueDRotation).plus(blueReefCenter); 
    static Translation2d blueDLeft = blueALeft.rotateBy(blueDRotation).plus(blueReefCenter);
    static Translation2d blueDRight = blueARight.rotateBy(blueDRotation).plus(blueReefCenter);


    static Rotation2d blueERotation= Rotation2d.fromDegrees(-240);
    static Translation2d blueECenter = blueACenter.rotateBy(blueERotation).plus(blueReefCenter); 
    static Translation2d blueELeft = blueALeft.rotateBy(blueERotation).plus(blueReefCenter);
    static Translation2d blueERight = blueARight.rotateBy(blueERotation).plus(blueReefCenter);

    static Rotation2d blueFRotation= Rotation2d.fromDegrees(-300);
    static Translation2d blueFCenter = blueACenter.rotateBy(blueFRotation).plus(blueReefCenter); 
    static Translation2d blueFLeft = blueALeft.rotateBy(blueFRotation).plus(blueReefCenter);
    static Translation2d blueFRight = blueARight.rotateBy(blueFRotation).plus(blueReefCenter);

    static Translation2d[] blueReefLeftBranches = {blueALeft,blueBLeft,blueCLeft,blueDLeft,blueELeft,blueFLeft};
    static Translation2d[] blueReefRightBranches = {blueARight,blueBRight,blueCRight,blueDRight,blueERight,blueFRight};
    static Translation2d[] blueReefCenters = {blueACenter,blueBCenter,blueCCenter,blueDCenter,blueECenter,blueFCenter};
    static Rotation2d[] blueReefSnapAngles = {Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(120),Rotation2d.fromDegrees(60),Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(-60),Rotation2d.fromDegrees(-120)};

    static public Reef blueReef = new Reef(blueReefCenters, blueReefLeftBranches, blueReefRightBranches, blueReefSnapAngles);
    
    static Translation2d redReefCenter = new Translation2d(17.55-4.5,4);

    static Translation2d redACenter = new Translation2d(-1*reefWidth/2,0);
    static Translation2d redALeft = redACenter.plus(new Translation2d(0,branchSpacing/2));
    static Translation2d redARight = redACenter.plus(new Translation2d(0,-1*branchSpacing/2));

    static Rotation2d redBRotation= Rotation2d.fromDegrees(-60);
    static Translation2d redBCenter = redACenter.rotateBy(redBRotation).plus(redReefCenter); 
    static Translation2d redBLeft = redALeft.rotateBy(redBRotation).plus(redReefCenter);
    static Translation2d redBRight = redARight.rotateBy(redBRotation).plus(redReefCenter);


    static Rotation2d redCRotation= Rotation2d.fromDegrees(-120);
    static Translation2d redCCenter = redACenter.rotateBy(redCRotation).plus(redReefCenter); 
    static Translation2d redCLeft = redALeft.rotateBy(redCRotation).plus(redReefCenter);
    static Translation2d redCRight = redARight.rotateBy(redCRotation).plus(redReefCenter);

    static Rotation2d redDRotation= Rotation2d.fromDegrees(-180);
    static Translation2d redDCenter = redACenter.rotateBy(redDRotation).plus(redReefCenter); 
    static Translation2d redDLeft = redALeft.rotateBy(redDRotation).plus(redReefCenter);
    static Translation2d redDRight = redARight.rotateBy(redDRotation).plus(redReefCenter);


    static Rotation2d redERotation= Rotation2d.fromDegrees(-240);
    static Translation2d redECenter = redACenter.rotateBy(redERotation).plus(redReefCenter); 
    static Translation2d redELeft = redALeft.rotateBy(redERotation).plus(redReefCenter);
    static Translation2d redERight = redARight.rotateBy(redERotation).plus(redReefCenter);

    static Rotation2d redFRotation= Rotation2d.fromDegrees(-300);
    static Translation2d redFCenter = redACenter.rotateBy(redFRotation).plus(redReefCenter); 
    static Translation2d redFLeft = redALeft.rotateBy(redFRotation).plus(redReefCenter);
    static Translation2d redFRight = redARight.rotateBy(redFRotation).plus(redReefCenter);

    static Translation2d[] redReefLeftBranches = {redALeft,redBLeft,redCLeft,redDLeft,redELeft,redFLeft};
    static Translation2d[] redReefRightBranches = {redARight,redBRight,redCRight,redDRight,redERight,redFRight};
    static Translation2d[] redReefCenters = {redACenter,redBCenter,redCCenter,redDCenter,redECenter,redFCenter};
    static Rotation2d[] redReefSnapAngles = {Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(60),Rotation2d.fromDegrees(120),Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(240),Rotation2d.fromDegrees(300)};

    public static Reef redReef = new Reef(redReefCenters, redReefLeftBranches, redReefRightBranches, redReefSnapAngles);
  

    public static Reef getReef(){
      return FmsUtil.isRedAlliance() ? redReef:blueReef;
    }

    public static Pose2d addRobotOffset(Pose2d reefTarget){
      return new Pose2d(reefTarget.getTranslation().minus(robotToBranchOffset.rotateBy(reefTarget.getRotation())),reefTarget.getRotation());
    }
    
}