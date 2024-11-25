package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;


public class FieldUtil {

      public static final Pose2d ORIGINAL_RED_SPEAKER =
      new Pose2d(
          Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(180));
  public static final Pose2d ORIGINAL_BLUE_SPEAKER =
      new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0));


  public static Pose2d getSpeakerPose() {
    if (FmsUtil.isRedAlliance()) {
      return ORIGINAL_RED_SPEAKER;
    } else {
      return ORIGINAL_BLUE_SPEAKER;
    }
  }

  public static Rotation2d getAmpAngle() {
    return FmsUtil.isRedAlliance() ? Rotation2d.fromDegrees(90) : Rotation2d.fromDegrees(90.0);
  }

  public static Rotation2d getPodiumAngle() {
    // return whatever the amp angle is
    return FmsUtil.isRedAlliance() ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180.0);
  }

  public static Rotation2d getSubwooferAngle() {
    // return whatever the amp angle is
    return FmsUtil.isRedAlliance() ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180.0);
  }

  public static Rotation2d getFieldRelativeAngleToPose(Pose2d current, Pose2d target) {
    return Rotation2d.fromRadians(Math.atan2(target.getY() - current.getY(), target.getX() - current.getX()));
  }
  
}