package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldUtil {

  public static Pose2d getExamplePose() {
    return FmsUtil.isRedAlliance() ? new Pose2d(1.0, 1.0, Rotation2d.kZero) : new Pose2d(6, 1, Rotation2d.k180deg);
  }

  public static double angleBetweenRotation2ds(Rotation2d a, Rotation2d b) {
    double phi = Math.abs(a.getDegrees() - b.getDegrees()) % 360;
    return phi > 180 ? 360 - phi : phi;
  }

  public static Rotation2d getFieldRelativeAngleToPose(Pose2d current, Pose2d target) {
    return getFieldRelativeAngleToPose(current.getTranslation(), target.getTranslation());
  }

  public static Rotation2d getFieldRelativeAngleToPose(Pose2d current, Translation2d target) {
    return getFieldRelativeAngleToPose(current.getTranslation(), target);
  }

  public static Rotation2d getFieldRelativeAngleToPose(Translation2d current, Pose2d target) {
    return getFieldRelativeAngleToPose(current, target.getTranslation());
  }

  public static Rotation2d getFieldRelativeAngleToPose(Translation2d current, Translation2d target) {
    return Rotation2d.fromRadians(Math.atan2(target.getY() - current.getY(), target.getX() - current.getX()));
  }

}