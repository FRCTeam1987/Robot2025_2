package frc.robot.utils.localization.posearea;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public record RectanglePoseArea(Translation2d bottomLeft, Translation2d topRight) {
  /**
   * Create a 2D rectangular area for pose calculations.
   *
   * @param bottomLeft bottom left corner of the rectangle.
   * @param topRight top right corner of the rectangle.
   */
  public RectanglePoseArea {}

  public double getMinX() {
    return bottomLeft.getX();
  }

  public double getMaxX() {
    return topRight.getX();
  }

  public double getMinY() {
    return bottomLeft.getY();
  }

  public double getMaxY() {
    return topRight.getY();
  }

  public Translation2d getBottomRight() {
    return new Translation2d(topRight.getY(), bottomLeft.getY());
  }

  public Translation2d getTopLeft() {
    return new Translation2d(bottomLeft.getX(), topRight.getY());
  }

  public Translation2d getBottomLeftPoint() {
    return bottomLeft;
  }

  public Translation2d getTopRightPoint() {
    return topRight;
  }

  public boolean isPoseWithinArea(Pose2d pose) {
    return isTranslationWithinArea(pose.getTranslation());
  }

  public boolean isTranslationWithinArea(Translation2d translation) {
    return translation.getX() >= bottomLeft.getX()
        && translation.getX() <= topRight.getX()
        && translation.getY() >= bottomLeft.getY()
        && translation.getY() <= topRight.getY();
  }
}
