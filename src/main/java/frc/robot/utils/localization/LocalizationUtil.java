// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.localization.posearea.CircularPoseArea;
import frc.robot.utils.localization.posearea.HexagonalPoseArea;
import frc.robot.utils.localization.posearea.RectanglePoseArea;

/** Add your docs here. */
public class LocalizationUtil {

  public static final double FIELD_LENGTH = 17.548;
  public static final double FIELD_WIDTH = 8.052;

  public static RectanglePoseArea blueFlipToRed(RectanglePoseArea area) {
    return new RectanglePoseArea(
        blueFlipToRed(area.getBottomLeftPoint()), blueFlipToRed(area.getTopRightPoint()));
  }

  public static HexagonalPoseArea blueFlipToRed(HexagonalPoseArea area) {
    return new HexagonalPoseArea(blueFlipToRed(area.getOrigin()), area.getRadius());
  }

  public static CircularPoseArea blueFlipToRed(final CircularPoseArea area) {
    return new CircularPoseArea(blueFlipToRed(area.getOrigin()), area.getRadius());
  }

  public static Pose2d blueFlipToRed(final Pose2d blue) {
    return new Pose2d(
        blueFlipToRed(blue.getTranslation()),
        Rotation2d.fromDegrees(-180.0).minus(blue.getRotation()));
  }

  public static Translation2d blueFlipToRed(final Translation2d blue) {
    return new Translation2d(blueFlipXCoordinate(blue.getX()), blueFlipYCoordinate(blue.getY()));
  }

  public static Translation2d blueFlipToRedXOnly(final Translation2d blue) {
    return new Translation2d(blueFlipXCoordinate(blue.getX()), blue.getY());
  }

  public static Pose2d flipOverField(final Pose2d field) {
    return new Pose2d(flipOverField(field.getTranslation()), field.getRotation());
  }

  public static Translation2d flipOverField(final Translation2d field) {
    return new Translation2d(field.getX(), blueFlipYCoordinate(field.getY()));
  }

  public static double blueFlipXCoordinate(final double blueX) {
    return FIELD_LENGTH - blueX;
  }

  public static double blueFlipYCoordinate(final double blueY) {
    return FIELD_WIDTH - blueY;
  }

  /**
   * Calculate the rotation needed for the robot to poin towards the point of interest.
   *
   * @param robot
   * @param poi
   * @return
   */
  public static Rotation2d getRotationTowards(final Translation2d robot, final Translation2d poi) {
    Transform2d delta = new Transform2d(poi.minus(robot), new Rotation2d());
    return new Rotation2d(Math.atan2(delta.getY(), delta.getX()))
        .plus(Rotation2d.fromDegrees(180.0)); // Shooter shoots out the back.
  }
}
