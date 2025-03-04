// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.localization;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.localization.posearea.CircularPoseArea;
import frc.robot.utils.localization.posearea.RectanglePoseArea;

/** Adds your docs here. */
public class FieldZones {
  public enum Zone {
    ALLIANCE_SIDE,
    CENTER_BARGE,
    OPPONENT_SIDE,

    ALLIANCE_REEF,
    OPPONENT_REEF,

    ALLIANCE_CORAL_LEFT,
    ALLIANCE_CORAL_RIGHT,

    ALLIANCE_PROCESSOR,
    OPPONENT_PROCESSOR
  }

  public static final double X_MIN = 0.0;
  public static final double Y_MIN = 0.0;
  public static final double BLUE_SIDE_LINE = 7.582 + 0.30;
  public static final double RED_SIDE_LINE = 9.968 - 0.30;

  public static final RectanglePoseArea FIELD =
      new RectanglePoseArea(
          new Translation2d(X_MIN, Y_MIN),
          new Translation2d(LocalizationUtil.FIELD_LENGTH, LocalizationUtil.FIELD_WIDTH));

  public static final CircularPoseArea REEF_BLUE =
      new CircularPoseArea(new Translation2d(4.495, 4.02), Meters.of(2.2));
  public static final CircularPoseArea REEF_RED = LocalizationUtil.blueFlipToRed(REEF_BLUE);

  public static final Distance CORAL_STATION_RADIUS = Distance.ofBaseUnits(4.0, Meters);
  public static final CircularPoseArea LEFT_CORAL_STATION_BLUE =
      new CircularPoseArea(FIELD.bottomLeft(), CORAL_STATION_RADIUS);
  public static final CircularPoseArea RIGHT_CORAL_STATION_BLUE =
      new CircularPoseArea(FIELD.getTopLeft(), CORAL_STATION_RADIUS);
  public static final CircularPoseArea LEFT_CORAL_STATION_RED =
      LocalizationUtil.blueFlipToRed(LEFT_CORAL_STATION_BLUE);
  public static final CircularPoseArea RIGHT_CORAL_STATION_RED =
      LocalizationUtil.blueFlipToRed(RIGHT_CORAL_STATION_BLUE);

  public static Zone getZoneFromTranslation(
      final Alliance alliance, final Translation2d translation) {
    final double x = translation.getX();
    if (alliance == Alliance.Red) {
      if (x > RED_SIDE_LINE) {
        if (LEFT_CORAL_STATION_RED.contains(translation)) return Zone.ALLIANCE_CORAL_LEFT;
        if (RIGHT_CORAL_STATION_RED.contains(translation)) return Zone.ALLIANCE_CORAL_RIGHT;
        if (REEF_RED.contains(translation)) return Zone.ALLIANCE_REEF;
        return Zone.ALLIANCE_SIDE;
      }
      if (x < RED_SIDE_LINE && x > BLUE_SIDE_LINE) {
        return Zone.CENTER_BARGE;
      }
      if (x < BLUE_SIDE_LINE) {
        if (REEF_BLUE.contains(translation)) return Zone.OPPONENT_REEF;
        return Zone.OPPONENT_SIDE;
      }
    } else {
      if (x < BLUE_SIDE_LINE) {
        if (LEFT_CORAL_STATION_BLUE.contains(translation)) return Zone.ALLIANCE_CORAL_LEFT;
        if (RIGHT_CORAL_STATION_BLUE.contains(translation)) return Zone.ALLIANCE_CORAL_RIGHT;
        if (REEF_BLUE.contains(translation)) return Zone.ALLIANCE_REEF;
        return Zone.ALLIANCE_SIDE;
      }
      if (x < RED_SIDE_LINE && x > BLUE_SIDE_LINE) {
        return Zone.CENTER_BARGE;
      }
      if (x > RED_SIDE_LINE) {
        if (REEF_RED.contains(translation)) return Zone.OPPONENT_REEF;
        return Zone.OPPONENT_SIDE;
      }
    }
    return Zone.ALLIANCE_SIDE;
  }
}
