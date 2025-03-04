// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.localization.posearea;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class CircularPoseArea {
  private final Translation2d m_origin;
  private final Distance m_radius;

  public CircularPoseArea(final Translation2d origin, final Distance radius) {
    m_origin = origin;
    m_radius = radius;
  }

  public boolean contains(final Translation2d point) {
    return m_origin.getDistance(point) < m_radius.magnitude();
    // return Distance.ofBaseUnits(m_origin.getDistance(point), Meters).lte(m_radius);
    // && FieldZones.FIELD.isPoseWithinArea(point);
  }

  public boolean contains(final Pose2d point) {
    return contains(point.getTranslation());
  }

  public Translation2d getOrigin() {
    return m_origin;
  }

  public Distance getRadius() {
    return m_radius;
  }
}
