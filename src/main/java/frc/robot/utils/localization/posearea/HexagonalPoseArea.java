package frc.robot.utils.localization.posearea;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.OptionalInt;

public class HexagonalPoseArea {
  private final Translation2d origin;
  private final Distance radius;
  private Translation2d[] vertices;

  public HexagonalPoseArea(Translation2d origin, Distance radius) {
    this(origin, radius, Rotation2d.kZero);
  }

  public HexagonalPoseArea(Translation2d origin, Distance radius, Rotation2d rotation) {
    this.origin = origin;
    this.radius = radius;
    calculateVertices(rotation);
  }

  private void calculateVertices(Rotation2d rotation) {
    vertices = new Translation2d[6];
    double angleStep = Math.PI / 3;

    for (int i = 0; i < 6; i++) {
      double angle = i * angleStep;
      double x = radius.in(Meters) * Math.cos(angle);
      double y = radius.in(Meters) * Math.sin(angle);

      Translation2d basePoint = new Translation2d(x, y);
      Translation2d rotatedPoint = basePoint.rotateBy(rotation);

      vertices[i] =
          new Translation2d(
              rotatedPoint.getX() + origin.getX(), rotatedPoint.getY() + origin.getY());
    }
  }

  public Translation2d[] getVertices() {
    return vertices;
  }

  public Translation2d getOrigin() {
    return origin;
  }

  public Distance getRadius() {
    return radius;
  }
  /**
   * Determines if a given position is within the hexagonal area.
   *
   * @param position The position to check.
   * @return An OptionalInt containing the index of the triangle within the hexagon that contains
   *     the position, or an empty OptionalInt if the position is outside the hexagon.
   */
  public OptionalInt contains(Translation2d position) {
    if (origin.getDistance(position) > radius.in(Meters)) {
      return OptionalInt.empty();
    }

    for (int i = 0; i < 6; i++) {
      if (isPointInTriangle(position, origin, vertices[i], vertices[(i + 1) % 6])) {
        return OptionalInt.of(i);
      }
    }

    return OptionalInt.empty();
  }

  public boolean isPoseWithinArea(Pose2d pose) {
    return contains(pose.getTranslation()).isPresent();
  }

  private boolean isPointInTriangle(
      Translation2d p, Translation2d a, Translation2d b, Translation2d c) {
    Translation2d v0 = c.minus(a);
    Translation2d v1 = b.minus(a);
    Translation2d v2 = p.minus(a);

    double dot00 = v0.getX() * v0.getX() + v0.getY() * v0.getY();
    double dot01 = v0.getX() * v1.getX() + v0.getY() * v1.getY();
    double dot02 = v0.getX() * v2.getX() + v0.getY() * v2.getY();
    double dot11 = v1.getX() * v1.getX() + v1.getY() * v1.getY();
    double dot12 = v1.getX() * v2.getX() + v1.getY() * v2.getY();

    double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    return (u >= 0) && (v >= 0) && (u + v <= 1);
  }
}
