package frc.robot.state.logic.constants;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.RobotContainer.DRIVETRAIN;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.localization.LocalizationUtil;

public enum PositionConstant {
  BLUE_REEF(new Translation2d(4.495, 4.02), Degrees.of(0.0)),

  SIDE_1_ENTRY(new Translation2d(2.629 - 0.0274, 4.023), Degrees.of(0.0)),
  SIDE_1_A(new Translation2d(3.120 - 0.00, 4.192 + 0.005), SIDE_1_ENTRY.getAngle()),
  SIDE_1_ALGAE(new Translation2d(3.180 + 0.06, 4.023), SIDE_1_ENTRY.getAngle()),
  SIDE_1_B(new Translation2d(3.120 - 0.00, 3.862 + 0.005), SIDE_1_ENTRY.getAngle()),
  SIDE_1_A1(new Translation2d(3.2 - 0.00, 4.5), flip(SIDE_1_ENTRY.getAngle())),
  SIDE_1_B1(new Translation2d(3.2 - 0.00, 3.5), flip(SIDE_1_ENTRY.getAngle())),

  SIDE_2_ENTRY(rotateAroundBlueReef(SIDE_1_ENTRY, Degrees.of(60.0)), Degrees.of(60.0)),
  SIDE_2_C(rotateAroundBlueReef(SIDE_1_A, Degrees.of(60.0)), SIDE_2_ENTRY.getAngle()),
  SIDE_2_ALGAE(rotateAroundBlueReef(SIDE_1_ALGAE, Degrees.of(60.0)), SIDE_2_ENTRY.getAngle()),
  SIDE_2_D(rotateAroundBlueReef(SIDE_1_B, Degrees.of(60.0)), SIDE_2_ENTRY.getAngle()),
  SIDE_2_C1(rotateAroundBlueReef(SIDE_1_A1, Degrees.of(60.0)), flip(SIDE_2_ENTRY.getAngle())),
  SIDE_2_D1(rotateAroundBlueReef(SIDE_1_B1, Degrees.of(60.0)), flip(SIDE_2_ENTRY.getAngle())),

  SIDE_3_ENTRY(rotateAroundBlueReef(SIDE_2_ENTRY, Degrees.of(60.0)), Degrees.of(120.0)),
  SIDE_3_E(rotateAroundBlueReef(SIDE_2_C, Degrees.of(60.0)), SIDE_3_ENTRY.getAngle()),
  SIDE_3_ALGAE(rotateAroundBlueReef(SIDE_2_ALGAE, Degrees.of(60.0)), SIDE_3_ENTRY.getAngle()),
  SIDE_3_F(rotateAroundBlueReef(SIDE_2_D, Degrees.of(60.0)), SIDE_3_ENTRY.getAngle()),
  SIDE_3_E1(rotateAroundBlueReef(SIDE_2_C1, Degrees.of(60.0)), flip(SIDE_3_ENTRY.getAngle())),
  SIDE_3_F1(rotateAroundBlueReef(SIDE_2_D1, Degrees.of(60.0)), flip(SIDE_3_ENTRY.getAngle())),

  SIDE_4_ENTRY(rotateAroundBlueReef(SIDE_3_ENTRY, Degrees.of(60.0)), Degrees.of(180)),
  SIDE_4_G(rotateAroundBlueReef(SIDE_3_E, Degrees.of(60.0)), SIDE_4_ENTRY.getAngle()),
  SIDE_4_ALGAE(rotateAroundBlueReef(SIDE_3_ALGAE, Degrees.of(60.0)), SIDE_4_ENTRY.getAngle()),
  SIDE_4_H(rotateAroundBlueReef(SIDE_3_F, Degrees.of(60.0)), SIDE_4_ENTRY.getAngle()),
  SIDE_4_G1(rotateAroundBlueReef(SIDE_3_E1, Degrees.of(60.0)), flip(SIDE_4_ENTRY.getAngle())),
  SIDE_4_H1(rotateAroundBlueReef(SIDE_3_F1, Degrees.of(60.0)), flip(SIDE_4_ENTRY.getAngle())),

  SIDE_5_ENTRY(rotateAroundBlueReef(SIDE_4_ENTRY, Degrees.of(60.0)), Degrees.of(-120)),
  SIDE_5_I(rotateAroundBlueReef(SIDE_4_G, Degrees.of(60.0)), SIDE_5_ENTRY.getAngle()),
  SIDE_5_ALGAE(rotateAroundBlueReef(SIDE_4_ALGAE, Degrees.of(60.0)), SIDE_5_ENTRY.getAngle()),
  SIDE_5_J(rotateAroundBlueReef(SIDE_4_H, Degrees.of(60.0)), SIDE_5_ENTRY.getAngle()),
  SIDE_5_I1(rotateAroundBlueReef(SIDE_4_G1, Degrees.of(60.0)), flip(SIDE_5_ENTRY.getAngle())),
  SIDE_5_J1(rotateAroundBlueReef(SIDE_4_H1, Degrees.of(60.0)), flip(SIDE_5_ENTRY.getAngle())),

  SIDE_6_ENTRY(rotateAroundBlueReef(SIDE_5_ENTRY, Degrees.of(60.0)), Degrees.of(-60)),
  SIDE_6_K(rotateAroundBlueReef(SIDE_5_I, Degrees.of(60.0)), SIDE_6_ENTRY.getAngle()),
  SIDE_6_ALGAE(rotateAroundBlueReef(SIDE_5_ALGAE, Degrees.of(60.0)), SIDE_6_ENTRY.getAngle()),
  SIDE_6_L(rotateAroundBlueReef(SIDE_5_J, Degrees.of(60.0)), SIDE_6_ENTRY.getAngle()),
  SIDE_6_K1(rotateAroundBlueReef(SIDE_5_I1, Degrees.of(60.0)), flip(SIDE_6_ENTRY.getAngle())),
  SIDE_6_L1(rotateAroundBlueReef(SIDE_5_J1, Degrees.of(60.0)), flip(SIDE_6_ENTRY.getAngle())),

  RC2(new Translation2d(1.658, 1.001), Degrees.of(55)),
  LC2(LocalizationUtil.flipOverField(RC2.getTranslation()), Degrees.of(-55)),

  ALL_NET(new Translation2d(7.55, 6.157), Degrees.of(180)),
  OPP_NET(new Translation2d(LocalizationUtil.blueFlipXCoordinate(7.55), 6.157), Degrees.of(0)),

  C1(new Translation2d(7.5, 5.047), Degrees.of(270)),
  C2(new Translation2d(7.5, 6.159), Degrees.of(270)),
  C3(new Translation2d(7.5, 7.256), Degrees.of(270)),

  P1(new Translation2d(5.65, 0.63), Degrees.of(-55)),
  ;

  private final Translation2d BLUE_TRANSLATION;
  private final Rotation2d ROTATION;
  private final Angle ROTATION_ANGLE;
  private final Pose2d BLUE_POSE;
  private final Pose2d RED_POSE;

  public Translation2d getTranslation() {
    return BLUE_TRANSLATION;
  }

  public Rotation2d getRotation() {
    return ROTATION;
  }

  public Angle getAngle() {
    return ROTATION_ANGLE;
  }

  public Pose2d getBluePose() {
    return BLUE_POSE;
  }

  public Pose2d getRedPose() {
    return RED_POSE;
  }

  public Pose2d getAlliancePose() {
    return DRIVETRAIN.getAlliance() == DriverStation.Alliance.Blue ? BLUE_POSE : RED_POSE;
  }

  public Pose2d getOpposingPose() {
    return DRIVETRAIN.getAlliance() == DriverStation.Alliance.Blue ? RED_POSE : BLUE_POSE;
  }

  PositionConstant(Translation2d TRANSLATION, Angle HEADING) {
    this.BLUE_TRANSLATION = TRANSLATION;
    this.ROTATION_ANGLE = HEADING;
    this.ROTATION = new Rotation2d(HEADING);

    this.BLUE_POSE = new Pose2d(TRANSLATION, ROTATION.minus(new Rotation2d(Degrees.of(180))));
    this.RED_POSE =
        new Pose2d(LocalizationUtil.blueFlipToRed(BLUE_POSE.getTranslation()), ROTATION);
  }

  private static Translation2d rotateAroundBlueReef(PositionConstant POS, Angle ROTATION) {
    return POS.getTranslation().rotateAround(BLUE_REEF.BLUE_TRANSLATION, new Rotation2d(ROTATION));
  }

  private static Angle flip(Angle angle) {
    return angle.minus(Degrees.of(180.0));
  }
}
