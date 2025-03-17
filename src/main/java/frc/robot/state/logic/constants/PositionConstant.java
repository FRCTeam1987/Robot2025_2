package frc.robot.state.logic.constants;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.RobotContainer.DRIVETRAIN;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.state.commands.DriveToPose;
import frc.robot.utils.localization.LocalizationUtil;

public enum PositionConstant {
  BLUE_REEF(new Translation2d(4.495, 4.02), Degrees.of(0.0)),

  SIDE_1_ENTRY(new Translation2d(2.629 - 0.0274, 4.023 - 0.01), Degrees.of(0.0)),
  SIDE_1_A(new Translation2d(3.120 - 0.00, 4.192 + 0.005), SIDE_1_ENTRY.getAngle()),
  SIDE_1_ALGAE(new Translation2d(3.180 - 0.06, 4.023), SIDE_1_ENTRY.getAngle()),
  SIDE_1_B(new Translation2d(3.120 - 0.00, 3.862 + 0.005), SIDE_1_ENTRY.getAngle()),

  SIDE_2_ENTRY(rotateAroundBlueReef(SIDE_1_ENTRY, Degrees.of(60.0)), Degrees.of(60.0)),
  SIDE_2_C(rotateAroundBlueReef(SIDE_1_A, Degrees.of(60.0)), SIDE_2_ENTRY.getAngle()),
  SIDE_2_ALGAE(rotateAroundBlueReef(SIDE_1_ALGAE, Degrees.of(60.0)), SIDE_2_ENTRY.getAngle()),
  SIDE_2_D(rotateAroundBlueReef(SIDE_1_B, Degrees.of(60.0)), SIDE_2_ENTRY.getAngle()),

  SIDE_3_ENTRY(rotateAroundBlueReef(SIDE_2_ENTRY, Degrees.of(60.0)), Degrees.of(120.0)),
  SIDE_3_E(rotateAroundBlueReef(SIDE_2_C, Degrees.of(60.0)), SIDE_3_ENTRY.getAngle()),
  SIDE_3_ALGAE(rotateAroundBlueReef(SIDE_2_ALGAE, Degrees.of(60.0)), SIDE_3_ENTRY.getAngle()),
  SIDE_3_F(rotateAroundBlueReef(SIDE_2_D, Degrees.of(60.0)), SIDE_3_ENTRY.getAngle()),

  SIDE_4_ENTRY(rotateAroundBlueReef(SIDE_3_ENTRY, Degrees.of(60.0)), Degrees.of(180)),
  SIDE_4_G(rotateAroundBlueReef(SIDE_3_E, Degrees.of(60.0)), SIDE_4_ENTRY.getAngle()),
  SIDE_4_ALGAE(rotateAroundBlueReef(SIDE_3_ALGAE, Degrees.of(60.0)), SIDE_4_ENTRY.getAngle()),
  SIDE_4_H(rotateAroundBlueReef(SIDE_3_F, Degrees.of(60.0)), SIDE_4_ENTRY.getAngle()),

  SIDE_5_ENTRY(rotateAroundBlueReef(SIDE_4_ENTRY, Degrees.of(60.0)), Degrees.of(-120)),
  SIDE_5_I(rotateAroundBlueReef(SIDE_4_G, Degrees.of(60.0)), SIDE_5_ENTRY.getAngle()),
  SIDE_5_ALGAE(rotateAroundBlueReef(SIDE_4_ALGAE, Degrees.of(60.0)), SIDE_5_ENTRY.getAngle()),
  SIDE_5_J(rotateAroundBlueReef(SIDE_4_H, Degrees.of(60.0)), SIDE_5_ENTRY.getAngle()),

  SIDE_6_ENTRY(rotateAroundBlueReef(SIDE_5_ENTRY, Degrees.of(60.0)), Degrees.of(-60)),
  SIDE_6_K(rotateAroundBlueReef(SIDE_5_I, Degrees.of(60.0)), SIDE_6_ENTRY.getAngle()),
  SIDE_6_ALGAE(rotateAroundBlueReef(SIDE_5_ALGAE, Degrees.of(60.0)), SIDE_6_ENTRY.getAngle()),
  SIDE_6_L(rotateAroundBlueReef(SIDE_5_J, Degrees.of(60.0)), SIDE_6_ENTRY.getAngle()),

  LC1(new Translation2d(0.661, 6.66), Degrees.of(-55)),
  LC2(new Translation2d(1.277, 6.955), Degrees.of(-55)),
  LC3(new Translation2d(1.672, 6.398), Degrees.of(-55)),
  // RIGHT STATION
  RC1(new Translation2d(0.661, 1.367), Degrees.of(55)),
  RC2(new Translation2d(1.262, 1.170), Degrees.of(55)),
  RC3(new Translation2d(1.672, 0.651), Degrees.of(55)),
  // NET SCORING POS, CHANGE X FOR DISTANCE TO NET
  N1(new Translation2d(7.933, 7.266), Degrees.of(0.0)),
  N2(new Translation2d(7.933, 6.157), Degrees.of(0.0)),
  N3(new Translation2d(7.933, 5.079), Degrees.of(0.0)),
  // PROCESSOR
  P1(new Translation2d(5.987, 0.676), Degrees.of(90.0)),
  ;

  private final Translation2d BLUE_TRANSLATION;
  private final Rotation2d ROTATION;
  private final Angle ROTATION_ANGLE;

  private final Command BLUE_PATH;
  private final Command RED_PATH;
  private final DriveToPose BLUE_HOLO;
  private final DriveToPose RED_HOLO;
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

  public Command getRedPath() {
    return RED_PATH;
  }

  public Command getBluePath() {
    return BLUE_PATH;
  }

  public Command getAlliancePath() {
    return DRIVETRAIN.getAlliance() == DriverStation.Alliance.Blue ? BLUE_PATH : RED_PATH;
  }

  public DriveToPose getRedHolo() {
    return RED_HOLO;
  }

  public DriveToPose getBlueHolo() {
    return BLUE_HOLO;
  }

  public DriveToPose getAllianceHolo() {
    return DRIVETRAIN.getAlliance() == DriverStation.Alliance.Blue ? BLUE_HOLO : RED_HOLO;
  }

  PositionConstant(Translation2d TRANSLATION, Angle HEADING) {
    this.BLUE_TRANSLATION = TRANSLATION;
    this.ROTATION_ANGLE = HEADING;
    this.ROTATION = new Rotation2d(HEADING);

    this.BLUE_POSE = new Pose2d(TRANSLATION, ROTATION.minus(new Rotation2d(Degrees.of(180))));
    this.RED_POSE =
        new Pose2d(LocalizationUtil.blueFlipToRed(BLUE_POSE.getTranslation()), ROTATION);

    this.BLUE_PATH =
        AutoBuilder.pathfindToPose(
            BLUE_POSE,
            new PathConstraints(
                4.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
    this.RED_PATH =
        AutoBuilder.pathfindToPose(
            RED_POSE,
            new PathConstraints(
                4.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
    this.BLUE_HOLO = new DriveToPose(DRIVETRAIN, BLUE_POSE);
    this.RED_HOLO = new DriveToPose(DRIVETRAIN, RED_POSE);
  }

  private static Translation2d rotateAroundBlueReef(PositionConstant POS, Angle ROTATION) {
    return POS.getTranslation().rotateAround(BLUE_REEF.BLUE_TRANSLATION, new Rotation2d(ROTATION));
  }
}
