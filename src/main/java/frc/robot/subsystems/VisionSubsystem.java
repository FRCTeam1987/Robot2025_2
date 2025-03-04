// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

  // Constants that should be moved elsewhere after dumping.
  private static final String INTAKE_LIMELIGHT = "limelight-intake";
  private static final String SCORING_LIMELIGHT = "limelight-scoring";
  private static final double MAX_AMBIGUITY = 0.3;
  private static final double BASE_CONFIDENCE = 0.2;
  private static final double IGNORE_ROTATION_STD_DEV = 9999999;
  private static final double CONNECT_DEBOUNCER_SECONDS = 0.5;

  // move this to field zones
  public static final double FIELD_LENGTH = 17.548;
  public static final double FIELD_WIDTH = 8.052;

  // Logic
  private final Debouncer m_intakeConnectedDebouncer;
  private final Debouncer m_scoringConnectedDebouncer;
  private boolean m_shouldUpdateDrivetrain = true;

  /** Creates a new Vision. */
  public VisionSubsystem() {
    m_intakeConnectedDebouncer = new Debouncer(CONNECT_DEBOUNCER_SECONDS);
    m_scoringConnectedDebouncer = new Debouncer(CONNECT_DEBOUNCER_SECONDS);
  }

  public void updateOrientations() {
    final double yaw = RobotContainer.DRIVETRAIN.getState().Pose.getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation(INTAKE_LIMELIGHT, yaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    LimelightHelpers.SetRobotOrientation(SCORING_LIMELIGHT, yaw, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  public boolean bronco2024canUseVisionData() {
    final ChassisSpeeds chassisSpeeds = RobotContainer.DRIVETRAIN.getState().Speeds;
    return !(Math.abs(
                RobotContainer.DRIVETRAIN
                    .getPigeon2()
                    .getAngularVelocityZWorld()
                    .getValueAsDouble())
            > 540
        || (Math.abs(chassisSpeeds.vxMetersPerSecond) > 3.0
            || Math.abs(chassisSpeeds.vyMetersPerSecond) > 3.0));
  }

  public boolean maRejectVisionData(LimelightHelpers.PoseEstimate observation) {
    return observation == null
        || observation.tagCount == 0 // Must have at least one tag
        || (observation.tagCount == 1
            && observation.rawFiducials[0].ambiguity > MAX_AMBIGUITY) // Cannot be high ambiguity
        // || Math.abs(observation.pose.getZ())
        //     > maxZError // Must have realistic Z coordinate

        // Must be within the field boundaries
        || observation.pose.getX() < 0.0
        || observation.pose.getX() > FIELD_LENGTH
        || observation.pose.getY() < 0.0
        || observation.pose.getY() > FIELD_WIDTH;
  }

  public boolean shouldUpdateDrivetrain() {
    return m_shouldUpdateDrivetrain;
  }

  public void setShouldUpdateDrivetrain(final boolean shouldUpdateDrivetrain) {
    m_shouldUpdateDrivetrain = shouldUpdateDrivetrain;
  }

  public double bronco2024PoseConfidence(LimelightHelpers.PoseEstimate estimate) {
    return Math.pow(0.8, estimate.tagCount) * (estimate.avgTagDist) * 2;
  }

  public double maPoseConfidence(LimelightHelpers.PoseEstimate estimate) {
    return BASE_CONFIDENCE * Math.pow(estimate.avgTagDist, 2.0) / (double) estimate.tagCount;
  }

  public void updateDrivetrain(final String limelight) {
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
    if (maRejectVisionData(mt2)) {
      return;
    }
    // final double confidence = bronco2024PoseConfidence(mt2);
    final double confidence = maPoseConfidence(mt2);
    RobotContainer.DRIVETRAIN.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds,
        VecBuilder.fill(confidence, confidence, IGNORE_ROTATION_STD_DEV));
    DogLog.log("vision/" + limelight + "/pose", mt2.pose);
    DogLog.log("vision/" + limelight + "/timestampSeconds", mt2.timestampSeconds);
    DogLog.log("vision/" + limelight + "/tagCount", mt2.tagCount);
    DogLog.log("vision/" + limelight + "/avgTagDist", mt2.avgTagDist);
    DogLog.log("vision/" + limelight + "/confidence", confidence);
  }

  public boolean isLimelightConnected(final String limelight) {
    return LimelightHelpers.getLatency_Pipeline(limelight) > 0.0;
  }

  public void log() {
    DogLog.log(
        "vision/" + INTAKE_LIMELIGHT + "/isConnected",
        m_intakeConnectedDebouncer.calculate(isLimelightConnected(INTAKE_LIMELIGHT)));
    DogLog.log(
        "vision/" + SCORING_LIMELIGHT + "/isConnected",
        m_scoringConnectedDebouncer.calculate(isLimelightConnected(SCORING_LIMELIGHT)));
  }

  @Override
  public void periodic() {
    log();
    updateOrientations();
    if (!shouldUpdateDrivetrain()) {
      return;
    }
    if (!bronco2024canUseVisionData()) {
      return;
    }
    // TODO conditionally update drive depending on robot location / status (ex. auto align)
    updateDrivetrain(INTAKE_LIMELIGHT);
    updateDrivetrain(SCORING_LIMELIGHT);
  }
}
