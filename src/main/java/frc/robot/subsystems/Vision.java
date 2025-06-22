package frc.robot.subsystems;

import static frc.robot.subsystems.constants.SubsystemConstants.VisionConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utils.LimelightHelpers;
import java.util.function.DoubleSupplier;

public class Vision extends SubsystemBase {

  private boolean shouldUpdatePose = true;

  public Vision() {}

  private static final double BASE_CONFIDENCE = 0.2;

  DoubleSupplier yaw = () -> RobotContainer.DRIVETRAIN.getPose().getRotation().getDegrees();

  public boolean shouldUpdatePose() {
    return shouldUpdatePose;
  }

  public void setShouldUpdatePose(final boolean shouldUpdate) {
    shouldUpdatePose = shouldUpdate;
  }

  public double maPoseConfidence(LimelightHelpers.PoseEstimate estimate) {
    return BASE_CONFIDENCE * Math.pow(estimate.avgTagDist, 2.0) / (double) estimate.tagCount;
  }

  @Override
  public void periodic() {
    LimelightHelpers.SetRobotOrientation_NoFlush(
        LIMELIGHT_INTAKE_NAME, yaw.getAsDouble(), 0.0, 0.0, 0.0, 0.0, 0.0);
    LimelightHelpers.SetRobotOrientation(
        LIMELIGHT_SCORING_NAME, yaw.getAsDouble(), 0.0, 0.0, 0.0, 0.0, 0.0);

    LimelightHelpers.PoseEstimate scoringEst = getMegaTag2(LIMELIGHT_SCORING_NAME);
    LimelightHelpers.PoseEstimate intakeEst = getMegaTag2(LIMELIGHT_INTAKE_NAME);
    if (!processScoring(scoringEst)) {
      processIntake(intakeEst);
    }
  }

  public boolean processScoring(LimelightHelpers.PoseEstimate estimate) {
    if (estimate != null && estimate.rawFiducials != null && estimate.rawFiducials.length > 0) {
      if (estimate.avgTagDist < 0.65) {
        // MT1 Angle
        estimate = getMegaTag1(LIMELIGHT_SCORING_NAME);
        double confidence = maPoseConfidence(estimate);
        if (shouldUpdatePose()) {
          RobotContainer.DRIVETRAIN.addVisionMeasurement(
              estimate.pose,
              estimate.timestampSeconds,
              VecBuilder.fill(confidence, confidence, 0.0001));
        }
        if (RobotContainer.DEBUG) {
          DogLog.log("Vision/ScoringProcess/MT1Pose", estimate.pose);
          DogLog.log("Vision/ScoringProcess/MT1Degrees", estimate.pose.getRotation().getDegrees());
          DogLog.log("Vision/ScoringProcess/MT1Confidence", confidence);
          DogLog.log("Vision/ScoringProcess/MT1Active", true);
        }
        return true;
      } else if (estimate.avgTagDist < 3.75) {
        // MT2 Regular
        double confidence = maPoseConfidence(estimate);
        if (shouldUpdatePose()) {
          RobotContainer.DRIVETRAIN.addVisionMeasurement(
              estimate.pose,
              estimate.timestampSeconds,
              VecBuilder.fill(confidence, confidence, 99999999));
        }
        if (RobotContainer.DEBUG) {
          DogLog.log("Vision/ScoringProcess/MT2Pose", estimate.pose);
          DogLog.log("Vision/ScoringProcess/MT2Confidence", confidence);
          DogLog.log("Vision/ScoringProcess/MT1Active", false);
        }
        return true;
      }
    }
    return false;
  }

  public boolean processIntake(LimelightHelpers.PoseEstimate estimate) {

    if (estimate != null
        && estimate.rawFiducials != null
        && estimate.rawFiducials.length > 0
        && estimate.avgTagDist < 2.25) {
      double confidence = maPoseConfidence(estimate);
      if (shouldUpdatePose()) {
        RobotContainer.DRIVETRAIN.addVisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds,
            VecBuilder.fill(confidence, confidence, 99999999));
      }
      if (RobotContainer.DEBUG) {
        DogLog.log("Vision/IntakeProcess/Pose", estimate.pose);
        DogLog.log("Vision/IntakeProcess/Confidence", confidence);
      }
      return true;
    }
    return false;
  }

  public LimelightHelpers.PoseEstimate getMegaTag2(String LIMELIGHT_NAME) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
  }

  public LimelightHelpers.PoseEstimate getMegaTag1(String LIMELIGHT_NAME) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);
  }

  public void log() {}
}
