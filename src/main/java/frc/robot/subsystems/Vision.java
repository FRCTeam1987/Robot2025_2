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
    LimelightHelpers.PoseEstimate scoringEst = updateAndAcquirePoseEstimate(LIMELIGHT_SCORING_NAME);
    LimelightHelpers.PoseEstimate intakeEst = updateAndAcquirePoseEstimate(LIMELIGHT_INTAKE_NAME);
    if (!processScoringLimelight(scoringEst)) {
      processIntakeLimelight(intakeEst);
    }
  }

  public boolean processScoringLimelight(LimelightHelpers.PoseEstimate estimate) {

    if (estimate != null
        && estimate.rawFiducials != null
        && estimate.rawFiducials.length > 0
        && estimate.avgTagDist < 3.75) {
      double confidence = maPoseConfidence(estimate);
      if (shouldUpdatePose()) {
        RobotContainer.DRIVETRAIN.addVisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds,
            VecBuilder.fill(confidence, confidence, 99999999));
      }
      if (RobotContainer.DEBUG) {
        DogLog.log("Vision/ScoringProcess/Pose", estimate.pose);
        DogLog.log("Vision/ScoringProcess/Confidence", confidence);
      }
      return true;
    }
    return false;
  }

  public boolean processIntakeLimelight(LimelightHelpers.PoseEstimate estimate) {

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

  public LimelightHelpers.PoseEstimate updateAndAcquirePoseEstimate(String LIMELIGHT_NAME) {
    LimelightHelpers.SetRobotOrientation(
        LIMELIGHT_NAME, yaw.getAsDouble(), 0.0, 0.0, 0.0, 0.0, 0.0);
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
  }

  public void log() {}
}
