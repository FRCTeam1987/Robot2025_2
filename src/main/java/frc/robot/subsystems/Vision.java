package frc.robot.subsystems;

import static frc.robot.RobotContainer.DRIVETRAIN;
import static frc.robot.subsystems.constants.SubsystemConstants.VisionConstants.LIMELIGHTS;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utils.LimelightHelpers;

public class Vision extends SubsystemBase {

  public Vision() {}

  private static final double BASE_CONFIDENCE = 0.2;

  public double maPoseConfidence(LimelightHelpers.PoseEstimate estimate) {
    return BASE_CONFIDENCE * Math.pow(estimate.avgTagDist, 2.0) / (double) estimate.tagCount;
  }

  @Override
  public void periodic() {
    for (String limelight : LIMELIGHTS) {
      double yaw = RobotContainer.DRIVETRAIN.getPose().getRotation().getDegrees();
      LimelightHelpers.SetRobotOrientation(limelight, yaw, 0.0, 0.0, 0.0, 0.0, 0.0);
      LimelightHelpers.PoseEstimate estimate =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

      if (estimate != null
          && estimate.rawFiducials != null
          && estimate.rawFiducials.length > 0
          && estimate.avgTagDist < 3.75) {
        if (DRIVETRAIN.getPose().getTranslation().getDistance(estimate.pose.getTranslation())
            > 1.0) {
          DogLog.log("Pose Rejection", true);
          return;
        }
        double confidence = maPoseConfidence(estimate);
        RobotContainer.DRIVETRAIN.addVisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds,
            VecBuilder.fill(confidence, confidence, 99999999));
        DogLog.log("Vision/" + limelight + "Pose", estimate.pose);
        DogLog.log("Vision/" + limelight + "Confidence", confidence);
      }
    }
  }

  public void log() {}
}
