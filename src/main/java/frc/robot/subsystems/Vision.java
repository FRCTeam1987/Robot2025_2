package frc.robot.subsystems;

import static frc.robot.subsystems.constants.SubsystemConstants.VisionConstants.LIMELIGHTS;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import frc.robot.RobotContainer;
import frc.robot.utils.LimelightHelpers;

public class Vision {

  public Vision() {}

  private static final double BASE_CONFIDENCE = 0.2;

  public double maPoseConfidence(LimelightHelpers.PoseEstimate estimate) {
    return BASE_CONFIDENCE * Math.pow(estimate.avgTagDist, 2.0) / (double) estimate.tagCount;
  }

  public void cycle() {
    for (String limelight : LIMELIGHTS) {
      double yaw = RobotContainer.DRIVETRAIN.getPigeon2().getYaw().getValueAsDouble();
      LimelightHelpers.SetRobotOrientation(limelight, yaw, 0.0, 0.0, 0.0, 0.0, 0.0);
      LimelightHelpers.PoseEstimate estimate =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);

      if (estimate != null && estimate.rawFiducials != null && estimate.rawFiducials.length > 0) {
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
