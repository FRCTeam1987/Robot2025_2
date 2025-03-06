package frc.robot.subsystems;

import static frc.robot.subsystems.constants.SubsystemConstants.VisionConstants.LIMELIGHTS;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import frc.robot.RobotContainer;
import frc.robot.utils.LimelightHelpers;

public class Vision {

  public Vision() {}

  public void cycle() {
    for (String limelight : LIMELIGHTS) {
      double yaw = RobotContainer.DRIVETRAIN.getPigeon2().getYaw().getValueAsDouble();
      LimelightHelpers.SetRobotOrientation(limelight, yaw, 0.0, 0.0, 0.0, 0.0, 0.0);
      LimelightHelpers.PoseEstimate estimate =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
      RobotContainer.DRIVETRAIN.setVisionMeasurementStdDevs(VecBuilder.fill(.1, .1, 32));
      if (estimate != null && estimate.rawFiducials != null && estimate.rawFiducials.length > 0) {
        RobotContainer.DRIVETRAIN.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
        DogLog.log("Vision/" + limelight + "Pose", estimate.pose);
      }
    }
  }

  public void log() {}
}
