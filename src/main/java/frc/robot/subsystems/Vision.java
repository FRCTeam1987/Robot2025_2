package frc.robot.subsystems;

import static frc.robot.subsystems.constants.SubsystemConstants.VisionConstants.LIMELIGHTS;

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
      RobotContainer.DRIVETRAIN.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 99999999));
      RobotContainer.DRIVETRAIN.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
    }
  }

  public void log() {}
}
