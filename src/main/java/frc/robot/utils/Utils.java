package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import java.util.List;

public class Utils {
  public static Pose2d getNearest(List<Pose2d> posesRed, List<Pose2d> posesBlue) {
    return RobotContainer.DRIVETRAIN
        .getPose()
        .nearest(
            RobotContainer.DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
                ? posesRed
                : posesBlue);
  }
}
