package frc.robot.state.commands;

import static frc.robot.state.logic.constants.StateConstants.BLUE_TARGET_POSES_CORAL;
import static frc.robot.state.logic.constants.StateConstants.RED_TARGET_POSES_CORAL;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class DriveToNearest extends SequentialCommandGroup {
  public DriveToNearest() {
    super();
    addCommands(
        new DriveToPose(
            RobotContainer.DRIVETRAIN,
            () ->
                RobotContainer.DRIVETRAIN
                    .getPose()
                    .nearest(
                        RobotContainer.DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
                            ? RED_TARGET_POSES_CORAL
                            : BLUE_TARGET_POSES_CORAL)));
  }
}
