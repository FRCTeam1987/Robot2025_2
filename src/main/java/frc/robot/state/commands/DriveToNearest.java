package frc.robot.state.commands;

import static frc.robot.state.logic.constants.StateConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.state.Abomination;
import frc.robot.state.logic.functional.FunctionalState;
import frc.robot.state.logic.mode.CollectMode;
import java.util.List;

public class DriveToNearest extends SequentialCommandGroup {
  public DriveToNearest() {
    super();
    addCommands(
        new DriveToPose(
            RobotContainer.DRIVETRAIN,
            () -> {
              if (Abomination.getPreviousState() == FunctionalState.COLLECT) {
                return RobotContainer.DRIVETRAIN
                    .getPose()
                    .nearest(
                        Abomination.getCollectMode() == CollectMode.HUMAN_PLAYER_STATION
                            ? RobotContainer.DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
                                ? RED_TARGET_POSES_COLLECT
                                : BLUE_TARGET_POSES_COLLECT
                            : RobotContainer.DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
                                ? RED_TARGET_POSES_ALGAE
                                : BLUE_TARGET_POSES_ALGAE);
              } else {
                return RobotContainer.DRIVETRAIN
                    .getPose()
                    .nearest(
                        RobotContainer.DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
                            ? RED_TARGET_POSES_CORAL
                            : BLUE_TARGET_POSES_CORAL);
              }
            }));
  }

  public DriveToNearest(List<Pose2d> poses) {
    super();
    addCommands(
        new DriveToPose(
            RobotContainer.DRIVETRAIN, () -> RobotContainer.DRIVETRAIN.getPose().nearest(poses)));
  }
}
