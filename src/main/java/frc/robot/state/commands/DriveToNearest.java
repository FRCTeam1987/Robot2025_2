package frc.robot.state.commands;

import static frc.robot.state.logic.constants.StateConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.state.Abomination;
import frc.robot.state.logic.constants.PositionConstant;
import frc.robot.state.logic.functional.FunctionalState;
import java.util.List;
import java.util.function.Supplier;

public class DriveToNearest extends SequentialCommandGroup {

  public DriveToNearest(Supplier<List<Pose2d>> poses) {
    super();
    addCommands(
        new DriveToPose(
            RobotContainer.DRIVETRAIN,
            () -> RobotContainer.DRIVETRAIN.getPose().nearest(poses.get())));
  }

  public DriveToNearest(boolean isLeft) {
    super();
    addCommands(calculateNearest(isLeft));
  }

  public DriveToPose calculateNearest(boolean isLeft) {
    if (Abomination.getPreviousState() == FunctionalState.COLLECT) {
      switch (Abomination.getCollectMode()) {
        case HUMAN_PLAYER_STATION -> {
          return getNearest(RED_COLLECT, BLUE_COLLECT);
        }
        case ALGAE_2, ALGAE_3 -> {
          return getNearest(RED_ALGAE, BLUE_ALGAE);
        }
      }
    } else if (Abomination.getPreviousState() == FunctionalState.CLIMB_DEPLOY) {
      new DriveToPose(
          RobotContainer.DRIVETRAIN,
          () ->
              RobotContainer.DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
                  ? PositionConstant.N2.getRedPose()
                  : PositionConstant.N2.getBluePose(),
          true);

    } else {
      switch (Abomination.getScoreMode()) {
        case L1, L2, L3, L4 -> {
          if (isLeft) {
            return getNearest(RED_CORAL_LEFT, BLUE_CORAL_LEFT);
          } else {
            return getNearest(RED_CORAL_RIGHT, BLUE_CORAL_RIGHT);
          }
        }
        case PROCESSOR -> {
          return new DriveToPose(
              RobotContainer.DRIVETRAIN,
              () ->
                  RobotContainer.DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
                      ? PositionConstant.P1.getRedPose()
                      : PositionConstant.P1.getBluePose());
        }
        case NET -> {}
      }
    }
    if (isLeft) {
      return getNearest(RED_CORAL_LEFT, BLUE_CORAL_LEFT);
    } else {
      return getNearest(RED_CORAL_RIGHT, BLUE_CORAL_RIGHT);
    }
  }

  public DriveToPose getNearest(List<Pose2d> posesRed, List<Pose2d> posesBlue) {
    return new DriveToPose(
        RobotContainer.DRIVETRAIN,
        () ->
            RobotContainer.DRIVETRAIN
                .getPose()
                .nearest(
                    RobotContainer.DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
                        ? posesRed
                        : posesBlue));
  }
}
