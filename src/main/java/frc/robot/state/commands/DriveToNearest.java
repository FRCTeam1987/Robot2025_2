package frc.robot.state.commands;

import static frc.robot.state.logic.constants.FieldConstants.*;
import static frc.robot.state.logic.constants.StateConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.state.Abomination;
import frc.robot.state.logic.constants.PositionConstant;
import frc.robot.state.logic.functional.FunctionalState;
import frc.robot.state.logic.mode.ScoreMode;
import frc.robot.utils.Utils;

public class DriveToNearest extends SequentialCommandGroup {

  public DriveToNearest(boolean isLeft, boolean includeOpposing) {
    super();
    addCommands(
        new DriveToPose(
            () -> calculateNearest(isLeft, includeOpposing),
            () ->
                Abomination.getScoreMode().equals(ScoreMode.CLIMB)
                    || (Abomination.getScoreMode() == ScoreMode.NET
                        && Abomination.getPreviousState() != FunctionalState.COLLECT)));
  }

  public Pose2d calculateNearest(boolean isLeft, boolean includeOpposing) {
    if (Abomination.getPreviousState() == FunctionalState.COLLECT) {
      switch (Abomination.getCollectMode()) {
        case HUMAN_PLAYER_STATION -> {
          if (isLeft) {
            return Utils.getNearest(RED_COLLECT_LEFT, BLUE_COLLECT_LEFT, includeOpposing);
          } else {
            return Utils.getNearest(RED_COLLECT_RIGHT, BLUE_COLLECT_RIGHT, includeOpposing);
          }
        }
        case ALGAE_2 -> {
          if (isLeft) {
            return Utils.getNearest(RED_PRE_2, BLUE_PRE_2, true);
          } else {
            return Utils.getNearest(RED_ALGAE_2, BLUE_ALGAE_2, true);
          }
        }
        case ALGAE_3 -> {
          if (isLeft) {
            return Utils.getNearest(RED_PRE_3, BLUE_PRE_3, true);
          } else {
            return Utils.getNearest(RED_ALGAE_3, BLUE_ALGAE_3, true);
          }
        }
      }
    } else {
      switch (Abomination.getScoreMode()) {
        case L1 -> {
          if (isLeft) {
            return Utils.getNearest(RED_CORAL_LEFT_L1, BLUE_CORAL_LEFT_L1, false);
          } else {
            return Utils.getNearest(RED_CORAL_RIGHT_L1, BLUE_CORAL_RIGHT_L1, false);
          }
        }
        case L2, L3, L4 -> {
          if (isLeft) {
            return Utils.getNearest(RED_CORAL_LEFT, BLUE_CORAL_LEFT, false);
          } else {
            return Utils.getNearest(RED_CORAL_RIGHT, BLUE_CORAL_RIGHT, false);
          }
        }
        case PROCESSOR -> {
          return RobotContainer.DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
              ? PositionConstant.P1.getRedPose()
              : PositionConstant.P1.getBluePose();
        }
        case NET -> {
          return Utils.getNearest(RED_NET, BLUE_NET, false);
        }
        case CLIMB -> {
          return Utils.getNearest(RED_CLIMB, BLUE_CLIMB, false);
        }
      }
    }
    if (isLeft) {
      return Utils.getNearest(RED_CORAL_LEFT, BLUE_CORAL_LEFT, false);
    } else {
      return Utils.getNearest(RED_CORAL_RIGHT, BLUE_CORAL_RIGHT, false);
    }
  }
}
