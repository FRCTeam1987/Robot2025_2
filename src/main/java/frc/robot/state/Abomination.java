package frc.robot.state;

import static frc.robot.RobotContainer.*;
import static frc.robot.state.logic.functional.FunctionalState.*;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.state.commands.DriveToPose;
import frc.robot.state.logic.actions.DesiredAction;
import frc.robot.state.logic.constants.FieldPosition;
import frc.robot.state.logic.functional.FunctionalState;
import frc.robot.state.logic.mode.CollectMode;
import frc.robot.state.logic.mode.DriveMode;
import frc.robot.state.logic.mode.ScoreMode;
import frc.robot.utils.localization.LocalizationState;

public class Abomination {

  private static CollectMode COLLECT_MODE = CollectMode.HUMAN_PLAYER_STATION;
  private static ScoreMode SCORE_MODE = ScoreMode.L4;
  private static DriveMode DRIVE_MODE = DriveMode.MANUAL;
  private static DesiredAction DESIRED_ACTION = DesiredAction.IDLE_CORAL;
  private static FunctionalState PREVIOUS_STATE = FunctionalState.COLLECT;
  private static Command PATHFINDING_COMMAND;
  private static DriveToPose HOLONOMIC_COMMAND;

  public static FunctionalState calculateRobotState() {
    LocalizationState LOC = RobotContainer.getLocalizationState();
    FieldPosition desiredPosition = Strategy.getCurrentFieldPosition();
    ScoreMode MODE = desiredPosition.getMode();

    PATHFINDING_COMMAND = desiredPosition.getPreLocation().getAlliancePath();
    HOLONOMIC_COMMAND = desiredPosition.getLocation().getAllianceHolo();

    if (SCORE_MODE != MODE && isAutomatic()) {
      SCORE_MODE = MODE;
    }
    switch (PREVIOUS_STATE) {
      case COLLECT -> {
        switch (COLLECT_MODE) {
          case HUMAN_PLAYER_STATION -> {
            if (isAutomatic()) schedulePathfind();
            if (ARM.hasGamePiece()) {
              Strategy.next();
              return COLLECTED_CORAL;
            }
            return COLLECT;
          }
          case ALGAE_2, ALGAE_3 -> {
            if (ARM.hasAlgae()) {
              Strategy.next();
              return COLLECTED_ALGAE;
            }
            return COLLECT;
          }
        }
      }
      case COLLECTED_ALGAE -> {
        if (!ARM.hasAlgae()) return COLLECT;
        switch (SCORE_MODE) {
          case NET -> {
            if (isAutomatic()) {
            } else {
              if (DESIRED_ACTION == DesiredAction.INIT || DESIRED_ACTION == DesiredAction.SCORE) {
                return NET_ELEVATE;
              }
            }
          }
          case PROCESSOR -> {
            if (isAutomatic()) {
            } else {
              if (DESIRED_ACTION == DesiredAction.INIT || DESIRED_ACTION == DesiredAction.SCORE) {
                return PROCESSOR_ELEVATE;
              }
            }
          }
        }
        return COLLECTED_ALGAE;
      }
      case COLLECTED_CORAL -> {
        if (!ARM.hasGamePieceEntrance()) return COLLECT;
        switch (SCORE_MODE) {
          case L1, L2, L3, L4 -> {
            if (DESIRED_ACTION == DesiredAction.INIT || DESIRED_ACTION == DesiredAction.SCORE) {
              return LEVEL_X_ELEVATE;
            }
          }
        }
        return COLLECTED_CORAL;
      }
      case NET_ELEVATE, NET_ROTATE, NET_SCORE, NET_UNROTATE, NET_UNELEVATE -> {
        switch (PREVIOUS_STATE) {
          case NET_ELEVATE -> {
            if (ELEVATOR.isAtTarget()) return NET_ROTATE;
            return NET_ELEVATE;
          }
          case NET_ROTATE -> {
            if (!ARM.isAtTarget()) return NET_ROTATE;
            if (isAutomatic()) {
              if (ARM.isAtTarget()) return NET_SCORE;
            }
            if (DESIRED_ACTION == DesiredAction.SCORE) {
              return NET_SCORE;
            }
            return NET_ROTATE;
          }
          case NET_SCORE -> {
            if (!ARM.hasAlgae()) {
              return NET_UNROTATE;
            }
            return NET_SCORE;
          }
          case NET_UNROTATE -> {
            if (ARM.isAtTarget()) {
              return NET_UNELEVATE;
            }
            return NET_UNROTATE;
          }
          case NET_UNELEVATE -> {
            if (ELEVATOR.isAtTarget()) {
              setAction(DesiredAction.IDLE_CORAL);
              setCollectMode(CollectMode.HUMAN_PLAYER_STATION);
              return COLLECT;
            }
            return NET_UNELEVATE;
          }
        }
      }
      case PROCESSOR_ELEVATE, PROCESSOR_ROTATE, PROCESSOR_SCORE -> {
        switch (PREVIOUS_STATE) {
          case PROCESSOR_ELEVATE -> {
            if (ELEVATOR.isAtTarget()) return PROCESSOR_ROTATE;
            return PROCESSOR_ELEVATE;
          }
          case PROCESSOR_ROTATE -> {
            if (!ARM.isAtTarget()) return PROCESSOR_ROTATE;
            if (DESIRED_ACTION == DesiredAction.SCORE) {
              return PROCESSOR_SCORE;
            }
            return PROCESSOR_ROTATE;
          }
          case PROCESSOR_SCORE -> {
            if (!ARM.hasAlgae()) {
              setCollectMode(CollectMode.HUMAN_PLAYER_STATION);
              return COLLECT;
            }
            return PROCESSOR_SCORE;
          }
        }
      }
      case LEVEL_X_ELEVATE, LEVEL_X_ROTATE, LEVEL_X_SCORE, LEVEL_X_UNROTATE, LEVEL_X_UNELEVATE -> {
        switch (PREVIOUS_STATE) {
          case LEVEL_X_ELEVATE -> {
            if (ELEVATOR.isAtTarget()) return LEVEL_X_ROTATE;
            return LEVEL_X_ELEVATE;
          }
          case LEVEL_X_ROTATE -> {
            if (!ARM.isAtTarget()) return LEVEL_X_ROTATE;
            if (DESIRED_ACTION == DesiredAction.SCORE) {
              return LEVEL_X_SCORE;
            }
            return LEVEL_X_ROTATE;
          }
          case LEVEL_X_SCORE -> {
            if (!ARM.hasGamePieceEntrance()) return LEVEL_X_UNROTATE;
            return LEVEL_X_SCORE;
          }
          case LEVEL_X_UNROTATE -> {
            if (ARM.isAtTarget()) return LEVEL_X_UNELEVATE;
            return LEVEL_X_UNROTATE;
          }
          case LEVEL_X_UNELEVATE -> {
            if (!ELEVATOR.isAtTarget()) return LEVEL_X_UNELEVATE;
            return COLLECT;
          }
        }
      }
    }
    return COLLECT;
  }

  public static void setScoreMode(ScoreMode MODE) {
    SCORE_MODE = MODE;
  }

  public static ScoreMode getScoreMode() {
    return SCORE_MODE;
  }

  public static void setDriveMode(DriveMode MODE) {
    DRIVE_MODE = MODE;
  }

  public static DriveMode getDriveMode() {
    return DRIVE_MODE;
  }

  public static void setAction(DesiredAction ACTION) {
    DESIRED_ACTION = ACTION;
  }

  public static DesiredAction getAction() {
    return DESIRED_ACTION;
  }

  public static void setCollectMode(CollectMode MODE) {
    COLLECT_MODE = MODE;
  }

  public static CollectMode getCollectMode() {
    return COLLECT_MODE;
  }

  public static boolean isAutomatic() {
    return getDriveMode() == DriveMode.AUTOMATIC;
  }

  public static FunctionalState getState() {

    FunctionalState STATE = calculateRobotState();
    DogLog.log("STATE/Robot State", STATE.toString());
    DogLog.log("STATE/Drive Mode", DRIVE_MODE.toString());
    DogLog.log("STATE/Score Mode", SCORE_MODE.toString());
    DogLog.log("STATE/Desired Action", DESIRED_ACTION.toString());
    DogLog.log("STATE/Strategy State", Strategy.getCurrentFieldPosition().toString());
    PREVIOUS_STATE = STATE;
    return STATE;
  }

  public static FunctionalState getPreviousState() {
    return PREVIOUS_STATE;
  }

  public static void schedulePathfind() {
    if (!PATHFINDING_COMMAND.isScheduled()) {
      HOLONOMIC_COMMAND.cancel();
      PATHFINDING_COMMAND.schedule();
    }
  }

  public static void scheduleHolo() {
    PATHFINDING_COMMAND.cancel();
    if (!HOLONOMIC_COMMAND.isScheduled()) {
      HOLONOMIC_COMMAND.schedule();
    }
  }
}
