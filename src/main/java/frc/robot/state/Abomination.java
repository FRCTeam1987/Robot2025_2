package frc.robot.state;

import static frc.robot.RobotContainer.*;
import static frc.robot.state.logic.functional.FunctionalState.*;

import dev.doglog.DogLog;
import frc.robot.state.logic.actions.DesiredAction;
import frc.robot.state.logic.functional.FunctionalState;
import frc.robot.state.logic.mode.CollectMode;
import frc.robot.state.logic.mode.ScoreMode;

public class Abomination {

  private static CollectMode COLLECT_MODE = CollectMode.HUMAN_PLAYER_STATION;
  private static ScoreMode SCORE_MODE = ScoreMode.L4;
  private static DesiredAction DESIRED_ACTION = DesiredAction.IDLE_CORAL;
  private static FunctionalState PREVIOUS_STATE = FunctionalState.COLLECT;

  public static FunctionalState calculateRobotState() {
    // master overrides
    if (isDesired(DesiredAction.RECOVERY)) {
      setAction(DesiredAction.IDLE_CORAL);
      return COLLECT;
    }

    if (SCORE_MODE == ScoreMode.CLIMB
        && PREVIOUS_STATE != CLIMB_DEPLOY
        && PREVIOUS_STATE != CLIMB_CLIMB) {
      return CLIMB_DEPLOY;
    }

    switch (PREVIOUS_STATE) {
      case COLLECT -> {
        switch (COLLECT_MODE) {
          case HUMAN_PLAYER_STATION -> {
            if (ARM.hasGamePiece()) {
              if (SCORE_MODE != ScoreMode.L1
                  && SCORE_MODE != ScoreMode.L2
                  && SCORE_MODE != ScoreMode.L3
                  && SCORE_MODE != ScoreMode.L4) {
                setScoreMode(ScoreMode.L4, true);
                setAction(DesiredAction.IDLE_CORAL);
              }
              return COLLECTED_CORAL;
            }
            return COLLECT;
          }
          case ALGAE_2, ALGAE_3 -> {
            if (ARM.hasAlgae()) {
              if (SCORE_MODE != ScoreMode.NET && SCORE_MODE != ScoreMode.PROCESSOR) {
                setScoreMode(ScoreMode.PROCESSOR, true);
                setAction(DesiredAction.IDLE_ALGAE);
              }
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
            if (isDesired(DesiredAction.INIT, DesiredAction.SCORE)) {
              return NET_ELEVATE;
            }
          }
          case PROCESSOR -> {
            if (isDesired(DesiredAction.INIT, DesiredAction.SCORE)) {
              return PROCESSOR_ELEVATE;
            }
          }
        }
        return COLLECTED_ALGAE;
      }
      case COLLECTED_CORAL -> {
        if (!ARM.hasGamePieceEntrance()) return COLLECT;
        switch (SCORE_MODE) {
          case L1, L2, L3, L4 -> {
            if (isDesired(DesiredAction.INIT) || shouldScore()) {
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
            if (shouldScore()) {
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
            if (ELEVATOR.isNearTarget()) {
              setAction(DesiredAction.IDLE_ALGAE);
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
            if (ELEVATOR.isNearTarget()) return PROCESSOR_ROTATE;
            return PROCESSOR_ELEVATE;
          }
          case PROCESSOR_ROTATE -> {
            if (!ARM.isNearTarget()) return PROCESSOR_ROTATE;
            if (shouldScore()) {
              return PROCESSOR_SCORE;
            }
            return PROCESSOR_ROTATE;
          }
          case PROCESSOR_SCORE -> {
            if (!ARM.hasAlgae()) {
              setAction(DesiredAction.IDLE_ALGAE);
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
            if (ELEVATOR.isNearTarget()) return LEVEL_X_ROTATE;
            return LEVEL_X_ELEVATE;
          }
          case LEVEL_X_ROTATE -> {
            if (!ARM.isAtTarget()) return LEVEL_X_ROTATE;
            if (shouldScore()) {
              return LEVEL_X_SCORE;
            }
            return LEVEL_X_ROTATE;
          }
          case LEVEL_X_SCORE -> {
            if (!ARM.hasGamePieceEntrance()) return LEVEL_X_UNROTATE;
            return LEVEL_X_SCORE;
          }
          case LEVEL_X_UNROTATE -> {
            if (ARM.getArmPosition().lt(getScoreMode().getMechanismConstant().getArmAngle()))
              return LEVEL_X_UNELEVATE;
            return LEVEL_X_UNROTATE;
          }
          case LEVEL_X_UNELEVATE -> {
            setAction(DesiredAction.IDLE_CORAL);
            if (!ELEVATOR.isNearTarget()) return LEVEL_X_UNELEVATE;
            return COLLECT;
          }
        }
      }
      case CLIMB_DEPLOY, CLIMB_CLIMB -> {
        switch (PREVIOUS_STATE) {
          case CLIMB_DEPLOY -> {
            if (CLIMBER.isAtTarget()) {
              if (shouldScore()) {
                return CLIMB_CLIMB;
              }
              if (isDesired(DesiredAction.RECOVERY)) {
                return CLIMB_STOW;
              }
            }
            return CLIMB_DEPLOY;
          }
          case CLIMB_CLIMB -> {
            if (isDesired(DesiredAction.INIT)) {
              return CLIMB_DEPLOY;
            }
            return CLIMB_CLIMB;
          }
        }
      }
    }
    return COLLECT;
  }

  public static void setScoreMode(ScoreMode MODE, boolean force) {
    if (force) {
      SCORE_MODE = MODE;
      return;
    }
    switch (MODE) {
      case L1, L2, L3, L4 -> {
        if (stateContains(
            NET_ELEVATE,
            NET_ROTATE,
            NET_SCORE,
            PROCESSOR_ELEVATE,
            PROCESSOR_ROTATE,
            PROCESSOR_SCORE,
            COLLECTED_ALGAE)) {
          return;
        }
      }
      case PROCESSOR, NET -> {
        if (stateContains(LEVEL_X_ELEVATE, LEVEL_X_ROTATE, LEVEL_X_SCORE, COLLECTED_CORAL)) {
          return;
        }
      }
      case CLIMB -> {
        if (!(stateContains(COLLECT, COLLECTED_ALGAE, COLLECTED_CORAL))) return;
      }
    }
    SCORE_MODE = MODE;
  }

  public static ScoreMode getScoreMode() {
    return SCORE_MODE;
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

  public static boolean shouldScore() {
    return DESIRED_ACTION == DesiredAction.SCORE;
  }

  public static boolean stateContains(FunctionalState... ACTION) {
    for (FunctionalState functionalState : ACTION) {
      if (PREVIOUS_STATE == functionalState) return true;
    }
    return false;
  }

  public static boolean isDesired(DesiredAction... ACTION) {
    for (DesiredAction desiredAction : ACTION) {
      if (DESIRED_ACTION == desiredAction) return true;
    }
    return false;
  }

  public static FunctionalState getState() {

    FunctionalState STATE = calculateRobotState();
    DogLog.log("STATE/Robot State", STATE.toString());
    DogLog.log("STATE/Score Mode", SCORE_MODE.toString());
    DogLog.log("STATE/Desired Action", DESIRED_ACTION.toString());
    PREVIOUS_STATE = STATE;
    return STATE;
  }

  public static FunctionalState getPreviousState() {
    return PREVIOUS_STATE;
  }
}
