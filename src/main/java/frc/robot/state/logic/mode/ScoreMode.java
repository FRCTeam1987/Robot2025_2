package frc.robot.state.logic.mode;

import frc.robot.state.logic.constants.MechanismConstant;

public enum ScoreMode {
  L1(MechanismConstant.L1, MechanismConstant.L1),
  L2(MechanismConstant.L2, MechanismConstant.L2),
  L3(MechanismConstant.L3, MechanismConstant.L3),
  L4(MechanismConstant.L4, MechanismConstant.L4),
  PROCESSOR(MechanismConstant.PROCESSOR, MechanismConstant.IDLE_ALGAE_PROCESSOR),
  NET(MechanismConstant.NET, MechanismConstant.IDLE_ALGAE_NET),
  DEFENSE(MechanismConstant.DEFENSE),
  CLIMB(MechanismConstant.CLIMB);

  public MechanismConstant getMechanismConstant() {
    return SCORE_POS;
  }

  public MechanismConstant getIdleMechanismConstant() {
    return IDLE_POS;
  }

  private final MechanismConstant SCORE_POS;
  private final MechanismConstant IDLE_POS;

  ScoreMode(MechanismConstant SCORING) {
    this(SCORING, MechanismConstant.IDLE_ALGAE_PROCESSOR);
  }

  ScoreMode(MechanismConstant SCORING, MechanismConstant COLLECT) {
    SCORE_POS = SCORING;
    IDLE_POS = COLLECT;
  }
}
