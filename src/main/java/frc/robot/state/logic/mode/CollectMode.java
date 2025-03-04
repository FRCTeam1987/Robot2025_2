package frc.robot.state.logic.mode;

import frc.robot.state.logic.constants.MechanismConstant;

public enum CollectMode {
  HUMAN_PLAYER_STATION(MechanismConstant.HP_INTAKE),
  ALGAE_2(MechanismConstant.A2),
  ALGAE_3(MechanismConstant.A3);

  public MechanismConstant getMechanismConstant() {
    return COLLECT_POS;
  }

  private final MechanismConstant COLLECT_POS;

  CollectMode(MechanismConstant INTAKE) {
    COLLECT_POS = INTAKE;
  }
}
