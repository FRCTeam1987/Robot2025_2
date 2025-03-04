package frc.robot.state.logic.constants;

import frc.robot.state.logic.mode.CollectMode;
import frc.robot.state.logic.mode.ScoreMode;

public enum FieldPosition {
  A2(PositionConstant.SIDE_1_A, ScoreMode.L2, PositionConstant.SIDE_1_ENTRY),
  A3(PositionConstant.SIDE_1_A, ScoreMode.L3, PositionConstant.SIDE_1_ENTRY),
  A4(PositionConstant.SIDE_1_A, ScoreMode.L4, PositionConstant.SIDE_1_ENTRY),
  B2(PositionConstant.SIDE_1_B, ScoreMode.L2, PositionConstant.SIDE_1_ENTRY),
  B3(PositionConstant.SIDE_1_B, ScoreMode.L3, PositionConstant.SIDE_1_ENTRY),
  B4(PositionConstant.SIDE_1_B, ScoreMode.L4, PositionConstant.SIDE_1_ENTRY),
  C2(PositionConstant.SIDE_2_C, ScoreMode.L2, PositionConstant.SIDE_2_ENTRY),
  C3(PositionConstant.SIDE_2_C, ScoreMode.L3, PositionConstant.SIDE_2_ENTRY),
  C4(PositionConstant.SIDE_2_C, ScoreMode.L4, PositionConstant.SIDE_2_ENTRY),
  D2(PositionConstant.SIDE_2_D, ScoreMode.L2, PositionConstant.SIDE_2_ENTRY),
  D3(PositionConstant.SIDE_2_D, ScoreMode.L3, PositionConstant.SIDE_2_ENTRY),
  D4(PositionConstant.SIDE_2_D, ScoreMode.L4, PositionConstant.SIDE_2_ENTRY),
  E2(PositionConstant.SIDE_3_E, ScoreMode.L2, PositionConstant.SIDE_3_ENTRY),
  E3(PositionConstant.SIDE_3_E, ScoreMode.L3, PositionConstant.SIDE_3_ENTRY),
  E4(PositionConstant.SIDE_3_E, ScoreMode.L4, PositionConstant.SIDE_3_ENTRY),
  F2(PositionConstant.SIDE_3_F, ScoreMode.L2, PositionConstant.SIDE_3_ENTRY),
  F3(PositionConstant.SIDE_3_F, ScoreMode.L3, PositionConstant.SIDE_3_ENTRY),
  F4(PositionConstant.SIDE_3_F, ScoreMode.L4, PositionConstant.SIDE_3_ENTRY),
  G2(PositionConstant.SIDE_4_G, ScoreMode.L2, PositionConstant.SIDE_4_ENTRY),
  G3(PositionConstant.SIDE_4_G, ScoreMode.L3, PositionConstant.SIDE_4_ENTRY),
  G4(PositionConstant.SIDE_4_G, ScoreMode.L4, PositionConstant.SIDE_4_ENTRY),
  H2(PositionConstant.SIDE_4_H, ScoreMode.L2, PositionConstant.SIDE_4_ENTRY),
  H3(PositionConstant.SIDE_4_H, ScoreMode.L3, PositionConstant.SIDE_4_ENTRY),
  H4(PositionConstant.SIDE_4_H, ScoreMode.L4, PositionConstant.SIDE_4_ENTRY),
  I2(PositionConstant.SIDE_5_I, ScoreMode.L2, PositionConstant.SIDE_5_ENTRY),
  I3(PositionConstant.SIDE_5_I, ScoreMode.L3, PositionConstant.SIDE_5_ENTRY),
  I4(PositionConstant.SIDE_5_I, ScoreMode.L4, PositionConstant.SIDE_5_ENTRY),
  J2(PositionConstant.SIDE_5_J, ScoreMode.L2, PositionConstant.SIDE_5_ENTRY),
  J3(PositionConstant.SIDE_5_J, ScoreMode.L3, PositionConstant.SIDE_5_ENTRY),
  J4(PositionConstant.SIDE_5_J, ScoreMode.L4, PositionConstant.SIDE_5_ENTRY),
  K2(PositionConstant.SIDE_6_K, ScoreMode.L2, PositionConstant.SIDE_6_ENTRY),
  K3(PositionConstant.SIDE_6_K, ScoreMode.L3, PositionConstant.SIDE_6_ENTRY),
  K4(PositionConstant.SIDE_6_K, ScoreMode.L4, PositionConstant.SIDE_6_ENTRY),
  L2(PositionConstant.SIDE_6_L, ScoreMode.L2, PositionConstant.SIDE_6_ENTRY),
  L3(PositionConstant.SIDE_6_L, ScoreMode.L3, PositionConstant.SIDE_6_ENTRY),
  L4(PositionConstant.SIDE_6_L, ScoreMode.L4, PositionConstant.SIDE_6_ENTRY),

  N1(PositionConstant.N1, ScoreMode.NET, PositionConstant.N1),
  N2(PositionConstant.N2, ScoreMode.NET, PositionConstant.N2),
  N3(PositionConstant.N3, ScoreMode.NET, PositionConstant.N3),

  P1(PositionConstant.P1, ScoreMode.PROCESSOR, PositionConstant.P1),

  LC1(PositionConstant.LC1, CollectMode.HUMAN_PLAYER_STATION, PositionConstant.LC1),
  LC2(PositionConstant.LC2, CollectMode.HUMAN_PLAYER_STATION, PositionConstant.LC2),
  LC3(PositionConstant.LC3, CollectMode.HUMAN_PLAYER_STATION, PositionConstant.LC3),

  RC1(PositionConstant.RC1, CollectMode.HUMAN_PLAYER_STATION, PositionConstant.RC1),
  RC2(PositionConstant.RC2, CollectMode.HUMAN_PLAYER_STATION, PositionConstant.RC2),
  RC3(PositionConstant.RC3, CollectMode.HUMAN_PLAYER_STATION, PositionConstant.RC3),
  ;

  public PositionConstant getLocation() {
    return LOCATION;
  }

  public PositionConstant getPreLocation() {
    return PRE_LOCATION != null ? PRE_LOCATION : LOCATION;
  }

  public ScoreMode getMode() {
    return SCORE_MODE;
  }

  private final ScoreMode SCORE_MODE;
  private final CollectMode COLLECT_MODE;
  private final PositionConstant LOCATION;
  private final PositionConstant PRE_LOCATION;

  FieldPosition(PositionConstant location, ScoreMode mode, PositionConstant preloc) {
    this.SCORE_MODE = mode;
    this.LOCATION = location;
    this.PRE_LOCATION = preloc;

    this.COLLECT_MODE = CollectMode.HUMAN_PLAYER_STATION;
  }

  FieldPosition(PositionConstant location, CollectMode mode, PositionConstant preloc) {
    this.COLLECT_MODE = mode;
    this.LOCATION = location;
    this.PRE_LOCATION = preloc;

    this.SCORE_MODE = ScoreMode.DEFENSE;
  }
}
