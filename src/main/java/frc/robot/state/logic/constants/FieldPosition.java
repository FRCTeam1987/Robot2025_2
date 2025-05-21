package frc.robot.state.logic.constants;

import frc.robot.RobotContainer;
import frc.robot.state.logic.mode.CollectMode;
import frc.robot.state.logic.mode.ScoreMode;

public enum FieldPosition {
  A1(PositionConstant.SIDE_1_A1, ScoreMode.L1, PositionConstant.SIDE_1_ENTRY),
  A2(PositionConstant.SIDE_1_A, ScoreMode.L2, PositionConstant.SIDE_1_ENTRY),
  A3(PositionConstant.SIDE_1_A, ScoreMode.L3, PositionConstant.SIDE_1_ENTRY),
  A4(PositionConstant.SIDE_1_A, ScoreMode.L4, PositionConstant.SIDE_1_ENTRY),
  B1(PositionConstant.SIDE_1_B1, ScoreMode.L1, PositionConstant.SIDE_1_ENTRY),
  B2(PositionConstant.SIDE_1_B, ScoreMode.L2, PositionConstant.SIDE_1_ENTRY),
  B3(PositionConstant.SIDE_1_B, ScoreMode.L3, PositionConstant.SIDE_1_ENTRY),
  B4(PositionConstant.SIDE_1_B, ScoreMode.L4, PositionConstant.SIDE_1_ENTRY),
  C1(PositionConstant.SIDE_2_C1, ScoreMode.L1, PositionConstant.SIDE_2_ENTRY),
  C2(PositionConstant.SIDE_2_C, ScoreMode.L2, PositionConstant.SIDE_2_ENTRY),
  C3(PositionConstant.SIDE_2_C, ScoreMode.L3, PositionConstant.SIDE_2_ENTRY),
  C4(PositionConstant.SIDE_2_C, ScoreMode.L4, PositionConstant.SIDE_2_ENTRY),
  D1(PositionConstant.SIDE_2_D1, ScoreMode.L1, PositionConstant.SIDE_2_ENTRY),
  D2(PositionConstant.SIDE_2_D, ScoreMode.L2, PositionConstant.SIDE_2_ENTRY),
  D3(PositionConstant.SIDE_2_D, ScoreMode.L3, PositionConstant.SIDE_2_ENTRY),
  D4(PositionConstant.SIDE_2_D, ScoreMode.L4, PositionConstant.SIDE_2_ENTRY),
  E1(PositionConstant.SIDE_3_E1, ScoreMode.L1, PositionConstant.SIDE_3_ENTRY),
  E2(PositionConstant.SIDE_3_E, ScoreMode.L2, PositionConstant.SIDE_3_ENTRY),
  E3(PositionConstant.SIDE_3_E, ScoreMode.L3, PositionConstant.SIDE_3_ENTRY),
  E4(PositionConstant.SIDE_3_E, ScoreMode.L4, PositionConstant.SIDE_3_ENTRY),
  F1(PositionConstant.SIDE_3_F1, ScoreMode.L1, PositionConstant.SIDE_3_ENTRY),
  F2(PositionConstant.SIDE_3_F, ScoreMode.L2, PositionConstant.SIDE_3_ENTRY),
  F3(PositionConstant.SIDE_3_F, ScoreMode.L3, PositionConstant.SIDE_3_ENTRY),
  F4(PositionConstant.SIDE_3_F, ScoreMode.L4, PositionConstant.SIDE_3_ENTRY),
  G1(PositionConstant.SIDE_4_G1, ScoreMode.L1, PositionConstant.SIDE_4_ENTRY),
  G2(PositionConstant.SIDE_4_G, ScoreMode.L2, PositionConstant.SIDE_4_ENTRY),
  G3(PositionConstant.SIDE_4_G, ScoreMode.L3, PositionConstant.SIDE_4_ENTRY),
  G4(PositionConstant.SIDE_4_G, ScoreMode.L4, PositionConstant.SIDE_4_ENTRY),
  H1(PositionConstant.SIDE_4_H1, ScoreMode.L1, PositionConstant.SIDE_4_ENTRY),
  H2(PositionConstant.SIDE_4_H, ScoreMode.L2, PositionConstant.SIDE_4_ENTRY),
  H3(PositionConstant.SIDE_4_H, ScoreMode.L3, PositionConstant.SIDE_4_ENTRY),
  H4(PositionConstant.SIDE_4_H, ScoreMode.L4, PositionConstant.SIDE_4_ENTRY),
  I1(PositionConstant.SIDE_5_I1, ScoreMode.L1, PositionConstant.SIDE_5_ENTRY),
  I2(PositionConstant.SIDE_5_I, ScoreMode.L2, PositionConstant.SIDE_5_ENTRY),
  I3(PositionConstant.SIDE_5_I, ScoreMode.L3, PositionConstant.SIDE_5_ENTRY),
  I4(PositionConstant.SIDE_5_I, ScoreMode.L4, PositionConstant.SIDE_5_ENTRY),
  J1(PositionConstant.SIDE_5_J1, ScoreMode.L1, PositionConstant.SIDE_5_ENTRY),
  J2(PositionConstant.SIDE_5_J, ScoreMode.L2, PositionConstant.SIDE_5_ENTRY),
  J3(PositionConstant.SIDE_5_J, ScoreMode.L3, PositionConstant.SIDE_5_ENTRY),
  J4(PositionConstant.SIDE_5_J, ScoreMode.L4, PositionConstant.SIDE_5_ENTRY),
  K1(PositionConstant.SIDE_6_K1, ScoreMode.L1, PositionConstant.SIDE_6_ENTRY),
  K2(PositionConstant.SIDE_6_K, ScoreMode.L2, PositionConstant.SIDE_6_ENTRY),
  K3(PositionConstant.SIDE_6_K, ScoreMode.L3, PositionConstant.SIDE_6_ENTRY),
  K4(PositionConstant.SIDE_6_K, ScoreMode.L4, PositionConstant.SIDE_6_ENTRY),
  L1(PositionConstant.SIDE_6_L1, ScoreMode.L1, PositionConstant.SIDE_6_ENTRY),
  L2(PositionConstant.SIDE_6_L, ScoreMode.L2, PositionConstant.SIDE_6_ENTRY),
  L3(PositionConstant.SIDE_6_L, ScoreMode.L3, PositionConstant.SIDE_6_ENTRY),
  L4(PositionConstant.SIDE_6_L, ScoreMode.L4, PositionConstant.SIDE_6_ENTRY),

  AL1(PositionConstant.SIDE_1_ALGAE, CollectMode.ALGAE_3, PositionConstant.SIDE_1_ENTRY),
  AL2(PositionConstant.SIDE_2_ALGAE, CollectMode.ALGAE_2, PositionConstant.SIDE_2_ENTRY),
  AL3(PositionConstant.SIDE_3_ALGAE, CollectMode.ALGAE_3, PositionConstant.SIDE_3_ENTRY),
  AL4(PositionConstant.SIDE_4_ALGAE, CollectMode.ALGAE_2, PositionConstant.SIDE_4_ENTRY),
  AL5(PositionConstant.SIDE_5_ALGAE, CollectMode.ALGAE_3, PositionConstant.SIDE_5_ENTRY),
  AL6(PositionConstant.SIDE_6_ALGAE, CollectMode.ALGAE_2, PositionConstant.SIDE_6_ENTRY),

  ALL_NET(PositionConstant.ALL_NET, ScoreMode.NET, PositionConstant.ALL_NET),
  OPP_NET(PositionConstant.OPP_NET, ScoreMode.NET, PositionConstant.OPP_NET),

  P1(PositionConstant.P1, ScoreMode.PROCESSOR, PositionConstant.P1),

  LC2(PositionConstant.LC2, CollectMode.HUMAN_PLAYER_STATION, PositionConstant.LC2),

  RC2(PositionConstant.RC2, CollectMode.HUMAN_PLAYER_STATION, PositionConstant.RC2),
  ;

  public PositionConstant getLocation() {
    return LOCATION;
  }

  public boolean isScored() {
    return RobotContainer.TRACKER.isScored(this);
  }

  public void setScored(boolean setScored) {
    RobotContainer.TRACKER.setScored(this, setScored);
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
