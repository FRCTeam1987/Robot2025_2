package frc.robot.state.logic.constants;

import java.util.ArrayList;
import java.util.Arrays;

public class FieldConstants {

  public static ArrayList<FieldPosition> CORAL_RIGHT_L1 =
      new ArrayList<>(
          Arrays.asList(
              FieldPosition.B1,
              FieldPosition.D1,
              FieldPosition.F1,
              FieldPosition.H1,
              FieldPosition.J1,
              FieldPosition.L1));

  public static ArrayList<FieldPosition> CORAL_LEFT_L1 =
      new ArrayList<>(
          Arrays.asList(
              FieldPosition.A1,
              FieldPosition.C1,
              FieldPosition.E1,
              FieldPosition.G1,
              FieldPosition.I1,
              FieldPosition.K1));

  public static ArrayList<FieldPosition> CORAL_RIGHT_L2 =
      new ArrayList<>(
          Arrays.asList(
              FieldPosition.B2,
              FieldPosition.D2,
              FieldPosition.F2,
              FieldPosition.H2,
              FieldPosition.J2,
              FieldPosition.L2));

  public static ArrayList<FieldPosition> CORAL_LEFT_L2 =
      new ArrayList<>(
          Arrays.asList(
              FieldPosition.A2,
              FieldPosition.C2,
              FieldPosition.E2,
              FieldPosition.G2,
              FieldPosition.I2,
              FieldPosition.K2));

  public static ArrayList<FieldPosition> CORAL_RIGHT_L3 =
      new ArrayList<>(
          Arrays.asList(
              FieldPosition.B3,
              FieldPosition.D3,
              FieldPosition.F3,
              FieldPosition.H3,
              FieldPosition.J3,
              FieldPosition.L3));

  public static ArrayList<FieldPosition> CORAL_LEFT_L3 =
      new ArrayList<>(
          Arrays.asList(
              FieldPosition.A3,
              FieldPosition.C3,
              FieldPosition.E3,
              FieldPosition.G3,
              FieldPosition.I3,
              FieldPosition.K3));

  public static ArrayList<FieldPosition> CORAL_RIGHT_L4 =
      new ArrayList<>(
          Arrays.asList(
              FieldPosition.B4,
              FieldPosition.D4,
              FieldPosition.F4,
              FieldPosition.H4,
              FieldPosition.J4,
              FieldPosition.L4));

  public static ArrayList<FieldPosition> CORAL_LEFT_L4 =
      new ArrayList<>(
          Arrays.asList(
              FieldPosition.A4,
              FieldPosition.C4,
              FieldPosition.E4,
              FieldPosition.G4,
              FieldPosition.I4,
              FieldPosition.K4));

  public static ArrayList<FieldPosition> ALGAE_2 =
      new ArrayList<>(Arrays.asList(FieldPosition.AL2, FieldPosition.AL4, FieldPosition.AL6));

  public static ArrayList<FieldPosition> ALGAE_3 =
      new ArrayList<>(Arrays.asList(FieldPosition.AL1, FieldPosition.AL3, FieldPosition.AL5));

  public static ArrayList<FieldPosition> COLLECT_LEFT =
      new ArrayList<>(Arrays.asList(FieldPosition.LC2));
  public static ArrayList<FieldPosition> COLLECT_RIGHT =
      new ArrayList<>(Arrays.asList(FieldPosition.RC2));

  public static ArrayList<FieldPosition> CLIMB =
      new ArrayList<>(Arrays.asList(FieldPosition.C1, FieldPosition.C2, FieldPosition.C3));
}
