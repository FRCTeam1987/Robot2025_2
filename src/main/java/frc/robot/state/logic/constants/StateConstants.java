package frc.robot.state.logic.constants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.localization.FieldZones;
import java.util.List;

public class StateConstants {
  public static final List<FieldZones.Zone> COLLECT_ZONES =
      List.of(
          FieldZones.Zone.ALLIANCE_CORAL_LEFT,
          FieldZones.Zone.ALLIANCE_CORAL_RIGHT,
          FieldZones.Zone.ALLIANCE_REEF,
          FieldZones.Zone.OPPONENT_REEF);

  public static final List<Pose2d> BLUE_CORAL_RIGHT =
      List.of(
          PositionConstant.SIDE_1_B.getBluePose(),
          PositionConstant.SIDE_2_D.getBluePose(),
          PositionConstant.SIDE_3_F.getBluePose(),
          PositionConstant.SIDE_4_H.getBluePose(),
          PositionConstant.SIDE_5_J.getBluePose(),
          PositionConstant.SIDE_6_L.getBluePose());
  public static final List<Pose2d> RED_CORAL_RIGHT =
      List.of(
          PositionConstant.SIDE_1_B.getRedPose(),
          PositionConstant.SIDE_2_D.getRedPose(),
          PositionConstant.SIDE_3_F.getRedPose(),
          PositionConstant.SIDE_4_H.getRedPose(),
          PositionConstant.SIDE_5_J.getRedPose(),
          PositionConstant.SIDE_6_L.getRedPose());

  public static final List<Pose2d> BLUE_CORAL_LEFT =
      List.of(
          PositionConstant.SIDE_1_A.getBluePose(),
          PositionConstant.SIDE_2_C.getBluePose(),
          PositionConstant.SIDE_3_E.getBluePose(),
          PositionConstant.SIDE_4_G.getBluePose(),
          PositionConstant.SIDE_5_I.getBluePose(),
          PositionConstant.SIDE_6_K.getBluePose());
  public static final List<Pose2d> RED_CORAL_LEFT =
      List.of(
          PositionConstant.SIDE_1_A.getRedPose(),
          PositionConstant.SIDE_2_C.getRedPose(),
          PositionConstant.SIDE_3_E.getRedPose(),
          PositionConstant.SIDE_4_G.getRedPose(),
          PositionConstant.SIDE_5_I.getRedPose(),
          PositionConstant.SIDE_6_K.getRedPose());

  public static final List<Pose2d> BLUE_CORAL_RIGHT_L1 =
      List.of(
          PositionConstant.SIDE_1_B1.getBluePose(),
          PositionConstant.SIDE_2_D1.getBluePose(),
          PositionConstant.SIDE_3_F1.getBluePose(),
          PositionConstant.SIDE_4_H1.getBluePose(),
          PositionConstant.SIDE_5_J1.getBluePose(),
          PositionConstant.SIDE_6_L1.getBluePose());
  public static final List<Pose2d> RED_CORAL_RIGHT_L1 =
      List.of(
          PositionConstant.SIDE_1_B1.getRedPose(),
          PositionConstant.SIDE_2_D1.getRedPose(),
          PositionConstant.SIDE_3_F1.getRedPose(),
          PositionConstant.SIDE_4_H1.getRedPose(),
          PositionConstant.SIDE_5_J1.getRedPose(),
          PositionConstant.SIDE_6_L1.getRedPose());

  public static final List<Pose2d> BLUE_CORAL_LEFT_L1 =
      List.of(
          PositionConstant.SIDE_1_A1.getBluePose(),
          PositionConstant.SIDE_2_C1.getBluePose(),
          PositionConstant.SIDE_3_E1.getBluePose(),
          PositionConstant.SIDE_4_G1.getBluePose(),
          PositionConstant.SIDE_5_I1.getBluePose(),
          PositionConstant.SIDE_6_K1.getBluePose());
  public static final List<Pose2d> RED_CORAL_LEFT_L1 =
      List.of(
          PositionConstant.SIDE_1_A1.getRedPose(),
          PositionConstant.SIDE_2_C1.getRedPose(),
          PositionConstant.SIDE_3_E1.getRedPose(),
          PositionConstant.SIDE_4_G1.getRedPose(),
          PositionConstant.SIDE_5_I1.getRedPose(),
          PositionConstant.SIDE_6_K1.getRedPose());

  public static final List<Pose2d> BLUE_TARGET_POSES_CORAL =
      List.of(
          PositionConstant.SIDE_1_A.getBluePose(),
          PositionConstant.SIDE_1_B.getBluePose(),
          PositionConstant.SIDE_2_C.getBluePose(),
          PositionConstant.SIDE_2_D.getBluePose(),
          PositionConstant.SIDE_3_E.getBluePose(),
          PositionConstant.SIDE_3_F.getBluePose(),
          PositionConstant.SIDE_4_G.getBluePose(),
          PositionConstant.SIDE_4_H.getBluePose(),
          PositionConstant.SIDE_5_I.getBluePose(),
          PositionConstant.SIDE_5_J.getBluePose(),
          PositionConstant.SIDE_6_K.getBluePose(),
          PositionConstant.SIDE_6_L.getBluePose());
  public static final List<Pose2d> RED_TARGET_POSES_CORAL =
      List.of(
          PositionConstant.SIDE_1_A.getRedPose(),
          PositionConstant.SIDE_1_B.getRedPose(),
          PositionConstant.SIDE_2_C.getRedPose(),
          PositionConstant.SIDE_2_D.getRedPose(),
          PositionConstant.SIDE_3_E.getRedPose(),
          PositionConstant.SIDE_3_F.getRedPose(),
          PositionConstant.SIDE_4_G.getRedPose(),
          PositionConstant.SIDE_4_H.getRedPose(),
          PositionConstant.SIDE_5_I.getRedPose(),
          PositionConstant.SIDE_5_J.getRedPose(),
          PositionConstant.SIDE_6_K.getRedPose(),
          PositionConstant.SIDE_6_L.getRedPose());

  public static final List<Pose2d> BLUE_ALGAE =
      List.of(
          PositionConstant.SIDE_1_ALGAE.getBluePose(),
          PositionConstant.SIDE_2_ALGAE.getBluePose(),
          PositionConstant.SIDE_3_ALGAE.getBluePose(),
          PositionConstant.SIDE_4_ALGAE.getBluePose(),
          PositionConstant.SIDE_5_ALGAE.getBluePose(),
          PositionConstant.SIDE_6_ALGAE.getBluePose());
  public static final List<Pose2d> RED_ALGAE =
      List.of(
          PositionConstant.SIDE_1_ALGAE.getRedPose(),
          PositionConstant.SIDE_2_ALGAE.getRedPose(),
          PositionConstant.SIDE_3_ALGAE.getRedPose(),
          PositionConstant.SIDE_4_ALGAE.getRedPose(),
          PositionConstant.SIDE_5_ALGAE.getRedPose(),
          PositionConstant.SIDE_6_ALGAE.getRedPose());

  public static final List<Pose2d> BLUE_PRE =
      List.of(
          PositionConstant.SIDE_1_ENTRY.getBluePose(),
          PositionConstant.SIDE_2_ENTRY.getBluePose(),
          PositionConstant.SIDE_3_ENTRY.getBluePose(),
          PositionConstant.SIDE_4_ENTRY.getBluePose(),
          PositionConstant.SIDE_5_ENTRY.getBluePose(),
          PositionConstant.SIDE_6_ENTRY.getBluePose());
  public static final List<Pose2d> RED_PRE =
      List.of(
          PositionConstant.SIDE_1_ENTRY.getRedPose(),
          PositionConstant.SIDE_2_ENTRY.getRedPose(),
          PositionConstant.SIDE_3_ENTRY.getRedPose(),
          PositionConstant.SIDE_4_ENTRY.getRedPose(),
          PositionConstant.SIDE_5_ENTRY.getRedPose(),
          PositionConstant.SIDE_6_ENTRY.getRedPose());

  public static final List<Pose2d> BLUE_PRE_3 =
      List.of(
          PositionConstant.SIDE_1_ENTRY.getBluePose(),
          PositionConstant.SIDE_3_ENTRY.getBluePose(),
          PositionConstant.SIDE_5_ENTRY.getBluePose());
  public static final List<Pose2d> RED_PRE_3 =
      List.of(
          PositionConstant.SIDE_1_ENTRY.getRedPose(),
          PositionConstant.SIDE_3_ENTRY.getRedPose(),
          PositionConstant.SIDE_5_ENTRY.getRedPose());
  public static final List<Pose2d> BLUE_PRE_2 =
      List.of(
          PositionConstant.SIDE_2_ENTRY.getBluePose(),
          PositionConstant.SIDE_4_ENTRY.getBluePose(),
          PositionConstant.SIDE_6_ENTRY.getBluePose());
  public static final List<Pose2d> RED_PRE_2 =
      List.of(
          PositionConstant.SIDE_2_ENTRY.getRedPose(),
          PositionConstant.SIDE_4_ENTRY.getRedPose(),
          PositionConstant.SIDE_6_ENTRY.getRedPose());

  public static final List<Pose2d> BLUE_ALGAE_3 =
      List.of(
          PositionConstant.SIDE_1_ALGAE.getBluePose(),
          PositionConstant.SIDE_3_ALGAE.getBluePose(),
          PositionConstant.SIDE_5_ALGAE.getBluePose());
  public static final List<Pose2d> RED_ALGAE_3 =
      List.of(
          PositionConstant.SIDE_1_ALGAE.getRedPose(),
          PositionConstant.SIDE_3_ALGAE.getRedPose(),
          PositionConstant.SIDE_5_ALGAE.getRedPose());
  public static final List<Pose2d> BLUE_ALGAE_2 =
      List.of(
          PositionConstant.SIDE_2_ALGAE.getBluePose(),
          PositionConstant.SIDE_4_ALGAE.getBluePose(),
          PositionConstant.SIDE_6_ALGAE.getBluePose());
  public static final List<Pose2d> RED_ALGAE_2 =
      List.of(
          PositionConstant.SIDE_2_ALGAE.getRedPose(),
          PositionConstant.SIDE_4_ALGAE.getRedPose(),
          PositionConstant.SIDE_6_ALGAE.getRedPose());

  public static final List<Pose2d> RED_COLLECT_LEFT = List.of(PositionConstant.LC2.getRedPose());
  public static final List<Pose2d> BLUE_COLLECT_LEFT = List.of(PositionConstant.LC2.getBluePose());

  public static final List<Pose2d> RED_COLLECT_RIGHT = List.of(PositionConstant.RC2.getRedPose());
  public static final List<Pose2d> BLUE_COLLECT_RIGHT = List.of(PositionConstant.RC2.getBluePose());

  public static final List<Pose2d> RED_CLIMB =
      List.of(
          PositionConstant.C1.getRedPose(),
          PositionConstant.C2.getRedPose(),
          PositionConstant.C2.getRedPose());
  public static final List<Pose2d> BLUE_CLIMB =
      List.of(
          PositionConstant.C1.getBluePose(),
          PositionConstant.C2.getBluePose(),
          PositionConstant.C2.getBluePose());

  public static final List<Pose2d> RED_NET =
      List.of(PositionConstant.ALL_NET.getRedPose(), PositionConstant.OPP_NET.getRedPose());
  public static final List<Pose2d> BLUE_NET =
      List.of(PositionConstant.ALL_NET.getBluePose(), PositionConstant.OPP_NET.getBluePose());
}
