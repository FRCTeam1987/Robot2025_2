package frc.robot.state.logic.constants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.localization.FieldZones;
import java.util.List;

public class StateConstants {
  public static final List<FieldZones.Zone> COLLECT_ZONES =
      List.of(
          FieldZones.Zone.ALLIANCE_CORAL_LEFT,
          FieldZones.Zone.ALLIANCE_CORAL_RIGHT,
          FieldZones.Zone.ALLIANCE_REEF);

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

  public static final List<Pose2d> BLUE_TARGET_POSES_ALGAE =
      List.of(
          PositionConstant.SIDE_1_ALGAE.getBluePose(),
          PositionConstant.SIDE_2_ALGAE.getBluePose(),
          PositionConstant.SIDE_3_ALGAE.getBluePose(),
          PositionConstant.SIDE_4_ALGAE.getBluePose(),
          PositionConstant.SIDE_5_ALGAE.getBluePose(),
          PositionConstant.SIDE_6_ALGAE.getBluePose());
  public static final List<Pose2d> RED_TARGET_POSES_ALGAE =
      List.of(
          PositionConstant.SIDE_1_ALGAE.getRedPose(),
          PositionConstant.SIDE_2_ALGAE.getRedPose(),
          PositionConstant.SIDE_3_ALGAE.getRedPose(),
          PositionConstant.SIDE_4_ALGAE.getRedPose(),
          PositionConstant.SIDE_5_ALGAE.getRedPose(),
          PositionConstant.SIDE_6_ALGAE.getRedPose());

  public static final List<Pose2d> RED_TARGET_POSES_COLLECT =
      List.of(PositionConstant.RC2.getRedPose(), PositionConstant.RC2.getRedPose());
  public static final List<Pose2d> BLUE_TARGET_POSES_COLLECT =
      List.of(PositionConstant.RC2.getBluePose(), PositionConstant.RC2.getBluePose());
}
