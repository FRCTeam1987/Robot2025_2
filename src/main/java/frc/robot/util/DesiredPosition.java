package frc.robot.util;

import static frc.robot.state.logic.constants.FieldPosition.*;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.state.logic.constants.FieldPosition;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class DesiredPosition {

  public static final List<FieldPosition> TARGET_CORAL_POSES =
      List.of(
          A2, A3, A4, B2, B3, B4, C2, C3, C4, D2, D3, D4, E2, E3, E4, F2, F3, F4, G2, G3, G4, H2,
          H3, H4, I2, I3, I4, J2, J3, J4, K2, K3, K4, L2, L3, L4);

  public static FieldPosition nearest(Pose2d pos, List<FieldPosition> poses) {
    return Collections.min(
        poses,
        Comparator.comparing(
                (FieldPosition other) ->
                    pos.getTranslation()
                        .getDistance(other.getLocation().getAlliancePose().getTranslation()))
            .thenComparing(
                (FieldPosition other) ->
                    Math.abs(
                        pos.getRotation()
                            .minus(other.getLocation().getAlliancePose().getRotation())
                            .getRadians())));
  }
}
