package frc.robot.state.logic.constants;

import frc.robot.utils.localization.FieldZones;
import java.util.List;

public class StateConstants {
  public static final List<FieldZones.Zone> COLLECT_ZONES =
      List.of(
          FieldZones.Zone.ALLIANCE_CORAL_LEFT,
          FieldZones.Zone.ALLIANCE_CORAL_RIGHT,
          FieldZones.Zone.ALLIANCE_REEF);
}
