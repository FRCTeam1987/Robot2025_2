package frc.robot.state;

import static frc.robot.state.logic.constants.FieldPosition.*;

import edu.wpi.first.math.Pair;
import frc.robot.RobotContainer;
import frc.robot.state.logic.constants.FieldPosition;
import java.util.List;
import java.util.ListIterator;
import java.util.Optional;

public class Strategy {

  public static final List<FieldPosition> STRATEGY =
      List.of(RC2, A3, RC2, B3, RC2, A2, RC2, B2, RC2, A4, RC2, B4);
  static final ListIterator<FieldPosition> ITERATOR = STRATEGY.listIterator();
  static FieldPosition CURRENT;

  static List<FieldPosition> level4Priority = List.of(A4, B4, C4, D4, E4, F4, G4, H4, J4, K4, L4);
  static List<FieldPosition> level3Priority = List.of(A3, B3, C3, D3, E3, F3, G3, H3, J3, K3, L3);
  static List<FieldPosition> level2Priority = List.of(A2, B2, C2, D2, E2, F2, G2, H2, J2, K2, L2);
  static List<List<FieldPosition>> levelPriorities =
      List.of(level4Priority, level3Priority, level2Priority);

  static {
    CURRENT = next();
  }

  public static FieldPosition next() {
    if (ITERATOR.hasNext()) {
      CURRENT = ITERATOR.next();
      return CURRENT;
    } else {
      return A2;
    }
  }

  public static FieldPosition previous() {
    if (ITERATOR.hasPrevious()) {
      CURRENT = ITERATOR.previous();
      return CURRENT;
    } else {
      return A2;
    }
  }

  public static FieldPosition getCurrentFieldPosition() {
    return CURRENT;
  }

  public static FieldPosition getNextPriorityReefPosition() {
    FieldPosition nextBest = null;
    for (List<FieldPosition> level : levelPriorities) {
      Optional<Pair<Boolean, FieldPosition>> priority = getNextPriorityLevelPosition(level, 5);
      if (priority.isPresent()) {
        if (priority.get().getFirst()) {
          return priority.get().getSecond();
        } else {
          nextBest = priority.get().getSecond();
        }
      }
    }
    return nextBest;
  }

  public static Optional<Pair<Boolean, FieldPosition>> getNextPriorityLevelPosition(
      List<FieldPosition> level, int priorityCount) {
    for (FieldPosition position : level) {
      int scored = 0;
      if (RobotContainer.TRACKER.isScored(position)) {
        scored++;
      } else {
        return Optional.of(Pair.of(scored < priorityCount, position));
      }
    }
    return Optional.empty();
  }
}
