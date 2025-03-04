package frc.robot.state;

import static frc.robot.state.logic.constants.FieldPosition.*;

import frc.robot.state.logic.constants.FieldPosition;
import java.util.List;
import java.util.ListIterator;

public class Strategy {

  public static final List<FieldPosition> STRATEGY =
      List.of(RC2, A3, RC2, B3, RC2, A2, RC2, B2, RC2, A4, RC2, B4);
  static final ListIterator<FieldPosition> ITERATOR = STRATEGY.listIterator();
  static FieldPosition CURRENT;

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
}
