package frc.robot.state.logic.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public enum MechanismConstant {
  // CORAL SCORING LOCATIONS
  L1(Inches.of(0.0), Degrees.of(140)),
  L2(Inches.of(1.0), Degrees.of(296.5)),
  L3(Inches.of(17.75), Degrees.of(297)),
  L4(Inches.of(47.95), Degrees.of(314.25)),

  // ALGAE SCORING LOCATIONS
  NET(Inches.of(50.5), Degrees.of(-45)),
  PROCESSOR(Inches.of(0.0), Degrees.of(77)),

  // CORAL INTAKING LOCATIONS
  HP_INTAKE(Inches.of(0.0), Degrees.of(98.347)),
  IDLE_CORAL(Inches.of(0.0), Degrees.of(280)),

  // ALGAE INTAKING LOCATIONS
  A2(Inches.of(13), Degrees.of(54)),
  A3(Inches.of(28), Degrees.of(54)),
  IDLE_ALGAE_PROCESSOR(Inches.of(3.0), Degrees.of(34)),
  IDLE_ALGAE_NET(Inches.of(3.0), Degrees.of(-45)),

  CLIMB(Inches.of(0.0), Degrees.of(100.0)),
  DEFENSE(Inches.of(0.0), Degrees.of(0)),
  ;

  public Distance getElevatorDistance() {
    return ELEVATOR_DISTANCE;
  }

  public Angle getArmAngle() {
    return ARM_ANGLE;
  }

  private final Distance ELEVATOR_DISTANCE;
  private final Angle ARM_ANGLE;

  MechanismConstant(Distance ELEV, Angle ARM) {
    this.ELEVATOR_DISTANCE = ELEV;
    this.ARM_ANGLE = ARM;
  }
}
