package frc.robot.state.logic.functional;

public class FunctionalAction {
  public final Runnable ELEVATOR_RUNNABLE;
  public final Runnable ARM_RUNNABLE;
  public final Runnable ROLLERCLAW_RUNNABLE;
  public final Runnable INTAKE_RUNNABLE;
  public final Runnable CLIMBER_RUNNABLE;

  public FunctionalAction(Runnable ELEV, Runnable ARM, Runnable ROLL, Runnable INTAKE) {
    this(ELEV, ARM, ROLL, INTAKE, () -> {});
  }

  public FunctionalAction(
      Runnable ELEV, Runnable ARM, Runnable ROLL, Runnable INTAKE, Runnable CLIMB) {
    this.ELEVATOR_RUNNABLE = ELEV;
    this.ARM_RUNNABLE = ARM;
    this.ROLLERCLAW_RUNNABLE = ROLL;
    this.INTAKE_RUNNABLE = INTAKE;
    this.CLIMBER_RUNNABLE = CLIMB;
  }

  public Runnable getElev() {
    return ELEVATOR_RUNNABLE;
  }

  public Runnable getArm() {
    return ARM_RUNNABLE;
  }

  public Runnable getRoller() {
    return ROLLERCLAW_RUNNABLE;
  }

  public Runnable getIntake() {
    return INTAKE_RUNNABLE;
  }

  public Runnable getClimb() {
    return CLIMBER_RUNNABLE;
  }
}
