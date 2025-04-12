package frc.robot.state.logic.functional;

import static frc.robot.RobotContainer.*;

public class FunctionalAction {
  public final Runnable ELEVATOR_RUNNABLE;
  public final Runnable ARM_RUNNABLE;
  public final Runnable ROLLERCLAW_RUNNABLE;
  public final Runnable INTAKE_RUNNABLE;
  public final Runnable CLIMBER_RUNNABLE;
  public final Runnable LIGHTS_RUNNABLE;

  public FunctionalAction(
      Runnable ELEV, Runnable ARM, Runnable ROLL, Runnable INTAKE, Runnable LIGHTS) {
    this(
        ELEV,
        ARM,
        ROLL,
        INTAKE,
        () -> {
          if (JOYSTICK.back().getAsBoolean()) {
            CLIMBER.stow();
          } else {
            CLIMBER.stop();
          }
        },
        LIGHTS);
  }

  public FunctionalAction(
      Runnable ELEV,
      Runnable ARM,
      Runnable ROLL,
      Runnable INTAKE,
      Runnable CLIMB,
      Runnable LIGHTS) {
    this.ELEVATOR_RUNNABLE = ELEV;
    this.ARM_RUNNABLE = ARM;
    this.ROLLERCLAW_RUNNABLE = ROLL;
    this.INTAKE_RUNNABLE = INTAKE;
    this.CLIMBER_RUNNABLE = CLIMB;
    this.LIGHTS_RUNNABLE = LIGHTS;
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

  public Runnable getLights() {
    return LIGHTS_RUNNABLE;
  }
}
