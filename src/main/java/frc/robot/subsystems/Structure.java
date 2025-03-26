package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.state.Abomination;
import frc.robot.state.logic.functional.FunctionalState;
import frc.robot.util.NetworkTableTimer;

public class Structure extends SubsystemBase {

  public Structure() {}

  @Override
  public void periodic() {
    NetworkTableTimer.wrap("ARM.cycle", () -> RobotContainer.ARM.cycle()).run();
    NetworkTableTimer.wrap("ELEVATOR.cycle", () -> RobotContainer.ELEVATOR.cycle()).run();
    NetworkTableTimer.wrap("INTAKE.cycle", () -> RobotContainer.INTAKE.cycle()).run();
    NetworkTableTimer.wrap("CLIMBER.cycle", () -> RobotContainer.CLIMBER.cycle()).run();
    FunctionalState STATE = Abomination.getState();

    NetworkTableTimer.wrap("STATE.ACTION.getArm", STATE.ACTION.getArm()).run();
    NetworkTableTimer.wrap("STATE.ACTION.getRoller", STATE.ACTION.getRoller()).run();
    NetworkTableTimer.wrap("STATE.ACTION.getElev", STATE.ACTION.getElev()).run();
    NetworkTableTimer.wrap("STATE.ACTION.getIntake", STATE.ACTION.getIntake()).run();
    NetworkTableTimer.wrap("STATE.ACTION.getClimb", STATE.ACTION.getClimb()).run();
  }
}
