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
    NetworkTableTimer.wrap("ARM.preCycle", RobotContainer.ARM::preCycle).run();
    NetworkTableTimer.wrap("ELEVATOR.preCycle", RobotContainer.ELEVATOR::preCycle).run();
    NetworkTableTimer.wrap("INTAKE.preCycle", RobotContainer.INTAKE::preCycle).run();
    NetworkTableTimer.wrap("CLIMBER.preCycle", RobotContainer.CLIMBER::preCycle).run();
    NetworkTableTimer.wrap("LIGHTS.preCycle", RobotContainer.LIGHTS::preCycle).run();
    NetworkTableTimer.wrap("ALGAE.preCycle", RobotContainer.ALGAE::preCycle).run();

    FunctionalState STATE = Abomination.getState();

    NetworkTableTimer.wrap("STATE.ACTION.getArm", STATE.ACTION.getArm()).run();
    NetworkTableTimer.wrap("STATE.ACTION.getRoller", STATE.ACTION.getRoller()).run();
    NetworkTableTimer.wrap("STATE.ACTION.getElev", STATE.ACTION.getElev()).run();
    NetworkTableTimer.wrap("STATE.ACTION.getIntake", STATE.ACTION.getIntake()).run();
    NetworkTableTimer.wrap("STATE.ACTION.getAlgae", STATE.ACTION.getAlgae()).run();
    NetworkTableTimer.wrap("STATE.ACTION.getClimb", STATE.ACTION.getClimb()).run();
    NetworkTableTimer.wrap("STATE.ACTION.getLights", STATE.ACTION.getLights()).run();

    NetworkTableTimer.wrap("LIGHTS.postCycle", RobotContainer.LIGHTS::postCycle).run();
  }
}
