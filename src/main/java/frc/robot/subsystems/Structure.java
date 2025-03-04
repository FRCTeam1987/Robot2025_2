package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.state.Abomination;
import frc.robot.state.logic.functional.FunctionalState;

public class Structure extends SubsystemBase {

  public Structure() {}

  @Override
  public void periodic() {
    RobotContainer.ARM.cycle();
    RobotContainer.ELEVATOR.cycle();
    RobotContainer.INTAKE.cycle();
    RobotContainer.CLIMBER.cycle();
    RobotContainer.VISION.cycle();
    FunctionalState STATE = Abomination.getState();

    STATE.ACTION.getArm().run();
    STATE.ACTION.getRoller().run();
    STATE.ACTION.getElev().run();
    STATE.ACTION.getIntake().run();
  }
}
