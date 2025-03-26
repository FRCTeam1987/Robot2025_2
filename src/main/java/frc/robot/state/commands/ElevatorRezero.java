package frc.robot.state.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.RobotContainer.ELEVATOR;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utils.InstCmd;

public class ElevatorRezero extends SequentialCommandGroup {
  public ElevatorRezero() {
    // TODO: Add your sequential commands in the super() call, e.g.
    //           super(new OpenClawCommand(), new MoveArmCommand());
    super();
    addCommands(
        new InstCmd(
            () -> {
              ELEVATOR.setRecovery(true);
              ELEVATOR.setVoltage(Volts.of(-0.5));
            }),
        new WaitUntilCommand(() -> ELEVATOR.getVelocity().lt(RotationsPerSecond.of(0.002))),
        new InstCmd(
            () -> {
              ELEVATOR.zero();
              ELEVATOR.setRecovery(false);
            }));
  }
}
