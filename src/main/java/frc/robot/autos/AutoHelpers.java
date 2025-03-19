// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.state.Abomination;
import frc.robot.state.logic.actions.DesiredAction;
import frc.robot.state.logic.constants.FieldPosition;
import frc.robot.state.logic.constants.MechanismConstant;
import frc.robot.state.logic.mode.ScoreMode;
import frc.robot.utils.InstCmd;
import java.util.List;

/** Add your docs here. */
public class AutoHelpers {

  public static Distance DRIVING_MAX_HEIGHT =
      MechanismConstant.L4.getElevatorDistance().minus(Inches.of(1.0));
  private static boolean WANTS_CORAL = true;

  public static void setScoreMode(ScoreMode mode) {
    Abomination.setScoreMode(ScoreMode.L4);
  }

  public static double matchTimeIncrement = 0.0;

  public static void setWantsCoral(final boolean wantsCoral) {
    WANTS_CORAL = wantsCoral;
  }

  public static boolean getWantsCoral() {
    return WANTS_CORAL;
  }

  public static boolean hasScoredCoral() {
    return RobotContainer.ELEVATOR.getPosition().lte(DRIVING_MAX_HEIGHT);
  }

  public static boolean hasAlgae() {
    return RobotContainer.ARM.hasAlgae();
  }

  public static boolean hasCoral() {
    return RobotContainer.ARM.hasGamePiece();
  }

  public static void registerNamedCommands() {
    NamedCommands.registerCommand(
        "WaitUntilScoredCoral", new WaitUntilCommand(AutoHelpers::hasScoredCoral));
    NamedCommands.registerCommand("WaitUntilAlgae", new WaitUntilCommand(AutoHelpers::hasAlgae));
    NamedCommands.registerCommand("AutoAlignAlgae", new InstCmd());
    NamedCommands.registerCommand("AutoAlignCoral", new InstCmd());
    NamedCommands.registerCommand(
        "PreElevate",
        new WaitUntilCommand(AutoHelpers::hasCoral)
            .andThen(new WaitCommand(0.1)) // TODO: check arm rotation instead of delay
            .andThen(
                new InstCmd(
                    () -> {
                      Abomination.setScoreMode(ScoreMode.L3);
                      Abomination.setAction(DesiredAction.INIT);
                    })));
    NamedCommands.registerCommand(
        "InitL4",
        new InstCmd(
            () -> {
              Abomination.setScoreMode(ScoreMode.L4);
              Abomination.setAction(DesiredAction.INIT);
            }));
    NamedCommands.registerCommand(
        "AutoScore",
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new AutoAlignCoral(),
                    new InstCmd(() -> Abomination.setAction(DesiredAction.INIT)))
                .withTimeout(2.5),
            new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L4)),
            new WaitUntilCommand(RobotContainer.ELEVATOR::isAtTarget).withTimeout(1.0),
            new InstCmd(() -> Abomination.setAction(DesiredAction.SCORE)),
            new WaitUntilCommand(AutoHelpers::hasScoredCoral),
            new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L3))));
  }

  public static List<FieldPosition> processorQueue =
      List.of(FieldPosition.F4, FieldPosition.C4, FieldPosition.D4, FieldPosition.B4);

  public static List<FieldPosition> climbQueue =
      List.of(FieldPosition.I4, FieldPosition.L4, FieldPosition.K4, FieldPosition.A4);
}
