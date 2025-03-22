// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.units.measure.Angle;
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

  private static final Angle ARM_UNROTATE_ANGLE =
      MechanismConstant.L4.getArmAngle().minus(Degrees.of(2.5));

  public static boolean hasScoredCoral() {
    return RobotContainer.ARM.getArmPosition().lte(ARM_UNROTATE_ANGLE);
    // return RobotContainer.ELEVATOR.getPosition().lte(DRIVING_MAX_HEIGHT);
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
        new ConditionalCommand(
            new InstCmd(
                () -> {
                  Abomination.setScoreMode(ScoreMode.L4);
                  Abomination.setAction(DesiredAction.INIT);
                }),
            new InstCmd(),
            AutoHelpers::hasCoral));
    NamedCommands.registerCommand(
        "InitL2",
        new InstCmd(
            () -> {
              Abomination.setScoreMode(ScoreMode.L2);
              Abomination.setAction(DesiredAction.INIT);
            }));
    //        new ConditionalCommand(
    //            new InstCmd(
    //                () -> {
    //                  Abomination.setScoreMode(ScoreMode.L2);
    //                  Abomination.setAction(DesiredAction.INIT);
    //                }),
    //            new InstCmd(),
    //            () -> RobotContainer.ARM.hasGamePieceEntrance()));
    NamedCommands.registerCommand(
        "InitL4",
        new ConditionalCommand(
            new InstCmd(
                () -> {
                  Abomination.setScoreMode(ScoreMode.L4);
                  Abomination.setAction(DesiredAction.INIT);
                }),
            new InstCmd(),
            () -> RobotContainer.ARM.hasGamePieceEntrance()));
    NamedCommands.registerCommand(
        "AutoScoreL2",
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new AutoAlignCoral(),
                    new InstCmd(
                        () -> {
                          Abomination.setScoreMode(ScoreMode.L2);
                          Abomination.setAction(DesiredAction.INIT);
                        }))
                .withTimeout(2.5),
            new InstCmd(() -> Abomination.setAction(DesiredAction.SCORE))));
    NamedCommands.registerCommand(
        "AutoScore",
        new ConditionalCommand(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new AutoAlignCoral(),
                        new InstCmd(
                            () -> {
                              Abomination.setScoreMode(ScoreMode.L4);
                              Abomination.setAction(DesiredAction.INIT);
                            }))
                    .withTimeout(2.5),
                new WaitUntilCommand(RobotContainer.ELEVATOR::isAtTarget).withTimeout(1.0),
                new InstCmd(() -> Abomination.setAction(DesiredAction.SCORE)),
                new WaitUntilCommand(AutoHelpers::hasScoredCoral)
                // new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L3))),
                ),
            new InstCmd(),
            RobotContainer.ARM::hasGamePiece));
  }

  public static List<FieldPosition> processorQueue =
      List.of(FieldPosition.F4, FieldPosition.C4, FieldPosition.D4, FieldPosition.B4);

  public static List<FieldPosition> climbQueue =
      List.of(FieldPosition.I4, FieldPosition.L4, FieldPosition.K4, FieldPosition.A4);
}
