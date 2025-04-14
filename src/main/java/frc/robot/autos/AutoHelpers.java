// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.state.Abomination;
import frc.robot.state.logic.actions.DesiredAction;
import frc.robot.state.logic.constants.FieldPosition;
import frc.robot.state.logic.constants.MechanismConstant;
import frc.robot.state.logic.functional.FunctionalState;
import frc.robot.state.logic.mode.CollectMode;
import frc.robot.state.logic.mode.ScoreMode;
import frc.robot.utils.InstCmd;
import java.util.List;

/** Add your docs here. */
public class AutoHelpers {

  public static DoubleEntry CONFIGURABLE_DELAY =
      NetworkTableInstance.getDefault().getDoubleTopic("autoDelaySeconds").getEntry(0.0);

  public static Distance DRIVING_MAX_HEIGHT =
      MechanismConstant.L4.getElevatorDistance().minus(Inches.of(1.0));
  private static boolean WANTS_CORAL = true;

  public static Distance ELEVATOR_BELOW_NET_HEIGHT = MechanismConstant.L3.getElevatorDistance();

  public static void setScoreMode(ScoreMode mode) {
    Abomination.setScoreMode(ScoreMode.L4, false);
  }

  public static double matchTimeIncrement = 0.0;

  public static void setWantsCoral(final boolean wantsCoral) {
    WANTS_CORAL = wantsCoral;
  }

  public static boolean getWantsCoral() {
    return WANTS_CORAL;
  }

  public static boolean hasScoredAlgae() {
    return Abomination.getPreviousState().equals(FunctionalState.NET_UNELEVATE);
  }

  public static boolean hasScoredCoral() {
    return Abomination.getPreviousState().equals(FunctionalState.LEVEL_X_UNROTATE);
  }

  public static boolean hasAlgae() {
    return RobotContainer.ARM.hasAlgae();
  }

  public static boolean hasCoral() {
    return RobotContainer.ARM.hasGamePiece();
  }

  public static void registerNamedCommands() {
    NamedCommands.registerCommand("DelayStart", new WaitCommand(matchTimeIncrement));
    NamedCommands.registerCommand(
        "WaitUntilHasAlgae", new WaitUntilCommand(AutoHelpers::hasAlgae).withTimeout(2));
    NamedCommands.registerCommand(
        "WaitUntilScoredCoral", new WaitUntilCommand(AutoHelpers::hasScoredCoral));
    NamedCommands.registerCommand("WaitUntilAlgae", new WaitUntilCommand(AutoHelpers::hasAlgae));
    NamedCommands.registerCommand(
        "PreElevate",
        new ConditionalCommand(
            new InstCmd(
                () -> {
                  Abomination.setScoreMode(ScoreMode.L4, false);
                  Abomination.setAction(DesiredAction.INIT);
                }),
            new InstCmd(),
            AutoHelpers::hasCoral));
    NamedCommands.registerCommand(
        "InitL2",
        new InstCmd(
            () -> {
              Abomination.setScoreMode(ScoreMode.L2, false);
              Abomination.setAction(DesiredAction.INIT);
            }));
    NamedCommands.registerCommand(
        "InitL4",
        new ConditionalCommand(
            new InstCmd(
                () -> {
                  Abomination.setScoreMode(ScoreMode.L4, false);
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
                          Abomination.setScoreMode(ScoreMode.L2, false);
                          Abomination.setAction(DesiredAction.INIT);
                        }))
                .withTimeout(1),
            new InstCmd(() -> Abomination.setAction(DesiredAction.SCORE))));
    NamedCommands.registerCommand(
        "AutoScore",
        new ConditionalCommand(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new AutoAlignCoral(),
                        new InstCmd(
                            () -> {
                              Abomination.setScoreMode(ScoreMode.L4, false);
                              Abomination.setAction(DesiredAction.INIT);
                            }))
                    .withTimeout(1.5),
                new WaitUntilCommand(RobotContainer.ELEVATOR::isAtTarget).withTimeout(1.0),
                new InstCmd(() -> Abomination.setAction(DesiredAction.SCORE)),
                new WaitUntilCommand(AutoHelpers::hasScoredCoral)
                // new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L3))),
                ),
            new InstCmd(),
            RobotContainer.ARM::hasGamePiece));
    NamedCommands.registerCommand(
        "EnableVisionUpdates", new InstCmd(() -> RobotContainer.VISION.setShouldUpdatePose(true)));
    NamedCommands.registerCommand(
        "DisableVisionUpdates",
        new InstCmd(() -> RobotContainer.VISION.setShouldUpdatePose(false)));
    NamedCommands.registerCommand(
        "SetCollectModeAlgae2", new InstCmd(() -> Abomination.setCollectMode(CollectMode.ALGAE_2)));
    NamedCommands.registerCommand(
        "SetCollectModeAlgae3", new InstCmd(() -> Abomination.setCollectMode(CollectMode.ALGAE_3)));
    NamedCommands.registerCommand("AutoAlignAlgae", new AutoAlignAlgae());
    NamedCommands.registerCommand(
        "InitNet",
        new ConditionalCommand(
            new InstCmd(
                () -> {
                  Abomination.setScoreMode(ScoreMode.NET, false);
                  Abomination.setAction(DesiredAction.INIT);
                }),
            new InstCmd(),
            RobotContainer.ARM::hasAlgae));
    NamedCommands.registerCommand(
        "ScoreAlgae",
        new WaitUntilCommand(() -> RobotContainer.ELEVATOR.isAtTarget())
            .andThen(new WaitCommand(0.04))
            .andThen(
                new InstCmd(() -> Abomination.setAction(DesiredAction.SCORE))
                    .andThen(new WaitUntilCommand(AutoHelpers::hasScoredAlgae))));
    NamedCommands.registerCommand(
        "WaitUntilElevatorIsBelowNet",
        new WaitUntilCommand(
            () -> RobotContainer.ELEVATOR.getPosition().lte(ELEVATOR_BELOW_NET_HEIGHT)));
  }

  public static List<FieldPosition> processorQueue =
      List.of(FieldPosition.F4, FieldPosition.C4, FieldPosition.D4, FieldPosition.B4);

  public static List<FieldPosition> climbQueue =
      List.of(FieldPosition.I4, FieldPosition.L4, FieldPosition.K4, FieldPosition.A4);
}
