// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.state.Abomination;
import frc.robot.state.logic.constants.FieldPosition;
import frc.robot.state.logic.constants.MechanismConstant;
import frc.robot.state.logic.mode.ScoreMode;
import java.util.List;

/** Add your docs here. */
public class AutoHelpers {

  private static Distance DRIVING_MAX_HEIGHT = MechanismConstant.L3.getElevatorDistance();
  private static boolean WANTS_CORAL = true;

  public static void setScoreMode(ScoreMode mode) {
    Abomination.setScoreMode(ScoreMode.L4);
  }

  public static void setWantsCoral(final boolean wantsCoral) {
    WANTS_CORAL = wantsCoral;
  }

  public static boolean getWantsCoral() {
    return WANTS_CORAL;
  }

  public static boolean hasScored() {
    return !RobotContainer.ARM.hasGamePiece()
        && !RobotContainer.ARM.hasAlgae()
        && RobotContainer.ELEVATOR.getPosition().lte(DRIVING_MAX_HEIGHT);
  }

  public static boolean hasAlgae() {
    return RobotContainer.ARM.hasAlgae();
  }

  public static void registerNamedCommands() {
    NamedCommands.registerCommand("WaitUntilScored", new WaitUntilCommand(AutoHelpers::hasScored));
    NamedCommands.registerCommand("WaitUntilAlgae", new WaitUntilCommand(AutoHelpers::hasAlgae));
    NamedCommands.registerCommand("AutoAlignAlgae", new AutoAlignAlgae());
    NamedCommands.registerCommand("AutoAlignCoral", new AutoAlignCoral());
  }

  public static List<FieldPosition> processorQueue =
      List.of(FieldPosition.F4, FieldPosition.C4, FieldPosition.D4, FieldPosition.B4);

  public static List<FieldPosition> climbQueue =
      List.of(FieldPosition.I4, FieldPosition.L4, FieldPosition.K4, FieldPosition.A4);
}
