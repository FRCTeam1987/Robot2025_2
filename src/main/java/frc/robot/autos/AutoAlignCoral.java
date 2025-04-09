// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import static frc.robot.state.logic.constants.StateConstants.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.state.commands.DriveToPose;
import frc.robot.utils.Utils;

/** Add your docs here. */
public class AutoAlignCoral extends SequentialCommandGroup {

  public AutoAlignCoral() {
    addCommands(
        new DriveToPose(
            () -> Utils.getNearest(RED_TARGET_POSES_CORAL, BLUE_TARGET_POSES_CORAL, true)));
  }
}
