// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import static frc.robot.state.logic.constants.StateConstants.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.state.commands.DriveToPose;
import frc.robot.utils.Utils;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAlignAlgae extends SequentialCommandGroup {

  /** Creates a new AutoAlignAlgae. */
  public AutoAlignAlgae() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveToPose(() -> Utils.getNearest(RED_ALGAE, BLUE_ALGAE, true)));
  }
}
