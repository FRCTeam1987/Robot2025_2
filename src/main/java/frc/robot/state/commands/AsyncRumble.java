// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.state.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class AsyncRumble extends Command {
  private final XboxController controller;
  private final double strength;
  private final RumbleType type;
  private final long durationMs;
  private long finishtime;

  /** Rumbles the controller for a given amount of time. */
  public AsyncRumble(XboxController controller, RumbleType type, double strength, long durationMs) {
    this.controller = controller;
    this.strength = strength;
    this.durationMs = durationMs;
    this.type = type;
  }

  @Override
  public void initialize() {
    long time = System.currentTimeMillis();
    this.finishtime = time + durationMs;
  }

  @Override
  public void execute() {
    controller.setRumble(type, strength);
  }

  @Override
  public void end(boolean interrupted) {
    controller.setRumble(RumbleType.kBothRumble, 0.0);
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() >= finishtime;
  }
}
