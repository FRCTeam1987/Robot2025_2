// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.RobotContainer.TEST_MODE;

import au.grapplerobotics.CanBridge;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.AutoHelpers;
import frc.robot.utils.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command AUTONOMOUS_COMMAND;

  private final RobotContainer ROBOT_CONTAINER;

  private double timeToCoast;

  public Robot() {
    CanBridge.runTCP();
    RobotController.setBrownoutVoltage(6.0);

    FollowPathCommand.warmupCommand().schedule();
    PathfindingCommand.warmupCommand().schedule();
    ROBOT_CONTAINER = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    RobotContainer.updateLocalizationState();
  }

  @Override
  public void disabledInit() {
    if (TEST_MODE) {
      LimelightHelpers.SetThrottle("limelight-scoring", 200);
    }
    timeToCoast = Timer.getFPGATimestamp();
  }

  @Override
  public void disabledPeriodic() {
    if (timeToCoast + 15 < Timer.getFPGATimestamp()) {
      RobotContainer.CLIMBER.coast();
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if (TEST_MODE) {
      LimelightHelpers.SetThrottle("limelight-scoring", 0);
    }
    RobotContainer.ELEVATOR.setConfigAuto();
    AUTONOMOUS_COMMAND = ROBOT_CONTAINER.getAutonomousCommand();
    AutoHelpers.matchTimeIncrement = Timer.getFPGATimestamp();

    if (AUTONOMOUS_COMMAND != null) {
      AUTONOMOUS_COMMAND.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (TEST_MODE) {
      LimelightHelpers.SetThrottle("limelight-scoring", 0);
    }
    RobotContainer.CLIMBER.brake();
    RobotContainer.ELEVATOR.setConfigTeleop();
    if (AUTONOMOUS_COMMAND != null) {
      AUTONOMOUS_COMMAND.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
