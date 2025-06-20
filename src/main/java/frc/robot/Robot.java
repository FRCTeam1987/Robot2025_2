// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.RobotContainer.ARM;
import static frc.robot.RobotContainer.DRIVETRAIN;

import au.grapplerobotics.CanBridge;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.AutoHelpers;

public class Robot extends TimedRobot {
  private PathPlannerAuto AUTONOMOUS_COMMAND;

  private final RobotContainer ROBOT_CONTAINER;

  private double timeToCoast;

  public static boolean hasClimberCoasted = false;

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
    //    if (TEST_MODE) {
    //      LimelightHelpers.SetThrottle("limelight-scoring", 200);
    //    }
    timeToCoast = Timer.getFPGATimestamp();
  }

  @Override
  public void disabledPeriodic() {
    if (timeToCoast + 15 < Timer.getFPGATimestamp() && !hasClimberCoasted) {
      RobotContainer.CLIMBER.coast();
      hasClimberCoasted = true;
    }
    if (timeToCoast + 5 < Timer.getFPGATimestamp()) {
      if (ARM.hasAlgae() || (ARM.hasGamePieceEntrance() && !ARM.hasGamePieceBack())) {
        ARM.coast();
      } else {
        ARM.brake();
      }
    } else {
      ARM.brake();
    }
  }

  @Override
  public void disabledExit() {
    RobotContainer.CLIMBER.brake();
    RobotContainer.ARM.brake();
    hasClimberCoasted = false;
  }

  @Override
  public void autonomousInit() {
    RobotContainer.autoTime = Timer.getFPGATimestamp();

    RobotContainer.VISION.setShouldUpdatePose(false);
    AUTONOMOUS_COMMAND = RobotContainer.getAutonomousCommand();

    AutoHelpers.matchTimeIncrement = Timer.getFPGATimestamp();

    DRIVETRAIN.tareEverything();
    if (AUTONOMOUS_COMMAND != null) {
      AUTONOMOUS_COMMAND.schedule();
    }
    RobotContainer.ELEVATOR.setConfigAuto();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    RobotContainer.autoTime = Timer.getFPGATimestamp();
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
