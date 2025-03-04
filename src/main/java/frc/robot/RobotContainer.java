// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  private final double MAX_SPEED =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private final double MAX_ANGULAR_RATE =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric DRIVE =
      new SwerveRequest.FieldCentric()
          .withDeadband(MAX_SPEED * 0.1)
          .withRotationalDeadband(MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt POINT = new SwerveRequest.PointWheelsAt();

  private final Telemetry LOGGER = new Telemetry(MAX_SPEED);

  private final CommandXboxController JOYSTICK = new CommandXboxController(0);

  public final CommandSwerveDrivetrain DRIVETRAIN = TunerConstants.createDrivetrain();
  public final IntakeSubsystem INTAKE = new IntakeSubsystem();

  public RobotContainer() {
    // https://doglog.dev
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
    DogLog.setPdh(new PowerDistribution());
    // Disable the logger if desired
    // DogLog.setEnabled(false);
    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    DRIVETRAIN.setDefaultCommand(
        // Drivetrain will execute this command periodically
        DRIVETRAIN.applyRequest(
            () ->
                DRIVE
                    .withVelocityX(
                        -JOYSTICK.getLeftY() * MAX_SPEED) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -JOYSTICK.getLeftX() * MAX_SPEED) // Drive left with negative X (left)
                    .withRotationalRate(
                        -JOYSTICK.getRightX()
                            * MAX_ANGULAR_RATE) // Drive counterclockwise with negative X (left)
            ));

    JOYSTICK.a().whileTrue(DRIVETRAIN.applyRequest(() -> BRAKE));
    JOYSTICK
        .b()
        .whileTrue(
            DRIVETRAIN.applyRequest(
                () ->
                    POINT.withModuleDirection(
                        new Rotation2d(-JOYSTICK.getLeftY(), -JOYSTICK.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    JOYSTICK.back().and(JOYSTICK.y()).whileTrue(DRIVETRAIN.sysIdDynamic(Direction.kForward));
    JOYSTICK.back().and(JOYSTICK.x()).whileTrue(DRIVETRAIN.sysIdDynamic(Direction.kReverse));
    JOYSTICK.start().and(JOYSTICK.y()).whileTrue(DRIVETRAIN.sysIdQuasistatic(Direction.kForward));
    JOYSTICK.start().and(JOYSTICK.x()).whileTrue(DRIVETRAIN.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    JOYSTICK.leftBumper().onTrue(DRIVETRAIN.runOnce(DRIVETRAIN::seedFieldCentric));

    DRIVETRAIN.registerTelemetry(LOGGER::telemeterize);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
