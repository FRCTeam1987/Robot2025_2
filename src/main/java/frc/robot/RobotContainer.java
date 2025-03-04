// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.state.Strategy;
import frc.robot.subsystems.*;
import frc.robot.subsystems.constants.TunerConstants;
import frc.robot.utils.localization.FieldZones;
import frc.robot.utils.localization.LocalizationState;

public class RobotContainer {
  public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
  public static final AngularVelocity MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75);

  public static final SwerveRequest.FieldCentric DRIVE =
      new SwerveRequest.FieldCentric()
          .withDeadband(MAX_SPEED.in(MetersPerSecond) * 0.1)
          .withRotationalDeadband(MAX_ANGULAR_RATE.in(RotationsPerSecond) * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  public static final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();
  public static final SwerveRequest.PointWheelsAt POINT = new SwerveRequest.PointWheelsAt();

  public static final CommandXboxController JOYSTICK = new CommandXboxController(0);

  public static final Drivetrain DRIVETRAIN = TunerConstants.createDrivetrain();
  public static final Elevator ELEVATOR = new Elevator();
  public static final Arm ARM = new Arm();
  public static final Intake INTAKE = new Intake();
  public static final Climber CLIMBER = new Climber();
  public static final Vision VISION = new Vision();

  public static final Structure STRUCTURE = new Structure();

  public RobotContainer() {
    DRIVETRAIN.registerTelemetry(DRIVETRAIN.LOGGER::telemeterize);
    Bindings.configureBindings();
  }

  private static LocalizationState localizationState =
      new LocalizationState(FieldZones.Zone.ALLIANCE_SIDE, Distance.ofRelativeUnits(0, Meters));

  public static LocalizationState getLocalizationState() {
    return localizationState;
  }

  public static void updateLocalizationState() {
    final DriverStation.Alliance alliance = DRIVETRAIN.getAlliance();
    final Translation2d robotTranslation = DRIVETRAIN.getPose().getTranslation();
    localizationState =
        new LocalizationState(
            FieldZones.getZoneFromTranslation(alliance, robotTranslation),
            Distance.ofRelativeUnits(
                robotTranslation.getDistance(
                    alliance == DriverStation.Alliance.Blue
                        ? Strategy.getCurrentFieldPosition()
                            .getLocation()
                            .getBluePose()
                            .getTranslation()
                        : Strategy.getCurrentFieldPosition()
                            .getLocation()
                            .getRedPose()
                            .getTranslation()),
                Meters));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
