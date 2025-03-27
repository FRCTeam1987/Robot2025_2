// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoHelpers;
import frc.robot.state.commands.ElevatorRezero;
import frc.robot.subsystems.*;
import frc.robot.subsystems.constants.TunerConstants;
import frc.robot.util.Tracker;
import frc.robot.utils.InstCmd;
import frc.robot.utils.Utils;
import frc.robot.utils.localization.FieldZones;
import frc.robot.utils.localization.LocalizationState;

public class RobotContainer {

  public static final boolean DEBUG = true;
  public static final boolean TEST_MODE = false;
  public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
  public static final AngularVelocity MAX_ANGULAR_RATE = RotationsPerSecond.of(1.15);
  private static SendableChooser<PathPlannerAuto> autoChooser;
  public static final SwerveRequest.FieldCentric DRIVE =
      new SwerveRequest.FieldCentric()
          .withDeadband(MAX_SPEED.in(MetersPerSecond) * 0.1)
          .withRotationalDeadband(MAX_ANGULAR_RATE.in(RotationsPerSecond) * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  public static final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();
  public static final SwerveRequest.PointWheelsAt POINT = new SwerveRequest.PointWheelsAt();

  public static final CommandXboxController JOYSTICK = new CommandXboxController(0);
  public static final CommandXboxController CODRIVER_JOYSTICK = new CommandXboxController(1);

  public static final Drivetrain DRIVETRAIN = TunerConstants.createDrivetrain();
  public static final Vision VISION = new Vision();
  public static final Elevator ELEVATOR = new Elevator();
  public static final Arm ARM = new Arm();
  public static final Intake INTAKE = new Intake();
  public static final Climber CLIMBER = new Climber();

  public static final Structure STRUCTURE = new Structure();

  public static final Tracker TRACKER = new Tracker();

  public RobotContainer() {
    DRIVETRAIN.registerTelemetry(DRIVETRAIN.LOGGER::telemeterize);
    configureAutos();
    Bindings.configureBindings();
    SmartDashboard.putData(
        "Init Processor Side",
        new InstCmd(
                () ->
                    DRIVETRAIN.resetPose(
                        new Pose2d(new Translation2d(7.14, 3.0), Rotation2d.fromDegrees(0))))
            .ignoringDisable(true));
    SmartDashboard.putData("Home Elevator", new ElevatorRezero());

    //    ArrayList<Pose2d> posesRed = new ArrayList<Pose2d>();
    //    ArrayList<Pose2d> posesBlu = new ArrayList<Pose2d>();
    //    for (PositionConstant value : PositionConstant.values()) {
    //      posesRed.add(value.getRedPose());
    //      posesBlu.add(value.getBluePose());
    //    }
    //    Pose2d[] listRed = posesRed.toArray(new Pose2d[0]);
    //    Pose2d[] listBlu = posesBlu.toArray(new Pose2d[0]);
    //    DogLog.log("Poses/Red", listRed);
    //    DogLog.log("Poses/Blue", listBlu);
  }

  private static LocalizationState localizationState =
      new LocalizationState(FieldZones.Zone.ALLIANCE_SIDE, Distance.ofRelativeUnits(0, Meters));

  public static LocalizationState getLocalizationState() {
    return localizationState;
  }

  public void configureAutos() {
    AutoHelpers.registerNamedCommands();
    // Set up auto routines
    autoChooser = Utils.buildAutoChooser("None");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public static void updateLocalizationState() {
    final DriverStation.Alliance alliance = DRIVETRAIN.getAlliance();
    final Translation2d robotTranslation = DRIVETRAIN.getPose().getTranslation();
    localizationState =
        new LocalizationState(
            FieldZones.getZoneFromTranslation(alliance, robotTranslation), Inches.of(5.0));
    //            Distance.ofRelativeUnits(
    //                robotTranslation.getDistance(
    //                    alliance == DriverStation.Alliance.Blue
    //                        ? Strategy.getCurrentFieldPosition()
    //                            .getLocation()
    //                            .getBluePose()
    //                            .getTranslation()
    //                        : Strategy.getCurrentFieldPosition()
    //                            .getLocation()
    //                            .getRedPose()
    //                            .getTranslation()),
    //                Meters));
  }

  public static PathPlannerAuto getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
