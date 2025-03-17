// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {

  private static final HolonomicDriveController HOLONOMIC_CONTROLLER =
      new HolonomicDriveController(
          new PIDController(6, 0, 0.00015),
          new PIDController(6, 0, 0.00015),
          new ProfiledPIDController(
              1.0,
              0,
              0,
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(180.0), Units.degreesToRadians(180.0))));
  private static final SwerveRequest.ApplyRobotSpeeds ROBOT_RELATIVE =
      new SwerveRequest.ApplyRobotSpeeds();
  private static final ChassisSpeeds STOP = new ChassisSpeeds();
  private static final Distance TOLERANCE_TRANSLATION = Inches.of(1.25);
  private static final Angle TOLERANCE_ANGLE = Degrees.of(2.0);
  private static final LinearVelocity TOLERANCE_TRANSLATION_VELOCITY = InchesPerSecond.of(10.0);
  private static final AngularVelocity TOLERANCE_ANGULAR_VELOCITY = DegreesPerSecond.of(20.0);

  public static Pose2d getTargetPose() {
    return TARGET_POSE;
  }

  private static Pose2d TARGET_POSE = new Pose2d();

  public static void setTargetPose(final Pose2d newPose) {
    TARGET_POSE = newPose;
  }

  /** Creates a new AutoAlign. */
  public AutoAlign() {
    addRequirements(RobotContainer.DRIVETRAIN);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    HOLONOMIC_CONTROLLER.getXController().reset();
    HOLONOMIC_CONTROLLER.getYController().reset();
    HOLONOMIC_CONTROLLER
        .getThetaController()
        .reset(RobotContainer.DRIVETRAIN.getPose().getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //    System.out.println("is running " + TARGET_POSE);
    RobotContainer.DRIVETRAIN.setControl(
        ROBOT_RELATIVE.withSpeeds(
            HOLONOMIC_CONTROLLER.calculate(
                RobotContainer.DRIVETRAIN.getPose(), TARGET_POSE, 0, TARGET_POSE.getRotation())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //    System.out.println("COMPLETE");
    RobotContainer.DRIVETRAIN.setControl(ROBOT_RELATIVE.withSpeeds(STOP));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final Pose2d currentPose = RobotContainer.DRIVETRAIN.getPose();
    final ChassisSpeeds currentChassisSpeeds = RobotContainer.DRIVETRAIN.getChassisSpeeds();
    return Inches.of(currentPose.getTranslation().getDistance(TARGET_POSE.getTranslation()))
            .lte(TOLERANCE_TRANSLATION)
        && currentPose
            .getRotation()
            .getMeasure()
            .isNear(TARGET_POSE.getRotation().getMeasure(), TOLERANCE_ANGLE)
        && MetersPerSecond.of(currentChassisSpeeds.vxMetersPerSecond)
            .lte(TOLERANCE_TRANSLATION_VELOCITY)
        && MetersPerSecond.of(currentChassisSpeeds.vyMetersPerSecond)
            .lte(TOLERANCE_TRANSLATION_VELOCITY)
        && RadiansPerSecond.of(currentChassisSpeeds.omegaRadiansPerSecond)
            .lte(TOLERANCE_ANGULAR_VELOCITY);
  }
}
