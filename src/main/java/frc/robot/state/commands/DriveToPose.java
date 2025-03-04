package frc.robot.state.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import java.util.function.Supplier;

public class DriveToPose extends Command {
  private final Drivetrain DRIVE;
  private final Supplier<Pose2d> TARGET;
  private final Supplier<Pose2d> ROBOT;
  private final HolonomicDriveController HOLONOMIC =
      new HolonomicDriveController(
          // PID constants for translation
          new PIDController(4.25, 0, 0),
          // PID constants for rotation
          new PIDController(4.25, 0, 0),
          new ProfiledPIDController(
              4.0,
              0,
              0,
              new TrapezoidProfile.Constraints(Units.degreesToRadians(360.0), 8.0),
              0.02));
  boolean running = false;
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private Translation2d lastSetpointTranslation = new Translation2d();
  private Debouncer hasTargetDebounce;

  public DriveToPose(Drivetrain drive, Pose2d target) {
    addRequirements(drive);
    this.DRIVE = drive;
    this.TARGET = () -> target;
    this.ROBOT = RobotContainer.DRIVETRAIN::getPose;
  }

  public DriveToPose() {
    this.DRIVE = null;
    this.TARGET = null;
    this.ROBOT = null;
  }

  @Override
  public void initialize() {
    this.hasTargetDebounce = new Debouncer(1.5);
    Pose2d currentPose = ROBOT.get();

    ChassisSpeeds fieldVelocity = RobotContainer.DRIVETRAIN.getState().Speeds;
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

    HOLONOMIC.getXController().reset();
    HOLONOMIC.getYController().reset();
    HOLONOMIC.getThetaController().reset(currentPose.getRotation().getRadians());
    lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    // Get current pose and target pose
    Pose2d currentPose = ROBOT.get();
    Pose2d targetPose = TARGET.get();

    // Calculate drive speed

    driveErrorAbs = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    SwerveRequest request =
        new SwerveRequest.ApplyRobotSpeeds()
            .withSpeeds(
                HOLONOMIC.calculate(currentPose, targetPose, 0.0, targetPose.getRotation()));

    // Command speeds
    DRIVE.setControl(request);
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    // Clear logs
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running
        && DRIVE.getPose().getTranslation().getDistance(TARGET.get().getTranslation()) < 0.02
        && DRIVE.getState().Speeds.vxMetersPerSecond < 0.01
        && DRIVE.getState().Speeds.vyMetersPerSecond < 0.01;
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}
