package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.state.Abomination;
import frc.robot.state.commands.AsyncRumble;
import frc.robot.state.commands.DriveToNearest;
import frc.robot.state.logic.actions.DesiredAction;
import frc.robot.state.logic.constants.PositionConstant;
import frc.robot.state.logic.mode.CollectMode;
import frc.robot.state.logic.mode.ScoreMode;
import frc.robot.utils.InstCmd;

public class Bindings extends RobotContainer {
  private static DriveToNearest NEAREST = new DriveToNearest();

  public static void configureBindings() {

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    DRIVETRAIN.setDefaultCommand(
        // Drivetrain will execute this command periodically
        DRIVETRAIN.applyRequest(
            () ->
                DRIVE
                    .withVelocityX(
                        MAX_SPEED.times(
                            -JOYSTICK.getLeftY())) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MAX_SPEED.times(-JOYSTICK.getLeftX())) // Drive left with negative X (left)
                    .withRotationalRate(
                        MAX_ANGULAR_RATE.times(
                            -JOYSTICK
                                .getRightX())))); // Drive counterclockwise with negative X (left)

    JOYSTICK.a().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.PROCESSOR)));
    JOYSTICK.b().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L2)));
    JOYSTICK.x().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L4)));
    JOYSTICK.y().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L3)));

    JOYSTICK.rightBumper().onTrue(new InstCmd(() -> Abomination.setAction(DesiredAction.SCORE)));
    JOYSTICK.leftBumper().onTrue(new InstCmd(() -> Abomination.setAction(DesiredAction.INIT)));

    JOYSTICK.rightTrigger().whileTrue(NEAREST);
    new Trigger(() -> NEAREST.isFinished())
        .whileTrue(
            new AsyncRumble(JOYSTICK.getHID(), GenericHID.RumbleType.kBothRumble, 1.0, 3000));
    JOYSTICK
        .povUp()
        .onTrue(new InstCmd(() -> Abomination.setCollectMode(CollectMode.HUMAN_PLAYER_STATION)));
    JOYSTICK.povLeft().onTrue(new InstCmd(() -> Abomination.setCollectMode(CollectMode.ALGAE_2)));
    JOYSTICK.povRight().onTrue(new InstCmd(() -> Abomination.setCollectMode(CollectMode.ALGAE_3)));

    JOYSTICK
        .start()
        .onTrue(
            new InstCmd(() -> DRIVETRAIN.resetPose(PositionConstant.SIDE_1_ALGAE.getRedPose())));
  }
}
