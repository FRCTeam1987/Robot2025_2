package frc.robot;

import frc.robot.state.Abomination;
import frc.robot.state.logic.actions.DesiredAction;
import frc.robot.state.logic.constants.PositionConstant;
import frc.robot.state.logic.mode.CollectMode;
import frc.robot.state.logic.mode.DriveMode;
import frc.robot.state.logic.mode.ScoreMode;
import frc.robot.utils.InstCmd;

public class Bindings extends RobotContainer {
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

    JOYSTICK
        .rightTrigger()
        .onTrue(new InstCmd(() -> Abomination.setDriveMode(DriveMode.AUTOMATIC)));
    JOYSTICK.leftTrigger().onTrue(new InstCmd(() -> Abomination.setDriveMode(DriveMode.MANUAL)));
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
