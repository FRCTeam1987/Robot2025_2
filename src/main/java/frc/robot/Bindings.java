package frc.robot;

import frc.robot.state.Abomination;
import frc.robot.state.commands.DriveToNearest;
import frc.robot.state.logic.actions.DesiredAction;
import frc.robot.state.logic.constants.PositionConstant;
import frc.robot.state.logic.mode.CollectMode;
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
                            -JOYSTICK.getLeftY()
                                * (1
                                    - (JOYSTICK.getLeftTriggerAxis()
                                        * 0.5)))) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MAX_SPEED.times(
                            -JOYSTICK.getLeftX()
                                * (1
                                    - (JOYSTICK.getLeftTriggerAxis()
                                        * 0.5)))) // Drive left with negative X (left)
                    .withRotationalRate(
                        MAX_ANGULAR_RATE.times(
                            -JOYSTICK.getRightX()
                                * (1
                                    - (JOYSTICK.getLeftTriggerAxis()
                                        * 0.5)))))); // Drive counterclockwise with negative X
    // (left)

    JOYSTICK.rightBumper().onTrue(new InstCmd(() -> Abomination.setAction(DesiredAction.SCORE)));
    JOYSTICK.leftBumper().onTrue(new InstCmd(() -> Abomination.setAction(DesiredAction.INIT)));

    JOYSTICK.rightTrigger().whileTrue(new DriveToNearest());
    //    new Trigger(() -> NEAREST.isFinished())
    //        .whileTrue(
    //            new AsyncRumble(JOYSTICK.getHID(), GenericHID.RumbleType.kBothRumble, 1.0, 3000));
    JOYSTICK
        .povUp()
        .onTrue(new InstCmd(() -> Abomination.setCollectMode(CollectMode.HUMAN_PLAYER_STATION)));
    JOYSTICK.povLeft().onTrue(new InstCmd(() -> Abomination.setCollectMode(CollectMode.ALGAE_2)));
    JOYSTICK.povRight().onTrue(new InstCmd(() -> Abomination.setCollectMode(CollectMode.ALGAE_3)));

    JOYSTICK
        .start()
        .onTrue(
            new InstCmd(
                () -> DRIVETRAIN.resetPose(PositionConstant.SIDE_1_ALGAE.getAlliancePose())));

    JOYSTICK
        .back()
        .onTrue(
            new InstCmd(
                () -> {
                  Abomination.setAction(DesiredAction.RECOVERY);
                  Abomination.setScoreMode(ScoreMode.L3);
                  Abomination.setCollectMode(CollectMode.HUMAN_PLAYER_STATION);
                }));

    CODRIVER_JOSYTICK.x().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L3)));
    CODRIVER_JOSYTICK.y().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L4)));
    CODRIVER_JOSYTICK.b().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L2)));
    CODRIVER_JOSYTICK
        .povLeft()
        .onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.PROCESSOR)));
    CODRIVER_JOSYTICK.povRight().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.NET)));
    CODRIVER_JOSYTICK
        .back()
        .onTrue(
            new InstCmd(
                () -> {
                  Abomination.setAction(DesiredAction.RECOVERY);
                  Abomination.setScoreMode(ScoreMode.L3);
                  Abomination.setCollectMode(CollectMode.HUMAN_PLAYER_STATION);
                }));
    CODRIVER_JOSYTICK
        .leftBumper()
        .onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.CLIMB)));
    CODRIVER_JOSYTICK
        .rightBumper()
        .onTrue(new InstCmd(() -> Abomination.setAction(DesiredAction.SCORE)));

    //    CODRIVER_JOSYTICK
    //        .start()
    //        .whileTrue(
    //            new DriveToPose(
    //                DRIVETRAIN,
    //                LocalizationUtil.blueFlipToRed(new Pose2d(7.15, 5.07, new Rotation2d(0.0)))));
  }
}
