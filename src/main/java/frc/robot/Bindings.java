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
                            -JOYSTICK.getLeftY())) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MAX_SPEED.times(-JOYSTICK.getLeftX())) // Drive left with negative X (left)
                    .withRotationalRate(
                        MAX_ANGULAR_RATE.times(
                            -JOYSTICK.getRightX())))); // Drive counterclockwise with negative X
    // (left)

    JOYSTICK.rightBumper().onTrue(new InstCmd(() -> Abomination.setAction(DesiredAction.SCORE)));
    JOYSTICK.leftBumper().onTrue(new InstCmd(() -> Abomination.setAction(DesiredAction.INIT)));

    JOYSTICK.rightTrigger().whileTrue(new DriveToNearest(false));
    JOYSTICK.leftTrigger().whileTrue(new DriveToNearest(true));
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
                  Abomination.setScoreMode(ScoreMode.L4);
                  Abomination.setCollectMode(CollectMode.HUMAN_PLAYER_STATION);
                }));

    JOYSTICK.x().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L3)));
    JOYSTICK.y().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L4)));
    JOYSTICK.b().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L2)));
    JOYSTICK.a().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L1)));
    CODRIVER_JOYSTICK.x().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L3)));
    CODRIVER_JOYSTICK.y().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L4)));
    CODRIVER_JOYSTICK.b().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L2)));
    CODRIVER_JOYSTICK.a().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L1)));
    JOYSTICK.leftStick().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.PROCESSOR)));
    JOYSTICK.rightStick().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.NET)));
    CODRIVER_JOYSTICK
        .back()
        .onTrue(
            new InstCmd(
                () -> {
                  Abomination.setAction(DesiredAction.RECOVERY);
                  Abomination.setScoreMode(ScoreMode.L4);
                  Abomination.setCollectMode(CollectMode.HUMAN_PLAYER_STATION);
                }));
    JOYSTICK.povDown().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.CLIMB)));
    CODRIVER_JOYSTICK
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
