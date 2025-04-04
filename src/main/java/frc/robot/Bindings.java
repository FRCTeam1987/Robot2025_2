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
            new InstCmd(() -> DRIVETRAIN.resetPose(PositionConstant.SIDE_1_ALGAE.getAlliancePose()))
                .ignoringDisable(true));

    JOYSTICK
        .back()
        .onTrue(
            new InstCmd(
                    () -> {
                      Abomination.setScoreMode(ScoreMode.L4, true);
                      Abomination.setAction(DesiredAction.RECOVERY);
                      Abomination.setCollectMode(CollectMode.HUMAN_PLAYER_STATION);
                    })
                .ignoringDisable(true));

    JOYSTICK.x().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L3, false)));
    JOYSTICK.y().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L4, false)));
    JOYSTICK.b().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L2, false)));
    JOYSTICK.a().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L1, false)));
    CODRIVER_JOYSTICK.x().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L3, false)));
    CODRIVER_JOYSTICK.y().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L4, false)));
    CODRIVER_JOYSTICK.b().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L2, false)));
    CODRIVER_JOYSTICK.a().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.L1, false)));
    JOYSTICK
        .leftStick()
        .onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.PROCESSOR, false)));
    JOYSTICK.rightStick().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.NET, false)));
    CODRIVER_JOYSTICK
        .back()
        .onTrue(
            new InstCmd(
                () -> {
                  Abomination.setScoreMode(ScoreMode.L4, true);
                  Abomination.setAction(DesiredAction.RECOVERY);
                  Abomination.setCollectMode(CollectMode.HUMAN_PLAYER_STATION);
                }));
    JOYSTICK.povDown().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.CLIMB, true)));
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
