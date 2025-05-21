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

    DRIVETRAIN.setDefaultCommand(
        DRIVETRAIN.applyRequest(
            () ->
                DRIVE
                    .withVelocityX(MAX_SPEED.times(-JOYSTICK.getLeftY()))
                    .withVelocityY(MAX_SPEED.times(-JOYSTICK.getLeftX()))
                    .withRotationalRate(MAX_ANGULAR_RATE.times(-JOYSTICK.getRightX()))));

    JOYSTICK.rightBumper().onTrue(new InstCmd(() -> Abomination.setAction(DesiredAction.SCORE)));
    JOYSTICK.leftBumper().onTrue(new InstCmd(() -> Abomination.setAction(DesiredAction.INIT)));

    JOYSTICK.rightTrigger().whileTrue(new DriveToNearest(false, false));
    JOYSTICK.leftTrigger().whileTrue(new DriveToNearest(true, false));
    JOYSTICK
        .povUp()
        .onTrue(new InstCmd(() -> Abomination.setCollectMode(CollectMode.HUMAN_PLAYER_STATION)));
    JOYSTICK.povLeft().onTrue(new InstCmd(() -> Abomination.setCollectMode(CollectMode.ALGAE_2)));
    JOYSTICK.povRight().onTrue(new InstCmd(() -> Abomination.setCollectMode(CollectMode.ALGAE_3)));
    JOYSTICK.povDown().onTrue(new InstCmd(() -> Abomination.setScoreMode(ScoreMode.CLIMB, true)));

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
  }
}
