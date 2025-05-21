// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.RobotContainer.DRIVETRAIN;
import static frc.robot.subsystems.constants.SubsystemConstants.LightsConstants.UPRIGHTS;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.state.Abomination;
import frc.robot.state.logic.functional.FunctionalState;
import frc.robot.state.logic.mode.CollectMode;
import frc.robot.state.logic.mode.ScoreMode;
import frc.robot.subsystems.constants.SubsystemConstants;

/** Add your docs here. */
public class Lights extends BroncSystem {
  public static ScoreMode PREVIOUS_SCORE_MODE = Abomination.getScoreMode();
  public static FunctionalState PREVIOUS_STATE = FunctionalState.COLLECT;
  public static CollectMode PREVIOUS_COLLECT_MODE = Abomination.getCollectMode();
  public static boolean PREVIOUSLY_ENABLED = false;
  public static DriverStation.Alliance PREVIOUS_ALLIANCE = DriverStation.Alliance.Red;

  private static final CANdle CANDLE =
      new CANdle(
          SubsystemConstants.LightsConstants.CANDLE_ID,
          SubsystemConstants.LightsConstants.CANBUS_NAME);

  public Lights() {}

  @Override
  public void preCycle() {
    log();
  }

  @Override
  public void postCycle() {
    PREVIOUS_STATE = Abomination.getPreviousState();
    PREVIOUS_SCORE_MODE = Abomination.getScoreMode();
    PREVIOUS_COLLECT_MODE = Abomination.getCollectMode();
    PREVIOUSLY_ENABLED = DriverStation.isEnabled();
    PREVIOUS_ALLIANCE = DRIVETRAIN.getAlliance();
  }

  @Override
  public void log() {

  }

  public void applyAnimationUpright(Animation animation) {
    if (shouldUpdate()) {
      CANDLE.clearAnimation(1);
      CANDLE.animate(animation, 1);
    }
  }

  public void applyAnimationSide(Animation animation) {
    if (shouldUpdate()) {
      CANDLE.clearAnimation(0);
      CANDLE.animate(animation, 0);
    }
  }

  public boolean shouldUpdate() {

    return Abomination.getScoreMode() != PREVIOUS_SCORE_MODE
        || Abomination.getCollectMode() != PREVIOUS_COLLECT_MODE
        || DriverStation.isEnabled() != PREVIOUSLY_ENABLED
        || Abomination.getPreviousState() != PREVIOUS_STATE
        || DRIVETRAIN.getAlliance() != PREVIOUS_ALLIANCE;
  }

  public void applyAnimations(Animation animationSide, Animation animationUpright) {
    applyAnimationSide(animationSide);
    applyAnimationUpright(animationUpright);
  }

  public void setColor(Color8Bit color) {
    CANDLE.clearAnimation(0);
    CANDLE.clearAnimation(1);
    CANDLE.setLEDs(
        color.red,
        color.green,
        color.blue,
        0,
        0,
        SubsystemConstants.LightsConstants.SIDE + UPRIGHTS);
  }

  public void off() {
    CANDLE.clearAnimation(0);
    CANDLE.setLEDs(0, 0, 0);
  }
}
