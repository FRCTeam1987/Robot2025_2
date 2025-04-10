// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.subsystems.constants.SubsystemConstants.IntakeConstants.*;
import static frc.robot.subsystems.constants.SubsystemConstants.LightsConstants.LEDS_UPRIGHTS;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.constants.SubsystemConstants;

/** Add your docs here. */
public class Lights {
  private static Animation CURRENT_ANIMATION_SIDE = new FireAnimation();
  private static Animation CURRENT_ANIMATION_UPRIGHTS = new FireAnimation();
  private static final CANdle CANDLE =
      new CANdle(
          SubsystemConstants.LightsConstants.CANDLE_ID,
          SubsystemConstants.LightsConstants.CANBUS_NAME);

  public Lights() {}

  public void cycle() {
    log();
  }

  public void log() {
    //        StatusCode motorStatus = BaseStatusSignal.refreshAll(INTAKE_SUPPLY_CURRENT,
    // INTAKE_VELOCITY);
    //        if (RobotContainer.DEBUG) {
    //            DogLog.log("Intake/supplyCurrent", INTAKE_SUPPLY_CURRENT.getValueAsDouble());
    //            DogLog.log("Intake/velocity", INTAKE_VELOCITY.getValueAsDouble());
    //            DogLog.log("Intake/isConnected", motorStatus.isOK());
    //        }
  }

  public void applyAnimationUpright(Animation animation) {
    if (CURRENT_ANIMATION_UPRIGHTS != animation) {
      CANDLE.clearAnimation(1);
      CANDLE.animate(animation, 1);
      CURRENT_ANIMATION_UPRIGHTS = animation;
    }
  }

  public void applyAnimationSide(Animation animation) {
    if (CURRENT_ANIMATION_SIDE != animation) {
      CANDLE.clearAnimation(0);
      CANDLE.animate(animation, 0);
      CURRENT_ANIMATION_SIDE = animation;
    }
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
        SubsystemConstants.LightsConstants.LEDS_SIDE + LEDS_UPRIGHTS);
  }

  public void off() {
    CANDLE.clearAnimation(0);
    CANDLE.setLEDs(0, 0, 0);
  }
}
