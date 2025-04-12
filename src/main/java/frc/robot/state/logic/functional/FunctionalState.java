package frc.robot.state.logic.functional;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotContainer.*;
import static frc.robot.state.Abomination.getCollectMode;
import static frc.robot.state.Abomination.getScoreMode;
import static frc.robot.state.logic.constants.StateConstants.COLLECT_ZONES;
import static frc.robot.subsystems.constants.SubsystemConstants.LightsConstants.*;

import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import frc.robot.state.Abomination;
import frc.robot.state.logic.constants.MechanismConstant;
import frc.robot.state.logic.mode.CollectMode;

public enum FunctionalState {
  COLLECT(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getCollectMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(getCollectMode().getMechanismConstant().getArmAngle()),
          () ->
              ARM.setClawVoltage(
                  COLLECT_ZONES.contains(getLocalizationState().fieldZone())
                      ? Abomination.getCollectMode().equals(CollectMode.HUMAN_PLAYER_STATION)
                          ? ARM.hasGamePieceEntrance() ? Volts.of(9.0) : Volts.of(2.5)
                          : Volts.of(14.0)
                      : Volts.of(0.0)),
          () -> {
            if (JOYSTICK.back().getAsBoolean()) {
              INTAKE.reverse();
            } else {
              INTAKE.start();
            }
          },
          () -> {
            if (DriverStation.isDisabled()) {
              Color8Bit C =
                  DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
                      ? new Color8Bit(255, 0, 0)
                      : new Color8Bit(0, 0, 255);
              LIGHTS.applyAnimationSide(
                  new SingleFadeAnimation(
                      C.red, C.green, C.blue, 0, IDLE_SPEED, SIDE, SIDE_OFFSET));
              LIGHTS.applyAnimationUpright(
                  new SingleFadeAnimation(
                      C.red, C.green, C.blue, 0, IDLE_SPEED, UPRIGHTS, UPRIGHTS_OFFSET));
            } else {
              LIGHTS.applyAnimationSide(
                  new LarsonAnimation(
                      0,
                      255,
                      255,
                      0,
                      UP_SPEED,
                      SIDE,
                      LarsonAnimation.BounceMode.Front,
                      LARSON_SIZE,
                      SIDE_OFFSET));
              LIGHTS.applyAnimationUpright(
                  new LarsonAnimation(
                      0,
                      255,
                      255,
                      0,
                      UP_SPEED,
                      UPRIGHTS,
                      LarsonAnimation.BounceMode.Front,
                      LARSON_SIZE,
                      UPRIGHTS_OFFSET));
            }
          })),

  COLLECTED_CORAL(
      new FunctionalAction(
          // stroke behaivor

          () -> {
            //            if (DriverStation.isAutonomous()
            //                && Timer.getFPGATimestamp() < AutoHelpers.matchTimeIncrement + 3.0) {
            //              ELEVATOR.setDistance(MechanismConstant.L3.getElevatorDistance());
            //              return;
            //            }
            ELEVATOR.setDistance(MechanismConstant.IDLE_CORAL.getElevatorDistance());
          },
          () -> ARM.setArmPosition(getScoreMode().getIdleMechanismConstant().getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.9)),
          INTAKE::stop,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new SingleFadeAnimation(
                    COLOR.red, COLOR.green, COLOR.blue, 0, COLLECTED_SPEED, SIDE, SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new SingleFadeAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    COLLECTED_SPEED,
                    UPRIGHTS,
                    UPRIGHTS_OFFSET));
          })),
  COLLECTED_ALGAE(
      new FunctionalAction(
          () ->
              ELEVATOR.setDistance(getScoreMode().getIdleMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(getScoreMode().getIdleMechanismConstant().getArmAngle()),
          ARM::dynamicHold,
          INTAKE::stop,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new SingleFadeAnimation(
                    COLOR.red, COLOR.green, COLOR.blue, 0, COLLECTED_SPEED, SIDE, SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new SingleFadeAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    COLLECTED_SPEED,
                    UPRIGHTS,
                    UPRIGHTS_OFFSET));
          })),
  PROCESSOR_ELEVATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.A2.getArmAngle()),
          ARM::dynamicHold,
          INTAKE::stop,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    UP_SPEED,
                    SIDE,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    UP_SPEED,
                    UPRIGHTS,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    UPRIGHTS_OFFSET));
          })),
  PROCESSOR_ROTATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(getScoreMode().getMechanismConstant().getArmAngle()),
          ARM::dynamicHold,
          INTAKE::stop,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    UP_SPEED,
                    SIDE,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    UP_SPEED,
                    UPRIGHTS,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    UPRIGHTS_OFFSET));
          })),
  PROCESSOR_SCORE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(getScoreMode().getMechanismConstant().getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(-6.0)),
          INTAKE::stop,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new StrobeAnimation(
                    COLOR.red, COLOR.green, COLOR.blue, 0, SCORE_STROBE_SPEED, SIDE, SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new StrobeAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    SCORE_STROBE_SPEED,
                    UPRIGHTS,
                    UPRIGHTS_OFFSET));
          })),
  NET_ELEVATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () ->
              ARM.setArmPosition(
                  getScoreMode().getMechanismConstant().getArmAngle().minus(Degrees.of(35))),
          ARM::dynamicHold,
          INTAKE::stop,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    UP_SPEED,
                    SIDE,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    UP_SPEED,
                    UPRIGHTS,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    UPRIGHTS_OFFSET));
          })),
  NET_ROTATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () ->
              ARM.setArmPosition(
                  getScoreMode().getMechanismConstant().getArmAngle().minus(Degrees.of(35))),
          () -> ARM.setClawVoltage(Volts.of(4.25)),
          INTAKE::stop,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    UP_SPEED,
                    SIDE,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    UP_SPEED,
                    UPRIGHTS,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    UPRIGHTS_OFFSET));
          })),
  NET_SCORE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          //   () ->
          //       ARM.setArmPosition(
          //           getScoreMode().getMechanismConstant().getArmAngle().plus(Degrees.of(15))),
          () -> ARM.setArmPosition(MechanismConstant.HP_INTAKE.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(-16.0)),
          INTAKE::stop,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new StrobeAnimation(
                    COLOR.red, COLOR.green, COLOR.blue, 0, SCORE_STROBE_SPEED, SIDE, SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new StrobeAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    SCORE_STROBE_SPEED,
                    UPRIGHTS,
                    UPRIGHTS_OFFSET));
          })),
  NET_UNROTATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.HP_INTAKE.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(1.0)),
          INTAKE::stop,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    DOWN_SPEED,
                    SIDE,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    DOWN_SPEED,
                    UPRIGHTS,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    UPRIGHTS_OFFSET));
          })),
  NET_UNELEVATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(MechanismConstant.IDLE_CORAL.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.HP_INTAKE.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(1.0)),
          INTAKE::stop,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    DOWN_SPEED,
                    SIDE,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    DOWN_SPEED,
                    UPRIGHTS,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    UPRIGHTS_OFFSET));
          })),
  LEVEL_X_ELEVATE(
      new FunctionalAction(
          () -> {
            ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance());
          },
          () -> ARM.setArmPosition(getScoreMode().getIdleMechanismConstant().getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(1)),
          INTAKE::stop,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    UP_SPEED,
                    SIDE,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    UP_SPEED,
                    UPRIGHTS,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    UPRIGHTS_OFFSET));
          })),
  LEVEL_X_ROTATE(
      new FunctionalAction(
          () -> {
            ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance());
          },
          () -> ARM.setArmPosition(getScoreMode().getMechanismConstant().getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(1)),
          INTAKE::stop,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new StrobeAnimation(
                    COLOR.red, COLOR.green, COLOR.blue, 0, SCORE_STROBE_SPEED, SIDE, SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new StrobeAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    SCORE_STROBE_SPEED,
                    UPRIGHTS,
                    UPRIGHTS_OFFSET));
          })),
  LEVEL_X_SCORE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(getScoreMode().getMechanismConstant().getArmAngle()),
          () -> {
            // this isn't pretty, but it should be efficient per the JVM
            boolean inBack = ARM.hasGamePieceBack();
            ARM.setClawVoltage(
                Volts.of(
                    switch (getScoreMode()) {
                      case L1 -> inBack ? -2.75 : -2.75;
                      case L4 -> inBack ? -3.0 : -2.5;
                      default -> inBack ? -2.0 : -1.4;
                    }));
          },
          INTAKE::stop,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new StrobeAnimation(
                    COLOR.red, COLOR.green, COLOR.blue, 0, SCORE_STROBE_SPEED, SIDE, SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new StrobeAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    SCORE_STROBE_SPEED,
                    UPRIGHTS,
                    UPRIGHTS_OFFSET));
          })),
  LEVEL_X_UNROTATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.IDLE_CORAL.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(-6.0)),
          INTAKE::stop,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    DOWN_SPEED,
                    SIDE,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    DOWN_SPEED,
                    UPRIGHTS,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    UPRIGHTS_OFFSET));
          })),
  LEVEL_X_UNELEVATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(MechanismConstant.IDLE_CORAL.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.HP_INTAKE.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.0)),
          INTAKE::stop,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    DOWN_SPEED,
                    SIDE,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    DOWN_SPEED,
                    UPRIGHTS,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    UPRIGHTS_OFFSET));
          })),
  CLIMB_DEPLOY(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(MechanismConstant.CLIMB.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.CLIMB.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.0)),
          INTAKE::stop,
          CLIMBER::deploy,
          () -> {
            Color8Bit COLOR = getScoreColor();
            LIGHTS.applyAnimationSide(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    UP_SPEED,
                    SIDE,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    SIDE_OFFSET));
            LIGHTS.applyAnimationUpright(
                new LarsonAnimation(
                    COLOR.red,
                    COLOR.green,
                    COLOR.blue,
                    0,
                    UP_SPEED,
                    UPRIGHTS,
                    LarsonAnimation.BounceMode.Front,
                    LARSON_SIZE,
                    UPRIGHTS_OFFSET));
          })),
  CLIMB_CLIMB(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(MechanismConstant.CLIMB.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.CLIMB.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.0)),
          INTAKE::stop,
          CLIMBER::climb,
          () -> {
            if (!Robot.hasCoasted) {
              LIGHTS.applyAnimationSide(
                  new SingleFadeAnimation(255, 0, 0, 0, CLIMB_CLIMB_SPEED, SIDE, SIDE_OFFSET));
              LIGHTS.applyAnimationUpright(
                  new SingleFadeAnimation(
                      255, 0, 0, 0, CLIMB_CLIMB_SPEED, UPRIGHTS, UPRIGHTS_OFFSET));
            } else {
              LIGHTS.applyAnimationSide(
                  new SingleFadeAnimation(255, 0, 0, 0, IDLE_SPEED, SIDE, SIDE_OFFSET));
              LIGHTS.applyAnimationUpright(
                  new SingleFadeAnimation(255, 0, 0, 0, IDLE_SPEED, UPRIGHTS, UPRIGHTS_OFFSET));
            }
          })),
  ;

  public final FunctionalAction ACTION;

  FunctionalState(FunctionalAction ACTION) {
    this.ACTION = ACTION;
  }
}
