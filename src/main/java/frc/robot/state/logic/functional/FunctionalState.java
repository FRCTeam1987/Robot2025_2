package frc.robot.state.logic.functional;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotContainer.*;
import static frc.robot.state.Abomination.getCollectMode;
import static frc.robot.state.Abomination.getScoreMode;
import static frc.robot.state.logic.constants.StateConstants.COLLECT_ZONES;
import static frc.robot.subsystems.constants.SubsystemConstants.LightsConstants.*;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.DriverStation;
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
              RGBWColor C =
                  DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
                      ? new RGBWColor(255, 0, 0)
                      : new RGBWColor(0, 0, 255);
              LIGHTS.setControl(new EmptyAnimation(1));
              LIGHTS.setControl(
                  new SingleFadeAnimation(SIDE_START, UPRIGHTS_END)
                      .withSlot(0)
                      .withColor(C)
                      .withFrameRate(60));
            } else {
              LIGHTS.setControl(
                  new LarsonAnimation(SIDE_START, SIDE_END)
                      .withSlot(0)
                      .withColor(COLLECT_COLOR)
                      .withBounceMode(LarsonBounceValue.Center)
                      .withFrameRate(IDLE_SPEED));
              LIGHTS.setControl(
                  new LarsonAnimation(UPRIGHTS_START, UPRIGHTS_END)
                      .withSlot(1)
                      .withColor(COLLECT_COLOR)
                      .withBounceMode(LarsonBounceValue.Center)
                      .withFrameRate(IDLE_SPEED));
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
            if (DriverStation.isDisabled()) {
              RGBWColor C =
                  DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
                      ? new RGBWColor(255, 0, 0)
                      : new RGBWColor(0, 0, 255);
              LIGHTS.setControl(new EmptyAnimation(1));
              LIGHTS.setControl(
                  new SingleFadeAnimation(SIDE_START, UPRIGHTS_END)
                      .withSlot(0)
                      .withColor(C)
                      .withFrameRate(60));
            } else {
              RGBWColor C = getScoreColor();
              LIGHTS.setControl(
                  new LarsonAnimation(SIDE_START, SIDE_END)
                      .withSlot(0)
                      .withColor(C)
                      .withBounceMode(LarsonBounceValue.Center)
                      .withFrameRate(COLLECTED_SPEED));
              LIGHTS.setControl(
                  new LarsonAnimation(UPRIGHTS_START, UPRIGHTS_END)
                      .withSlot(1)
                      .withColor(C)
                      .withBounceMode(LarsonBounceValue.Center)
                      .withFrameRate(COLLECTED_SPEED));
            }
          })),
  COLLECTED_ALGAE(
      new FunctionalAction(
          () ->
              ELEVATOR.setDistance(getScoreMode().getIdleMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(getScoreMode().getIdleMechanismConstant().getArmAngle()),
          ARM::dynamicHold,
          INTAKE::stop,
          () -> {
            if (DriverStation.isDisabled()) {
              RGBWColor C =
                  DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
                      ? new RGBWColor(255, 0, 0)
                      : new RGBWColor(0, 0, 255);
              LIGHTS.setControl(new EmptyAnimation(1));
              LIGHTS.setControl(
                  new SingleFadeAnimation(SIDE_START, UPRIGHTS_END)
                      .withSlot(0)
                      .withColor(C)
                      .withFrameRate(60));
            } else {
              RGBWColor C = getScoreColor();
              LIGHTS.setControl(
                  new LarsonAnimation(SIDE_START, SIDE_END)
                      .withSlot(0)
                      .withColor(C)
                      .withBounceMode(LarsonBounceValue.Center)
                      .withFrameRate(COLLECTED_SPEED));
              LIGHTS.setControl(
                  new LarsonAnimation(UPRIGHTS_START, UPRIGHTS_END)
                      .withSlot(1)
                      .withColor(C)
                      .withBounceMode(LarsonBounceValue.Center)
                      .withFrameRate(COLLECTED_SPEED));
            }
          })),
  PROCESSOR_ELEVATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.A2.getArmAngle()),
          ARM::dynamicHold,
          INTAKE::stop,
          () -> {
            RGBWColor C = getScoreColor();
            LIGHTS.setControl(
                new LarsonAnimation(SIDE_START, SIDE_END)
                    .withSlot(0)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(UP_SPEED));
            LIGHTS.setControl(
                new LarsonAnimation(UPRIGHTS_START, UPRIGHTS_END)
                    .withSlot(1)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(UP_SPEED));
          })),
  PROCESSOR_ROTATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(getScoreMode().getMechanismConstant().getArmAngle()),
          ARM::dynamicHold,
          INTAKE::stop,
          () -> {
            RGBWColor C = getScoreColor();
            LIGHTS.setControl(
                new LarsonAnimation(SIDE_START, SIDE_END)
                    .withSlot(0)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(UP_SPEED));
            LIGHTS.setControl(
                new LarsonAnimation(UPRIGHTS_START, UPRIGHTS_END)
                    .withSlot(1)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(UP_SPEED));
          })),
  PROCESSOR_SCORE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(getScoreMode().getMechanismConstant().getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(-6.0)),
          INTAKE::stop,
          () -> {
            RGBWColor C = getScoreColor();
            LIGHTS.setControl(
                new StrobeAnimation(SIDE_START, SIDE_END)
                    .withSlot(0)
                    .withColor(C)
                    .withFrameRate(SCORE_STROBE_SPEED));
            LIGHTS.setControl(
                new StrobeAnimation(UPRIGHTS_START, UPRIGHTS_END)
                    .withSlot(1)
                    .withColor(C)
                    .withFrameRate(SCORE_STROBE_SPEED));
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
            RGBWColor C = getScoreColor();
            LIGHTS.setControl(
                new LarsonAnimation(SIDE_START, SIDE_END)
                    .withSlot(0)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(UP_SPEED));
            LIGHTS.setControl(
                new LarsonAnimation(UPRIGHTS_START, UPRIGHTS_END)
                    .withSlot(1)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(UP_SPEED));
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
            RGBWColor C = getScoreColor();
            LIGHTS.setControl(
                new LarsonAnimation(SIDE_START, SIDE_END)
                    .withSlot(0)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(UP_SPEED));
            LIGHTS.setControl(
                new LarsonAnimation(UPRIGHTS_START, UPRIGHTS_END)
                    .withSlot(1)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(UP_SPEED));
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
            RGBWColor C = getScoreColor();
            LIGHTS.setControl(
                new StrobeAnimation(SIDE_START, SIDE_END)
                    .withSlot(0)
                    .withColor(C)
                    .withFrameRate(SCORE_STROBE_SPEED));
            LIGHTS.setControl(
                new StrobeAnimation(UPRIGHTS_START, UPRIGHTS_END)
                    .withSlot(1)
                    .withColor(C)
                    .withFrameRate(SCORE_STROBE_SPEED));
          })),
  NET_UNROTATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.HP_INTAKE.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(1.0)),
          INTAKE::stop,
          () -> {
            RGBWColor C = getScoreColor();
            LIGHTS.setControl(
                new LarsonAnimation(SIDE_START, SIDE_END)
                    .withSlot(0)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(DOWN_SPEED));
            LIGHTS.setControl(
                new LarsonAnimation(UPRIGHTS_START, UPRIGHTS_END)
                    .withSlot(1)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(DOWN_SPEED));
          })),
  NET_UNELEVATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(MechanismConstant.IDLE_CORAL.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.HP_INTAKE.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(1.0)),
          INTAKE::stop,
          () -> {
            RGBWColor C = getScoreColor();
            LIGHTS.setControl(
                new LarsonAnimation(SIDE_START, SIDE_END)
                    .withSlot(0)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(DOWN_SPEED));
            LIGHTS.setControl(
                new LarsonAnimation(UPRIGHTS_START, UPRIGHTS_END)
                    .withSlot(1)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(DOWN_SPEED));
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
            RGBWColor C = getScoreColor();
            LIGHTS.setControl(
                new LarsonAnimation(SIDE_START, SIDE_END)
                    .withSlot(0)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(UP_SPEED));
            LIGHTS.setControl(
                new LarsonAnimation(UPRIGHTS_START, UPRIGHTS_END)
                    .withSlot(1)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(UP_SPEED));
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
            RGBWColor C = getScoreColor();
            LIGHTS.setControl(
                new LarsonAnimation(SIDE_START, SIDE_END)
                    .withSlot(0)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(UP_SPEED));
            LIGHTS.setControl(
                new LarsonAnimation(UPRIGHTS_START, UPRIGHTS_END)
                    .withSlot(1)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(UP_SPEED));
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
                      case L1 -> inBack ? -3.75 : -3.75;
                      case L4 -> inBack ? -3.0 : -2.5;
                      default -> inBack ? -3.0 : -1.8;
                    }));
          },
          INTAKE::stop,
          () -> {
            RGBWColor C = getScoreColor();
            LIGHTS.setControl(
                new StrobeAnimation(SIDE_START, SIDE_END)
                    .withSlot(0)
                    .withColor(C)
                    .withFrameRate(SCORE_STROBE_SPEED));
            LIGHTS.setControl(
                new StrobeAnimation(UPRIGHTS_START, UPRIGHTS_END)
                    .withSlot(1)
                    .withColor(C)
                    .withFrameRate(SCORE_STROBE_SPEED));
          })),
  LEVEL_X_UNROTATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.IDLE_CORAL.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(-6.0)),
          INTAKE::stop,
          () -> {
            RGBWColor C = getScoreColor();
            LIGHTS.setControl(
                new LarsonAnimation(SIDE_START, SIDE_END)
                    .withSlot(0)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(DOWN_SPEED));
            LIGHTS.setControl(
                new LarsonAnimation(UPRIGHTS_START, UPRIGHTS_END)
                    .withSlot(1)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(DOWN_SPEED));
          })),
  LEVEL_X_UNELEVATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(MechanismConstant.IDLE_CORAL.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.HP_INTAKE.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.0)),
          INTAKE::stop,
          () -> {
            RGBWColor C = getScoreColor();
            LIGHTS.setControl(
                new LarsonAnimation(SIDE_START, SIDE_END)
                    .withSlot(0)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(DOWN_SPEED));
            LIGHTS.setControl(
                new LarsonAnimation(UPRIGHTS_START, UPRIGHTS_END)
                    .withSlot(1)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(DOWN_SPEED));
          })),
  CLIMB_DEPLOY(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(MechanismConstant.CLIMB.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.CLIMB.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.0)),
          INTAKE::stop,
          CLIMBER::deploy,
          () -> {
            RGBWColor C = getScoreColor();
            LIGHTS.setControl(
                new LarsonAnimation(SIDE_START, SIDE_END)
                    .withSlot(0)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(UP_SPEED));
            LIGHTS.setControl(
                new LarsonAnimation(UPRIGHTS_START, UPRIGHTS_END)
                    .withSlot(1)
                    .withColor(C)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(UP_SPEED));
          })),
  CLIMB_CLIMB(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(MechanismConstant.CLIMB.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.CLIMB.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.0)),
          INTAKE::stop,
          CLIMBER::climb,
          () -> {
            if (!Robot.hasClimberCoasted) {
              RGBWColor C =
                  DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
                      ? new RGBWColor(255, 0, 0)
                      : new RGBWColor(0, 0, 255);
              LIGHTS.setControl(new EmptyAnimation(1));
              LIGHTS.setControl(
                  new SingleFadeAnimation(SIDE_START, UPRIGHTS_END)
                      .withSlot(0)
                      .withColor(C)
                      .withFrameRate(60));
            } else {
              RGBWColor C = getScoreColor();
              LIGHTS.setControl(
                  new LarsonAnimation(SIDE_START, SIDE_END)
                      .withSlot(0)
                      .withColor(C)
                      .withBounceMode(LarsonBounceValue.Center)
                      .withFrameRate(CLIMB_CLIMB_SPEED));
              LIGHTS.setControl(
                  new LarsonAnimation(UPRIGHTS_START, UPRIGHTS_END)
                      .withSlot(1)
                      .withColor(C)
                      .withBounceMode(LarsonBounceValue.Center)
                      .withFrameRate(UP_SPEED));
            }
          })),
  ;

  public final FunctionalAction ACTION;

  FunctionalState(FunctionalAction ACTION) {
    this.ACTION = ACTION;
  }
}
