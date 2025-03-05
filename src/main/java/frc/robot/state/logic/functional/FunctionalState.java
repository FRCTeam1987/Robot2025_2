package frc.robot.state.logic.functional;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotContainer.*;
import static frc.robot.state.Abomination.getCollectMode;
import static frc.robot.state.Abomination.getScoreMode;
import static frc.robot.state.logic.constants.StateConstants.COLLECT_ZONES;

import frc.robot.state.logic.constants.MechanismConstant;

public enum FunctionalState {
  COLLECT(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getCollectMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(getCollectMode().getMechanismConstant().getArmAngle()),
          () ->
              ARM.setClawVoltage(
                  COLLECT_ZONES.contains(getLocalizationState().fieldZone())
                      ? Volts.of(4.0)
                      : Volts.of(0.0)),
              COLLECT_ZONES.contains(getLocalizationState().fieldZone()) ? INTAKE::start : INTAKE::stop)),
  COLLECTED_CORAL(
      new FunctionalAction(
          // stroke behaivor
          // ELEVATOR.setDistance(
          //                  Meters.of(
          //                      Math.max(
          //                          0,
          //                          Math.max(
          //                                  getScoreMode()
          //                                      .getMechanismConstant()
          //                                      .getElevatorDistance()
          //                                      .in(Meters),
          //                                  MechanismConstant.L3.getElevatorDistance().in(Meters))
          //                              * (1
          //                                  - (RobotContainer.getLocalizationState()
          //                                          .distanceToGoal()
          //                                          .in(Meters)
          //                                      / 4))))
          () -> ELEVATOR.setDistance(MechanismConstant.IDLE_CORAL.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.IDLE_CORAL.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.45)),
          INTAKE::stop)),
  COLLECTED_ALGAE(
      new FunctionalAction(
          () ->
              ELEVATOR.setDistance(getScoreMode().getIdleMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(getScoreMode().getIdleMechanismConstant().getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.75)),
          INTAKE::stop)),
  PROCESSOR_ELEVATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.A2.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(1.0)),
          INTAKE::stop)),
  PROCESSOR_ROTATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(getScoreMode().getMechanismConstant().getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(1.0)),
          INTAKE::stop)),
  PROCESSOR_SCORE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(getScoreMode().getMechanismConstant().getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(-6.0)),
          INTAKE::stop)),
  NET_ELEVATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.IDLE_ALGAE_NET.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(1.0)),
          INTAKE::stop)),
  NET_ROTATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(getScoreMode().getMechanismConstant().getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(1.0)),
          INTAKE::stop)),
  NET_SCORE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(getScoreMode().getMechanismConstant().getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(-16.0)),
          INTAKE::stop)),
  NET_UNROTATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.HP_INTAKE.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(1.0)),
          INTAKE::stop)),
  NET_UNELEVATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(MechanismConstant.IDLE_CORAL.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.HP_INTAKE.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(1.0)),
          INTAKE::stop)),
  LEVEL_X_ELEVATE(
      new FunctionalAction(
          () -> {
            ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance());
          },
          () -> ARM.setArmPosition(MechanismConstant.IDLE_CORAL.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.0)),
          INTAKE::stop)),
  LEVEL_X_ROTATE(
      new FunctionalAction(
          () -> {
            ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance());
          },
          () -> ARM.setArmPosition(getScoreMode().getMechanismConstant().getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.0)),
          INTAKE::stop)),
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
                      case L1 -> inBack ? -8.0 : -16.0;
                      case L4 -> inBack ? -8.0 : -2.5;
                      default -> inBack ? -8.0 : -1.4;
                    }));
          },
          INTAKE::stop)),
  LEVEL_X_UNROTATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(getScoreMode().getMechanismConstant().getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.IDLE_CORAL.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(-0.3)),
          INTAKE::stop)),
  LEVEL_X_UNELEVATE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(MechanismConstant.IDLE_CORAL.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.IDLE_CORAL.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.0)),
          INTAKE::stop)),
  DEFENSE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(MechanismConstant.DEFENSE.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.DEFENSE.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.0)),
          INTAKE::stop)),
  CLIMB_DEPLOY(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(MechanismConstant.CLIMB.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.CLIMB.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.0)),
          INTAKE::stop,
          () -> CLIMBER.setVoltage(Volt.of(4.0)))),
  CLIMB_RETRACT(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(MechanismConstant.CLIMB.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.CLIMB.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.0)),
          INTAKE::stop,
          () -> CLIMBER.setVoltage(Volt.of(4.0)))),
  CLIMB_LATCH(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(MechanismConstant.CLIMB.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.CLIMB.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.0)),
          INTAKE::stop,
          () -> CLIMBER.setVoltage(Volt.of(4.0)))),
  CLIMB_RAISE(
      new FunctionalAction(
          () -> ELEVATOR.setDistance(MechanismConstant.CLIMB.getElevatorDistance()),
          () -> ARM.setArmPosition(MechanismConstant.CLIMB.getArmAngle()),
          () -> ARM.setClawVoltage(Volts.of(0.0)),
          INTAKE::stop,
          () -> CLIMBER.setVoltage(Volt.of(4.0)))),
  ;

  public final FunctionalAction ACTION;

  FunctionalState(FunctionalAction ACTION) {
    this.ACTION = ACTION;
  }
}
