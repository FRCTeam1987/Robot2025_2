// Copyright FRC 5712
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.constants.SubsystemConstants.ArmConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotContainer;
import frc.robot.state.Abomination;
import frc.robot.state.logic.functional.FunctionalState;
import frc.robot.state.logic.mode.ScoreMode;
import frc.robot.utils.Utils;

public class Arm extends BroncSystem {

  public final TalonFX ARM_MOTOR = new TalonFX(LEADER_MOTOR_ID, CANBUS_NAME);

  public final TalonFX EFFECTOR_MOTOR = new TalonFX(FOLLOWER_MOTOR_ID, CANBUS_NAME);

  public final CANcoder ENCODER = new CANcoder(ENCODER_ID, CANBUS_NAME);

  public final CANdi CORAL_CANDI = new CANdi(CORAL_CANDI_ID, CANBUS_NAME);

  public final CANdi ALGAE_CANDI = new CANdi(ALGAE_CANDI_ID, CANBUS_NAME);

  private final StatusSignal<Angle> ARM_POSITION = ARM_MOTOR.getPosition();
  private final StatusSignal<AngularVelocity> ARM_VELOCITY = ARM_MOTOR.getVelocity();
  private final StatusSignal<Angle> EFFECTOR_POSITION = EFFECTOR_MOTOR.getPosition();
  private final StatusSignal<Current> ARM_SUPPLY_CURRENT = ARM_MOTOR.getSupplyCurrent();
  private final StatusSignal<Current> EFFECTOR_SUPPLY_CURRENT = EFFECTOR_MOTOR.getSupplyCurrent();
  private final StatusSignal<Angle> ENCODER_POSITION = ENCODER.getPosition();
  private final StatusSignal<S1StateValue> CORAL_S1_SIGNAL = CORAL_CANDI.getS1State();
  private final StatusSignal<S2StateValue> CORAL_S2_SIGNAL = CORAL_CANDI.getS2State();
  private final StatusSignal<S1StateValue> ALGAE_S1_SIGNAL = ALGAE_CANDI.getS1State();

  private final Debouncer isAtTargetDebouncer = new Debouncer(0.06);
  private final Debouncer isNearTargetDebouncer = new Debouncer(0.06);
  public final Debouncer ENTRANCE_DEBOUNCER =
      new Debouncer(CORAL_PIECE_DEBOUNCE_SECONDS, CORAL_DEBOUNCE_TYPE);
  public final Debouncer BACK_DEBOUNCER =
      new Debouncer(CORAL_PIECE_DEBOUNCE_SECONDS, CORAL_DEBOUNCE_TYPE);
  public final Debouncer ALGAE_DEBOUNCE =
      new Debouncer(ALGAE_PIECE_DEBOUNCE_SECONDS, ALGAE_DEBOUNCE_TYPE);

  private final Angle AT_TARGET_TOLERANCE = Degrees.of(1);
  private final Angle NEAR_TARGET_TOLERANCE = Degrees.of(8.0);

  public Angle target = Degrees.of(0.0);
  private boolean isFollowing = true;
  private boolean isAtTarget = false;
  private boolean isNearTarget = false;
  private boolean isCoast;

  private final DynamicMotionMagicVoltage SLOW_MOTION =
      new DynamicMotionMagicVoltage(target, SLOW_CRUISE, SLOW_ACCEL, CARNAGE);
  private final DynamicMotionMagicVoltage FAST_MOTION =
      new DynamicMotionMagicVoltage(target, FAST_CRUISE, FAST_ACCEL, CARNAGE);

  public Arm() {
    ENCODER.getConfigurator().apply(encoderConfig());
    ARM_MOTOR.getConfigurator().apply(armConfig());
    EFFECTOR_MOTOR.getConfigurator().apply(effectorConfig());

    CORAL_CANDI
        .getConfigurator()
        .apply(
            new DigitalInputsConfigs()
                .withS1FloatState(S1FloatStateValue.PullHigh)
                .withS2FloatState(S2FloatStateValue.PullHigh));
    ALGAE_CANDI
        .getConfigurator()
        .apply(
            new DigitalInputsConfigs()
                .withS1FloatState(S1FloatStateValue.PullHigh)
                .withS2FloatState(S2FloatStateValue.PullHigh));

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        ARM_POSITION,
        ARM_VELOCITY,
        ARM_SUPPLY_CURRENT,
        EFFECTOR_POSITION,
        EFFECTOR_SUPPLY_CURRENT,
        ENCODER_POSITION,
        CORAL_S1_SIGNAL,
        CORAL_S2_SIGNAL,
        ALGAE_S1_SIGNAL);

    isCoast = armConfig().MotorOutput.NeutralMode.equals(NeutralModeValue.Coast);
  }

  @Override
  public void log() {
    // Update and log inputs from hardware
    DogLog.log("Arm/armSupplyCurrent", ARM_SUPPLY_CURRENT.getValueAsDouble());
    //    DogLog.log("Arm/armPosition", ARM_POSITION.getValueAsDouble());
    //    DogLog.log("Arm/armVelocity", ARM_VELOCITY.getValueAsDouble());
    DogLog.log("Arm/effectorSupplyCurrent", EFFECTOR_SUPPLY_CURRENT.getValueAsDouble());
    //    DogLog.log("Arm/effectorPosition", EFFECTOR_POSITION.getValueAsDouble());
    DogLog.log("Arm/encoderPosition", ENCODER_POSITION.getValueAsDouble());
    DogLog.log("Arm/isAtTarget", isAtTarget);
    DogLog.log("Arm/coralS1", CORAL_S1_SIGNAL.getValue());
    DogLog.log("Arm/coralS2", CORAL_S2_SIGNAL.getValue());
    DogLog.log("Arm/algaeS1", ALGAE_S1_SIGNAL.getValue());
    //    DogLog.log("Arm/armIsConnected", leaderStatus.isOK());
    //    DogLog.log("Arm/effectorIsConnected", followerStatus.isOK());
    //    DogLog.log("Arm/encoderIsConnected", encoderStatus.isOK());
    //    DogLog.log("Arm/coralIsConnected", coralStatus.isOK());
    //    DogLog.log("Arm/algaeIsConnected", algaeStatus.isOK());
  }

  @Override
  public void preCycle() {
    StatusCode allStatus =
        BaseStatusSignal.refreshAll(
            ARM_POSITION,
            ARM_VELOCITY,
            ARM_SUPPLY_CURRENT,
            EFFECTOR_POSITION,
            EFFECTOR_SUPPLY_CURRENT,
            ENCODER_POSITION,
            CORAL_S1_SIGNAL,
            CORAL_S2_SIGNAL,
            ALGAE_S1_SIGNAL);

    Angle ANGLE = getArmPosition();

    isNearTarget = isNearTargetDebouncer.calculate(ANGLE.isNear(target, NEAR_TARGET_TOLERANCE));
    if (!isNearTarget) {
      isAtTarget = isAtTargetDebouncer.calculate(false);
    } else {
      isAtTarget = isAtTargetDebouncer.calculate(ANGLE.isNear(target, AT_TARGET_TOLERANCE));
    }

    log();
  }

  public void setClawVoltage(Voltage voltage) {
    if (voltage.magnitude() == 0.0) {
      if (isFollowing) return;
      this.isFollowing = true;
      resetEffectorPosition(getArmPosition());
      setEffectorPosition(getArmPosition());
    } else {
      isFollowing = false;
      setEffectorVoltage(voltage);
    }
  }

  public void setArmPosition(Angle angle) {
    if (Abomination.getScoreMode() == ScoreMode.L4
        && Abomination.stateContains(
            FunctionalState.LEVEL_X_ELEVATE,
            FunctionalState.LEVEL_X_ROTATE,
            FunctionalState.LEVEL_X_SCORE)) {
      target = angle.plus(Degrees.of(Utils.getArmOverride()));
    } else {
      target = angle;
    }

    if (RobotContainer.ELEVATOR.getPosition().gt(SLOW_HEIGHT)) {
      ARM_MOTOR.setControl(SLOW_MOTION.withPosition(target));
    } else {
      ARM_MOTOR.setControl(FAST_MOTION.withPosition(target));
    }
  }

  public Angle getArmPosition() {
    return ARM_POSITION.getValue();
  }

  public Angle getTarget() {
    return target;
  }

  public boolean isAtTarget() {
    return isAtTarget;
  }

  public boolean isNearTarget() {
    return isNearTarget;
  }

  public boolean hasAlgae() {
    return ALGAE_DEBOUNCE.calculate(
        ALGAE_S1_SIGNAL.getValue().equals(S1StateValue.High) && !hasGamePiece());
  }

  public boolean hasGamePieceBack() {
    return BACK_DEBOUNCER.calculate(CORAL_S1_SIGNAL.getValue().equals(S1StateValue.High));
  }

  public boolean hasGamePieceEntrance() {
    return ENTRANCE_DEBOUNCER.calculate(CORAL_S2_SIGNAL.getValue().equals(S2StateValue.High));
  }

  public boolean hasGamePiece() {
    return hasGamePieceBack() && hasGamePieceEntrance();
  }

  private void setEffectorVoltage(Voltage voltage) {
    EFFECTOR_MOTOR.setVoltage(voltage.in(Volts));
  }

  private void setEffectorPosition(Angle angle) {
    if (!EFFECTOR_POSITION.getValue().isNear(angle, Angle.ofBaseUnits(0.25, Degrees))) {
      EFFECTOR_MOTOR.setControl(new PositionVoltage(angle));
    }
  }

  private void resetEffectorPosition(Angle angle) {
    EFFECTOR_MOTOR.setPosition(angle);
  }

  public void dynamicHold() {
    setClawVoltage(Volts.of(3.0));
  }

  public void coast() {
    if (isCoast) {
      return;
    }
    ARM_MOTOR
        .getConfigurator()
        .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
    isCoast = true;
  }

  public void brake() {
    if (!isCoast) {
      return;
    }
    ARM_MOTOR
        .getConfigurator()
        .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    isCoast = false;
  }
}
