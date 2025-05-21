// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.subsystems.constants.SubsystemConstants.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class Intake extends BroncSystem {

  public enum MotorStatus {
    isRunning,
    isStopped,
    isReversing
  }

  private static final VoltageOut VOLTAGE_OUT_START = new VoltageOut(MOTOR_RUN_VOLTS);
  private static final VoltageOut VOLTAGE_OUT_STOP = new VoltageOut(0.0);
  private static final VoltageOut VOLTAGE_OUT_REVERSE = new VoltageOut(-MOTOR_RUN_VOLTS);

  public final TalonFX INTAKE_MOTOR = new TalonFX(MOTOR_ID, CAN_BUS);

  private final StatusSignal<Current> INTAKE_SUPPLY_CURRENT = INTAKE_MOTOR.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> INTAKE_VELOCITY = INTAKE_MOTOR.getVelocity();

  private MotorStatus motorStatus = MotorStatus.isStopped;

  public Intake() {
    INTAKE_MOTOR.getConfigurator().apply(intakeConfig());

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, INTAKE_SUPPLY_CURRENT, INTAKE_VELOCITY);
  }

  @Override
  public void preCycle() {
    log();
  }

  @Override
  public void log() {
    StatusCode motorStatus = BaseStatusSignal.refreshAll(INTAKE_SUPPLY_CURRENT, INTAKE_VELOCITY);
    if (RobotContainer.DEBUG) {
      DogLog.log("Intake/supplyCurrent", INTAKE_SUPPLY_CURRENT.getValueAsDouble());
      DogLog.log("Intake/velocity", INTAKE_VELOCITY.getValueAsDouble());
      DogLog.log("Intake/isConnected", motorStatus.isOK());
    }
  }

  public void start() {
    if (motorStatus.equals(MotorStatus.isRunning)) {
      return;
    }
    INTAKE_MOTOR.setControl(VOLTAGE_OUT_START);
    motorStatus = MotorStatus.isRunning;
  }

  public void reverse() {
    if (motorStatus.equals(MotorStatus.isReversing)) {
      return;
    }
    INTAKE_MOTOR.setControl(VOLTAGE_OUT_REVERSE);
    motorStatus = MotorStatus.isReversing;
  }

  public void stop() {
    if (motorStatus.equals(MotorStatus.isStopped)) {
      return;
    }
    INTAKE_MOTOR.setControl(VOLTAGE_OUT_STOP);
    motorStatus = MotorStatus.isStopped;
  }
}
