// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  // Constants that should be moved elsewhere after dumping.
  private static final int MOTOR_ID = 20;
  private static final CANBus CAN_BUS = new CANBus();
  private static final double CONNECT_DEBOUNCER_SECONDS = 0.5;
  private static final VoltageOut MOTOR_RUN_VOLTAGE_OUT = new VoltageOut(Volts.of(6.0));
  private static final VoltageOut MOTOR_STOP_VOLTAGE_OUT = new VoltageOut(Volts.of(0.0));

  private static TalonFXConfiguration createConfig() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    cfg.CurrentLimits.withSupplyCurrentLimit(Amps.of(20.0));
    cfg.CurrentLimits.withSupplyCurrentLowerTime(Seconds.of(2.0));
    cfg.CurrentLimits.withSupplyCurrentLimitEnable(true);
    return cfg;
  }

  // Devices
  private final TalonFX m_motor;

  // Signals
  private final StatusSignal<Voltage> m_signalMotorAppliedVoltage;
  private final StatusSignal<Current> m_signalMotorStatorCurrent;
  private final StatusSignal<Current> m_signalMotorSupplyCurrent;
  private final StatusSignal<AngularVelocity> m_signalMotorVelocity;

  // Logic
  private final Debouncer m_motorConnectedDebouncer;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_motor = new TalonFX(MOTOR_ID, CAN_BUS);
    m_motor.getConfigurator().apply(createConfig());
    m_signalMotorAppliedVoltage = m_motor.getMotorVoltage();
    m_signalMotorStatorCurrent = m_motor.getStatorCurrent();
    m_signalMotorSupplyCurrent = m_motor.getSupplyCurrent();
    m_signalMotorVelocity = m_motor.getRotorVelocity();
    m_motorConnectedDebouncer = new Debouncer(CONNECT_DEBOUNCER_SECONDS);
    configureStatusSignals();
  }

  /** Configures the update frequency for all status signals. */
  private void configureStatusSignals() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_signalMotorAppliedVoltage,
        m_signalMotorStatorCurrent,
        m_signalMotorSupplyCurrent,
        m_signalMotorVelocity);
    m_motor.optimizeBusUtilization();
  }

  public void start() {
    m_motor.setControl(MOTOR_RUN_VOLTAGE_OUT);
  }

  public void stop() {
    m_motor.setControl(MOTOR_STOP_VOLTAGE_OUT);
  }

  public void log() {
    StatusCode motorStatus =
        BaseStatusSignal.refreshAll(
            m_signalMotorAppliedVoltage,
            m_signalMotorStatorCurrent,
            m_signalMotorSupplyCurrent,
            m_signalMotorVelocity);
    // These signals are logged by default,
    // but this lets us view them on NT with less effor than CTRE's Telemetry until we have more
    // time
    DogLog.log("intake/appliedVoltage", m_signalMotorAppliedVoltage.getValueAsDouble());
    DogLog.log("intake/statorCurrent", m_signalMotorStatorCurrent.getValueAsDouble());
    DogLog.log("intake/supplyCurrent", m_signalMotorSupplyCurrent.getValueAsDouble());
    DogLog.log("intake/velocity", m_signalMotorVelocity.getValueAsDouble());
    DogLog.log("intake/isConnected", m_motorConnectedDebouncer.calculate(motorStatus.isOK()));
  }

  @Override
  public void periodic() {
    log();
  }
}
