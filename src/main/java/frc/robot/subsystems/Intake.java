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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

/** Add your docs here. */
public class Intake {

  public final TalonFX INTAKE_MOTOR = new TalonFX(MOTOR_ID, CAN_BUS);

  private final StatusSignal<Current> INTAKE_SUPPLY_CURRENT = INTAKE_MOTOR.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> INTAKE_VELOCITY = INTAKE_MOTOR.getVelocity();

  public Intake() {
    INTAKE_MOTOR.getConfigurator().apply(intakeConfig());

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, INTAKE_SUPPLY_CURRENT, INTAKE_VELOCITY);

    INTAKE_MOTOR.optimizeBusUtilization();
  }

  public void cycle() {}

  public void log() {
    StatusCode motorStatus = BaseStatusSignal.refreshAll(INTAKE_SUPPLY_CURRENT, INTAKE_VELOCITY);
  }

  public void start() {
    INTAKE_MOTOR.setControl(new VoltageOut(MOTOR_RUN_VOLTS));
  }

  public void stop() {
    INTAKE_MOTOR.setControl(new VoltageOut(0.0));
  }
}
