// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.subsystems.constants.SubsystemConstants.AlgaeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class Algae {

  private static final VoltageOut VOLTAGE_OUT_START = new VoltageOut(MOTOR_RUN_VOLTS);
  private static final VoltageOut VOLTAGE_OUT_STOP = new VoltageOut(0.0);
  private static final VoltageOut VOLTAGE_OUT_REVERSE = new VoltageOut(-MOTOR_RUN_VOLTS);

  public final TalonFX DEPLOYER_MOTOR = new TalonFX(DEPLOYER_ID, CAN_BUS);
  public final TalonFX ROLLER_MOTOR = new TalonFX(ROLLER_ID, CAN_BUS);

  private final StatusSignal<Current> ROLLER_SUPPLY_CURRENT = ROLLER_MOTOR.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> ROLLER_VELOCITY = ROLLER_MOTOR.getVelocity();

  private final StatusSignal<Current> DEPLOYER_SUPPLY_CURRENT = DEPLOYER_MOTOR.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> DEPLOYER_VELOCITY = DEPLOYER_MOTOR.getVelocity();
  private final StatusSignal<Angle> DEPLOYER_POSITION = DEPLOYER_MOTOR.getPosition();

  public Algae() {
    DEPLOYER_MOTOR.getConfigurator().apply(deployerConfig());
    ROLLER_MOTOR.getConfigurator().apply(rollerConfig());

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        ROLLER_SUPPLY_CURRENT,
        ROLLER_VELOCITY,
        DEPLOYER_SUPPLY_CURRENT,
        DEPLOYER_VELOCITY,
        DEPLOYER_POSITION);
  }

  public void cycle() {
    log();
  }

  public void log() {
    StatusCode motorStatus =
        BaseStatusSignal.refreshAll(
            ROLLER_SUPPLY_CURRENT,
            ROLLER_VELOCITY,
            DEPLOYER_SUPPLY_CURRENT,
            DEPLOYER_VELOCITY,
            DEPLOYER_POSITION);
    if (RobotContainer.DEBUG) {
      DogLog.log("Algae/rollerCurrent", ROLLER_SUPPLY_CURRENT.getValueAsDouble());
      DogLog.log("Algae/rollerVelocity", ROLLER_VELOCITY.getValueAsDouble());
      DogLog.log("Algae/deployerCurrent", DEPLOYER_SUPPLY_CURRENT.getValueAsDouble());
      DogLog.log("Algae/deployerVelocity", DEPLOYER_VELOCITY.getValueAsDouble());
      DogLog.log("Algae/deployerPosition", DEPLOYER_POSITION.getValueAsDouble());
      DogLog.log("Algae/isConnected", motorStatus.isOK());
    }
  }

  public void run(Angle angle, boolean runRollers) {
    DEPLOYER_MOTOR.setControl(new PositionVoltage(angle));
    if (runRollers) {
      ROLLER_MOTOR.setControl(VOLTAGE_OUT_START);
    } else {
      ROLLER_MOTOR.setControl(VOLTAGE_OUT_STOP);
    }
  }
}
