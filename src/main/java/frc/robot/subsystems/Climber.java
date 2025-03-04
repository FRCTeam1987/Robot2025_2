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

import static frc.robot.subsystems.constants.SubsystemConstants.ClimberConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class Climber {

  public final TalonFX LEADER = new TalonFX(LEADER_MOTOR_ID, CANBUS_NAME);

  public final TalonFX FOLLOWER = new TalonFX(FOLLOWER_MOTOR_ID, CANBUS_NAME);

  public final CANcoder ENCODER = new CANcoder(ENCODER_ID, CANBUS_NAME);

  private final StatusSignal<Angle> ENCODER_POSITION = ENCODER.getPosition();
  private final StatusSignal<Angle> LEADER_POSITION = LEADER.getPosition();
  private final StatusSignal<Angle> FOLLOWER_POSITION = FOLLOWER.getPosition();
  private final StatusSignal<Current> LEADER_SUPPLY_CURRENT = LEADER.getSupplyCurrent();
  private final StatusSignal<Current> FOLLOWER_SUPPLY_CURRENT = FOLLOWER.getSupplyCurrent();

  public Climber() {
    TalonFXConfiguration config = climberConfig();
    LEADER.getConfigurator().apply(config);
    FOLLOWER.getConfigurator().apply(config);
    ENCODER.getConfigurator().apply(encoderConfig());

    FOLLOWER.setControl(new Follower(LEADER.getDeviceID(), false));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        ENCODER_POSITION,
        LEADER_POSITION,
        FOLLOWER_POSITION,
        LEADER_SUPPLY_CURRENT,
        FOLLOWER_SUPPLY_CURRENT);

    LEADER.optimizeBusUtilization();
    FOLLOWER.optimizeBusUtilization();
    LEADER.setPosition(0.0);
    FOLLOWER.setPosition(0.0);
  }

  public void cycle() {}

  public void log() {
    StatusCode leaderStatus = BaseStatusSignal.refreshAll(LEADER_POSITION, LEADER_SUPPLY_CURRENT);

    StatusCode followerStatus =
        BaseStatusSignal.refreshAll(FOLLOWER_POSITION, FOLLOWER_SUPPLY_CURRENT);

    StatusCode encoderStatus = BaseStatusSignal.refreshAll(ENCODER_POSITION);
  }

  public void setVoltage(Voltage volts) {
    LEADER.setControl(new VoltageOut(volts));
  }
}
