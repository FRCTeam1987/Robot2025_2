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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.constants.SubsystemConstants.ClimberConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotContainer;

public class Climber {

  public final TalonFX LEADER = new TalonFX(LEADER_MOTOR_ID, CANBUS_NAME);

  public final CANcoder ENCODER = new CANcoder(ENCODER_ID, CANBUS_NAME);

  private final StatusSignal<Angle> ENCODER_POSITION = ENCODER.getPosition();
  private final StatusSignal<Angle> LEADER_POSITION = LEADER.getPosition();
  private final StatusSignal<Current> LEADER_SUPPLY_CURRENT = LEADER.getSupplyCurrent();
  private Angle target = FULLY_STOWED;
  private Angle current;

  public Climber() {
    TalonFXConfiguration config = climberConfig();
    LEADER.getConfigurator().apply(config);
    ENCODER.getConfigurator().apply(encoderConfig());
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, ENCODER_POSITION, LEADER_POSITION, LEADER_SUPPLY_CURRENT);

    LEADER.optimizeBusUtilization();
    LEADER.setPosition(0.0);
  }

  public void cycle() {
    if (RobotContainer.DEBUG) log();

    //        double voltage = Math.abs(getPosition().minus(target).in(Rotations)) * 100;
    //        if (getPosition().lt(FULLY_STOWED)) {
    //
    //            System.out.println("nope");
    //            return;
    //        }
    //        if (getPosition().gt(FULLY_EXTENDED)) {
    //            System.out.println("nope");
    //            return;
    //        }
    //        if (getPosition().gt(target)) {
    //            LEADER.setControl(new VoltageOut(voltage));
    //            System.out.println("GOING TO" + voltage);
    //        } else {
    //            LEADER.setControl(new VoltageOut(-voltage));
    //            System.out.println("GOING TO -" + voltage);
    //        }
  }

  public void deploy() {
    setPosition(FULLY_EXTENDED);
  }

  public void climb() {
    setPosition(FULLY_CLIMBED);
  }

  public void stow() {
    setPosition(FULLY_STOWED);
  }

  public void stop() {
    LEADER.stopMotor();
  }

  public boolean isAtTarget() {
    return getPosition().isNear(target, Degrees.of(3.5));
  }

  public void log() {
    StatusCode leaderStatus = BaseStatusSignal.refreshAll(LEADER_POSITION, LEADER_SUPPLY_CURRENT);

    StatusCode encoderStatus = BaseStatusSignal.refreshAll(ENCODER_POSITION);

    DogLog.log("Climber/leaderPosition", LEADER_POSITION.getValueAsDouble());
    DogLog.log("Climber/encoderPosition", ENCODER_POSITION.getValueAsDouble());
    DogLog.log("Climber/leaderCurrent", LEADER_SUPPLY_CURRENT.getValueAsDouble());
    DogLog.log("Climber/isAtTarget", isAtTarget());
    DogLog.log("Climber/target", target.in(Rotations));
  }

  public void setPosition(Angle pos) {
    LEADER.setControl(new MotionMagicVoltage(pos));
    target = pos;
  }

  public Angle getPosition() {
    return ENCODER_POSITION.getValue();
  }

  public void setVoltage(Voltage volts) {
    LEADER.setControl(new VoltageOut(volts));
  }
}
