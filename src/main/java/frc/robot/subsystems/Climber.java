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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.utils.InstCmd;

public class Climber {

    public final TalonFX LEADER = new TalonFX(LEADER_MOTOR_ID, CANBUS_NAME);

    public final TalonFX FOLLOWER = new TalonFX(FOLLOWER_MOTOR_ID, CANBUS_NAME);

    public final CANcoder ENCODER = new CANcoder(ENCODER_ID);

    private final StatusSignal<Angle> ENCODER_POSITION = ENCODER.getPosition();
    private final StatusSignal<Angle> LEADER_POSITION = LEADER.getPosition();
    private final StatusSignal<Angle> FOLLOWER_POSITION = FOLLOWER.getPosition();
    private final StatusSignal<Current> LEADER_SUPPLY_CURRENT = LEADER.getSupplyCurrent();
    private final StatusSignal<Current> FOLLOWER_SUPPLY_CURRENT = FOLLOWER.getSupplyCurrent();

    private Angle target = FULLY_STOWED;
    private Angle current;

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

        SmartDashboard.putData("stow", new InstCmd(this::stow));
        SmartDashboard.putData("deploy", new InstCmd(this::deploy));
        SmartDashboard.putData("climb", new InstCmd(this::climb));
    }

    public void cycle() {
        if (RobotContainer.DEBUG) log();

        double voltage = Math.abs(getPosition().minus(target).in(Rotations)) * 100;
        if (getPosition().lt(FULLY_STOWED)) {

            System.out.println("nope");
            return;
        }
        if (getPosition().gt(FULLY_EXTENDED)) {
            System.out.println("nope");
            return;
        }
        if (getPosition().gt(target)) {
            LEADER.setControl(new VoltageOut(voltage));
            System.out.println("GOING TO" + voltage);
        } else {
            LEADER.setControl(new VoltageOut(-voltage));
            System.out.println("GOING TO -" + voltage);
        }
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

    public boolean isAtTarget() {
        return getPosition().isNear(target, Degrees.of(2));
    }

    public void log() {
        StatusCode leaderStatus = BaseStatusSignal.refreshAll(LEADER_POSITION, LEADER_SUPPLY_CURRENT);

        StatusCode followerStatus =
                BaseStatusSignal.refreshAll(FOLLOWER_POSITION, FOLLOWER_SUPPLY_CURRENT);

        StatusCode encoderStatus = BaseStatusSignal.refreshAll(ENCODER_POSITION);
    }

    public void setPosition(Angle pos) {
        target = pos;
    }

    public Angle getPosition() {
        return ENCODER_POSITION.getValue();
    }

    public void setVoltage(Voltage volts) {
        LEADER.setControl(new VoltageOut(volts));
    }
}
