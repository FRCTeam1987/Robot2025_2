package frc.robot.subsystems.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.*;
import frc.robot.state.logic.constants.MechanismConstant;
import java.util.List;

public class SubsystemConstants {
  public static class ElevatorConstants {

    // CANBus constants
    public static final CANBus CAN_BUS = new CANBus("canfd");
    public static final int LEADER_CAN_ID = 18;
    public static final int FOLLOWER_CAN_ID = 19;

    // Ratios & calculations
    public static final double ELEVATOR_REDUCTION = (54.0 / 8.0);
    public static final double PULLEY_DIAMETER_INCHES = 1.88;
    public static final Distance PULLEY_RADIUS = Inches.of(PULLEY_DIAMETER_INCHES / 2.0);

    // Mechanism properties
    public static final boolean FOLLOWER_OPPOSE_LEADER = false;
    public static final Distance MINIMUM_HEIGHT = Inches.of(0.0);
    public static final Distance MAXIMUM_HEIGHT = Inches.of(50.5);

    // Simulation properties
    public static final Mass MOVING_MASS = Pounds.of(10.0);

    // Configs
    public static TalonFXConfiguration elevatorConfig() {
      final TalonFXConfiguration CFG = new TalonFXConfiguration();

      // MotorOutput
      CFG.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

      // Slot0
      CFG.Slot0.withKP(20.0);
      CFG.Slot0.withKI(0.0);
      CFG.Slot0.withKD(0.1);
      CFG.Slot0.withKS(0.0);
      CFG.Slot0.withKV(0.15);
      CFG.Slot0.withKA(0.0);
      CFG.Slot0.withKG(0.2);
      CFG.Slot0.withGravityType(GravityTypeValue.Elevator_Static);

      // Feedback
      CFG.Feedback.withSensorToMechanismRatio(ELEVATOR_REDUCTION);

      // CurrentLimits
      CFG.CurrentLimits.withSupplyCurrentLimit(Amps.of(15.0));
      CFG.CurrentLimits.withSupplyCurrentLimitEnable(true);

      return CFG;
    }
  }

  public static class ArmConstants {
    // CANBus constants
    public static final int CORAL_CANDI_ID = 1;
    public static final int ALGAE_CANDI_ID = 2;
    public static final int ENCODER_ID = 5;
    public static final int LEADER_MOTOR_ID = 12;
    public static final int FOLLOWER_MOTOR_ID = 11;
    public static final String CANBUS_NAME = "canfd";

    // Ratios and calculations
    public static final double GEAR_RATIO = 30;
    public static final double ARM_RATIO = (60.0 / 12.0) * (60.0 / 18.0) * (60.0 / 18.0);
    public static final double EFFECTOR_RATIO = (48.0 / 15.0);
    public static final Distance SLOW_HEIGHT =
        MechanismConstant.L4
            .getElevatorDistance()
            .minus(MechanismConstant.L3.getElevatorDistance());

    // Sensor configurations
    public static final double CORAL_PIECE_DEBOUNCE_SECONDS = 0.08;
    public static final DebounceType CORAL_DEBOUNCE_TYPE = DebounceType.kBoth;
    public static final double ALGAE_PIECE_DEBOUNCE_SECONDS = 0.9;
    public static final DebounceType ALGAE_DEBOUNCE_TYPE = DebounceType.kBoth;

    // Mechanism properties
    public static final Angle ARM_MAGNET_OFFSET = Rotations.of(-0.463379);

    // Dynamic configs
    public static final AngularAcceleration FAST_ACCEL = RotationsPerSecondPerSecond.of(10.0);
    public static final AngularVelocity FAST_CRUISE = RotationsPerSecond.of(5.0);
    public static final AngularAcceleration SLOW_ACCEL = RotationsPerSecondPerSecond.of(5.0);
    public static final AngularVelocity SLOW_CRUISE = RotationsPerSecond.of(2.5);
    public static final Velocity<AngularAccelerationUnit> CARNAGE =
        RotationsPerSecondPerSecond.per(Second).of(0.0);

    // Configs
    public static CANcoderConfiguration encoderConfig() {
      final CANcoderConfiguration CFG = new CANcoderConfiguration();
      // MagnetSensor
      CFG.MagnetSensor.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
      CFG.MagnetSensor.withMagnetOffset(ARM_MAGNET_OFFSET);
      CFG.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1.0));

      return CFG;
    }

    public static TalonFXConfiguration armConfig() {
      final TalonFXConfiguration CFG = new TalonFXConfiguration();

      // MotorOutput
      CFG.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

      // Slot0
      CFG.Slot0.withKP(120);
      CFG.Slot0.withKI(0.0);
      CFG.Slot0.withKD(0.1);

      // ClosedLoop
      CFG.ClosedLoopGeneral.withContinuousWrap(false);

      // SoftwareLimitSwitch
      CFG.SoftwareLimitSwitch.withForwardSoftLimitThreshold(Rotations.of(1.0));
      CFG.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
      CFG.SoftwareLimitSwitch.withReverseSoftLimitThreshold(Rotations.of(-1.0));
      CFG.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);

      // Feedback
      CFG.Feedback.withFeedbackRemoteSensorID(ENCODER_ID);
      CFG.Feedback.withSensorToMechanismRatio(1.0);
      CFG.Feedback.withRotorToSensorRatio(ARM_RATIO);
      CFG.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

      // CurrentLimits
      CFG.CurrentLimits.withStatorCurrentLimit(Amps.of(30.0));
      CFG.CurrentLimits.withStatorCurrentLimitEnable(true);

      return CFG;
    }

    public static TalonFXConfiguration effectorConfig() {
      TalonFXConfiguration CFG = new TalonFXConfiguration();

      // MotorOutput
      CFG.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

      // Slot0
      CFG.Slot0.withKP(40);
      CFG.Slot0.withKI(0.0);
      CFG.Slot0.withKD(0.1);
      CFG.Slot0.withKV(0.08);
      CFG.Slot0.withKS(2.0);

      // Slot1
      CFG.Slot1.withKP(0.003);
      CFG.Slot1.withKI(0.0);
      CFG.Slot1.withKD(0.0);

      // CurrentLimits
      CFG.CurrentLimits.withSupplyCurrentLimit(Amps.of(20.0));
      CFG.CurrentLimits.withSupplyCurrentLowerTime(Seconds.of(4.0));
      CFG.CurrentLimits.withSupplyCurrentLimitEnable(false);

      // Feedback
      CFG.Feedback.withSensorToMechanismRatio(EFFECTOR_RATIO);

      return CFG;
    }
  }

  public static class IntakeConstants {
    public static final int MOTOR_ID = 20;
    public static final String CAN_BUS = "rio";
    public static final double MOTOR_CONNECTED_DEBOUNCE_SECONDS = 0.5;
    public static final double MOTOR_RUN_VOLTS = -6.5;

    public static TalonFXConfiguration intakeConfig() {
      TalonFXConfiguration CFG = new TalonFXConfiguration();
      CFG.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
      CFG.CurrentLimits.withSupplyCurrentLimit(Amps.of(20.0));
      CFG.CurrentLimits.withSupplyCurrentLowerTime(Seconds.of(2.0));
      CFG.CurrentLimits.withSupplyCurrentLimitEnable(true);
      return CFG;
    }
  }

  public static class ClimberConstants {
    public static final int LEADER_MOTOR_ID = 9;
    public static final String CANBUS_NAME = "canfd";
    public static final int ENCODER_ID = 6;

    public static final Angle FULLY_STOWED = Degrees.of(90.0);
    public static final Angle FULLY_EXTENDED = Degrees.of(180.0);
    public static final Angle FULLY_CLIMBED = Degrees.of(97.0);

    // public static final double CLIMBER_REDUCTION = (54.0 / 8.0);

    public static CANcoderConfiguration encoderConfig() {
      final CANcoderConfiguration CFG = new CANcoderConfiguration();
      // MagnetSensor
      CFG.MagnetSensor.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
      CFG.MagnetSensor.withMagnetOffset(Rotations.of(-0.347168).plus(Degrees.of(90.0)));
      CFG.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1.0));

      return CFG;
    }

    public static TalonFXConfiguration climberConfig() {
      final TalonFXConfiguration CFG = new TalonFXConfiguration();

      // MotorOutput
      CFG.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

      // Slot0
      CFG.Slot0.withKP(400);
      CFG.Slot0.withKI(0.0);
      CFG.Slot0.withKD(0.1);

      CFG.MotionMagic.withMotionMagicAcceleration(200);
      CFG.MotionMagic.withMotionMagicCruiseVelocity(60);

      // Feedback
      CFG.Feedback.withFeedbackRemoteSensorID(ENCODER_ID);
      CFG.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);
      // CFG.Feedback.withSensorToMechanismRatio(CLIMBER_REDUCTION);

      // CurrentLimits
      CFG.CurrentLimits.withSupplyCurrentLimit(Amps.of(15.0));
      CFG.CurrentLimits.withSupplyCurrentLimitEnable(true);

      return CFG;
    }
  }

  public static class VisionConstants {
    public static final String LIMELIGHT_SCORING_NAME = "limelight-scoring";
    // public static final String LIMELIGHT_INTAKE_NAME = "limelight-fr";

    public static final List<String> LIMELIGHTS = List.of(LIMELIGHT_SCORING_NAME);
  }
}
