package frc.robot.state.logic.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.constants.SubsystemConstants.AlgaeConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public enum MechanismConstant {
  // CORAL SCORING LOCATIONS
  L1(Inches.of(0.0), Degrees.of(140), DEPLOYER_STOWED_ANGLE),
  L2(Inches.of(0.0), Degrees.of(296), DEPLOYER_STOWED_ANGLE),
  L3(Inches.of(17.25), Degrees.of(297), DEPLOYER_STOWED_ANGLE),
  L4(Inches.of(47.95), Degrees.of(313.75), DEPLOYER_STOWED_ANGLE),

  // ALGAE SCORING LOCATIONS
  NET(Inches.of(50.5), Degrees.of(-45), DEPLOYER_SEMIDEPLOYED_ANGLE),
  PROCESSOR(Inches.of(0.0), Degrees.of(77), DEPLOYER_SEMIDEPLOYED_ANGLE),

  // CORAL INTAKING LOCATIONS
  HP_INTAKE(Inches.of(0.0), Degrees.of(93), DEPLOYER_STOWED_ANGLE),
  IDLE_CORAL(Inches.of(0.0), Degrees.of(280), DEPLOYER_STOWED_ANGLE),

  // ALGAE INTAKING LOCATIONS
  A2(Inches.of(13), Degrees.of(54), DEPLOYER_SEMIDEPLOYED_ANGLE),
  A3(Inches.of(28), Degrees.of(54), DEPLOYER_SEMIDEPLOYED_ANGLE),
  AGROUND(Inches.of(0.0), Degrees.of(74), DEPLOYER_DEPLOYED_ANGLE),
  IDLE_ALGAE_PROCESSOR(Inches.of(3.0), Degrees.of(34), DEPLOYER_SEMIDEPLOYED_ANGLE),
  IDLE_ALGAE_NET(Inches.of(3.0), Degrees.of(-45), DEPLOYER_SEMIDEPLOYED_ANGLE),

  CLIMB(Inches.of(0.0), Degrees.of(100.0), DEPLOYER_STOWED_ANGLE),
  DEFENSE(Inches.of(0.0), Degrees.of(0), DEPLOYER_STOWED_ANGLE),
  ;

  public Distance getElevatorDistance() {
    return ELEVATOR_DISTANCE;
  }

  public Angle getArmAngle() {
    return ARM_ANGLE;
  }

  public Angle getDeployerAngle() {
    return DEPLOYER_ANGLE;
  }

  private final Distance ELEVATOR_DISTANCE;
  private final Angle ARM_ANGLE;
  private final Angle DEPLOYER_ANGLE;

  MechanismConstant(Distance ELEV, Angle ARM, Angle DEPLOYER_ANGLE) {
    this.ELEVATOR_DISTANCE = ELEV;
    this.ARM_ANGLE = ARM;
    this.DEPLOYER_ANGLE = DEPLOYER_ANGLE;
  }
}
