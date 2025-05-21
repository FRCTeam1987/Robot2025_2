# Robot 2025 (2)

The official repository for Team 1987's second-generation robot code, for the 2025 season Reefscape.

Our first iteration of robot code was built with AdvantageKit, which ended up lagging our robot code out and having suboptimal performance. We were most likely using it wrong, but found that an alternative logging utility, DogLog does what we need with significantly less overhead.

## Why?
For the game Reefscape, we decided that a state based robot would be absolutely critical for effective manipulation and scoring of game pieces, especially on a scoring location with tens of different precise positions.

## How?
This year, we containerized our robot state, into a class named FunctionalState.java. This class contained enum constants with all the possible "states" of the robot, with runnables passed as parameters. We would then compile all of our sensor data from all our subsystems in a class called Abomination.java, to compute what state we need. The state that was returned would have its runnables executed, and those runnables contained instructions for the subsystems. Bam, state based robot.

### RobotState & Subsystems
The FunctionalState enum contains all possible robot states, such as "COLLECT, COLLECTED_CORAL, NET_ELEVATE, NET_ROTATE, NET_SCORE", etc.
The enum paramaters contained runnables with subsystem instructions. An example is below.
```java
public enum FunctionalState {
COLLECT(
      new FunctionalAction(
            //Set elevator to height
        () -> ELEVATOR.setDistance(getCollectMode().getMechanismConstant().getElevatorDistance()),
            //Set arm to position
        () -> ARM.setArmPosition(getCollectMode().getMechanismConstant().getArmAngle()),
            //Set claw speed based on where we're at on the field
        () ->
        ARM.setClawVoltage(
        COLLECT_ZONES.contains(getLocalizationState().fieldZone())
        ? Abomination.getCollectMode().equals(CollectMode.HUMAN_PLAYER_STATION)
                          ? ARM.hasGamePieceEntrance() ? Volts.of(9.0) : Volts.of(2.5)
                          : Volts.of(14.0)
                      : Volts.of(0.0))))
}
```

Last year, we assigned each of our subsystems a default command. This was messy and required us to edit the file for every subsystem when we made any changes to the robot state. This year, we used a SuperStructure subsystem to execute the runnables from FunctionalState in each subsystem.
```java
public class Structure extends SubsystemBase {

    public Structure() {}

    @Override
    public void periodic() {
        ARM.preCycle();
        ELEVATOR.preCycle();
        INTAKE.preCycle();
        CLIMBER.preCycle();
        LIGHTS.preCycle();

        FunctionalState STATE = Abomination.getState();

        STATE.ACTION.getArm().run();
        STATE.ACTION.getRoller().run();
        STATE.ACTION.getElev().run();
        STATE.ACTION.getIntake().run();
        STATE.ACTION.getClimb().run();
        STATE.ACTION.getLights().run();

        LIGHTS.postCycle();
    }
}
```

## Constants?
Our robot code contains many other enum classes, such as CollectMode, ScoreMode, FieldPosition, PositionConstant, MechanismConstant, DesiredAction and so on.
These enums contain constant data that we can pass around in our state calculator. For instance, ScoreMode contains values like "L2, L3, L4", and has parameters passed like MechanismConstant.L4. And then FieldPosition takes that ScoreMode and pairs it with a PositionConstant, so you can have a FieldPosition that not only has the physical field position for scoring on peg A, but also easy access to the elevator and arm positions of an L4 score.
An example is provided below of some of these files.
```java
public enum MechanismConstant {
    //Physical robot mechanism properties
    L1(Inches.of(0.0), Degrees.of(140)),
    L2(Inches.of(0.0), Degrees.of(296)),
    L3(Inches.of(17.25), Degrees.of(297)),
    L4(Inches.of(47.95), Degrees.of(313.75)),
}
public enum PositionConstant {
    BLUE_REEF(new Translation2d(4.495, 4.02), Degrees.of(0.0)),

    SIDE_1_ENTRY(new Translation2d(2.629 - 0.0274, 4.023), Degrees.of(0.0)),
    SIDE_1_A(new Translation2d(3.120 - 0.00, 4.192 + 0.005), SIDE_1_ENTRY.getAngle()),
    SIDE_1_ALGAE(new Translation2d(3.180 + 0.06, 4.023), SIDE_1_ENTRY.getAngle()),
    SIDE_1_B(new Translation2d(3.120 - 0.00, 3.862 + 0.005), SIDE_1_ENTRY.getAngle()),
    SIDE_1_A1(new Translation2d(3.2 - 0.00, 4.5), flip(SIDE_1_ENTRY.getAngle())),
    SIDE_1_B1(new Translation2d(3.2 - 0.00, 3.5), flip(SIDE_1_ENTRY.getAngle())),

    //Use math to compute the rest of the values by rotating
    SIDE_2_ENTRY(rotateAroundBlueReef(SIDE_1_ENTRY, Degrees.of(60.0)), Degrees.of(60.0)),
    SIDE_2_C(rotateAroundBlueReef(SIDE_1_A, Degrees.of(60.0)), SIDE_2_ENTRY.getAngle()),
    SIDE_2_ALGAE(rotateAroundBlueReef(SIDE_1_ALGAE, Degrees.of(60.0)), SIDE_2_ENTRY.getAngle()),
    SIDE_2_D(rotateAroundBlueReef(SIDE_1_B, Degrees.of(60.0)), SIDE_2_ENTRY.getAngle()),
    SIDE_2_C1(rotateAroundBlueReef(SIDE_1_A1, Degrees.of(60.0)), flip(SIDE_2_ENTRY.getAngle())),
    SIDE_2_D1(rotateAroundBlueReef(SIDE_1_B1, Degrees.of(60.0)), flip(SIDE_2_ENTRY.getAngle())),
}
public enum FieldPosition {
    A1(PositionConstant.SIDE_1_A1, ScoreMode.L1, PositionConstant.SIDE_1_ENTRY),
    //SIDE_1_A1 is different as we score L1's from a different position
    A2(PositionConstant.SIDE_1_A, ScoreMode.L2, PositionConstant.SIDE_1_ENTRY),
    A3(PositionConstant.SIDE_1_A, ScoreMode.L3, PositionConstant.SIDE_1_ENTRY),
    A4(PositionConstant.SIDE_1_A, ScoreMode.L4, PositionConstant.SIDE_1_ENTRY),
    B1(PositionConstant.SIDE_1_B1, ScoreMode.L1, PositionConstant.SIDE_1_ENTRY),
    B2(PositionConstant.SIDE_1_B, ScoreMode.L2, PositionConstant.SIDE_1_ENTRY),
    B3(PositionConstant.SIDE_1_B, ScoreMode.L3, PositionConstant.SIDE_1_ENTRY),
    B4(PositionConstant.SIDE_1_B, ScoreMode.L4, PositionConstant.SIDE_1_ENTRY),
    C1(PositionConstant.SIDE_2_C1, ScoreMode.L1, PositionConstant.SIDE_2_ENTRY),
    C2(PositionConstant.SIDE_2_C, ScoreMode.L2, PositionConstant.SIDE_2_ENTRY),
    C3(PositionConstant.SIDE_2_C, ScoreMode.L3, PositionConstant.SIDE_2_ENTRY),
}
```

### Problems with this method
Other than the carnal java sins we committed to get to where we are with this state machine, it performed very well. It did not have any huge performance issues (that weren't our own fault).
### Advantages of this method
In last years roundup, I said "A more centralized way to manage the state would be nice, but that'll be next year's endeavor."
I believe we have successfully accomplished this goal by migration to FunctionalState and Abomination.
