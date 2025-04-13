package frc.robot.subsystems;

public abstract class BroncSystem {

  private final String NAME;

  public BroncSystem() {
    String name = this.getClass().getSimpleName();
    NAME = name.substring(name.lastIndexOf('.') + 1);
  }

  public void preCycle() {}

  public void postCycle() {}

  public void log() {}
  ;
}
