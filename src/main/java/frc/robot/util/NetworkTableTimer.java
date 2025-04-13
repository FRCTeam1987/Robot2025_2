package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class NetworkTableTimer {
  public static Runnable wrap(String label, Runnable runnable) {
    NetworkTableEntry entry = NetworkTableInstance.getDefault().getTable("timing").getEntry(label);

    return () -> {
      double startTime = Timer.getFPGATimestamp();
      runnable.run();
      entry.setDouble(Timer.getFPGATimestamp() - startTime);
    };
  }
}
