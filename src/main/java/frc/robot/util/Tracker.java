// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;
import frc.robot.state.logic.constants.FieldPosition;
import java.util.HashMap;
import java.util.List;

/** Add your docs here. */
public class Tracker {

  private static BooleanSubscriber getSubscriber(final String branch, final String level) {
    return NetworkTableInstance.getDefault()
        .getTable("tracker")
        .getSubTable("reef")
        .getSubTable("branches")
        .getSubTable(branch.toLowerCase())
        .getBooleanTopic(level)
        .subscribe(false);
  }

  private static BooleanPublisher getPublisher(final String branch, final String level) {
    BooleanPublisher pub =
        NetworkTableInstance.getDefault()
            .getTable("tracker")
            .getSubTable("reef")
            .getSubTable("branches")
            .getSubTable(branch.toLowerCase())
            .getBooleanTopic(level)
            .publish();
    pub.set(false);
    return pub;
  }

  final HashMap<String, BooleanSubscriber> subscribers;
  final HashMap<String, BooleanPublisher> publishers;

  public Tracker() {
    subscribers = new HashMap<String, BooleanSubscriber>();
    publishers = new HashMap<String, BooleanPublisher>();
    final List<String> branches =
        List.of("A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L");
    final List<String> levels = List.of("2", "3", "4");
    branches.forEach(
        (branch) -> {
          levels.forEach(
              (level) -> {
                subscribers.put(branch + level, getSubscriber(branch, level));
                publishers.put(branch + level, getPublisher(branch, level));
              });
        });
  }

  public boolean isScored(final FieldPosition position) {
    return isScored(position.name());
  }

  public void setScored(final FieldPosition position, boolean isScored) {
    setScored(position.toString(), isScored);
  }

  public boolean isScored(final String branchLevel) {
    if (RobotContainer.BYPASS_TRACKER) {
      return false;
    } else {
      return subscribers.get(branchLevel.toUpperCase()).get();
    }
  }

  public boolean isScored(final String branch, final String level) {
    return subscribers.get(branch.toUpperCase() + level).get();
  }

  public void setScored(final String branch, final String level, final boolean isScored) {
    publishers.get(branch.toUpperCase() + level).set(isScored);
  }

  public void setScored(final String branchlevel, final boolean isScored) {
    publishers.get(branchlevel.toUpperCase()).set(isScored);
  }
}
