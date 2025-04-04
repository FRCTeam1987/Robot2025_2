package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.state.Abomination;
import frc.robot.state.logic.constants.FieldPosition;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Stream;

public class Utils {

  private static double armIncrement = 1.0;

  public static void incrementArm(double amount) {
    armIncrement += amount;
    DogLog.log("Arm/OverrideValue", armIncrement);
  }

  public static double getArmOverride() {
    return armIncrement;
  }

  public static Pose2d getNearest(List<Pose2d> posesRed, List<Pose2d> posesBlue) {
    return RobotContainer.DRIVETRAIN
        .getPose()
        .nearest(
            RobotContainer.DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
                ? posesRed
                : posesBlue);
  }

  public static Pose2d getNearestFieldPos(List<FieldPosition> fieldPos) {
    return RobotContainer.DRIVETRAIN
        .getPose()
        .nearest(
            (fieldPos)
                .stream()
                    .map(
                        (pos) ->
                            RobotContainer.DRIVETRAIN.getAlliance() == DriverStation.Alliance.Red
                                ? pos.getLocation().getRedPose()
                                : pos.getLocation().getBluePose())
                    .toList());
  }

  public static FieldPosition getNearest(List<FieldPosition> fieldPos) {
    Pose2d bot = RobotContainer.DRIVETRAIN.getPose();
    return Collections.min(
        fieldPos,
        Comparator.comparing(
                (FieldPosition other) ->
                    bot.getTranslation()
                        .getDistance(other.getLocation().getAlliancePose().getTranslation()))
            .thenComparing(
                (FieldPosition other) ->
                    Math.abs(
                        bot.getRotation()
                            .minus(other.getLocation().getAlliancePose().getRotation())
                            .getRadians())));
  }

  public static Pose2d processAndReturn(List<FieldPosition> list) {
    FieldPosition current = Utils.getNearest(list);
    if (RobotContainer.TRACKER.isScored(current)) {
      ArrayList<FieldPosition> newList = new ArrayList<>(list);
      newList.remove(current);
      return processAndReturn(newList);
    } else {
      if (!list.contains(current)) {
        list.add(current);
      }
      Abomination.setLastScoredPosition(current);
      return current.getLocation().getAlliancePose();
    }
  }

  public static SendableChooser<PathPlannerAuto> buildAutoChooser(String defaultAutoName) {
    return buildAutoChooserWithOptionsModifier(defaultAutoName, (stream) -> stream);
  }

  public static SendableChooser<PathPlannerAuto> buildAutoChooserWithOptionsModifier(
      String defaultAutoName,
      Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {
    if (!AutoBuilder.isConfigured()) {
      throw new RuntimeException(
          "AutoBuilder was not configured before attempting to build an auto chooser");
    }

    SendableChooser<PathPlannerAuto> chooser = new SendableChooser<>();
    List<String> autoNames = AutoBuilder.getAllAutoNames();

    PathPlannerAuto defaultOption = null;
    List<PathPlannerAuto> options = new ArrayList<>();

    for (String autoName : autoNames) {
      PathPlannerAuto auto = new PathPlannerAuto(autoName);

      if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
        defaultOption = auto;
      } else {
        options.add(auto);
      }
    }

    if (defaultOption == null) {
      chooser.setDefaultOption("None", new PathPlannerAuto(Commands.none()));
    } else {
      chooser.setDefaultOption(defaultOption.getName(), defaultOption);
      chooser.addOption("None", new PathPlannerAuto(Commands.none()));
    }

    optionsModifier
        .apply(options.stream())
        .forEach(auto -> chooser.addOption(auto.getName(), auto));

    return chooser;
  }
}
