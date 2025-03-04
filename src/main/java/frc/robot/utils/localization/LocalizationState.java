// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.localization;

import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public record LocalizationState(FieldZones.Zone fieldZone, Distance distanceToGoal) {}
