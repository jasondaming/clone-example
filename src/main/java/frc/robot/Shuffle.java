// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Shuffle {
  public ShuffleboardTab shuffleTab = Shuffleboard.getTab("tooning");

  private GenericEntry slew = shuffleTab.addPersistent("xy slew",
      ShuffleValues.slewrate_translation)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0.1,
          "max", 10))
      .getEntry();
  private GenericEntry maxSpeed = shuffleTab.addPersistent("max speed",
      ShuffleValues.kMaxSpeedMetersPerSecond)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0,
          "max", 10))
      .getEntry();
  private GenericEntry maxRot = shuffleTab.addPersistent("max rot per s",
      ShuffleValues.kMaxAngularSpeed)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0,
          "max", 10))
      .getEntry();
  private GenericEntry rotSlew = shuffleTab.addPersistent("rotation slew",
      ShuffleValues.slewrate_rotation)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0.1,
          "max", 10))
      .getEntry();
	private GenericEntry flywheelSpeed = shuffleTab.addPersistent("Flywheel Speed",
      ShuffleValues.flywheel_speed)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1,
          "max", 6784))
      .getEntry();
public void refreshValue(double desiredpos, double actualpos) {
if (ShuffleValues.SHUFFLE_MANAGER_ENABLED) {

      Double maxSpeedDouble = maxSpeed.getDouble(ShuffleValues.kMaxSpeedMetersPerSecond);
      if (ShuffleValues.kMaxSpeedMetersPerSecond != maxSpeedDouble) {
        ShuffleValues.kMaxSpeedMetersPerSecond = maxSpeedDouble;
      }

      Double maxSpeedRotationDouble = maxRot.getDouble(ShuffleValues.kMaxAngularSpeed);
      if (ShuffleValues.kMaxAngularSpeed != maxSpeedRotationDouble) {
        ShuffleValues.kMaxAngularSpeed = maxSpeedRotationDouble;
      }

			Double flywheelSpeedDouble = flywheelSpeed.getDouble(ShuffleValues.flywheel_speed);
      if (ShuffleValues.flywheel_speed != flywheelSpeedDouble) {
        ShuffleValues.flywheel_speed = flywheelSpeedDouble;
      }

      Double slewTranslationDouble = slew.getDouble(ShuffleValues.slewrate_translation);
      if (ShuffleValues.slewrate_translation != slewTranslationDouble) {
        ShuffleValues.slewrate_translation = slewTranslationDouble;
        ShuffleValues.translationfilterx = new SlewRateLimiter(ShuffleValues.slewrate_translation);
        ShuffleValues.translationfiltery = new SlewRateLimiter(ShuffleValues.slewrate_translation);
      }

      Double slewRotationDouble = rotSlew.getDouble(ShuffleValues.slewrate_rotation);
      if (ShuffleValues.slewrate_rotation != slewRotationDouble) {
        ShuffleValues.slewrate_rotation = slewRotationDouble;
        ShuffleValues.rotationfilter = new SlewRateLimiter(slewRotationDouble);
      }
    }
  }
}
