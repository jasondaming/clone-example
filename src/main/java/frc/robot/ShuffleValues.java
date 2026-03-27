package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class ShuffleValues {
  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds
  public static double kMaxSpeedMetersPerSecond = 0.06;
  public static double kMaxAngularSpeed = 0.1 * Math.PI; // radians per second
  public static double kDriveDeadband = 0.075;
  public static double slewrate_translation = 0.5;
  public static double slewrate_rotation = 2.2;
	public static double flywheel_speed = 3392; //	rpm

  public static SlewRateLimiter translationfilterx = new SlewRateLimiter(ShuffleValues.slewrate_translation);
  public static SlewRateLimiter translationfiltery = new SlewRateLimiter(ShuffleValues.slewrate_translation);
  public static SlewRateLimiter rotationfilter = new SlewRateLimiter(ShuffleValues.slewrate_rotation);
  public static boolean SHUFFLE_MANAGER_ENABLED = true;
}