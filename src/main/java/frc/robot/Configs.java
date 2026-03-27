package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
        public static final class MAXSwerveModule {
                public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
                public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

                static {
                        // Use module constants to calculate conversion factors and feed forward gain.
                        double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                                        / ModuleConstants.kDrivingMotorReduction;
                        double turningFactor = 2 * Math.PI;
                        // double nominalVoltage = 12.0;
                        // double flywheelVelocityFeedForward = nominalVoltage / 6784;
                        // double drivingVelocityFeedForward = nominalVoltage /
                        // ModuleConstants.kDriveWheelFreeSpeedRps;


                        drivingConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(50);
                        drivingConfig.encoder
                                        .positionConversionFactor(drivingFactor) // meters
                                        .velocityConversionFactor(drivingFactor / 60.0); // meters per second
                        drivingConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        // These are example gains you may need to them for your own robot!
                                        .pid(0.4, 0, 0.0)
                                        .outputRange(-1, 1);
                        // .feedForward.kV(drivingVelocityFeedForward);

                        turningConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(20);

                        turningConfig.absoluteEncoder
                                        // Invert the turning encoder, since the output shaft rotates in the opposite
                                        // direction of the steering motor in the MAXSwerve Module.
                                        .inverted(true)
                                        .positionConversionFactor(turningFactor) // radians
                                        .velocityConversionFactor(turningFactor / 60.0) // radians per second
                                        // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for
                                        // V1):
                                        .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

                        turningConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        // These are example gains you may need to them for your own robot!
                                        .pid(0.03, 0.001, 0.4)
                                        .outputRange(-1, 1)
                                        // Enable PID wrap around for the turning motor. This will allow the PID
                                        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                                        // to 10 degrees will go through 0 rather than the other direction which is a
                                        // longer route.
                                        .positionWrappingEnabled(true)
                                        .positionWrappingInputRange(0, turningFactor);
                }

        }

        public static final class HopperConfigs {
                public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
                public static final SparkMaxConfig agitatorConfig = new SparkMaxConfig();
                public static final SparkFlexConfig flywheelConfig = new SparkFlexConfig();

                static {
                        double nominalVoltage = 12.0;
                        double flywheelVelocityFeedForward = nominalVoltage / 6784;
                        intakeConfig
                                        .idleMode(IdleMode.kCoast)
                                        .smartCurrentLimit(80);

                        agitatorConfig
                                        .idleMode(IdleMode.kCoast)
                                        .smartCurrentLimit(60);
                        agitatorConfig.encoder
                                        .positionConversionFactor(1.0)
                                        .velocityConversionFactor(1.0); // RPM
                        agitatorConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(0.0003, 0, 0.0)
                                        .outputRange(-1, 1).feedForward.kV(flywheelVelocityFeedForward);

                        flywheelConfig
                                        .idleMode(IdleMode.kCoast)
                                        .smartCurrentLimit(60);
                        flywheelConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        //TODO: maybe tune this pid a little, it's still oscillating a bit
                                        .pid(0.001, 0, 0.0)
                                        .outputRange(-1, 1).feedForward.kV(flywheelVelocityFeedForward);
                }
        }

        public static final class ClimberConfig {
                public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

                static {
                        motorConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(80);

                        motorConfig.encoder
                                        .positionConversionFactor(ClimberConstants.kGearRatio);
                }
        }
}
