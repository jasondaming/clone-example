// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.ShuffleValues;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  public final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      DriveConstants.kFrontLeftAbsoluteEncoderCanId);

  public final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      DriveConstants.kFrontRightAbsoluteEncoderCanId);

  public final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftChassisAngularOffset,
      DriveConstants.kRearLeftAbsoluteEncoderCanId);

  public final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightChassisAngularOffset,
      DriveConstants.kRearRightAbsoluteEncoderCanId);

  // The gyro sensor
  public final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGyroCanID);

  // These are updated in `drive()` and used in `getRobotRelativeSpeeds()`
  private SwerveModuleState[] m_swerveModuleStates = DriveConstants.kDriveKinematics
      .toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
  private final SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[4]; // To avoid GC in periodic

  private double overrideYaw = 0;

  // Odometry class for tracking robot pose
  public SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getGyroYaw(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }, new Pose2d(0, 0, new Rotation2d()));

  /** Creates a new DriveSubsystem. */
  RobotConfig config;

  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                              // ChassisSpeeds. Also optionally outputs individual
                                                              // module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants(4.7, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.25, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    m_modulePositions[0] = m_frontLeft.getPosition();
    m_modulePositions[1] = m_frontRight.getPosition();
    m_modulePositions[2] = m_rearLeft.getPosition();
    m_modulePositions[3] = m_rearRight.getPosition();
    // Update the odometry in the periodic block
    m_poseEstimator.update(getGyroYaw(), m_modulePositions); // Use the pre-allocated array
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getGyroYaw(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * ShuffleValues.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * ShuffleValues.kMaxSpeedMetersPerSecond;

    rot = (overrideYaw != 0)
      ? overrideYaw
      : rot;

    double rotDelivered = rot * ShuffleValues.kMaxAngularSpeed;

    m_swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getPose().getRotation())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        m_swerveModuleStates, ShuffleValues.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(m_swerveModuleStates[0]);
    m_frontRight.setDesiredState(m_swerveModuleStates[1]);
    m_rearLeft.setDesiredState(m_swerveModuleStates[2]);
    m_rearRight.setDesiredState(m_swerveModuleStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    // System.out.println(speeds.vxMetersPerSecond);
    // System.out.println(speeds.vyMetersPerSecond);
    m_swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    // SwerveDriveKinematics.desaturateWheelSpeeds(m_swerveModuleStates,
    // ShuffleValues.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(m_swerveModuleStates[0]);
    m_frontRight.setDesiredState(m_swerveModuleStates[1]);
    m_rearLeft.setDesiredState(m_swerveModuleStates[2]);
    m_rearRight.setDesiredState(m_swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, ShuffleValues.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_rearLeft.stop();
    m_rearRight.stop();
  }

  /** Zeroes the heading of the robot. */
  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getYaw().getValueAsDouble();
  }

  private final Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_rearLeft.getState();
    states[3] = m_rearRight.getState();
    ChassisSpeeds chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(states);
    return chassisSpeeds;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   * 
   *         public double getTurnRate() {
   *         return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ?
   *         -1.0 : 1.0);
   *         }
   */

   public void setYawOverride(double override) {
    overrideYaw = override;
   }

   public void clearYawOverride() {
    overrideYaw = 0;
   }
}
