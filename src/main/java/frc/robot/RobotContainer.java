// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// import java.util.jar.Attributes.Name;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants.ClimberConstants;
// import frc.robot.commands.ClimberCommand;
// import frc.robot.commands.VariableShootCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.VariableShootCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final HopperSubsystem m_hopper = new HopperSubsystem();
  public final ClimberSubsystem m_climber = new ClimberSubsystem();

  XboxController m_driverController = new XboxController(0);
  XboxController m_operatorController = new XboxController(1);

  Trigger startButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  Trigger intakeButton = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);
  Trigger variableShootButton = new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.8);
  Trigger dumpButton = new Trigger(() -> m_operatorController.getLeftTriggerAxis() > 0.8);
  Trigger clearJamButton = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);
  Trigger shootButton = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
  Trigger autoAimButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  Trigger fullSpeedDump = new JoystickButton(m_operatorController, XboxController.Button.kX.value);

  // Trigger climbButton = new JoystickButton(m_operatorController,
  // XboxController.Button.kLeftBumper.value);
  // Trigger cancelButton = new JoystickButton(m_operatorController,
  // XboxController.Button.kRightBumper.value);

  // public final SequentialCommandGroup climbCommand = new
  // SequentialCommandGroup(
  // new ClimberCommand(m_climber, ClimberConstants.kDesiredPosOne),
  // new ClimberCommand(m_climber, ClimberConstants.kRetractedDesiredPos));

  public Shuffle m_shuffle = new Shuffle();

  public LimelightHelpers.PoseEstimate limelightMeasurement;

  private final Field2d m_field = new Field2d();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    new EventTrigger("Run Intake").whileTrue(
        m_hopper.shootCommand(HopperConstants.kIntakeFlywheelMotorSpeed, HopperConstants.kIntakeFeederMotorSpeed, 0.0));
    NamedCommands.registerCommand("Variable Shoot",
        (new VariableShootCommand(m_hopper, m_robotDrive)));
   NamedCommands.registerCommand("Variable Shoot Timed",
        (Commands.sequence(new VariableShootCommand(m_hopper, m_robotDrive), Commands.waitSeconds(2))));
    NamedCommands.registerCommand("Shoot", m_hopper.shootCommand(HopperConstants.kShooterFlywheelMotorSpeed,
        HopperConstants.kShooterFeederMotorSpeed, 2.0)); // TODO: tune delay!!
    new EventTrigger("Cooler Shoot").onTrue(m_hopper.theCoolerShootCommand(HopperConstants.kShooterFlywheelMotorSpeed,
        HopperConstants.kShooterFeederMotorSpeed, 2.0));
    new EventTrigger("Dump").whileTrue(m_hopper.shootCommand(HopperConstants.kReverseIntakeFlywheelMotorSpeed,
        HopperConstants.kReverseIntakeFeederMotorSpeed, 0.0));
    new EventTrigger("Auto Aim").whileTrue(new AutoAimCommand(m_robotDrive));

    // Configure the trigger bindings
    configureBindings();
    m_robotDrive.setDefaultCommand(new RunCommand(
        () -> {
          double leftY = ShuffleValues.translationfiltery.calculate(m_driverController.getLeftY());
          double leftX = ShuffleValues.translationfilterx.calculate(m_driverController.getLeftX());
          double rightX = ShuffleValues.rotationfilter.calculate(m_driverController.getRightX());
          // Apply a round deadband, based on the x/y distance from the origin
          double distanceFromZero = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)); // Pythagoras
          if (distanceFromZero < DriveConstants.kDriveDeadband) {
            leftX = 0;
            leftY = 0;
          }

          leftY = Math.pow(leftY, 3);
          leftX = Math.pow(leftX, 3);
          rightX = Math.pow(MathUtil.applyDeadband(rightX, DriveConstants.kDriveDeadband), 3);

          m_robotDrive.drive(
              -leftY,
              -leftX,
              -rightX,
              DriveConstants.kfieldRelative);
        },
        m_robotDrive));

    LimelightHelpers.SetRobotOrientation("limelight",
        m_robotDrive.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    m_robotDrive.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
    m_robotDrive.m_poseEstimator.addVisionMeasurement(
        limelightMeasurement.pose,
        limelightMeasurement.timestampSeconds);
    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    startButton.onTrue(Commands.runOnce(() -> {
      m_robotDrive.resetGyro();
    }, m_robotDrive));

    intakeButton.whileTrue(
        m_hopper.shootCommand(HopperConstants.kIntakeFlywheelMotorSpeed, HopperConstants.kIntakeFeederMotorSpeed, 0.0));
    variableShootButton.whileTrue(new VariableShootCommand(m_hopper, m_robotDrive));
    shootButton.whileTrue(m_hopper.shootCommand(HopperConstants.kShooterFlywheelMotorSpeed,
        HopperConstants.kShooterFeederMotorSpeed, 2.0)); // TODO: tune delay!!
    dumpButton.whileTrue(m_hopper.shootCommand(HopperConstants.kReverseIntakeFlywheelMotorSpeed,
        HopperConstants.kReverseIntakeFeederMotorSpeed, 0.0));
    clearJamButton.whileTrue(m_hopper.shootCommand(HopperConstants.kReverseIntakeFlywheelMotorSpeed, 0.0, 5.0));
    autoAimButton.whileTrue(new AutoAimCommand(m_robotDrive));
    fullSpeedDump.whileTrue(m_hopper.fullSpeedDump());

    // climbButton.onTrue(climbCommand);
    // cancelButton.onTrue(Commands.runOnce(climbCommand::cancel));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
   * public Command getAutonomousCommand() {
   * // An example command will be run in autonomous
   * return Autos.exampleAuto(m_exampleSubsystem);
   * }
   */

  public void updateOdometry() {
    m_robotDrive.periodic();
    LimelightHelpers.SetRobotOrientation("limelight",
        m_robotDrive.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    if (limelightMeasurement.tagCount >= 1) { // Only trust measurement if we see multiple tags
      m_robotDrive.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      m_robotDrive.m_poseEstimator.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds);
    }
  }

  Pose2d startPose = null;

  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    // return new RunCommand(() -> {
    // if (startPose == null) {
    // startPose = m_robotDrive.m_poseEstimator.getEstimatedPosition();
    // m_robotDrive.drive(0.0, -0.3, 0, false);
    // return;
    // }
    // Transform2d cur =
    // m_robotDrive.m_poseEstimator.getEstimatedPosition().minus(startPose);
    // if (Math.abs(cur.getX()) >= 2 || Math.abs(cur.getY()) >= 2) {
    // m_robotDrive.drive(0.0, 0.0, 0, false);
    // return;
    // }
    // System.out.println("X, Y: " + cur.getX() + ", " + cur.getY());
    // m_robotDrive.drive(0.0, -0.4, 0, false);
    // }, m_robotDrive);

    return new PathPlannerAuto("test");
  }

  public void refresh_shuffleboard() {
    m_shuffle.refreshValue(m_robotDrive.m_frontLeft.getState().angle.getRadians(),
        m_robotDrive.m_frontLeft.getPosition().angle.getRadians());

    m_shuffle.refreshValue(m_robotDrive.m_frontRight.getState().angle.getRadians(),
        m_robotDrive.m_frontRight.getPosition().angle.getRadians());
    m_field.setRobotPose(m_robotDrive.getPose());
    SmartDashboard.putNumber("Gyro Yaw (raw)", m_robotDrive.getHeading());
    SmartDashboard.putNumber("Pose Heading (deg)", m_robotDrive.getPose().getRotation().getDegrees());
  }

  public void printLimeLight() {
    System.out.println(LimelightHelpers.getTX(""));
  }
}
