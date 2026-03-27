package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;

public class VariableShootCommand extends Command {
    private HopperSubsystem m_hopperSubsystem;
    private DriveSubsystem m_driveSubsystem;
    private double m_distance;
    private double m_power;
    private final Translation2d kHubPosition;

    public VariableShootCommand(HopperSubsystem hopperSubsystem, DriveSubsystem driveSubsystem) {
        m_hopperSubsystem = hopperSubsystem;
        m_driveSubsystem = driveSubsystem;
        kHubPosition = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? HopperConstants.kRedHubPosition : HopperConstants.kBlueHubPosition;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
        m_hopperSubsystem.setFeederMotorSpeed(HopperConstants.kShooterFeederMotorSpeed, 2.0).schedule();
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_driveSubsystem.getPose();
        m_distance = robotPose.getTranslation().getDistance(kHubPosition);
        System.out.println(m_distance);
        m_power = m_hopperSubsystem.getFlywheelPower(m_distance);
        m_hopperSubsystem.setFlywheelMotorSpeed(m_power);
        // m_hopperSubsystem.runAgitator();
    }

    @Override
    public void end(boolean interrupted) {
        m_hopperSubsystem.stopMotors();
    }
}
