package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAimCommand extends Command {
    private DriveSubsystem m_driveSubsystem;
    private final Translation2d kHubPosition;
    private PIDController thetaController;

    public AutoAimCommand(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        kHubPosition = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? HopperConstants.kRedHubPosition : HopperConstants.kBlueHubPosition;
    }

    @Override
    public void initialize() {
        thetaController = new PIDController(0.075, 0, 0.0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(2 * Math.PI * 5/360);
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_driveSubsystem.getPose();
        double angle = Math.atan2(kHubPosition.getY() - robotPose.getY(), kHubPosition.getX() - robotPose.getX());
        thetaController.setSetpoint(angle);
        double angleOverride = thetaController.calculate(m_driveSubsystem.getPose().getRotation().getRadians());
        m_driveSubsystem.setYawOverride(angleOverride);
        //System.out.println(angleOverride);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.clearYawOverride();
    }
}
