package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.Constants.ClimberConstants;

public class ClimberCommand extends Command {
    private ClimberSubsystem m_climberSubsystem;
    private double m_desiredPos;
    private PIDController m_PIDController;
    private double m_speedOutput;

    public ClimberCommand(ClimberSubsystem climberSubsystem, double desiredPos) {
        m_climberSubsystem = climberSubsystem;
        m_desiredPos = desiredPos;
        m_PIDController = new PIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);
        m_PIDController.setTolerance(ClimberConstants.kPositionTolerance);

        addRequirements(m_climberSubsystem);
    }

    @Override
    public void initialize() {
        m_PIDController.reset();
        m_PIDController.setSetpoint(m_desiredPos);
    }

    @Override
    public void execute() {
        m_speedOutput = m_PIDController.calculate(m_climberSubsystem.getEncoderPos(), m_desiredPos);
        m_climberSubsystem.setMotorSpeed(m_speedOutput);
    }

    @Override
    public boolean isFinished() {
        return m_PIDController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_climberSubsystem.stop();
    }
}