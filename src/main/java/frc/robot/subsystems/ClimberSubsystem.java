package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ClimberConfig;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax m_leftMotor;
    private final SparkMax m_rightMotor;

    private final RelativeEncoder m_encoder;

    public ClimberSubsystem() {
        m_leftMotor = new SparkMax(ClimberConstants.kLeftCANId, MotorType.kBrushless);
        m_rightMotor = new SparkMax(ClimberConstants.kRightCANId, MotorType.kBrushless);

        m_leftMotor.configure(ClimberConfig.motorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_rightMotor.configure(ClimberConfig.motorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_encoder = m_leftMotor.getEncoder();
    }

    public void stop() {
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
    }

    public void setMotorSpeed(double speed) {
        m_leftMotor
                .set(MathUtil.clamp(speed, -ClimberConstants.kMaxMotorSpeed, ClimberConstants.kMaxMotorSpeed));
        m_rightMotor
                .set(MathUtil.clamp(speed, -ClimberConstants.kMaxMotorSpeed, ClimberConstants.kMaxMotorSpeed));
    }

    public double getEncoderPos() {
        return m_encoder.getPosition();
    }

}