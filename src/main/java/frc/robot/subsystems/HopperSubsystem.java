package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Configs.HopperConfigs;
import frc.robot.Constants.HopperConstants;
import frc.robot.ShuffleValues;

public class HopperSubsystem extends SubsystemBase {

    private final SparkFlex m_feederMotor;
    private final SparkFlex m_leftFlywheelSpark;
    private final SparkFlex m_rightFlywheelSpark;
    private final SparkMax m_agitatorMotor;
    private final RelativeEncoder m_agitatorEncoder;
    private final RelativeEncoder m_leftFlywheelEncoder;
    private final RelativeEncoder m_rightFlywheelEncoder;
    private final InterpolatingDoubleTreeMap m_shooterFlywheelPower;
    private PIDController agitatorPID;

    private final SparkClosedLoopController m_rightFlywheelClosedLoopController;
    private final SparkClosedLoopController m_leftFlywheelClosedLoopController;
    private final SparkClosedLoopController m_agitatorClosedLoopController;

    public HopperSubsystem() {
        m_feederMotor = new SparkFlex(HopperConstants.kFeederCANID, MotorType.kBrushless);

        m_rightFlywheelSpark = new SparkFlex(HopperConstants.kRightFlywheelCANID, MotorType.kBrushless);
        m_rightFlywheelEncoder = m_rightFlywheelSpark.getEncoder();
        m_rightFlywheelEncoder.setPosition(0);
        m_rightFlywheelClosedLoopController = m_rightFlywheelSpark.getClosedLoopController();
        m_rightFlywheelSpark.configure(Configs.HopperConfigs.flywheelConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_leftFlywheelSpark = new SparkFlex(HopperConstants.kLeftFlywheelCANID, MotorType.kBrushless);
        m_leftFlywheelEncoder = m_leftFlywheelSpark.getEncoder();
        m_leftFlywheelEncoder.setPosition(0);
        m_leftFlywheelClosedLoopController = m_leftFlywheelSpark.getClosedLoopController();
        m_leftFlywheelSpark.configure(Configs.HopperConfigs.flywheelConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_agitatorMotor = new SparkMax(HopperConstants.kAgitatorCANID, MotorType.kBrushless);
        m_agitatorEncoder = m_agitatorMotor.getEncoder();
        m_agitatorClosedLoopController = m_agitatorMotor.getClosedLoopController();


        m_feederMotor.configure(HopperConfigs.intakeConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_agitatorMotor.configure(HopperConfigs.agitatorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_shooterFlywheelPower = new InterpolatingDoubleTreeMap(); // In Meters

        //speeds are measured in rpm, max value is 6784

        //TODO: add shuffleboard command for adjusting shooter speed
        m_shooterFlywheelPower.put(9.186, 3200.0);
        m_shooterFlywheelPower.put( 10.171, 3450.0);
        m_shooterFlywheelPower.put( 11.402, 4000.0);

        setDefaultCommand(
                runOnce(
                        () -> {
                            m_leftFlywheelSpark.disable();
                            m_rightFlywheelSpark.disable();
                            m_feederMotor.disable();
                            m_agitatorMotor.disable();
                        })
                        // .andThen(Commands.sequence(
                        // Commands.waitSeconds(HopperConstants.agitatorSwitchingDelay).andThen(runOnce(()
                        // -> agitatorPID.setSetpoint(-HopperConstants.kAgitatorMotorSpeed))),
                        // Commands.waitSeconds(HopperConstants.agitatorSwitchingDelay).andThen(runOnce(()
                        // -> agitatorPID.setSetpoint(HopperConstants.kAgitatorMotorSpeed)))
                        // ))
                        .withName("Idle"));

    }

    public Command shootCommand(double flywheelSpeed, double feederMotorSpeed, double delay) {
        return Commands.parallel(

                // Run the shooter flywheel at the desired setpoint using feedforward and
                // feedback
                run(
                        () -> {
                            m_rightFlywheelClosedLoopController.setSetpoint(flywheelSpeed, ControlType.kVelocity);
                            m_leftFlywheelClosedLoopController.setSetpoint(flywheelSpeed, ControlType.kVelocity);
                        }),

                // Wait until the shooter has reached the setpoint, and then run the feeder
                Commands.waitSeconds(delay).andThen(() -> {
                    m_feederMotor.set(feederMotorSpeed);
                    m_agitatorClosedLoopController.setSetpoint(HopperConstants.kAgitatorMotorSpeed * HopperConstants.agitatorFactor, ControlType.kVelocity);
                }))
                .withName("Shoot");
    }

    public Command fullSpeedDump(){
        return Commands.sequence(
            run(
                ()->{
                    System.out.println("Running command");
                    m_feederMotor.set(1);
                    m_rightFlywheelSpark.set(-1);
                    m_leftFlywheelSpark.set(-1);
            })
        );
    }

    public Command theCoolerShootCommand(double flywheelSpeed, double feederMotorSpeed, double delay) {
        return Commands.sequence(

                // Run the shooter flywheel at the desired setpoint using feedforward and
                // feedback
                runOnce(
                        () -> {
                            m_rightFlywheelClosedLoopController.setSetpoint(flywheelSpeed, ControlType.kVelocity);
                            m_leftFlywheelClosedLoopController.setSetpoint(flywheelSpeed, ControlType.kVelocity);
                        }),

                // Wait until the shooter has reached the setpoint, and then run the feeder
                Commands.waitSeconds(delay).andThen(() -> {
                    m_feederMotor.set(feederMotorSpeed);
                    m_agitatorClosedLoopController.setSetpoint(HopperConstants.kAgitatorMotorSpeed * HopperConstants.agitatorFactor, ControlType.kVelocity);
                }),
                Commands.waitSeconds(5))
                .withName("Cooler Shoot");
    }

    public void stopMotors() {
        m_rightFlywheelSpark.set(0);
        m_leftFlywheelSpark.set(0);
        m_feederMotor.set(0);
        m_rightFlywheelSpark.stopMotor();
        m_leftFlywheelSpark.stopMotor();
        m_feederMotor.stopMotor();
    }

    public double getFlywheelPower(double distance) {
        return m_shooterFlywheelPower.get(distance);
    }

    public void setFlywheelMotorSpeed(double power) {
        m_rightFlywheelClosedLoopController.setSetpoint(power, ControlType.kVelocity);
        m_leftFlywheelClosedLoopController.setSetpoint(power, ControlType.kVelocity);
    }

    public Command setFeederMotorSpeed(double feederSpeed, double delay) {
        return Commands.waitSeconds(delay).andThen(() -> {
            m_feederMotor.set(feederSpeed);
            runAgitator();
        });
    }

    public void runAgitator() {
        m_agitatorClosedLoopController.setSetpoint(HopperConstants.kAgitatorMotorSpeed * HopperConstants.agitatorFactor, ControlType.kVelocity);
    }
}
