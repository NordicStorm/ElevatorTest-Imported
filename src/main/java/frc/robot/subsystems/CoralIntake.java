package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class CoralIntake extends SubsystemBase {
    //
    // Hardware
    //
    private final SparkMax m_rightIntakeMotor = new SparkMax(17, MotorType.kBrushless);
    private final SparkMax m_leftIntakeMotor = new SparkMax(16, MotorType.kBrushless);
    private SparkMaxConfig m_rightIntakeMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig m_leftIntakeMotorConfig = new SparkMaxConfig();

    //
    // State
    //
    private double m_intakeDemand;

    public CoralIntake() {
        m_rightIntakeMotorConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(20);
        m_leftIntakeMotorConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(20);

        m_leftIntakeMotor.configure(m_leftIntakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_rightIntakeMotor.configure(m_rightIntakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

    public void stop() {
        m_intakeDemand = 0.0;
    }

    public void setIntakeVoltage(double IntakeVoltage) {
        m_intakeDemand = IntakeVoltage;
    }

    public Command openLoopIntakeCommand(DoubleSupplier IntakeVoltageSupplier) {
        return Commands.startEnd(
                () -> this.setIntakeVoltage(IntakeVoltageSupplier.getAsDouble()), this::stop, this);
    }

    public Command openLoopIntakeCommand(double IntakeVoltage) {
        return openLoopIntakeCommand(() -> IntakeVoltage);
    }

    @Override
    public void periodic() {
        m_rightIntakeMotor.set(m_intakeDemand);
        //m_leftIntakeMotor.set(m_intakeDemand);
        SmartDashboard.putNumber("Voltage", m_intakeDemand);
        SmartDashboard.putNumber("Velocity", m_leftIntakeMotor.getEncoder().getVelocity());

    }
}
