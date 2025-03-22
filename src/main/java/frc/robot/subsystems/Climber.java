package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    //
    // Hardware
    //
    private final SparkMax m_liftMotor = new SparkMax(23, MotorType.kBrushless);
    private SparkMaxConfig m_Config = new SparkMaxConfig();


    //
    // State
    //
    private double m_intakeDemand;

    public Climber() {
        m_Config.smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake);

        m_liftMotor.configure(m_Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void stop() {
        m_intakeDemand = 0.0;
    }

    public void setIntakeVoltage(double IntakeVoltage) {
        m_intakeDemand = IntakeVoltage;
    }

    public Command openLoopClimbCommand(DoubleSupplier IntakeVoltageSupplier) {
        return Commands.startEnd(
                () -> this.setIntakeVoltage(IntakeVoltageSupplier.getAsDouble()), this::stop, this);
    }

    public Command openLoopClimbCommand(double IntakeVoltage) {
        return openLoopClimbCommand(() -> IntakeVoltage);
    }

    @Override
    public void periodic() {
        m_liftMotor.set(m_intakeDemand);
        SmartDashboard.putNumber("Climber Voltage", m_intakeDemand);
    }
}
