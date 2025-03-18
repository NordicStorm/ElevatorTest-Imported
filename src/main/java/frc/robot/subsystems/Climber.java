package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Climber extends SubsystemBase {
    //
    // Hardware
    //
    private final PWMSparkMax m_liftMotor = new PWMSparkMax(3);

    //
    // State
    //
    private double m_intakeDemand;

    public Climber() {
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
        if (m_intakeDemand > .01){
            m_liftMotor.setInverted(false);
            m_liftMotor.set(Math.abs(m_intakeDemand));
        }
        else if (m_intakeDemand < .01){
            m_liftMotor.setInverted(true);
            m_liftMotor.set(Math.abs(m_intakeDemand));
        }
        else
            this.stop();

        SmartDashboard.putNumber("Climber Voltage", m_intakeDemand);
    }
}
