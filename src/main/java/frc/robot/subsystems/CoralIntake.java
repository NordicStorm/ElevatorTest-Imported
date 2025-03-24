package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.RollingAverage;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.Status;

public class CoralIntake extends SubsystemBase {
    //
    // Hardware
    //
    private final SparkMax m_rightIntakeMotor = new SparkMax(17, MotorType.kBrushless);
    private final SparkMax m_leftIntakeMotor = new SparkMax(16, MotorType.kBrushless);
    private SparkMaxConfig m_rightIntakeMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig m_leftIntakeMotorConfig = new SparkMaxConfig();
    private TimeOfFlight TOF = new TimeOfFlight(0);
    private RollingAverage rollingAverage = new RollingAverage(3);

    //
    // State
    //
    private double m_intakeDemand;

    public CoralIntake() {
        m_rightIntakeMotorConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(25);
        m_leftIntakeMotorConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(25);

        m_leftIntakeMotor.configure(m_leftIntakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_rightIntakeMotor.configure(m_rightIntakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        TOF.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        TOF.setRangeOfInterest(8,8,12,12);
    }

    public void stop() {
        m_intakeDemand = 0.0;
    }

    public double coralRange() {
        return TOF.getRange();
    }

    public boolean hasCoral() {
        return (TOF.getRange() < 65 && TOF.getRange() > 45 && isRangeValid());
    }

    public boolean isInTrough(){
        return (TOF.getRange() < 200 && TOF.getRange() > 180 && isRangeValid()); //TODO
    }

    public void setIntakeVoltage(double IntakeVoltage) {
        m_intakeDemand = IntakeVoltage;
    }

    public boolean isRangeValid(){
        return rollingAverage.get() < 4 && (TOF.getStatus() == Status.Valid || TOF.getStatus() == Status.Invalid);
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
        rollingAverage.put(TOF.getRangeSigma());

        m_rightIntakeMotor.set(m_intakeDemand);
        m_leftIntakeMotor.set(m_intakeDemand);

        SmartDashboard.putNumber("Voltage", m_intakeDemand);
        SmartDashboard.putNumber("Velocity", m_leftIntakeMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("TOF Range", coralRange());
        SmartDashboard.putBoolean("Has coral", hasCoral());
        SmartDashboard.putBoolean("Is TOF reliable?", TOF.isRangeValid());
        SmartDashboard.putBoolean("Is TOF reliable alternate", isRangeValid());
        SmartDashboard.putString("TOF status", TOF.getStatus().toString());
        SmartDashboard.putNumber("Sigma", TOF.getRangeSigma());
    }
}
