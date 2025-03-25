package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.MechanismConstants;
import frc.robot.generated.TunerConstants;

public class Elevator extends SubsystemBase {

    private enum ControlMode {
        kStop, kOpenLoop, kPID
    }

    //
    // Hardware
    //
    private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    private final TalonFX m_motor = new TalonFX(MechanismConstants.kElevatorMotorID, "CANivore");
    private final TalonFX m_motorFollower = new TalonFX(MechanismConstants.kElevatorMotorFollowerID, "CANivore");
    private final DigitalInput m_retractLimit = new DigitalInput(MechanismConstants.kRetractLimitSwitchChannel);;;;;;;;;;;;;;;
   
    //
    // State
    //

    private ControlMode m_controlMode = ControlMode.kStop;
    private double m_demand;


    //
    // PID
    //

    private final ProfiledPIDController m_pidController = new ProfiledPIDController(2.4, 0, 0,
        new TrapezoidProfile.Constraints(50, 150));
    private double m_pidLastVelocitySetpoint = 0;
    private double m_pidLastTime;

    private ElevatorFeedforward m_feedforward =
        new ElevatorFeedforward(0, .28235, .2, .005);
    
    
    //
    // SysID
    //
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            // Use default ramp rate (1 V/s)
            null,
            // Reduce dynamic step voltage to 4 to prevent brownout
            Volts.of(2),
            // Use default timeout (10 s)
            null,
            // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> setOutputVoltage(volts.in(Volts)),
            null,
            this
        )
    );

    public Elevator() {

        

        //
        // Base Motor configuration: Brake Mode, Current limits, etc...
        //
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = 125;
        
        //
        // Setup Follower motor
        //
        m_motorFollower.getConfigurator().apply(m_motorConfig);
        m_motorFollower.optimizeBusUtilization();
        m_motorFollower.setControl(new Follower(MechanismConstants.kElevatorMotorID, true));

        //
        // Apply add extra leader configuration on top of the base config
        // - Any control related settings for PID, Motion Magic, Remote Sensors, etc.. 
        // - Soft limits should only be applied to the leader motor, if they are applied to the follower
        //   it may stop moving if their built in encoders are not in sync.
        //
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 113.5;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        m_motor.getConfigurator().apply(m_motorConfig);
        
        // We want to read position data from the leader motor
        m_motor.getPosition().setUpdateFrequency(50);
        m_motor.getVelocity().setUpdateFrequency(50);

        // These 3 are needed for the follower motor to work
        m_motor.getDutyCycle().setUpdateFrequency(50);
        m_motor.getMotorVoltage().setUpdateFrequency(50);
        m_motor.getTorqueCurrent().setUpdateFrequency(50);
        m_motor.optimizeBusUtilization();            
    }

    public boolean isAtRetractLimit() {
        return !m_retractLimit.get();
    }

    public boolean isAtGrabHeight() {
        return (getHeight() > 16.5 && getHeight() < 18.5);
    }

    public double getHeight() {
        return m_motor.getPosition().getValueAsDouble() * MechanismConstants.kRotationToInches;
    }

    public double getVelocity() {
        return m_motor.getVelocity().getValueAsDouble() * MechanismConstants.kRotationToInches;
    }

    public void stop() {
        m_controlMode = ControlMode.kStop;
        m_demand = 0.0;
    }

    public void setOutputVoltage(double OutputVoltage) {
        m_controlMode = ControlMode.kOpenLoop;
        m_demand = OutputVoltage;
    }

    public void setPID(double heightInches) {
        if (m_controlMode != ControlMode.kPID) {
            m_pidController.reset(getHeight());
            m_pidLastVelocitySetpoint = 0;
            m_pidLastTime = Timer.getFPGATimestamp();
        }
        m_controlMode = ControlMode.kPID;
        m_demand = heightInches;
    }

    public boolean isAtSetPoint(){
        return (Math.abs(m_demand - getHeight()) < .5);
    }

   
    //
    // Commands
    //

    
    public Command openLoopCommand(DoubleSupplier OutputVoltageSupplier) {
        return Commands.runEnd(
            () -> this.setOutputVoltage(OutputVoltageSupplier.getAsDouble()), this::stop, this);
    }

    public Command openLoopCommand(double OutputVoltage) {
        return openLoopCommand(() -> OutputVoltage);
    }

    public Command pidCommand(DoubleSupplier heightInchesSupplier) {
        return Commands.runEnd(
            () -> this.setPID(heightInchesSupplier.getAsDouble()), this::stop, this);
    }

    public Command pidCommand(double heightInches) {
        return pidCommand(() -> heightInches);
    }

    
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }



    @Override
    public void periodic() {
        // TODO add switch to disable break mode when disabled

        SmartDashboard.putString("Mechanism wanted control mode", m_controlMode.toString());
       

        SmartDashboard.putString("Mechanism control mode", m_controlMode.toString());
        SmartDashboard.putNumber("Mechanism demand", m_demand);
        SmartDashboard.putBoolean("Mechanism is at retract limit", isAtRetractLimit());
        SmartDashboard.putNumber("Mechanism velocity", getVelocity());
        SmartDashboard.putNumber("Mechanism height", getHeight());
        SmartDashboard.putNumber("Mechanism rotations", m_motor.getPosition().getValueAsDouble());

        if (isAtRetractLimit()){
            m_motor.setPosition(0);
        }

        switch (m_controlMode) {
            case kOpenLoop:
                // Do openloop stuff here
                m_motor.setVoltage(m_demand);    
                break;
            case kPID:
            
                double pidVoltage = m_pidController.calculate(getHeight(), m_demand);
                double dt = Timer.getFPGATimestamp() - m_pidLastTime;
                double accelerationSetpoint = (m_pidController.getSetpoint().velocity - m_pidLastVelocitySetpoint) / dt;
                double feedforwardVoltage = m_feedforward.calculate(m_pidController.getSetpoint().velocity, accelerationSetpoint);
        
                double outputVoltage = pidVoltage + feedforwardVoltage;
                m_motor.setVoltage(outputVoltage);

                SmartDashboard.putNumber("Elevator Output Voltage", pidVoltage);
                SmartDashboard.putNumber("Elevator PID Output Voltage", feedforwardVoltage);
                SmartDashboard.putNumber("Elevator Feedfoward Output Voltage", outputVoltage);
        		SmartDashboard.putNumber("Elevator Profile Position", m_pidController.getSetpoint().position);
        		SmartDashboard.putNumber("Elevator Profile Velocity", m_pidController.getSetpoint().velocity);

                m_pidLastVelocitySetpoint = m_pidController.getSetpoint().velocity;
                m_pidLastTime = Timer.getFPGATimestamp();
                break;
            case kStop:
                // Fall through to default
            default:
                m_motor.stopMotor();
        }      
    }
}