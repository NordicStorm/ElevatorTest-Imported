package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.MechanismConstants;

public class Arm extends SubsystemBase {

    private enum ControlMode {
        kStop, kOpenLoop, kPID
    }

    public enum GeneralArmPosition{
        blockingVertical(3), 
        clearOfBottom(2), 
        blockingDown(1),
        straightDown(0);


        public final int value;
        
        GeneralArmPosition(int angle){
            this.value = angle;
        }
    }

    //
    // Hardware
    //

    private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    private final TalonFX m_motor = new TalonFX(MechanismConstants.kArmID, "rio");
    private final MotionMagicVoltage m_voltage = new MotionMagicVoltage(0);

    // TODO find CAN id
    public final CANdi m_candi = new CANdi(0, "rio");
    public final CANdiConfiguration configs_CANdi = new CANdiConfiguration();

    private final ProfiledPIDController m_pidController = new ProfiledPIDController(2.4, 0, 0,
        new TrapezoidProfile.Constraints(50, 150));

    private double m_pidLastVelocitySetpoint = 0;
    private double m_pidLastTime;

    private ArmFeedforward m_feedforward =
        new ArmFeedforward(0, .28235, .2, .005);

    //
    // State
    //

    private ControlMode m_controlMode = ControlMode.kStop;
    private double demandVoltage;
    private double m_desiredState;

    public Arm() {
        m_motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANdiPWM1;
        // TODO find CAN ID m_motorConfig.Feedback.FeedbackRemoteSensorID();

        m_motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        m_motorConfig.Slot0.kG = 0.325; // TODO: Add 0.25 V output to overcome static friction
        m_motorConfig.Slot0.kS = .18;
        m_motorConfig.Slot0.kV = .12; // TODO: A velocity target of 1 rps results in 0.12 V output
        m_motorConfig.Slot0.kA = 10; // TODO: An acceleration of 1 rps/s requires 0.01 V output
        m_motorConfig.Slot0.kP = 44; // TODO: A position error of 2.5 rotations results in 12 V output
        m_motorConfig.Slot0.kI = 0; // TODO: no output for integrated error
        m_motorConfig.Slot0.kD = 0; // TODO: A velocity error of 1 rps results in 0.1 V output

        m_motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        m_motorConfig.Slot1.kG = .3; // TODO: Add 0.25 V output to overcome static friction
        m_motorConfig.Slot1.kS = .18;
        m_motorConfig.Slot1.kV = .1; // TODO: A velocity target of 1 rps results in 0.12 V output
        m_motorConfig.Slot1.kA = 10; // TODO: An acceleration of 1 rps/s requires 0.01 V output
        m_motorConfig.Slot1.kP = 25; // TODO: A position error of 2.5 rotations results in 12 V output
        m_motorConfig.Slot1.kI = 0; // TODO: no output for integrated error
        m_motorConfig.Slot1.kD = 0; // TODO: A velocity error of 1 rps results in 0.1 V output

        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_motorConfig.MotionMagic.MotionMagicCruiseVelocity = 80; // Rotations Per second
        m_motorConfig.MotionMagic.MotionMagicAcceleration = 10;
        m_motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;
        // velocity
        m_motorConfig.MotionMagic.MotionMagicExpo_kV = .12; // kV is around 0.12 V/rps
        m_motorConfig.MotionMagic.MotionMagicExpo_kA = .1; // Use a slower kA of 0.1 V/(rps/s)

        m_motorConfig.Feedback.RotorToSensorRatio = 140.0;

        m_motor.getConfigurator().apply(m_motorConfig);

        // TODO encoder and arm angle
        configs_CANdi.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow;
        configs_CANdi.DigitalInputs.S2FloatState = S2FloatStateValue.PullHigh;
        // configs_CANdi.PWM2.AbsoluteSensorDiscontinuityPoint = 0.5;
        configs_CANdi.PWM1.SensorDirection = false;
        configs_CANdi.PWM1.AbsoluteSensorOffset = 0.899688 - .25;

        m_candi.getConfigurator().apply(configs_CANdi);
        m_candi.getS1Closed().setUpdateFrequency(100);
        m_candi.getS2Closed().setUpdateFrequency(100);

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
        // Apply add extra leader configuration on top of the base config
        // - Any control related settings for PID, Motion Magic, Remote Sensors, etc..
        //

        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .26;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -.265;
    }

    public double getArmAngle() {
        return m_candi.getPWM1Position().getValueAsDouble();
    }

    public GeneralArmPosition getArmGeneralPosition(){
        if (getArmAngle() > .2){
            return GeneralArmPosition.blockingVertical;
        }
        else if (getArmAngle() > -.1){
            return GeneralArmPosition.clearOfBottom;
        }
        else if (getArmAngle() > -.25){
            return GeneralArmPosition.blockingDown;
        }
        else{
            return GeneralArmPosition.straightDown;
        }
    }



    public void stop() {
        m_controlMode = ControlMode.kStop;
        demandVoltage = 0.0;
        m_desiredState = 0.0;
    }

    public void setOutputVoltage(double OutputVoltage) {
        m_controlMode = ControlMode.kOpenLoop;
        demandVoltage = OutputVoltage;
    }

    public void setArmAngle(double heightInches) {
        if (m_controlMode != ControlMode.kPID) {
            m_pidController.reset(getArmAngle());
            m_pidLastVelocitySetpoint = 0;
            m_pidLastTime = Timer.getFPGATimestamp();
        }
        m_controlMode = ControlMode.kPID;
        m_desiredState = heightInches;
    }

    public boolean isAtSetPoint(){
        return (Math.abs(m_desiredState - getArmAngle()) < .02);
    }

    //
    // Commands
    //

    public Command setArmStraightUpVertical() {
        return Commands.runOnce(() -> setArmAngle(Constants.ArmConstants.kArmStraightUp), this);
    }

    public Command setArmStraightDownVertical() {
        return Commands.runOnce(() -> setArmAngle(Constants.ArmConstants.kArmStraightDown), this);
    }

    public Command setArmStraightOut() {
        return Commands.runOnce(() -> setArmAngle(Constants.ArmConstants.kArmStraightOut), this);
    }

    @Override
    public void periodic() {
        // TODO add switch to disable break mode when disabled

        SmartDashboard.putString("Mechanism control mode", m_controlMode.toString());
        SmartDashboard.putNumber("Position demand", m_desiredState);
        SmartDashboard.putNumber("Voltage demand", demandVoltage);
        SmartDashboard.putNumber("Arm Angle", getArmAngle());


        switch (m_controlMode) {
            case kOpenLoop:
                // Do openloop stuff here
                m_motor.setVoltage(demandVoltage);
                break;
            case kPID:

                /*double pidVoltage = m_pidController.calculate(getArmAngle(), m_desiredState);
                double dt = Timer.getFPGATimestamp() - m_pidLastTime;
                double accelerationSetpoint = (m_pidController.getSetpoint().velocity - m_pidLastVelocitySetpoint) / dt;
                double feedforwardVoltage = m_feedforward.calculate(m_pidController.getSetpoint().velocity, accelerationSetpoint);
        
                double outputVoltage = pidVoltage + feedforwardVoltage;
                m_motor.setVoltage(outputVoltage);*/

               if (getArmAngle() > m_desiredState + .05) {
                    m_motor.setControl(m_voltage.withPosition(m_desiredState).withSlot(1));
                    SmartDashboard.putBoolean("Is arm using slot 1?", true);
                } else {
                    m_motor.setControl(m_voltage.withPosition(m_desiredState).withSlot(0));
                    SmartDashboard.putBoolean("Is arm using slot 1?", false);
                }

                break;
            case kStop:
                // Fall through to default
            default:
                m_motor.stopMotor();
        }
    }
}