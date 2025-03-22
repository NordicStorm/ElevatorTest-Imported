package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
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

    public enum Position {
        HOPPER_INTAKE(1, 2, 3, 1),
        GROUND_INTAKE(4, 5, 6, 1),
        STATION_INTAKE(7, 8, 9, 1),
        L1(10, 11, 12, 1),
        L2(13, 14, 15, 1),
        L3(16, 17, 18, 1),
        L4(19, 20, 21, 1);

        public final double armAngle;
        public final double wristPos;
        public final double elevatorPos;
        public final double dist;

        Position(double armAngle, double wristPos, double elevatorPos, double dist) {
            this.armAngle = armAngle;
            this.wristPos = wristPos;
            this.elevatorPos = elevatorPos;
            this.dist = dist;
        }
    }

    //
    // Hardware
    //

    private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
    private final TalonFX m_motor = new TalonFX(MechanismConstants.kArmID, "rio");
    private final MotionMagicExpoVoltage m_voltage = new MotionMagicExpoVoltage(0);



    // TODO find CAN id
    public final CANdi m_candi = new CANdi(0, "rio");
    public final CANdiConfiguration configs_CANdi = new CANdiConfiguration();
    
    //
    // State
    //

    private ControlMode m_controlMode = ControlMode.kStop;
    private double demandVoltage;
    private double m_desiredState;
    

    public Arm() {
        m_motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANdiPWM1;
        //TODO find CAN ID m_motorConfig.Feedback.FeedbackRemoteSensorID();

        m_motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        m_motorConfig.Slot0.kG = 0.325; // TODO: Add 0.25 V output to overcome static friction
        m_motorConfig.Slot0.kS = .18;
        m_motorConfig.Slot0.kV = .12; // TODO: A velocity target of 1 rps results in 0.12 V output
        m_motorConfig.Slot0.kA = 10; // TODO: An acceleration of 1 rps/s requires 0.01 V output
        m_motorConfig.Slot0.kP = 37; // TODO: A position error of 2.5 rotations results in 12 V output
        m_motorConfig.Slot0.kI = 0; // TODO: no output for integrated error
        m_motorConfig.Slot0.kD = 0; // TODO: A velocity error of 1 rps results in 0.1 V output

        m_motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        m_motorConfig.Slot1.kG = .3; // TODO: Add 0.25 V output to overcome static friction
        m_motorConfig.Slot1.kS = .18;
        m_motorConfig.Slot1.kV = .1; // TODO: A velocity target of 1 rps results in 0.12 V output
        m_motorConfig.Slot1.kA = 10; // TODO: An acceleration of 1 rps/s requires 0.01 V output
        m_motorConfig.Slot1.kP = 20; // TODO: A position error of 2.5 rotations results in 12 V output
        m_motorConfig.Slot1.kI = 0; // TODO: no output for integrated error
        m_motorConfig.Slot1.kD = 0; // TODO: A velocity error of 1 rps results in 0.1 V output

        
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotionMagic.MotionMagicCruiseVelocity = 60; // Rotations Per second
        m_motorConfig.MotionMagic.MotionMagicAcceleration = 10;
        // velocity
        m_motorConfig.MotionMagic.MotionMagicExpo_kV = .12; // kV is around 0.12 V/rps
        m_motorConfig.MotionMagic.MotionMagicExpo_kA = .1; // Use a slower kA of 0.1 V/(rps/s)

        m_motorConfig.Feedback.RotorToSensorRatio = 140.0;

        m_motor.getConfigurator().apply(m_motorConfig);

        SmartDashboard.putNumber("Arm kP", m_motorConfig.Slot0.kP);
        SmartDashboard.putNumber("Arm kI", m_motorConfig.Slot0.kI);
        SmartDashboard.putNumber("Arm kD", m_motorConfig.Slot0.kD);
        SmartDashboard.putNumber("Arm kS", m_motorConfig.Slot0.kS);
        SmartDashboard.putNumber("Arm kV", m_motorConfig.Slot0.kV);
        SmartDashboard.putNumber("Arm kA", m_motorConfig.Slot0.kA);
        SmartDashboard.putNumber("Arm mmCV", m_motorConfig.MotionMagic.MotionMagicCruiseVelocity);
        SmartDashboard.putNumber("Arm mmkV", m_motorConfig.MotionMagic.MotionMagicExpo_kV);
        SmartDashboard.putNumber("Arm mmkA", m_motorConfig.MotionMagic.MotionMagicExpo_kA);

        // TODO encoder and arm angle
        configs_CANdi.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow;
        configs_CANdi.DigitalInputs.S2FloatState = S2FloatStateValue.PullHigh;
        //configs_CANdi.PWM2.AbsoluteSensorDiscontinuityPoint = 0.5;
        configs_CANdi.PWM1.SensorDirection = false;
        configs_CANdi.PWM1.AbsoluteSensorOffset = 0.899688-.25;

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

        //m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        //m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 135;
        //m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        //m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = .25;

        // PID Slot 0
        //
        // TODO may to convert gains from Recalc to something in rotations
        // https://www.reca.lc/linear?angle=%7B%22s%22%3A103.582964%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A30%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Kraken%20X60%20%28FOC%29%2A%22%7D&ratio=%7B%22magnitude%22%3A10%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1.751%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A30%2C%22u%22%3A%22in%22%7D

        // We want to read position data from the leader motor
        m_motor.getPosition().setUpdateFrequency(50);
        m_motor.getVelocity().setUpdateFrequency(50);

        // These 3 are needed for the follower motor to work
        m_motor.getDutyCycle().setUpdateFrequency(50);
        m_motor.getMotorVoltage().setUpdateFrequency(50);
        m_motor.getTorqueCurrent().setUpdateFrequency(50);
        m_motor.optimizeBusUtilization();
    }

    public double getArmAngle() {
        return m_candi.getPWM1Position().getValueAsDouble();
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

    private void setArmAngle(double position) {
        m_controlMode = ControlMode.kPID;
        m_desiredState = position;
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

                m_motorConfig.Slot0.kS = SmartDashboard.getNumber("Arm kS", 0);
                m_motorConfig.Slot0.kG = SmartDashboard.getNumber("Arm kG", 0);
                m_motorConfig.Slot0.kV = SmartDashboard.getNumber("Arm kV", 0);
                m_motorConfig.Slot0.kA = SmartDashboard.getNumber("Arm kA", 0);
                m_motorConfig.Slot0.kP = SmartDashboard.getNumber("Arm kP", 0);
                m_motorConfig.Slot0.kI = SmartDashboard.getNumber("Arm kI", 0);
                m_motorConfig.Slot0.kD = SmartDashboard.getNumber("Arm kD", 0);

                m_motor.getConfigurator().apply(m_motorConfig);

                m_motor.setControl(m_voltage.withPosition(m_desiredState));

                break;
            case kStop:
                // Fall through to default
            default:
                m_motor.stopMotor();
        }
    }
}