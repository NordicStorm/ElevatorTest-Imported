package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;
import frc.robot.subsystems.Arm.Position;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;


public class Wrist extends SubsystemBase {

    private enum ControlMode {
        kStop, kOpenLoop, kPID
    }
    //
    // Hardware
    //

    private double m_desiredState;

    private final TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    private final TalonFX m_wristMotor = new TalonFX(MechanismConstants.kWristID);
    private final MotionMagicExpoVoltage m_voltage = new MotionMagicExpoVoltage(0);

    //
    // State
    //
    private double m_Demand;
    private ControlMode controlMode = ControlMode.kStop;

    public Wrist() {
        wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANdiPWM2;
        // TODO find CAN ID wristConfig.Feedback.FeedbackRemoteSensorID();

        wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        wristConfig.Slot0.kS = 0.11; // TODO: Add 0.25 V output to overcome static friction
        wristConfig.Slot0.kV = 2.77; // TODO: A velocity target of 1 rps results in 0.12 V output
        wristConfig.Slot0.kA = 0.0113116; // TODO: An acceleration of 1 rps/s requires 0.01 V output
        wristConfig.Slot0.kP = 30; // TODO: A position error of 2.5 rotations results in 12 V output
        wristConfig.Slot0.kI = 0.1; // TODO: no output for integrated error
        wristConfig.Slot0.kD = 0.52411; // TODO: A velocity error of 1 rps results in 0.1 V output

        // Motion Magic
        wristConfig.MotionMagic.MotionMagicCruiseVelocity = 8.0; // Rotations Per second
        wristConfig.MotionMagic.MotionMagicAcceleration = 8.0; // Acceleration Rotations per second^2
        // wristConfig.MotionMagic.MotionMagicCruiseVelocity = 0; // Unlimited cruise
        // velocity
        wristConfig.MotionMagic.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        wristConfig.MotionMagic.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

        m_wristMotor.getConfigurator().apply(wristConfig);



    }

    public void stop() {
        m_Demand = 0.0;
        controlMode = ControlMode.kStop;
    }

    public void setWristRotation(Position position) {
        m_desiredState = position.wristPos;
        controlMode = ControlMode.kPID;

    }

    public void setIntakeVoltage(double IntakeVoltage) {
        m_Demand = IntakeVoltage;
        controlMode = ControlMode.kOpenLoop;
    }

    //TODO see if the absolute encoder is actually being used
    public double getWristRotation() {
        return m_wristMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        switch (controlMode) {
            case kOpenLoop:
                m_wristMotor.set(m_Demand);
                break;
            case kPID:
                m_wristMotor.setControl(m_voltage.withPosition(m_desiredState));
                break;
            case kStop:
            default:
                m_wristMotor.stopMotor();
        }
        SmartDashboard.putNumber("Wrist Position", m_wristMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Wrist Desired State", m_desiredState);
    }
}
