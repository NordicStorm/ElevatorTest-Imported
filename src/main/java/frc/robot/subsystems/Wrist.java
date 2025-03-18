package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MechanismConstants;
import frc.robot.subsystems.Arm.Position;

import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;


public class Wrist extends SubsystemBase {

    private enum ControlMode {
        kStop, kOpenLoop, kPID
    }
    //
    // Hardware
    //

    private double m_desiredState;

    private final TalonFXSConfiguration wristConfig = new TalonFXSConfiguration();
    private final TalonFXS m_wristMotor = new TalonFXS(MechanismConstants.kWristID);
    private final PositionVoltage m_voltage = new PositionVoltage(0);

    //
    // State
    //
    private double m_Demand;
    private ControlMode controlMode = ControlMode.kStop;

    public Wrist() {
        wristConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANdiPWM2;
        // TODO find CAN ID wristConfig.Feedback.FeedbackRemoteSensorID();
        
        wristConfig.Slot0.kP = 85; // TODO: A position error of 2.5 rotations results in 12 V output
        wristConfig.Slot0.kI = 0; // TODO: no output for integrated error
        wristConfig.Slot0.kD = 5; // TODO: A velocity error of 1 rps results in 0.1 V output
        wristConfig.Slot0.kS = .75;
        wristConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        wristConfig.Commutation.MotorArrangement = MotorArrangementValue.Brushed_DC;
        wristConfig.Commutation.BrushedMotorWiring = BrushedMotorWiringValue.Leads_A_and_B;

        m_wristMotor.getConfigurator().apply(wristConfig);
    }

    public void stop() {
        m_Demand = 0.0;
        controlMode = ControlMode.kStop;
    }

    private void setWristTarget(double position){
        m_desiredState = position;
        controlMode = ControlMode.kPID;
    }

    public Command setWristHorizontal() {
        return Commands.runOnce(() -> setWristTarget(Constants.MechanismConstants.kWristHorizontalPos), this);
    }

    public Command setWristVertical(){
        return Commands.runOnce(() -> setWristTarget(Constants.MechanismConstants.kWristVerticalPos), this);

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
