package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class MoveUpperSubsystems extends Command {

    private Position m_position;
    private Arm m_arm;
    private Elevator m_elevator;
    private Wrist m_wrist;

    public MoveUpperSubsystems(Position position, Arm arm, Elevator elevator, Wrist wrist) {
        this.m_position = position;
        this.m_elevator = elevator;
        this.m_arm = arm;
        this.m_wrist = wrist;
        addRequirements(m_arm, m_elevator, m_wrist);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        var constrained = getConstrainedTargetHeight(m_position);
        m_elevator.setPID(constrained.elevatorPos);
        m_arm.setArmAngle(m_position.armAngle);
        m_wrist.setWristTarget(m_position.wristPos);
    }

    @Override
    public boolean isFinished() {
        return m_arm.isAtSetPoint() && m_elevator.isAtSetPoint() && m_wrist.isAtSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
    }

    private Position getConstrainedTargetHeight(Position position) {

        if (position.elevatorPos == Constants.Position.INTERNAL_INTAKE.elevatorPos
                && (m_arm.getArmGeneralPosition() != Arm.GeneralArmPosition.straightDown || !m_wrist.isHorizontal())) {
            return Position.HOPPER_INTAKE;
        }

        else if (position.elevatorPos < Constants.Position.HOPPER_INTAKE.elevatorPos
                && m_arm.getArmGeneralPosition().value < Arm.GeneralArmPosition.clearOfBottom.value) {
            return Position.HOPPER_INTAKE;
        }

        else if (m_arm.getArmGeneralPosition().value == Arm.GeneralArmPosition.blockingVertical.value) {
            return Position.ELEVATOR_ZERO;
        }
        return position;
    }

}
