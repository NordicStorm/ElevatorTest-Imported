package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class MoveUpperSubsystems extends Command implements CommandPathPiece{

    private Supplier<Position> m_positionSupplier;
    private Arm m_arm;
    private Elevator m_elevator;
    private Wrist m_wrist;

    public MoveUpperSubsystems(Supplier<Position> position, Arm arm, Elevator elevator, Wrist wrist) {
        this.m_positionSupplier = position;
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
        var currentTarget = m_positionSupplier.get();
        var constrained = getConstrainedTargetHeight(currentTarget);
        m_elevator.setPID(constrained.elevatorPos);
        m_arm.setArmAngle(getConstrainedTargetAngle(currentTarget.armAngle));
        m_wrist.setWristTarget(currentTarget.wristPos);
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

    private double getConstrainedTargetAngle(double targetAngle){
        if (m_elevator.getHeight() < Constants.Position.L1.elevatorPos - 0.5 && m_arm.getArmAngle() < 0){
            return 0;
        }
        return targetAngle;
    }

}
