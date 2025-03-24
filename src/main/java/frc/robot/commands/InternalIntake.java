package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class InternalIntake extends SequentialCommandGroup{
    
    public InternalIntake(Arm arm, Elevator elevator, Wrist wrist, CoralIntake intake){
        addCommands(new MoveUpperSubsystems(Position.HOPPER_INTAKE, arm, elevator, wrist));
        addCommands(new Command(){
            @Override
            public void initialize() {
                intake.setIntakeVoltage(-.75);
                elevator.setPID(Constants.Position.INTERNAL_INTAKE.elevatorPos);
            }
            @Override
            public boolean isFinished() {
                return intake.hasCoral();
            }
            @Override
            public void end(boolean interrupted) {
                intake.stop();
                elevator.setPID(Constants.Position.HOPPER_INTAKE.elevatorPos);
            }
        });
    }
    
}
