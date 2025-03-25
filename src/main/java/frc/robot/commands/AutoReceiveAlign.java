package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;

public class AutoReceiveAlign extends SequentialCommandGroup implements CommandPathPiece{

    public AutoReceiveAlign(ProfiledPIDController rotationController, CommandSwerveDrivetrain drivetrain, CoralIntake intake){
        addCommands(new Command(){
           
            @Override
            public void execute(){
                drivetrain.drive(new ChassisSpeeds(1, 0, rotationController.calculate(drivetrain.getGyroRadians())));
            }

            @Override
            public boolean isFinished(){
                return drivetrain.getBackRange() < .5;
            }

            @Override
            public void end(boolean interrupted) {
                drivetrain.drive(new ChassisSpeeds());
                drivetrain.setBlinkinPark();
            }
        }.andThen(new WaitUntilCommand(() -> intake.isInTrough() || intake.hasCoral())));
    }

    @Override
    public double getRequestedStartSpeed() {
        return 1;
    }
    
}
