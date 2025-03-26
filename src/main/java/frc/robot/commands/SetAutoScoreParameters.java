package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.paths.CommandPathPiece;

public class SetAutoScoreParameters extends InstantCommand implements CommandPathPiece{
    public SetAutoScoreParameters(Constants.Position position, boolean isLeft, int whichAlgae){
        super(()-> {
            RobotContainer.targetLevel = position; 
            RobotContainer.alignmentLeft = isLeft;
            RobotContainer.rakeAlgae = whichAlgae;
        });
    }
}
