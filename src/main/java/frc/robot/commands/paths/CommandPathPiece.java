package frc.robot.commands.paths;

import edu.wpi.first.wpilibj2.command.*;

public class CommandPathPiece extends Command implements PathPiece {

    /**
     * Is used only needed if interruptsTrajectory is true.
     * 
     * @return the requested speed in meters per second
     */
    public double getRequestedStartSpeed() {
        return 0;
    }

    @Override
    public PieceType getPieceType() {
        return PieceType.Command;
    }

}
