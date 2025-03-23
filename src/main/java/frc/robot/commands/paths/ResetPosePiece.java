package frc.robot.commands.paths;

import edu.wpi.first.math.geometry.Pose2d;

public class ResetPosePiece extends CommandPathPiece{
    MultiPartPath path;
    Pose2d pose;
    public ResetPosePiece(MultiPartPath path, Pose2d pose){
        this.path = path;
        this.pose = pose;
    }
    @Override
    public void initialize() {
        path.drivetrain.setPose(pose);
        
    }

    public Pose2d getPose(){
        return pose;
    }

    public void setPose(Pose2d pose){
        this.pose = pose;
    }
    
}
