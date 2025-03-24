package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {

    @Override
    public void periodic() {
        // First, tell Limelight your robot's current orientation
        double robotYaw = RobotContainer.drivetrain.getGyroDegrees();
        LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Get the pose estimate
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
        // Add it to your pose estimator
        if (limelightMeasurement.tagCount >= 1) {
            RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.9, .9, 9999999));
            RobotContainer.drivetrain.addVisionMeasurement(
                    limelightMeasurement.pose,
                    limelightMeasurement.timestampSeconds);
        }

    }

    public int seenTagID() {
        return (int) LimelightHelpers.getFiducialID("");
    }

}
