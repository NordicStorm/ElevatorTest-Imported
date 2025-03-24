package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

import java.util.Map;
import static java.util.Map.entry;

import java.io.IOException;

public class AutoScoreSequence extends SequentialCommandGroup {
    Map<Integer, Double> angleMap = Map.ofEntries(
            entry(17, 60.0),
            entry(18, 0.0),
            entry(19, -60.0),
            entry(20, -120.0),
            entry(21, 180.0),
            entry(22, 120.0),

            entry(6, 120.0),
            entry(7, 180.0),
            entry(8, -120.0),
            entry(9, -60.0),
            entry(10, 0.0),
            entry(11, 60.0));
    private boolean invalidTag = false;
    private ProfiledPIDController rotationPID = new ProfiledPIDController(2, 0, 0, new Constraints(9, 2));
    private ProfiledPIDController xPID = new ProfiledPIDController(2, 0, 0, new Constraints(4, 2));
    private ProfiledPIDController yPID = new ProfiledPIDController(2, 0, 0, new Constraints(4, 2));

    public static AprilTagFieldLayout fieldLayout;

    public AutoScoreSequence(Arm arm, Elevator elevator, Wrist wrist, CoralIntake intake,
            CommandSwerveDrivetrain drivetrain, Vision vision) {

        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
        } catch (IOException e) {
        }

        // 1st Strafe rotate and move forwards to align
        // 2nd Raise and rotate all required mechanisms and further align above required
        // alignment
        // 3rd Lower and place the piece down, then back off and return mechanisms to
        // the neutral intake position
        addRequirements(arm, elevator, wrist, intake, drivetrain);
        addCommands(new Command() {
            @Override
            public void initialize() {
            }

            @Override
            public void execute() {
                int seenTagID = vision.seenTagID();
                if (!angleMap.containsKey(seenTagID)) {
                    invalidTag = true;
                    return;
                }
                double rotation = angleMap.get(seenTagID);

                int targetOffset;

                if (RobotContainer.alignmentRight) {
                    targetOffset = 10;
                } else {
                    targetOffset = -10;
                }
                Pose2d relativePose = drivetrain.getPose().rotateAround(
                        fieldLayout.getTagPose(seenTagID).get().getTranslation().toTranslation2d(),
                        Rotation2d.fromDegrees(rotation));

                ChassisSpeeds speeds = new ChassisSpeeds();
                speeds.vyMetersPerSecond = yPID.calculate(relativePose.getY(), targetOffset);

                double distance;

                if (drivetrain.getFrontRangeIsDetected()) {
                    distance = drivetrain.getFrontRange();
                } else {
                    distance = relativePose.getX();
                }
                speeds.vxMetersPerSecond = xPID.calculate(distance, 1);

            }

            @Override
            public boolean isFinished() {
                return invalidTag || true;
            }

            @Override
            public void end(boolean interrupted) {
                intake.stop();
                elevator.setPID(Constants.Position.HOPPER_INTAKE.elevatorPos);
            }
        });
    }

}
