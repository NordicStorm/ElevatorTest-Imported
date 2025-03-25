package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

import java.util.Map;
import static java.util.Map.entry;

import java.io.IOException;

public class AutoScoreSequence extends SequentialCommandGroup implements CommandPathPiece {
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
    private boolean invalidTag;
    private int targetTagID;

    private ProfiledPIDController rotationPID;
    private ProfiledPIDController xPID;
    private ProfiledPIDController yPID;
    private SimpleMotorFeedforward motorFF;

    private double xDistance = 999;
    private double yError = 999;
    private double yOffset;

    private boolean isFirstTime;
    private Constants.Position level;

    public static AprilTagFieldLayout fieldLayout;

    public AutoScoreSequence(Arm arm, Elevator elevator, Wrist wrist, CoralIntake intake,
            CommandSwerveDrivetrain drivetrain, Vision vision, boolean hopperImmediately,
            int forcedTagID) {

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
            private boolean done = false;

            @Override
            public void initialize() {
                targetTagID = forcedTagID;
                done = false;
                xDistance = 999;
                yError = 999;
                rotationPID = new ProfiledPIDController(10, 0, 0, new Constraints(400, 300));
                xPID = new ProfiledPIDController(5, 0, 0, new Constraints(4, 2));
                yPID = new ProfiledPIDController(5, 0, 0, new Constraints(4, 2));
                rotationPID.enableContinuousInput(-180, 180);
                rotationPID.reset(drivetrain.getGyroDegrees());
                isFirstTime = true;
                yOffset = 0;
                level = RobotContainer.targetLevel;
            }

            @Override
            public void execute() {
                int seenTagID = vision.seenTagID();
                if (targetTagID == 0 && !angleMap.containsKey(seenTagID)) {
                    AutoScoreSequence.this.cancel();
                    return;
                } else if (seenTagID != 0 && angleMap.containsKey(seenTagID)) {
                    targetTagID = seenTagID;
                }

                doMovement(.8, drivetrain, vision);
                SmartDashboard.putNumber("xdist", xDistance);
                SmartDashboard.putNumber("yError", yError);
                if (xDistance <= 1 && Math.abs(yError) < .01) {
                    done = true;
                }
            }

            @Override
            public boolean isFinished() {
                return done;
            }

            @Override
            public void end(boolean interrupted) {
            }
        });

        addCommands(new Command() {
            private boolean done = false;

            @Override
            public void initialize() {
                done = false;
            }

            @Override
            public void execute() {
                doMovement(.4, drivetrain, vision);

                if (xDistance <= .45 && Math.abs(yError) < .01) {
                    done = true;
                }
            }

            @Override
            public boolean isFinished() {
                return done;
            }

            @Override
            public void end(boolean interrupted) {
                drivetrain.drive(new ChassisSpeeds());
            }
        }.alongWith(new MoveUpperSubsystems(() -> level, arm, elevator, wrist)));

        addCommands(new Command() {

            @Override
            public void execute() {
                doMovement(level.dist, drivetrain, vision);
            }

            @Override
            public boolean isFinished() {
                return Math.abs(xDistance - level.dist) < .02;
            }

            @Override
            public void end(boolean interrupted) {
                drivetrain.drive(new ChassisSpeeds());
            }
        });

        addCommands(new Command() {

            long timeToEnd;
            long timeToSpin;

            @Override
            public void initialize() {
                timeToEnd = 0;
                timeToSpin = System.currentTimeMillis() + 300;
            }

            @Override
            public void execute() {
                if (level == Constants.Position.L1) {
                    intake.setIntakeVoltage(1);
                } else if (level == Constants.Position.L2 || level == Constants.Position.L3) {
                    intake.setIntakeVoltage(.9);
                    arm.setArmAngle(Constants.ArmConstants.kArmAfterMiddleCoralOutake);
                    elevator.setPID(level.elevatorPos - 1);
                } else if (level == Constants.Position.L4) {
                    arm.setArmAngle(0.04);
                    if (arm.isAtSetPoint() || timeToSpin < System.currentTimeMillis()) {
                        intake.setIntakeVoltage(.9);
                        elevator.setPID(level.elevatorPos - 2);
                        drivetrain.drive(new ChassisSpeeds(-0.5, 0, 0));
                    }
                }
                if (arm.isAtSetPoint() && timeToEnd == 0) {
                    timeToEnd = System.currentTimeMillis() + 200;
                }
            }

            @Override
            public boolean isFinished() {
                return timeToEnd != 0 && System.currentTimeMillis() >= timeToEnd && !intake.hasCoral();
            }

            @Override
            public void end(boolean interrupted) {
                drivetrain.drive(new ChassisSpeeds());
                intake.setIntakeVoltage(0);
            }

        });

        addCommands(new Command() {

            @Override
            public void execute() {
                drivetrain.drive(new ChassisSpeeds(-1, 0, 0));
            }

            @Override
            public boolean isFinished() {
                if (level == Constants.Position.L4 && RobotContainer.rakeAlgae > 0) {
                    return drivetrain.getFrontRange() > 0.5;
                } else {
                    return drivetrain.getFrontRange() > 0.5;
                }
            }

            @Override
            public void end(boolean interrupted) {
                drivetrain.drive(new ChassisSpeeds());
            }
        });
        addCommands(new ConditionalCommand((new Command() {

            long timeToEnd;

            @Override
            public void initialize() {
                timeToEnd = System.currentTimeMillis() + 500;
            }

            @Override
            public void execute() {
                if (RobotContainer.alignmentRight) {
                    drivetrain.drive(new ChassisSpeeds(0, 1, 0));
                } else {
                    drivetrain.drive(new ChassisSpeeds(0, -1, 0));
                }
            }

            @Override
            public boolean isFinished() {
                return System.currentTimeMillis() >= timeToEnd;
            }

        }.alongWith(new MoveUpperSubsystems(() -> RobotContainer.rakeAlgae == 1 ? Constants.Position.BEFORE_LOWER_ALGAE
                : Constants.Position.BEFORE_UPPER_ALGAE, arm, elevator, wrist))).andThen(new Command() {
                    @Override
                    public void execute() {
                        drivetrain.drive(new ChassisSpeeds(1, 0, 0));
                    }

                    @Override
                    public boolean isFinished() {
                        return drivetrain.getFrontRange() < .1;
                    }

                })
                .andThen(new MoveUpperSubsystems(
                        () -> RobotContainer.rakeAlgae == 1 ? Constants.Position.AFTER_LOWER_ALGAE
                                : Constants.Position.AFTER_UPPER_ALGAE,
                        arm, elevator, wrist)
                        .alongWith(new Command() {
                            @Override
                            public void execute() {
                                drivetrain.drive(new ChassisSpeeds(-1, 0, 0));
                            }

                            @Override
                            public boolean isFinished() {
                                return drivetrain.getFrontRange() > .3;
                            }
                        })

                ), (new MoveUpperSubsystems(() -> Constants.Position.ELEVATOR_ZERO, arm, elevator, wrist)
                        .andThen(new MoveUpperSubsystems(() -> Constants.Position.HOPPER_INTAKE, arm, elevator, wrist)))
                        .unless(() -> !hopperImmediately),
                () -> level == Constants.Position.L4 && RobotContainer.rakeAlgae > 0));
    }

    private void doMovement(double targetDistance, CommandSwerveDrivetrain drivetrain, Vision vision) {
        if (targetTagID == 0) {
            return;
        }

        double rotation = angleMap.get(targetTagID);

        double targetOffset;

        if (RobotContainer.alignmentRight) {
            targetOffset = -Constants.kSideOffset;
        } else {
            targetOffset = Constants.kSideOffset;
        }

        Pose2d relativePose = drivetrain.getPose().relativeTo(fieldLayout.getTagPose(targetTagID).get().toPose2d());

        ChassisSpeeds speeds = new ChassisSpeeds();

        SmartDashboard.putNumber("xrel", relativePose.getX());
        SmartDashboard.putNumber("yrel", relativePose.getY());

        if (drivetrain.getFrontRangeIsDetected()) {
            xDistance = drivetrain.getFrontRange();
        } else {
            xDistance = relativePose.getX() - .8125 / 2.0;
        }
        if (vision.seenTagID() != 0) {
            yOffset = -Math.tan(Math.toRadians(vision.getXOffset())) * xDistance - .184;
        } else {
            yOffset = targetOffset;
        }
        if (isFirstTime) {
            yPID.reset(yOffset);
            xPID.reset(xDistance);
            isFirstTime = false;
        }
        SmartDashboard.putNumber("Real X distance", xDistance);
        speeds.vxMetersPerSecond = -xPID.calculate(xDistance, targetDistance);
        // speeds.vxMetersPerSecond += Math.signum(speeds.vxMetersPerSecond) * 0.1;

        speeds.vyMetersPerSecond = -yPID.calculate(yOffset, targetOffset);
        // speeds.vyMetersPerSecond += Math.signum(speeds.vyMetersPerSecond) * 0.1;
        yError = yOffset - targetOffset;

        speeds.omegaRadiansPerSecond = Math.toRadians(rotationPID.calculate(drivetrain.getGyroDegrees(), rotation));

        drivetrain.drive(speeds);
    }

}
