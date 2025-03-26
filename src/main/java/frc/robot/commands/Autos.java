
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.commands.paths.MultiPartPath;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public final class Autos extends SequentialCommandGroup {

    private CommandSwerveDrivetrain m_drivetrain;
    private Arm m_arm;
    private Elevator m_elevator;
    private Wrist m_wrist;
    private Vision m_vision;
    private CoralIntake m_intake;

    public Autos(CommandSwerveDrivetrain drivetrain, Wrist wrist, Arm arm, Elevator elevator, Vision vision,
            CoralIntake intake) {
        m_drivetrain = drivetrain;
        m_wrist = wrist;
        m_arm = arm;
        m_elevator = elevator;
        m_vision = vision;
        m_intake = intake;
        initializeCommands();
    }

    static SendableChooser<String> chooser = new SendableChooser<String>();

    public static void putToDashboard() {
        SmartDashboard.putBoolean("First Coral Left?", false);
        
        chooser.addOption("Left", "Left");
        chooser.addOption("Center", "Center");
        chooser.addOption("Right", "Right");
        SmartDashboard.putData(chooser);
    }

    public void initializeCommands() {
        // !PATHWEAVER_INFO: {"trackWidth":0.962025,"gameName":"Reefscape"}
        MultiPartPath pathA;
        pathA = new MultiPartPath(m_drivetrain);
        boolean isBlue = DriverStation.getAlliance().get() == Alliance.Blue;
        m_drivetrain.resetRotation(m_drivetrain.getOperatorForwardDirection().plus(Rotation2d.fromDegrees(180)));
        int firstTagID;
        int secondTagID;
        double feederStationAngle;

        boolean isLeft = chooser.getSelected().equals("Left");
        boolean isCenter = chooser.getSelected().equals("Center");
        boolean isRight = chooser.getSelected().equals("Right");

        boolean firstCoralLeft = SmartDashboard.getBoolean("First Coral Left?", false);

        String startingPosition = chooser.getSelected();
        if (isLeft || isRight) { // path on
            pathA.resetPosition(7.620, 1.854); 

            if (isRight) {
                firstTagID = isBlue ? 22 : 9;
                secondTagID = isBlue ? 17 : 8;
                feederStationAngle = isBlue ? 54 : 234;
            } else {
                firstTagID = isBlue ? 20 : 11;
                secondTagID = isBlue ? 19 : 6;
                feederStationAngle = isBlue ? -54 : 126;
            }

            pathA.addParallelCommand(new SetAutoScoreParameters(Constants.Position.L4, firstCoralLeft, 0));
            pathA.setHeading(AutoScoreSequence.angleMap.get(firstTagID));
            pathA.addWaypoint(6.302, 2.249);
            pathA.addSequentialCommand(new AutoScoreSequence(m_arm, m_elevator, m_wrist, m_intake, m_drivetrain, m_vision, true, firstTagID));// ENDPOS:5.383,2.953
            pathA.setHeading(feederStationAngle);
            pathA.addWaypoint(4.893, 2.187);
            for (int i = 0; i < 2; i++) { // path on
                pathA.addParallelCommand(new SetAutoScoreParameters(Constants.Position.L4, i == 1, 0));
                pathA.addWaypoint(2.536, 1.927);
                pathA.addSequentialCommand(new AutoReceiveAlign(feederStationAngle, m_drivetrain, m_intake));// ENDPOS:1.235,0.948
                pathA.addWaypoint(2.689, 2.004);
                pathA.addSequentialCommand(new AutoScoreSequence(m_arm, m_elevator, m_wrist, m_intake, m_drivetrain, m_vision, true, secondTagID));// ENDPOS:3.699,2.922
            }

        } else { // path off
            if (isBlue) {
                firstTagID = 21;
            }
            else {
                firstTagID = 10;
            }

            pathA.resetPosition(7.620, 3.810);
            pathA.addParallelCommand(new SetAutoScoreParameters(Constants.Position.L4, firstCoralLeft, 0));
            pathA.addSequentialCommand(new AutoScoreSequence(m_arm, m_elevator, m_wrist, m_intake, m_drivetrain, m_vision, false, firstTagID));// ENDPOS:5.888,3.902
            pathA.addParallelCommand(new MoveUpperSubsystems(() -> Position.HOPPER_INTAKE, m_arm, m_elevator, m_wrist));
        }

        pathA.addStop();

        if (isBlue) {
            if(isLeft) {
                pathA.flipAllY();
            }
        } else {
            pathA.flipAllX();
            if(isRight) {
                pathA.flipAllY();
            }
        }
        addCommands(pathA.finalizePath());
    }
}