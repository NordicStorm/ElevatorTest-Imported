
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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

public final class Autos extends SequentialCommandGroup{

  private CommandSwerveDrivetrain m_drivetrain;
  private Arm m_arm;
  private Elevator m_elevator;
  private Wrist m_wrist;
  private Vision m_vision;
  private CoralIntake m_intake;

  public Autos(CommandSwerveDrivetrain drivetrain, Wrist wrist, Arm arm, Elevator elevator, Vision vision, CoralIntake intake) {
    m_drivetrain = drivetrain;
    m_wrist = wrist;
    m_arm = arm;
    m_elevator = elevator;
    m_vision = vision;
    m_intake = intake;
    initializeCommands();
  }

  public void initializeCommands(){ 
// !PATHWEAVER_INFO: {"trackWidth":0.962025,"gameName":"Reefscape"}
    MultiPartPath pathA;
    pathA = new MultiPartPath(m_drivetrain);
    pathA.addParallelCommand(new SetAutoScoreParameters(Constants.Position.L4, false, 0));
    pathA.resetPosition(7.863, 1.897);
    pathA.setHeading(60);
    pathA.addWaypoint(6.302, 2.249);
    pathA.addSequentialCommand(new AutoScoreSequence(m_arm, m_elevator, m_wrist, m_intake, m_drivetrain, m_vision, false, 22));// ENDPOS:5.383,2.953
    pathA.addParallelCommand(new MoveUpperSubsystems(() -> Position.HOPPER_INTAKE, m_arm, m_elevator, m_wrist));
    pathA.addWaypoint(4.893, 2.187);
    pathA.setHeading(126);
    pathA.addWaypoint(2.858, 1.407);
    pathA.addSequentialCommand(new AutoReceiveAlign(pathA.getRotationController(), m_drivetrain, m_intake));// ENDPOS:1.235,0.948
    pathA.addWaypoint(2.689, 2.004);
    pathA.addSequentialCommand(new AutoScoreSequence(m_arm, m_elevator, m_wrist, m_intake, m_drivetrain, m_vision, false, 17));// ENDPOS:3.699,2.922
    pathA.addWaypoint(1.388, 3.932);
    pathA.addParallelCommand(new MoveUpperSubsystems(() -> Position.HOPPER_INTAKE, m_arm, m_elevator, m_wrist));
    pathA.addStop();

    addCommands(pathA.finalizePath());
  }
}