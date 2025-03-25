
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    pathA.resetPosition(8.567, 5.938);
    pathA.setHeading(60);
    pathA.addWaypoint(5.950, 5.632);
    pathA.addSequentialCommand(new AutoScoreSequence(m_arm, m_elevator, m_wrist, m_intake, m_drivetrain, m_vision));
    pathA.addWaypoint(3.929, 6.519);
    pathA.addStop();

    addCommands(pathA.finalizePath());
  }
}