
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.paths.MultiPartPath;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public final class Autos extends SequentialCommandGroup{

  private CommandSwerveDrivetrain m_drivetrain;

  public Autos(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    initializeCommands();
  }

  public void initializeCommands(){ 
// !PATHWEAVER_INFO: {"trackWidth":0.962025,"gameName":"Reefscape"}
    MultiPartPath pathA;
    pathA = new MultiPartPath(m_drivetrain);
    pathA.resetPosition(8.567, 5.938);
    pathA.addWaypoint(6.000, 6.000);
    pathA.addStop();

    addCommands(pathA.finalizePath());
  }
}