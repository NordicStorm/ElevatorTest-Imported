
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

  public Autos(CommandSwerveDrivetrain drivetrain) {
    MultiPartPath pathA;
    pathA = new MultiPartPath(drivetrain);
    pathA.resetPosition(5, 5);
    pathA.addWaypoint(6, 6);
    pathA.addStop();

    addCommands(pathA.finalizePath());
  }
}
