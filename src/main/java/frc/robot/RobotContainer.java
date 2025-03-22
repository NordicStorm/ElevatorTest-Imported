// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Climber;


// import frc.robot.subsystems.Intake;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import static edu.wpi.first.units.Units.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Elevator m_elevator = new Elevator();
  private final Arm m_arm = new Arm();
  private final Wrist m_wrist = new Wrist();
  private final CoralIntake m_CoralIntake = new CoralIntake();
  private final Climber m_climber = new Climber();
  

  // Replace with CommandPS4Controller or Commandm_driverController if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * .25; // TODO we changed value
                                                                                      // kSpeedAt12Volts desired top
                                                                                      // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {
    SignalLogger.setPath("/media/sda1/ctre-logs/");
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.Commandm_driverController Flight
   * m_driverControllers}.
   */
  private void configureBindings() {

    //m_driverController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    //m_driverController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    
    //m_driverController Y = quasistatic forward
    //m_driverController A = quasistatic reverse
    //m_driverController B = dynamic forward
    //m_driverController X = dyanmic reverse
    
    m_driverController.y().whileTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController.a().whileTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController.b().whileTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController.x().whileTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putNumber("Arm test setpoint", 0);
    //m_driverController.leftTrigger().whileTrue(m_elevator.armAngleCommand(()->
    //SmartDashboard.getNumber("Arm test setpoint", 0)));

    m_driverController.back().whileTrue(m_elevator.homeCommand());
    //m_driverController.povUp().whileTrue(m_elevator.openLoopCommand(2));
    //m_driverController.povDown().whileTrue(m_elevator.openLoopCommand(-2));
   

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward
                                                                                                     // with negative Y
                                                                                                     // (forward)
            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
        ));

    //m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //m_driverController.b().whileTrue(drivetrain
    //    .applyRequest(() -> point
    //        .withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);


    m_driverController.rightTrigger().whileTrue(m_climber.openLoopClimbCommand(.25));
    m_driverController.leftTrigger().whileTrue(m_climber.openLoopClimbCommand(-.25));
    //m_driverController.rightBumper().whileTrue(m_CoralIntake.openLoopIntakeCommand(-.25)); //Intake
    //m_driverController.leftBumper().whileTrue(m_CoralIntake.openLoopIntakeCommand(1)); //Outake
    m_driverController.povUp().onTrue(m_wrist.setWristHorizontal()); 
    m_driverController.povDown().onTrue(m_wrist.setWristVertical());
    //m_driverController.rightTrigger().whileTrue(m_elevator.pidCommand(10));
    //m_driverController.leftTrigger().whileTrue(m_elevator.pidCommand(30));
    SmartDashboard.putNumber("Elevator test setpoint", 0);
    m_driverController.povRight().whileTrue(m_elevator.pidCommand(20));
    m_driverController.povLeft().whileTrue(m_elevator.pidCommand(40));
    //m_driverController.povLeft().onTrue(m_arm.setArmStraightDownVertical());
    //m_driverController.povRight().onTrue(m_arm.setArmStraightUpVertical());
  }



  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}