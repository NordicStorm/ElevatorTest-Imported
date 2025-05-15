// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoReceiveAlign;
import frc.robot.commands.AutoScoreSequence;
import frc.robot.commands.Autos;
import frc.robot.commands.InternalIntake;
import frc.robot.commands.MoveUpperSubsystems;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.GeneralArmPosition;
import frc.robot.subsystems.Climber;

// import frc.robot.subsystems.Intake;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private final Vision m_vision = new Vision();
  private Autos m_autos;

  public static boolean alignmentLeft = true; // True is right, False is left
  public static Constants.Position targetLevel = Constants.Position.L4;
  public static int algaeMode = 0; // 0 is none, 1 is low algae, 2 is high algae
  public static boolean isCoralMode = true;
  public static  boolean isSecondControllerActive = false;

  // Replace with CommandPS4Controller or Commandm_driverController if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_secondController = new CommandXboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * .55; // TODO we changed value
                                                                                      // kSpeedAt12Volts desired top
                                                                                      // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {
    SignalLogger.setPath("/media/sda1/ctre-logs/");
    // SignalLogger.start();
    Autos.putToDashboard();
    configureBindings();
    SmartDashboard.putBoolean("Is auto initialized?", false);
    SmartDashboard.putData("Set Auto", new InstantCommand(() -> {
      m_autos = new Autos(drivetrain, m_wrist, m_arm, m_elevator, m_vision, m_CoralIntake);
      SmartDashboard.putBoolean("Is auto initialized?", m_autos.isInitialized);
    }).ignoringDisable(true));
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
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(MaxSpeed * -m_driverController.getLeftY()) // Drive forward
                                                                                                     // with negative Y
                                                                                                     // (forward)
            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
        ));
    new Trigger(() -> m_CoralIntake.isInTrough() && m_arm.isAtSetPoint() //Auto trigger for grabbing the piece when it is in the trough
        && m_arm.getArmGeneralPosition() == GeneralArmPosition.straightDown
        && m_wrist.isHorizontal() && m_elevator.isAtGrabHeight()).debounce(.5)
        .onTrue(new InternalIntake(m_arm, m_elevator, m_wrist, m_CoralIntake));

    // reset the field-centric heading on left bumper press
    m_driverController.leftBumper().and(m_driverController.rightBumper()) //Reset Gyro
        .onTrue(drivetrain.runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero)));


    m_driverController.leftTrigger().and(() -> !isCoralMode).whileTrue(m_climber.openLoopClimbCommand(.5)); //Climber commands for when Climb mode is active
    m_driverController.rightTrigger().and(() -> !isCoralMode).whileTrue(m_climber.openLoopClimbCommand(-.4));

    m_driverController.leftTrigger().and(() -> isCoralMode).onTrue(new InstantCommand(() -> algaeMode = 0)).whileTrue(new InstantCommand(() -> alignmentLeft = true) //Auto score right/left
        .andThen(new AutoScoreSequence(m_arm, m_elevator, m_wrist, m_CoralIntake, drivetrain, m_vision, true, 0)));
    m_driverController.rightTrigger().and(() -> isCoralMode).onTrue(new InstantCommand(() -> algaeMode = 0)).whileTrue(new InstantCommand(() -> alignmentLeft = false)
        .andThen(new AutoScoreSequence(m_arm, m_elevator, m_wrist, m_CoralIntake, drivetrain, m_vision, true, 0)));


    m_driverController.leftBumper().and(() -> isCoralMode).onTrue(new InstantCommand(() -> algaeMode = 1)).whileTrue(new InstantCommand(() -> alignmentLeft = true) //Auto score right/left
        .andThen(new AutoScoreSequence(m_arm, m_elevator, m_wrist, m_CoralIntake, drivetrain, m_vision, true, 0)));
    m_driverController.rightBumper().and(() -> isCoralMode).onTrue(new InstantCommand(() -> algaeMode = 2)).whileTrue(new InstantCommand(() -> alignmentLeft = false)
        .andThen(new AutoScoreSequence(m_arm, m_elevator, m_wrist, m_CoralIntake, drivetrain, m_vision, true, 0)));



    m_driverController.povUp().onTrue(new InstantCommand(() -> targetLevel = Constants.Position.L1)); //Up = 1
    m_driverController.povRight().onTrue(new InstantCommand(() -> targetLevel = Constants.Position.L2)); //Right = 2
    m_driverController.povDown().onTrue(new InstantCommand(() -> targetLevel = Constants.Position.L3)); //Down = 3
    m_driverController.povLeft().onTrue(new InstantCommand(() -> targetLevel = Constants.Position.L4)); //Left = 4

    m_driverController.y()
        .onTrue(new MoveUpperSubsystems(() -> Constants.Position.HOPPER_INTAKE, m_arm, m_elevator, m_wrist)); //Hopper position on y press
    m_driverController.a().whileTrue(new AutoReceiveAlign(0, drivetrain, m_CoralIntake)); //Align with feeder station
    m_driverController.b().onTrue(new MoveUpperSubsystems(() -> Constants.Position.GROUND_INTAKE, m_arm, m_elevator, m_wrist)); //Ground intake on b press

    m_driverController.start() //Elevator zero on start when in coral mode
    .and(() -> isCoralMode)
    .onTrue(new MoveUpperSubsystems(() -> Constants.Position.ELEVATOR_ZERO, m_arm, m_elevator, m_wrist).andThen(new MoveUpperSubsystems(() -> Constants.Position.HOPPER_INTAKE, m_arm, m_elevator, m_wrist)));
    m_driverController.start() //Set to Climber position when in climb mode
    .and(() -> !isCoralMode)
    .onTrue(new MoveUpperSubsystems(() -> Constants.Position.CLIMBER, m_arm, m_elevator, m_wrist));

    m_driverController.back().onTrue(new InstantCommand(() -> isCoralMode = !isCoralMode)); //Switch coral/climb



    //m_driverController.rightBumper()
      //  .whileTrue(Commands.runEnd(() -> MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * .75,
        //    () -> MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * .55));
    //m_driverController.rightBumper().onTrue(new MoveUpperSubsystems(() -> targetLevel, m_arm, m_elevator, m_wrist));


    m_secondController.y().and(() -> isSecondControllerActive).whileTrue(m_elevator.openLoopCommand(() -> 1));
    m_secondController.a().and(() -> isSecondControllerActive).whileTrue(m_elevator.openLoopCommand(() -> -1));
    
    m_secondController.start().onTrue(new InstantCommand(() -> isSecondControllerActive = !isSecondControllerActive));

    m_secondController.povUp().and(() -> isSecondControllerActive).whileTrue(new MoveUpperSubsystems(() -> Constants.Position.L1, m_arm, m_elevator, m_wrist));
    m_secondController.povRight().and(() -> isSecondControllerActive).whileTrue(new MoveUpperSubsystems(() -> Constants.Position.L2, m_arm, m_elevator, m_wrist));
    m_secondController.povDown().and(() -> isSecondControllerActive).whileTrue(new MoveUpperSubsystems(() -> Constants.Position.L3, m_arm, m_elevator, m_wrist));
    m_secondController.povLeft().and(() -> isSecondControllerActive).whileTrue(new MoveUpperSubsystems(() -> Constants.Position.L4, m_arm, m_elevator, m_wrist));

    m_secondController.leftBumper().and(() -> isSecondControllerActive).whileTrue(Commands.runEnd(() -> m_CoralIntake.openLoopIntakeCommand(-.75), () -> m_CoralIntake.stop()));
    m_secondController.rightBumper().and(() -> isSecondControllerActive).whileTrue(Commands.runEnd(() -> m_CoralIntake.openLoopIntakeCommand(.9), () -> m_CoralIntake.stop()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    if (m_autos == null) {
      m_autos = new Autos(drivetrain, m_wrist, m_arm, m_elevator, m_vision, m_CoralIntake);
    }
    return m_autos;
  }
}