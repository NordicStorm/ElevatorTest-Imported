package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.paths.DriveTrainConfig;
import frc.robot.commands.paths.PathableDrivetrain;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem, PathableDrivetrain {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final CANrange m_frontCANrange = new CANrange(1, "CANivore");
    private final CANrange m_backCANrange = new CANrange(0, "CANivore");
    public Field2d m_fieldDisplay = new Field2d();

    private DriveTrainConfig m_driveTrainConfig = new DriveTrainConfig();
    private final SwerveRequest.RobotCentric m_drive = new SwerveRequest.RobotCentric()
            .withDeadband(0).withRotationalDeadband(0) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Spark blinkinPark = new Spark(0);
    

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        SmartDashboard.putData(m_fieldDisplay);
        initializeDriveTrainConfig();

        setBlinkinDrive();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        SmartDashboard.putData(m_fieldDisplay);
        initializeDriveTrainConfig();
        setBlinkinDrive();
    }

    private void initializeDriveTrainConfig() {
        m_driveTrainConfig.maxVelocity = 4.5;
        m_driveTrainConfig.maxAcceleration = 3;
        m_driveTrainConfig.maxAnglularVelocity = 10;
        m_driveTrainConfig.maxAngularAcceleration = 8;
        m_driveTrainConfig.maxCentripetalAcceleration = 7;

        m_driveTrainConfig.rotationCorrectionP = 10;

    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        SmartDashboard.putNumber("Front Left Encoder",
                getModule(0).getEncoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Front Right Encoder",
                getModule(1).getEncoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Back Left Encoder",
                getModule(2).getEncoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Back Right Encoder",
                getModule(3).getEncoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Voltage", getModule(3).getDriveMotor().getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Vx", getState().Speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("vY", getState().Speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Gyro", getGyroDegrees());

        SmartDashboard.putNumber("Front CAN range", m_frontCANrange.getDistance().getValueAsDouble());
        SmartDashboard.putNumber("Back CAN range", m_backCANrange.getDistance().getValueAsDouble());
        SmartDashboard.putString("Target Level", RobotContainer.targetLevel.toString());
        SmartDashboard.putBoolean("Target Left Side?", RobotContainer.alignmentLeft);
        SmartDashboard.putNumber("Algae mode?", RobotContainer.algaeMode);
        SmartDashboard.putBoolean("Coral Mode?", RobotContainer.isCoralMode);
        SmartDashboard.putBoolean("Is C2 active?", RobotContainer.isSecondControllerActive);
        m_fieldDisplay.setRobotPose(getState().Pose);

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    public double getFrontRange() {
        return m_frontCANrange.getDistance().getValueAsDouble() - 0.238 - 0.0127;
    }

    public double getBackRange() {
        return m_backCANrange.getDistance().getValueAsDouble();
    }

    public boolean getFrontRangeIsDetected() {
        return m_frontCANrange.getIsDetected().getValue();
    }

    public boolean getBackRangeIsDetected() {
        return m_backCANrange.getIsDetected().getValue();
    }

    public void setBlinkinPark() {
        blinkinPark.set(-0.05);
    }

    public void setBlinkinDrive() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            blinkinPark.set(0.87);
        }
        else {
            blinkinPark.set(.61);
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    public double getGyroDegrees() {
        return Math.toDegrees(getGyroRadians());
    }

    @Override
    public double getGyroRadians() {
        return getState().Pose.getRotation().getRadians();

    }

    @Override
    public Pose2d getPose() {
        return getState().Pose;
    }

    @Override
    public void setPose(Pose2d pose) {
        resetTranslation(pose.getTranslation());
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        setControl(m_drive.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        return getState().Speeds;
    }

    @Override
    public DriveTrainConfig getConfig() {
        return m_driveTrainConfig;
    }

}
