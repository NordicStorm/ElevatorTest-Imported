package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;

public class AutoReceiveAlign extends SequentialCommandGroup implements CommandPathPiece{
    /***
     * 
     * @param targetAngle Pass 0 if it should just round/snap to the nearest angle
     * @param drivetrain
     * @param intake
     */
    public AutoReceiveAlign(double targetAngle, CommandSwerveDrivetrain drivetrain, CoralIntake intake){
        addCommands(new Command(){
            ProfiledPIDController rotationController;
            boolean done;
            double m_targetAngle;
            ProfiledPIDController xController;
            boolean first;
            @Override
            public void initialize() {
                rotationController = new ProfiledPIDController(10, 0, 0, new Constraints(400, 300));
                xController = new ProfiledPIDController(7, 0, 0, new Constraints(4, 3));
                rotationController.reset(drivetrain.getGyroDegrees());
                rotationController.enableContinuousInput(-180, 180);
                done = false;
                if(targetAngle == 0) {
                    double currentAngle = MathUtil.inputModulus(drivetrain.getGyroDegrees(), -180, 180);
                    // 126, 54, -54, -126
                    if(DriverStation.getAlliance().get() == Alliance.Blue) {
                        if(currentAngle > 0) {
                            m_targetAngle = 54;
                        } else {
                            m_targetAngle = -54;
                        }
                    } else {
                        if(currentAngle > 0) {
                            m_targetAngle = 126;
                        } else {
                            m_targetAngle = -126;
                        }
                    }
                } else {
                    m_targetAngle = targetAngle;
                }
            }
            @Override
            public void execute(){
                var speeds = new ChassisSpeeds();
                speeds.omegaRadiansPerSecond = Math.toRadians(rotationController.calculate(drivetrain.getGyroDegrees(), m_targetAngle));
                var range = drivetrain.getBackRangeIsDetected() ? drivetrain.getBackRange() : 2;
                if (first) {
                    xController.reset(range);
                    first = false;
                }
                speeds.vxMetersPerSecond = xController.calculate(.2);
                if (range < Constants.kBackRangeMaxDist && range > Constants.kBackRangeMinDist){
                    done = true;
                }
                drivetrain.drive(speeds);
            }

            @Override
            public boolean isFinished(){
                return done;
            }

            @Override
            public void end(boolean interrupted) {
                drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
                drivetrain.setBlinkinPark();
                if (interrupted) {
                drivetrain.setBlinkinDrive();
                }
            }
        }.andThen(new Command(){
            @Override
            public boolean isFinished(){
                return intake.isInTrough() || intake.hasCoral();
            }

            @Override
            public void end(boolean interrupted) {
                drivetrain.setBlinkinDrive();
            }
        }
        ));

        
    }

    @Override
    public double getRequestedStartSpeed() {
        return 1;
    }
    
}
