// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MechanismConstants {
    public static final int kArmID = 0;
    public static final int kWristID = 1;
    public static final int kElevatorMotorID = 8;
    public static final int kElevatorMotorFollowerID = 9;
    public static final int kRetractLimitSwitchChannel = 0;
    public static final int kClimberID = 23;
    public static final double kRotationToInches = .418; //1.0/20.0 * ((68.8/25.4)/2) * Math.PI * 2.0;//TODO: Find the correct value
    public static final double kWristVerticalPos = 0.214;
    public static final double kWristHorizontalPos = -0.039063;
  }

  public static class ArmConstants {
    public static final double kArmStraightUp = 0.4934-.25; //TODO make Cam finish the constants
    public static final double kArmStraightOut = 0;
    public static final double kArmStraightDown = -.265;
  }

  public static class ElevatorConstants {
    public static final double kBottom = 0; 
    public static final double kTopOfFirstStage = 0;//TODO
    public static final double kTop = 0;  //TODO
  }

  public enum Position {
    ELEVATOR_ZERO(.17,MechanismConstants.kWristHorizontalPos,0,0),

    GROUND_INTAKE(-.084, MechanismConstants.kWristHorizontalPos, 2, 1),

    INTERNAL_INTAKE(ArmConstants.kArmStraightDown, MechanismConstants.kWristHorizontalPos, 14.0, 1),

    HOPPER_INTAKE(ArmConstants.kArmStraightDown, MechanismConstants.kWristHorizontalPos, 18.0, 1),
    
    L1(-.0539, MechanismConstants.kWristVerticalPos, 11.6, .515), // 305 = none, 190=resting, 

    L2(.132, MechanismConstants.kWristHorizontalPos, 2.5, .325), //.486

    L3(.132, MechanismConstants.kWristHorizontalPos, 19.0, .3), //.3 away when coral is on the reef

    L4(.1, MechanismConstants.kWristHorizontalPos, 46, .45);

    public final double armAngle;
    public final double wristPos;
    public final double elevatorPos;
    public final double dist;

    Position(double armAngle, double wristPos, double elevatorPos, double dist) {
        this.armAngle = armAngle;
        this.wristPos = wristPos;
        this.elevatorPos = elevatorPos;
        this.dist = dist;
    }
}
}