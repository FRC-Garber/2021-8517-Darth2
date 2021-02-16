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
    // System 
    public static final double kSchedularLoopRate = 0.02;

    // Drive System
    public static final double kChassisWheelDiameter_In = 6;
    public static final double kChassisGearRatio = 10.7;
    public static final double kChassisDriveMotorMaxSpeed_RPM = 6380;
    public static final double kChassisDriveMotorEncoderCntsPerRev = 2048;

    public static final double kTeleopSpeedScale = 0.6;
    public static final double kTeleopRotationScale = 0.5;

    // Chassis Autonomous Drive PID
    public static double kChassisAutoDrive_P = 0.03;
    public static double kChassisAutoDrive_I = 0.0015;//0.0012;
    public static double kChassisAutoDrive_D = 0.0;//.0000001;
    public static double kChassisAutoDriveMaxSpeed = 0.5;
    public static final double kChassisAutoDrivePIDToleranceDis_Inch = 0.5;
    public static final double kChassisAutoDrivePIDToleranceVel_InPerSec = 1;

    // Chasis Autonomous Drive Rotation PID
    public static double kChassisAutoDriveRotation_P = 0.0;
    public static double kChassisAutoDriveRotation_I = 0.0;
    public static double kChassisAutoDriveRotation_D = 0.0;
    public static double kChassisAutoDriveRotationPIDToleranceDis_Deg = 0.5;
    public static double kChassisAutoDriveRotationPIDToleranceVel_DegPerSec = 2;
    
    // Chassis Autonomous Rotate PID
    public static double kChassisAutoRotate_P = 0.003;
    public static double kChassisAutoRotate_I = 0.0047;
    public static double kChassisAutoRotate_D = 0;
    public static double kChassisAutoRotateMaxSpeed = 0.4;
    public static final double kChassisAutoRotatePIDToleranceAng_Deg = 1;
    public static final double kChassisAutoRotatePIDToleranceVel_DegPerSec = 10;
    public static double kChassisAutoRotateMaxIntegrator = 1.0;

    // Chassis Autonomous Profiled PID constraints
    public static final double kChassisAutoDriveConstraintVel_InPerSec = 10; // 187 is Max
    public static final double kChassisAutoDriveConstraintAccel_InPerSecSquared = 10;
    public static final double kChassisAutoRotateVel_DegPerSec = 100; // 766 is Max
    public static final double kChassisAutoRotateAccel_DegPerSecSquared = 2000;
    
    // Chassis Motor CAN IDs
	public static final int kFL_Motor_CANID = 10;
	public static final int kRL_Motor_CANID = 11;
	public static final int kFR_Motor_CANID = 12;
	public static final int kBR_Motor_CANID = 13;

	

    

	

}
