// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ChassisDefaultCommand extends CommandBase {
  LinearFilter filterSpeed =  LinearFilter.singlePoleIIR(0.1, Constants.kSchedularLoopRate);
  LinearFilter filterRotation =  LinearFilter.singlePoleIIR(0.1, Constants.kSchedularLoopRate);
  // double velocity_old = 0;
  // double distance_old = 0;
  // double velocityAng_old = 0;
  // double angle_old = 0;
  /** Creates a new DriveDefaultCommand. */
  public ChassisDefaultCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.chassisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double _speed = (RobotContainer.stickDriver.getRawAxis(3) - RobotContainer.stickDriver.getRawAxis(2)) * Constants.kTeleopSpeedScale;
    double _rotation = RobotContainer.stickDriver.getX(Hand.kLeft) * Constants.kTeleopRotationScale;
    _speed = filterSpeed.calculate(_speed);
    _rotation = filterRotation.calculate(_rotation);
    RobotContainer.chassisSubsystem.driveTeleopArcade(_speed, _rotation, false);
    SmartDashboard.putNumber("Chassis/Teleop Joystick Speed", _speed);
    SmartDashboard.putNumber("Chassis/Teleop Joystick Rotation", _rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
