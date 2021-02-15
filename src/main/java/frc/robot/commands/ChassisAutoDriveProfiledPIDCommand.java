package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ChassisAutoDriveProfiledPIDCommand extends ProfiledPIDCommand {
  /** Creates a new DriveAutoMoveProfiledPIDCommand. */
  public ChassisAutoDriveProfiledPIDCommand(double distance_in) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            Constants.kChassisAutoDrive_P, Constants.kChassisAutoDrive_I, Constants.kChassisAutoDrive_D,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Constants.kChassisAutoDriveConstraintVel_InPerSec,
                Constants.kChassisAutoDriveConstraintAccel_InPerSecSquared)),
        // This should return the measurement
        RobotContainer.chassisSubsystem::getDistance,
        // This should return the goal (can also be a constant)
        distance_in,
        // This uses the output
        (output, setpoint) -> {RobotContainer.chassisSubsystem.driveAutoProfiledArcade(output);
        }
    );
    addRequirements(RobotContainer.chassisSubsystem);
    getController().setTolerance(Constants.kChassisAutoDrivePIDToleranceDis_Inch, Constants.kChassisAutoDrivePIDToleranceVel_InPerSec);
  }
  @Override
  public void initialize() {
    super.initialize();
    RobotContainer.chassisSubsystem.reset();
    getController().reset(0);
  }
  @Override
  public void execute() {
    super.execute();
    SmartDashboard.putNumber("Chassis/Distance Error In", getController().getPositionError());
    SmartDashboard.putNumber("Chassis/Velocity Error InPerSec", getController().getVelocityError());
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(getController().atGoal()){
      getController().reset(RobotContainer.chassisSubsystem.getDistance());
      return true;
    }
    return false;
  }
}
