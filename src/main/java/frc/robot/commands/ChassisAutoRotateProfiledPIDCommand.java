package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ChassisAutoRotateProfiledPIDCommand extends ProfiledPIDCommand {
  /** Creates a new DriveAutoRotateProfiledPIDCommand. */
  public ChassisAutoRotateProfiledPIDCommand(double targetAngle_deg) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(Constants.kChassisAutoRotate_P,Constants.kChassisAutoRotate_I,Constants.kChassisAutoRotate_D,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Constants.kChassisAutoRotateVel_DegPerSec, Constants.kChassisAutoRotateAccel_DegPerSecSquared)),
        // This should return the measurement
        RobotContainer.chassisSubsystem::getAngle,
        // This should return the goal (can also be a constant)
        targetAngle_deg,
        // This uses the output
        (output, setpoint) -> { RobotContainer.chassisSubsystem.rotateAutoProfiledArcade(output);
          // Use the output (and setpoint, if desired) here
        });
        addRequirements(RobotContainer.chassisSubsystem);
        getController().setTolerance(Constants.kChassisAutoRotatePIDToleranceAng_Deg, Constants.kChassisAutoRotatePIDToleranceVel_DegPerSec);
        getController().setIntegratorRange(-0.5, 0.5);
  }
  @Override 
  public void initialize() {
    super.initialize();
    RobotContainer.chassisSubsystem.reset();
    getController().reset(0);
    getController().setPID(Constants.kChassisAutoRotate_P, Constants.kChassisAutoRotate_I, Constants.kChassisAutoRotate_D);
  }
  @Override
  public void execute() {
    super.execute();
    SmartDashboard.putNumber("Chassis/Angle Error Deg", getController().getPositionError());
    SmartDashboard.putNumber("Chassis/Angular Vel Error DegPerSec", getController().getVelocityError());
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(getController().atGoal()){
      getController().reset(0);
      return true;
    }
    return false;
  }
}
