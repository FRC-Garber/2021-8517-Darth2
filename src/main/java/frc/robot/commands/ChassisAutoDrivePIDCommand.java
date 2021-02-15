package frc.robot.commands;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ChassisAutoDrivePIDCommand extends PIDCommand {

  LinearFilter filter = LinearFilter.singlePoleIIR(0.01, Constants.kSchedularLoopRate);
 /** Creates a PID command that will drive to a distance in inches with a supplied max speed
   * @param distance_in Distance in inches to travel. This can be negative to go in reverse
   */
  public ChassisAutoDrivePIDCommand(double distance_in) {
    // Call the super class of PIDCommad constructor. This MUST be first statement in this constructor
    super(
        // The controller that the command will use
        new PIDController(Constants.kChassisAutoDrive_P, Constants.kChassisAutoDrive_I, Constants.kChassisAutoDrive_D),
        // This should return the measurement
        RobotContainer.chassisSubsystem::getDistance,
        // This should return the setpoint (can also be a constant)
        distance_in,
        // This uses the output
        output -> RobotContainer.chassisSubsystem.driveAutoArcade(output),
        RobotContainer.chassisSubsystem
    );
    getController().setTolerance(Constants.kChassisAutoDrivePIDToleranceDis_Inch, Constants.kChassisAutoDrivePIDToleranceVel_InPerSec);
  }
  @Override 
  public void initialize() {
    super.initialize();
    RobotContainer.chassisSubsystem.setLinearFIlterAutoSpeed(LinearFilter.singlePoleIIR(0.01, Constants.kSchedularLoopRate));
    RobotContainer.chassisSubsystem.reset();
    getController().reset();
    getController().setPID(Constants.kChassisAutoDrive_P, Constants.kChassisAutoDrive_I, Constants.kChassisAutoDrive_D);
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
    if(getController().atSetpoint()){
      getController().reset();
      return true;
    }
    return false;
  }
}
