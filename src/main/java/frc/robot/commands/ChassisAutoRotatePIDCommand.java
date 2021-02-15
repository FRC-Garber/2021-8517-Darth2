package frc.robot.commands;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ChassisAutoRotatePIDCommand extends PIDCommand {
  /** Creates a PID command that will turn to an angle with the supplied max speed.
   * @param angle_deg The angle to go to in Degrees
   */
  public ChassisAutoRotatePIDCommand(double angle_deg) {
    // Call the super class of PIDCommad constructor. This MUST be first statement in this constructor
    super(
        // The controller that the command will use
        new PIDController(Constants.kChassisAutoRotate_P, Constants.kChassisAutoRotate_I, Constants.kChassisAutoRotate_D),
        // This should return the measurement
        RobotContainer.chassisSubsystem::getAngle,
        // This should return the setpoint (can also be a constant)
        angle_deg,
        // This uses the output
        output -> RobotContainer.chassisSubsystem.rotateAutoArcade(output),
        RobotContainer.chassisSubsystem
    );
    getController().setTolerance(Constants.kChassisAutoRotatePIDToleranceAng_Deg, Constants.kChassisAutoRotatePIDToleranceVel_DegPerSec);
  }
  @Override 
  public void initialize() {
    super.initialize();
    RobotContainer.chassisSubsystem.setLinearFIlterAutoAngle(LinearFilter.singlePoleIIR(0.01, Constants.kSchedularLoopRate));
    RobotContainer.chassisSubsystem.reset();
    getController().reset();
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
    if(getController().atSetpoint()){
      getController().reset();
      return true;
    }
    return false;
  }
}
