package frc.robot.commands;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ChassisAutoDrivePICommand extends PIDCommand {
  double m_kP, m_kI;
  /** Creates a new ChassisAutoDrivePICommand. */
  public ChassisAutoDrivePICommand(double distance_in, double kP, double kI, double speed) {
     // Call the super class of PIDCommad constructor. This MUST be first statement in this constructor
     super(
      // The controller that the command will use
      new PIDController(kP, kI, 0.0),
      // This should return the measurement
      RobotContainer.chassisSubsystem::getDistance,
      // This should return the setpoint (can also be a constant)
      distance_in,
      // This uses the output
      output -> RobotContainer.chassisSubsystem.driveAutoArcade(output,speed),
      RobotContainer.chassisSubsystem
  );
  getController().setTolerance(Constants.kChassisAutoDrivePIDToleranceDis_Inch, Constants.kChassisAutoDrivePIDToleranceVel_InPerSec);
  m_kP = kP;
  m_kI = kI;

  }
  @Override 
  public void initialize() {
    super.initialize();
    RobotContainer.chassisSubsystem.setLinearFIlterAutoSpeed(LinearFilter.singlePoleIIR(0.01, Constants.kSchedularLoopRate));
    RobotContainer.chassisSubsystem.reset();
    getController().reset();
    getController().setPID(m_kP, m_kI, 0);
    
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
