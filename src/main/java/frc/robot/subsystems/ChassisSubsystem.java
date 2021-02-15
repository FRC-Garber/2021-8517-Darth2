package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

public class ChassisSubsystem extends SubsystemBase {

  // Declare and define TalonFX motor controllers for the drive system
  private WPI_TalonFX FL_Motor = new WPI_TalonFX(Constants.kFL_Motor_CANID);
  private WPI_TalonFX BL_Motor = new WPI_TalonFX(Constants.kRL_Motor_CANID);
  private WPI_TalonFX FR_Motor = new WPI_TalonFX(Constants.kFR_Motor_CANID);
  private WPI_TalonFX BR_Motor = new WPI_TalonFX(Constants.kBR_Motor_CANID);
  // Declare the drive system type we have. The DifferentialDrive class has the methods for driving this type of robot.
  DifferentialDrive robotDrive;
  // Declare the filters to be used by autonomous mode for PID driving and turning
  LinearFilter filterAutoSpeed;
  LinearFilter filterAutoAngle;
  // Define the Gyro which is a NavX Micro
  AHRS gyro;
  // Define the PIDController used for driving straight during an autonomous drive command
  PIDController driveRotationPID;
  // Module variables used to hold the last distnance and angle for calculations
  double m_distance_In;
  double m_angle_Deg;
  double m_velocity_InPerSec;
  double m_acceleration_InPerSecSquared;
  double m_angularVelocity_DegPerSec;
  double m_angularAcceleration_DegPerSecSquared;

  double distance_prev = 0;
  double angle_prev = 0;
  double velocity_prev = 0;
  double angularVel_prev = 0;
  /** Creates a new DriveSubsystem. */
  public ChassisSubsystem() {
    // Set Motor so they are in brake mode so they will slow down faster when no power is applied 
    FL_Motor.setNeutralMode(NeutralMode.Brake);
    BL_Motor.setNeutralMode(NeutralMode.Brake);
    FR_Motor.setNeutralMode(NeutralMode.Brake);
    BR_Motor.setNeutralMode(NeutralMode.Brake);

    // Tell the back motors to follow the fronts so we only need to deal with the front motors
    BL_Motor.follow(FL_Motor);
    BR_Motor.follow(FR_Motor);

    // Create a differential drive object that drives the motors with the type of robot we have.
    robotDrive = new DifferentialDrive(FL_Motor, FR_Motor);
    // Define the gyro object we have
    gyro = new AHRS(Port.kMXP);
    // Define the PID controller that is used to keep the autonomous driving straight
    driveRotationPID = new PIDController(Constants.kChassisAutoDriveRotation_P,Constants.kChassisAutoDriveRotation_I,Constants.kChassisAutoDriveRotation_D);
    driveRotationPID.setTolerance(Constants.kChassisAutoDriveRotationPIDToleranceDis_Deg,Constants.kChassisAutoDriveRotationPIDToleranceVel_DegPerSec );
    // Set SmartDashboard variables that belong to the driveSubsystem
    setSmartDashboard();;
  }
  /** Used to drive the robot in teleop with a speed and rotation value
   * 
   * @param speed The forward speed in +/- 1
   * @param rotation The rotation speed in +/- 1 
   * @param squareInputs Square the inputs or not.
   */
  public void driveTeleopArcade(double speed, double rotation, boolean squareInputs) {
    robotDrive.arcadeDrive(speed, rotation, squareInputs);
  }
  /** Drive the robot in autonomous with a speed.
   * <p>The input speed will be clamped at limits and filtered
   * This method should be called from a PID, Time limiting or distance limiting command.
   * The rotation will use a PID to keep the heading at Zero angle
   * @param speed The speed to drive
   */
  public void driveAutoArcade(double speed) {
    double _speed = speed;
    // limit the speed based on what was commanded
    _speed = MathUtil.clamp(_speed, -Constants.kChassisAutoDriveMaxSpeed, Constants.kChassisAutoDriveMaxSpeed);
    // Filter the input to prevent sudden changes
    _speed =  filterAutoSpeed.calculate(_speed);
    // drive the robot in arcade
    robotDrive.arcadeDrive(_speed, getRotationPIDControllerValue(), false);
  }
  /** Rotate the robot in autonomous with a speed input
   * The input speed will be clamped and filtered
   * This method should be called from a PID command, Time limit or angle limiting command.
   * @param speed The speed to rotate
   */
  public void rotateAutoArcade(double speed){
    double _speed = -speed;
    // limit the speed based on what was commanded
    _speed = MathUtil.clamp(_speed, -Constants.kChassisAutoRotateMaxSpeed, Constants.kChassisAutoRotateMaxSpeed);
    // Filter the input to prevent sudden changes
    _speed = filterAutoAngle.calculate(_speed);
    // rotate the robot in arcade
    robotDrive.arcadeDrive(0, _speed, false);
  }
  public void rotateAutoProfiledArcade(double speed){
    speed = MathUtil.clamp(speed, -Constants.kChassisAutoRotateMaxSpeed, Constants.kChassisAutoRotateMaxSpeed);
    robotDrive.arcadeDrive(0, speed, false);
  }
  public void driveAutoProfiledArcade(double speed){
    speed = MathUtil.clamp(speed, -Constants.kChassisAutoDriveMaxSpeed, Constants.kChassisAutoDriveMaxSpeed);
    robotDrive.arcadeDrive(speed, -getRotationPIDControllerValue(), false);
  }
  /** Calculate a new PID value for the rotation speed during a autonmous drive command
   * <p>The reset of the PID must be called before each start of the drive command to reset the integral term
   * @return get the latest PID value output
   */
  public double getRotationPIDControllerValue(){
    return driveRotationPID.calculate(getAngle());
  }
  private void calcDistance(){
    // Calculate the circumference of the wheel 2PI*R or PI*D. This is also the  Inches/Revolution of the wheel
    double wheelCircumference_In = Math.PI * Constants.kChassisWheelDiameter_In;
    // Get an average of the Left motor encoders
    double motorEncCnts = (FL_Motor.getSelectedSensorPosition() + FR_Motor.getSelectedSensorPosition()) / 2;
    // calculate the number of revolutions the motor has gone
    double motorRev = motorEncCnts / Constants.kChassisDriveMotorEncoderCntsPerRev;
    double wheelRev = motorRev / Constants.kChassisGearRatio;
    // Get Inches traveled
    m_distance_In = wheelCircumference_In * wheelRev;
  }
  private void calcAngle(){
    m_angle_Deg = -gyro.getAngle();
  }
  private void calcVelocity(){
    double in = getDistance();
    double inPerSec = (in - distance_prev) / Constants.kSchedularLoopRate;
    distance_prev = in;
    m_velocity_InPerSec = inPerSec;
  }
  private void calcAcceleration(){
    double velocity = getVelocity();
    double accel = (velocity - velocity_prev)/Constants.kSchedularLoopRate;
    velocity_prev = getVelocity();
    m_acceleration_InPerSecSquared = accel;

  }
  private void calcAngularVelocity(){
    double angle = getAngle();
    double degreesPerSec = (angle - angle_prev) /Constants.kSchedularLoopRate;
    angle_prev = angle;
    m_angularVelocity_DegPerSec = degreesPerSec;
  }
  private void calcAngularAcceleration(){
    double angularVel = getAngularVelocity();
    double accel = (angularVel - angularVel_prev) / Constants.kSchedularLoopRate;
    angularVel_prev = angularVel;
    m_angularAcceleration_DegPerSecSquared = accel;
  }
  public double getDistance(){
    return m_distance_In;
  }
  public double getAngle(){
    return m_angle_Deg;
  }
  public double getVelocity(){
    return m_velocity_InPerSec;
  }
  public double getAcceleration(){
    return m_acceleration_InPerSecSquared;
  }
  public double getAngularVelocity(){
    return m_angularVelocity_DegPerSec;
  }
  public double getAngularAcceleration(){
    return m_angularAcceleration_DegPerSecSquared;
  }

  /** 
   * @return Get the current angle in degrees of the robot
   */
  // public double getAngle() {
  //   return -gyro.getAngle();
  // }
  /**
   * Reset all the filters, PIDs, Gyro and encoders to 0 for the start of a new autonomous command
   */
  public void reset(){
    resetMotorEncodersToZero();
    resetGyro();
    resetDriveRotationPID();
    distance_prev = 0;
    angle_prev = 0;
    velocity_prev = 0;
    angularVel_prev = 0;
  }
  public void resetMotorEncodersToZero() {
    FL_Motor.setSelectedSensorPosition(0);
    FR_Motor.setSelectedSensorPosition(0);
    BL_Motor.setSelectedSensorPosition(0);
    BR_Motor.setSelectedSensorPosition(0);
  }
  public void resetAutoSpeedFilter(){
    filterAutoSpeed.reset();
  }
  public void resetAutoAngleFilter(){
    filterAutoAngle.reset();
  }
  public void resetDriveRotationPID(){
    driveRotationPID.setSetpoint(0);
    driveRotationPID.setTolerance(Constants.kChassisAutoDriveRotationPIDToleranceDis_Deg,Constants.kChassisAutoDriveRotationPIDToleranceVel_DegPerSec);
    driveRotationPID.reset();
  }
  /**
   * Reset the gyro to Zero angle or Yaw = 0
   */
  public void resetGyro() {
    gyro.reset();
  }
  public void setLinearFIlterAutoAngle(LinearFilter filter){
    filterAutoAngle = filter;
    filterAutoAngle.reset();
  }
  public void setLinearFIlterAutoSpeed(LinearFilter filter){
    filterAutoSpeed = filter;
    filterAutoSpeed.reset();
  }
  @Override
  public void periodic() {
    calcDistance();
    calcAngle();
    calcVelocity();
    calcAcceleration();
    calcAngularVelocity();
    calcAngularAcceleration();

    SmartDashboard.putNumber("Chassis/Distance In", getDistance());
    SmartDashboard.putNumber("Chassis/Velocity InPerSec", getVelocity());
    SmartDashboard.putNumber("Chassis/Acceleration InPerSecSquared", getAcceleration());

    SmartDashboard.putNumber("Chassis/Angle Deg", getAngle());
    SmartDashboard.putNumber("Chassis/Angular Velocity DegPerSec", getAngularVelocity());
    SmartDashboard.putNumber("Chassis/Angular Acceleration DegPerSecSquared", getAngularAcceleration());

    SmartDashboard.putNumber("Chassis/FL EncCnts", FL_Motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Chassis/BL EncCnts", BL_Motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Chassis/FR EncCnts", FR_Motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Chassis/BR EncCnts", BR_Motor.getSelectedSensorPosition());
    
    // Get the PID values
    if(SmartDashboard.getBoolean("Chassis/PID Update Enable", false)){
      getSmartDashboard();
    }
    if(SmartDashboard.getBoolean("Chassis/Calibrate Gyro", true)){
      gyro.calibrate();
      while(gyro.isCalibrating()){ }
      resetGyro();
      reset();
      SmartDashboard.putBoolean("Chassis/Calibrate Gyro", false);
    }
  }
  private void setSmartDashboard(){
    SmartDashboard.putNumber("Chassis/AutoDrive_kP", Constants.kChassisAutoDrive_P);
    SmartDashboard.putNumber("Chassis/AutoDrive_KI", Constants.kChassisAutoDrive_I);
    SmartDashboard.putNumber("Chassis/AutoDrive_kD", Constants.kChassisAutoDrive_D);
    SmartDashboard.putNumber("Chassis/AutoDrive_MaxSpeed", Constants.kChassisAutoDriveMaxSpeed);
    SmartDashboard.putNumber("Chassis/AutoRotate_kP", Constants.kChassisAutoRotate_P);
    SmartDashboard.putNumber("Chassis/AutoRotate_kI", Constants.kChassisAutoRotate_I);
    SmartDashboard.putNumber("Chassis/AutoRotate_kD", Constants.kChassisAutoRotate_D);
    SmartDashboard.putNumber("Chassis/AutoRotate_MaxSpeed", Constants.kChassisAutoRotateMaxSpeed);
    SmartDashboard.putNumber("Chassis/AutoDriveRotate_kP", Constants.kChassisAutoDriveRotation_P);
    SmartDashboard.putNumber("Chassis/AutoDriveRotate_kI", Constants.kChassisAutoDriveRotation_I);
    SmartDashboard.putNumber("Chassis/AutoDriveRotate_kD", Constants.kChassisAutoDriveRotation_D);
    
    SmartDashboard.putBoolean("Chassis/PID Update Enable", false);
    SmartDashboard.putBoolean("Chassis/Calibrate Gyro", false);
  }
  private void getSmartDashboard(){
    Constants.kChassisAutoDrive_P = SmartDashboard.getNumber("Chassis/AutoDrive_kP", 0);
    Constants.kChassisAutoDrive_I = SmartDashboard.getNumber("Chassis/AutoDrive_KI", 0);
    Constants.kChassisAutoDrive_D = SmartDashboard.getNumber("Chassis/AutoDrive_kD", 0);
    Constants.kChassisAutoDriveMaxSpeed = SmartDashboard.getNumber("Chassis/AutoDrive_MaxSpeed", 0);

    Constants.kChassisAutoRotate_P = SmartDashboard.getNumber("Chassis/AutoRotate_kP", 0);
    Constants.kChassisAutoRotate_I = SmartDashboard.getNumber("Chassis/AutoRotate_kI", 0);
    Constants.kChassisAutoRotate_D = SmartDashboard.getNumber("Chassis/AutoRotate_kD", 0);
    Constants.kChassisAutoRotateMaxSpeed = SmartDashboard.getNumber("Chassis/AutoRotate_MaxSpeed", 0);

    driveRotationPID.setP(SmartDashboard.getNumber("Chassis/AutoDriveRotate_kP", Constants.kChassisAutoDriveRotation_P));
    driveRotationPID.setI(SmartDashboard.getNumber("Chassis/AutoDriveRotate_kI", Constants.kChassisAutoDriveRotation_I));
    driveRotationPID.setD(SmartDashboard.getNumber("Chassis/AutoDriveRotate_kD", Constants.kChassisAutoDriveRotation_D));

    SmartDashboard.putBoolean("Chassis/PID Update Enable", false);
  }
}
