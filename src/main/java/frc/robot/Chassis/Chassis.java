package frc.robot.Chassis;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
/*/**
 * @author yuval rader
 */
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.limelight;
import frc.robot.utils.autonomous.OdometryHandler;
import frc.robot.utils.controllers.MAPidController;
import frc.robot.utils.subsystem.chassis.BasicChassisPIDSubsystem;
import frc.robot.utils.MAShuffleboard;
import frc.robot.utils.MACalculations;

public class Chassis extends SubsystemBase implements BasicChassisPIDSubsystem{
  private static Chassis chassis;
  
  private BaseMotorController leftFrontMotor;
  private BaseMotorController leftRearMotor;
  private BaseMotorController rightFrontMotor;
  private BaseMotorController rightRearMotor;

  private AHRS navx;

  private MAPidController rightVelocityPID;
  private MAPidController leftVelocityPID;

  private MAPidController anglePIDVision;
  private MAPidController distancePIDVision;

  public MAShuffleboard chassisShuffleboard;

  private OdometryHandler odometryHandler;
  public Field2d m_field;

  public static Chassis getinstance() {
    if (chassis == null) {
      chassis = new Chassis();
    }
    return chassis;
  }

  private Chassis() {
    chassisShuffleboard = new MAShuffleboard(ChassisConstants.KSUBSYSTEM_NAME);
    leftFrontMotor = new WPI_TalonFX(3);
    leftRearMotor = new WPI_TalonFX(4);
    rightFrontMotor = new WPI_TalonFX(1);
    rightRearMotor = new WPI_TalonFX(2);
    
    leftRearMotor.follow(leftFrontMotor);
    rightRearMotor.follow(rightFrontMotor);

    navx = new AHRS(Port.kMXP);
    odometryHandler = new OdometryHandler(this::getLeftDistance, this::getRightDistance, this::getAngle);
    
    rightVelocityPID = new MAPidController(ChassisConstants.KP_MAPATH_RIGHT_VELOCITY,
        ChassisConstants.KI_MAPATH_RIGHT_VELOCITY, ChassisConstants.KD_MAPATH_RIGHT_VELOCITY, 0, 20, -12, 12);
    leftVelocityPID = new MAPidController(ChassisConstants.KP_MAPATH_LEFT_VELOCITY,
        ChassisConstants.KI_MAPATH_LEFT_VELOCITY, ChassisConstants.KD_MAPATH_LEFT_VELOCITY, 0, 20, -12, 12);

    anglePIDVision = new MAPidController(ChassisConstants.KP_VISION_ANGLE, ChassisConstants.KI_VISION_ANGLE,
        ChassisConstants.KD_VISION_ANGLE, 0, 2, -12, 12);
    distancePIDVision = new MAPidController(ChassisConstants.KP_VISION_DISTANCE, ChassisConstants.KI_VISION_DISTANCE,
        ChassisConstants.KD_VISION_DISTANCE, 0, 2, -12, 12);

    anglePIDVision.enableContinuousInput(-ChassisConstants.KANGLE_PID_VISION_SET_INPUTRANGE,
        ChassisConstants.KANGLE_PID_VISION_SET_INPUTRANGE);
    
    resetSensors();
    m_field = new Field2d();
    rightFrontMotor.setInverted(true);
    rightRearMotor.setInverted(true);
  }

  public void setLeftVoltage(double voltage) {
    leftFrontMotor.set(ControlMode.PercentOutput ,voltage / 12);
  }

  public void setRightVoltage(double voltage) {
    rightFrontMotor.set(ControlMode.PercentOutput, voltage / 12);
  }

  public void setRightPercent(double power){
    rightFrontMotor.set(ControlMode.PercentOutput, power);
  }

  public void setLeftPercent(double power){
    leftFrontMotor.set(ControlMode.PercentOutput, power);
  }

  public void arcadeDrive(double angle, double distance) {
    double w = (100 - Math.abs(angle * 100)) * (distance) + distance * 100;
    double v = (100 - Math.abs(distance * 100)) * (angle) + angle * 100;
    double leftVoltage = (-(v + w) / 200);
    double rightVoltage = ((v - w) / 200);
    setLeftVoltage(leftVoltage);
    setRightVoltage(rightVoltage);
  }

  public double getLeftDistance() {
    return leftRearMotor.getSelectedSensorPosition() / ChassisConstants.KTICKS_PER_METER;
  }

  public double getRightDistance() {
    return rightRearMotor.getSelectedSensorPosition() / ChassisConstants.KTICKS_PER_METER;
  }
  
  public double getRightEncoder(){
    return MACalculations.toRPMFromEncoderConnectToTalon((rightRearMotor.getSelectedSensorVelocity() + rightFrontMotor.getSelectedSensorVelocity()) / 2, RobotConstants.KCTRE_MAG_ENCODER_TPR);
  }

  public double getLeftEncoder(){
    return MACalculations.toRPMFromEncoderConnectToTalon((leftRearMotor.getSelectedSensorVelocity() + leftFrontMotor.getSelectedSensorVelocity()) / 2, RobotConstants.KCTRE_MAG_ENCODER_TPR);
  }

  // TODO 
  public double getLeftVelocity() {
    return leftMPS();
  }

  // TODO
  public double getRightVelocity() {
    return rightMPS();
  }

  private double falconTicksToRPM(double falconTicks) {
    return (falconTicks * 600) / 2048;
  }

  private double falconTicksToWheelVelocity(double falconTicks) {
    return falconTicksToMeters(falconTicks);
  }

  private double falconTicksToMeters(double ticks){
    return ticks * ChassisConstants.KMETER_PER_TICKS;
  }

  public double rightRPM() {
    return falconTicksToRPM((rightFrontMotor.getSelectedSensorVelocity() + rightRearMotor.getSelectedSensorVelocity()) / 2);
  }

  public double leftRPM() {
    return falconTicksToRPM((leftFrontMotor.getSelectedSensorVelocity() + rightRearMotor.getSelectedSensorVelocity()) / 2);
  }

  public double leftMPS() {
    // System.out.println("LEFT " + falconTicksToWheelVelocity(leftRearMotor.getSelectedSensorVelocity()));
    return falconTicksToWheelVelocity(leftRearMotor.getSelectedSensorVelocity());
  }

  public double rightMPS() {
    // System.out.println("RIGHT " + falconTicksToWheelVelocity(rightRearMotor.getSelectedSensorVelocity()));
    return falconTicksToWheelVelocity(rightRearMotor.getSelectedSensorVelocity());
  }

  @Override
  public void setIdleMode(Boolean Break) {
    if (Break){
      leftRearMotor.setNeutralMode(NeutralMode.Brake);
      leftFrontMotor.setNeutralMode(NeutralMode.Brake);
      rightFrontMotor.setNeutralMode(NeutralMode.Brake);
      rightRearMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      leftRearMotor.setNeutralMode(NeutralMode.Coast);
      leftFrontMotor.setNeutralMode(NeutralMode.Coast);
      rightFrontMotor.setNeutralMode(NeutralMode.Coast);
      rightRearMotor.setNeutralMode(NeutralMode.Coast);
    }
    
    
  }

  public double getAngle() {
    return navx.getAngle();
  }

  // reset the value of the encoder and the navx
  public void resetSensors() {
    navx.reset();
    leftFrontMotor.setSelectedSensorPosition(0);
    leftRearMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
    rightRearMotor.setSelectedSensorPosition(0);

    odometryHandler.reset();
  }

  public void resetPID() {
    leftVelocityPID.reset();
    rightVelocityPID.reset();
    distancePIDVision.reset();
    anglePIDVision.reset();
  }

  public double getVisionAnglePIDOutput(double setpoint) {
    return anglePIDVision.calculate(limelight.x * -1, setpoint);
  }

  public double getVisionDistancePIDOutput(double setpoint) {
    return distancePIDVision.calculate(limelight.Tshort, setpoint);
  }

  public boolean isVisionAngleAtSetpoint() {
    return anglePIDVision.atSetpoint();
  }

  public boolean isVisionDistanceAtSetpoint() {
    return distancePIDVision.atSetpoint();
  }

  public boolean isPIDRightVelocityAtSetPoint() {
    return rightVelocityPID.atSetpoint();
  }

  public boolean isPIDLeftVelocityAtSetPoint() {
    return leftVelocityPID.atSetpoint();
  }

  public void setRightVelocitySetpoint(double setpoint){
    rightVelocityPID.setSetpoint(setpoint);
  }

  public void setLeftVelocitySetpoint(double setpoint){
    leftVelocityPID.setSetpoint(setpoint);
  }

  public double rightVelocityMApathPIDOutput() {
    return rightVelocityPID.calculate(getRightVelocity());
  }

  public double leftVelocityMApathPIDOutput() {
    return leftVelocityPID.calculate(getLeftVelocity());
  }

  public double getRightPID(double measurement){
    return rightVelocityPID.calculate(measurement);
  }

  public double getLeftPID(double measurement){
    return leftVelocityPID.calculate(measurement);
  }

  public double getLeftF(){
    return leftVelocityPID.getSetpoint() * ChassisConstants.KF_MAPATH_LEFT_VELOCITY;
  }

  public double getRightF(){
    return rightVelocityPID.getSetpoint() * ChassisConstants.KF_MAPATH_RIGHT_VELOCITY;
  }

  public OdometryHandler getOdomoteryHandler() {
    return odometryHandler;
  }

  @Override
  public void periodic() {
    odometryHandler.update();
    
    chassisShuffleboard.addNum("right distance", getRightDistance());
    chassisShuffleboard.addNum("left distance", getLeftDistance());
    chassisShuffleboard.addNum("right velocity", getRightVelocity());
    chassisShuffleboard.addNum("left velocity", getLeftVelocity());
    chassisShuffleboard.addNum("right velocity setpoint", rightVelocityPID.getSetpoint());
    chassisShuffleboard.addNum("left velocity setpoint",leftVelocityPID.getSetpoint());
    chassisShuffleboard.addNum("angle", getAngle());

    chassisShuffleboard.addNum("right encoder", rightRearMotor.getSelectedSensorPosition());
    chassisShuffleboard.addNum("left encoder", leftRearMotor.getSelectedSensorPosition());


   chassisShuffleboard.addNum("right power", rightFrontMotor.getMotorOutputPercent());
   chassisShuffleboard.addNum("left power", leftRearMotor.getMotorOutputPercent());

    chassisShuffleboard.addNum("left f", getLeftF());
    chassisShuffleboard.addNum("right F", getRightF());
    chassisShuffleboard.addString("Robot Point", odometryHandler.getCurrentPosition().toString());
    // SmartDashboard.putData("Field", m_field);
    // m_field.setRobotPose(currentPose);

    // chassisShuffleboard.addNum("Left MPS", leftMPS());
    // chassisShuffleboard.addNum("Right MPS", rightMPS());
    // System.out.println("Left Speed: " + leftRearMotor.getSelectedSensorVelocity());
    // System.out.println("Right Speed " + rightRearMotor.getSelectedSensorVelocity());
  }
}