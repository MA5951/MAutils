
package frc.robot.subsystems.Chassis;

/**
 * @author yuval rader
 */
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.limelight;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.Calculation.MACalculations;
import frc.robot.utils.Controlers.MAPidController;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;
import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.Autonomous.Path;
import frc.robot.utils.Autonomous.Point;
import frc.robot.commands.Chassis.MAPath;

public class Chassis extends MASubsystem {
  private MAMotorControler leftFrontMotor;
  private MAMotorControler leftMotor;

  private MAMotorControler rightFrontMotor;
  private MAMotorControler rightMotor;

  private AHRS navx;

  private MAPidController anglePidMApath;
  private MAPidController accelerationPidMApath;

  private MAPidController anglePIDVision;
  private MAPidController distancePIDVision;

  private MAShuffleboard chassisShuffleboard;

  private double curentV = 0;
  private double prevV = 0;

  private double VInit = 0;
  private double xInit = 0;

  private static Chassis chassis;

  private Chassis() {
    chassisShuffleboard = new MAShuffleboard(ChassisConstants.KSUBSYSTEM_NAME);
    leftFrontMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID1, true, 0, false,
        ENCODER.Encoder);
    leftMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID2);
    rightFrontMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID3, false, 0, false,
        ENCODER.Encoder);
    rightMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID4);

    leftMotor.follow(leftFrontMotor);
    rightMotor.follow(rightFrontMotor);

    navx = new AHRS(Port.kMXP);

    // the distance PID Pathfinder
    accelerationPidMApath = new MAPidController(ChassisConstants.KP_MAPATH_DISTANCE,
        ChassisConstants.KI_MAPATH_DISTANCE, ChassisConstants.KD_MAPATH_DISTANCE, 0, 0, -12, 12);

    // the angle PID pathfinder
    anglePidMApath = new MAPidController(ChassisConstants.KP_MAPATH_ANGLE, ChassisConstants.KI_MAPATH_ANGLE,
        ChassisConstants.KD_MAPATH_ANGLE, 0, 0, -12, 12);

    // the angle PID vison
    anglePIDVision = new MAPidController(ChassisConstants.KP_VISION_ANGLE, ChassisConstants.KI_VISION_ANGLE,
        ChassisConstants.KD_VISION_ANGLE, 0, 2, -12, 12);

    anglePIDVision.enableContinuousInput(-ChassisConstants.KANGLE_PID_VISION_SET_INPUTRANGE,
        ChassisConstants.KANGLE_PID_VISION_SET_INPUTRANGE);

    anglePidMApath.enableContinuousInput(-ChassisConstants.KANGLE_PID_MAPATH_SET_INPUTRANGE,
        ChassisConstants.KANGLE_PID_MAPATH_SET_INPUTRANGE);

    distancePIDVision = new MAPidController(ChassisConstants.KP_VISION_DISTANCE, ChassisConstants.KI_VISION_DISTANCE,
        ChassisConstants.KD_VISION_DISTANCE, 0, 2, -12, 12);

    resetValue();

  }

  public void rampRate(double rampRate) {
    rightFrontMotor.configRampRate(rampRate);
    leftFrontMotor.configRampRate(rampRate);
  }

  public double averageDistance() {
    return ((leftFrontMotor.getPosition() + rightFrontMotor.getPosition()) / 2) / ChassisConstants.KTICKS_PER_METER;
  }

  public double averageRPM() {
    return (leftFrontMotor.getVelocity() + rightFrontMotor.getVelocity()) / 2;
  }

  public double getAngle() {
    return navx.getYaw();
  }

  public void setidilmodeBrake(boolean onOf) {
    leftFrontMotor.changeMood(onOf);
    leftMotor.changeMood(onOf);
    rightFrontMotor.changeMood(onOf);
    rightMotor.changeMood(onOf);
  }

  // resat the value of the encoder and the navx
  public void resetValue() {
    navx.zeroYaw();
    rightFrontMotor.resetEncoder();
    leftFrontMotor.resetEncoder();
  }

  // pid
  public void reset() {
    anglePIDVision.reset();
  }

  public double anglePIDVisionOutput(double setpoint) {
    return anglePIDVision.calculate(limelight.x * -1, setpoint);
  }

  public double distancePIDVisionOutput(double setpoint) {
    return distancePIDVision.calculate(limelight.Tshort, setpoint);
  }

  public void arcadeDrive(double angle, double distacne) {
    double w = (100 - Math.abs(angle * 100)) * (distacne) + distacne * 100;
    double v = (100 - Math.abs(distacne * 100)) * (angle) + angle * 100;
    double leftVoltage = (-(v + w) / 200);
    double rightVoltage = ((v - w) / 200);
    leftcontrol(leftVoltage);
    rightcontrol(rightVoltage);
  }

  // the PIDvison
  public void pidVisionAngle(double angleSetpoint) {
    double voltage = anglePIDVisionOutput(angleSetpoint);
    leftcontrol(-voltage);
    rightcontrol(voltage);

  }

  public boolean isPIDVisionOnTargetAngle() {
    return anglePIDVision.atSetpoint(0.1);
  }

  public boolean isPIDVisionOnTargetDistance() {
    return distancePIDVision.atSetpoint(0.1);
  }

  public void setpoint(double distancesetpoint, double anglesetpoint, double Speedlimitdistance,
      double Speedlimitangle) {

    anglePidMApath.setSetpoint(anglesetpoint);
    accelerationPidMApath.setSetpoint(distancesetpoint);

    accelerationPidMApath.setP(ChassisConstants.KP_MAPATH_DISTANCE * Speedlimitdistance);
    accelerationPidMApath.setD(ChassisConstants.KD_MAPATH_DISTANCE * Speedlimitdistance);

    anglePidMApath.setP(ChassisConstants.KP_MAPATH_ANGLE * Speedlimitangle);
    anglePidMApath.setD(ChassisConstants.KD_MAPATH_ANGLE * Speedlimitangle);
  }

  public double angleEror() {
    return anglePidMApath.getPositionError();
  }

  public double distanceEror() {
    return accelerationPidMApath.getPositionError();
  }

  public double angleMApathPIDOutput() {
    return anglePidMApath.calculate(getAngle());
  }

  public double distanceMApathPIDOutput() {
    return accelerationPidMApath.calculate(acceleration());
  }

  public void pathfinder() {
    try {
      double angle = chassis.angleMApathPIDOutput() * Path.mainPath[MAPath.stage][5];
      double distance = chassis.distanceMApathPIDOutput() * Path.mainPath[MAPath.stage][4];
      chassis.arcadeDrive(angle, distance);
    } catch (Exception e) {
      leftcontrol(0);
      rightcontrol(0);
    }
  }

  public void leftcontrol(double voltage) {
    leftFrontMotor.setVoltage(voltage);
  }

  public void rightcontrol(double voltage) {
    rightFrontMotor.setVoltage(voltage);
  }

  public static Chassis getinstance() {
    if (chassis == null) {
      chassis = new Chassis();
    }
    return chassis;
  }

  public double deltaV() {
    curentV = MACalculations.fromRPMToLinearSpeed(averageRPM(), ChassisConstants.KCHASSIS_GEAR);
    double delta = curentV - prevV;
    prevV = curentV;
    return delta;

  }

  public double acceleration() {
    return deltaV() / RobotConstants.KDELTA_TIME;
  }

  public double getAccelerationSetPoint(Point point) {
    VInit = averageRPM();
    if (Math.abs(VInit) < Math.abs(point.getVelocity())) {
      return (point.getVelocity() - VInit) / point.getAccelerationTime();
    } else {
      return 0;
    }
  }

  public void setSetPoint(Point point) {
    anglePidMApath.setSetpoint(point.getAngle());
    accelerationPidMApath.setSetpoint(getAccelerationSetPoint(point));

  }

  @Override
  public void periodic() {
    printValues();
  }

  public void printValues() {
    chassisShuffleboard.addBoolean("isPIDVisionOnTargetAngle", isPIDVisionOnTargetAngle());
    chassisShuffleboard.addNum("Stage", MAPath.stage);
    chassisShuffleboard.addNum("fixedAngle", getAngle());
    chassisShuffleboard.addNum("distacne", averageDistance());
    chassisShuffleboard.addNum("RPM", deltaV());
    chassisShuffleboard.addNum("acceleration", acceleration());
    chassisShuffleboard.addNum("angleSetPoint", anglePidMApath.getSetpoint());
    chassisShuffleboard.addNum("accelerationSetPoint", accelerationPidMApath.getSetpoint());
  }
}
