
package frc.robot.subsystems.Chassis;

/**
 * @author yuval rader
 */
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.limelight;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.Controlers.MAPidController;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;
import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.Autonomous.Point;
import frc.robot.utils.Calculation.MACalculations;
import frc.robot.commands.Chassis.MAPath;

public class Chassis extends MASubsystem {
  private MAMotorControler leftFrontMotor;
  private MAMotorControler leftMotor;

  private MAMotorControler rightFrontMotor;
  private MAMotorControler rightMotor;

  private AHRS navx;

  private MAPidController anglePidMApath;
  private MAPidController rightVelocityControl;
  private MAPidController leftVelocityControl;

  private MAPidController anglePIDVision;
  private MAPidController distancePIDVision;

  private MAShuffleboard chassisShuffleboard;

  private static Chassis chassis;

  private double velocity = 0;

  private Chassis() {
    chassisShuffleboard = new MAShuffleboard(ChassisConstants.KSUBSYSTEM_NAME);
    leftFrontMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID1, true, 0, false,
        ENCODER.Alternate_Encoder);
    leftMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID2, false, 0, false, ENCODER.Encoder);

    rightFrontMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID3, false, 0, false,
        ENCODER.Alternate_Encoder);
    rightMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID4, false, 0, false, ENCODER.Encoder);

    leftMotor.follow(leftFrontMotor);
    rightMotor.follow(rightFrontMotor);
    leftFrontMotor.phaseSensor(true);
    navx = new AHRS(Port.kMXP);

    // the distance PID Pathfinder
    rightVelocityControl = new MAPidController(ChassisConstants.KP_MAPATH_RIGHT_VELOCITY,
        ChassisConstants.KI_MAPATH_RIGHT_VELOCITY, ChassisConstants.KD_MAPATH_RiGHT_VELOCITY, 0, 20, -12, 12);

    leftVelocityControl = new MAPidController(ChassisConstants.KP_MAPATH_LEFT_VELOCITY,
        ChassisConstants.KI_MAPATH_LEFT_VELOCITY, ChassisConstants.KD_MAPATH_LEFT_VELOCITY, 0, 20, -12, 12);

    // the angle PID pathfinder
    anglePidMApath = new MAPidController(ChassisConstants.KP_MAPATH_ANGLE, ChassisConstants.KI_MAPATH_ANGLE,
        ChassisConstants.KD_MAPATH_ANGLE, 0, 2, -12, 12);

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

  public double leftDistance() {
    return leftFrontMotor.getPosition() / ChassisConstants.KTICKS_PER_METER;
  }

  public double rigthDistance() {
    return rightFrontMotor.getPosition() / ChassisConstants.KTICKS_PER_METER;
  }

  public double rightRPM() {
    return (rightMotor.getVelocity() + rightFrontMotor.getVelocity()) / 2;
  }

  public double leftRPM() {
    return (leftFrontMotor.getVelocity() + leftMotor.getVelocity()) / 2;
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

  public void reserEncoder() {
    velocity = 0;
    rightFrontMotor.resetEncoder();
    leftFrontMotor.resetEncoder();
  }

  // resat the value of the encoder and the navx
  public void resetValue() {
    navx.zeroYaw();
    reserEncoder();
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
    return anglePIDVision.atSetpoint();
  }

  public boolean isPIDVisionOnTargetDistance() {
    return distancePIDVision.atSetpoint();
  }

  public void setpoint(Point point, double InitLinearSpeed, double initDistacn, double initTata) {
    double angleSetPoint = tataFromDistance(point.getDistance(), point.getAngle(), initDistacn, initTata);

    if (point.getAngle() - getAngle() > 0) {
      rightVelocityControl.setSetpoint(MACalculations.fromLinearSpeedToRPM(
          accelerationVelocitySetPoint(InitLinearSpeed, point.getVelocity(), point.getAcceleration()),
          ChassisConstants.KCHASSIS_GEAR));

      leftVelocityControl.setSetpoint(MACalculations.fromLinearSpeedToRPM(
          theOtherVelocitySetPoint(angleSetPoint, InitLinearSpeed, point.getVelocity(), point.getAcceleration()),
          ChassisConstants.KCHASSIS_GEAR));

      anglePidMApath.setSetpoint(angleSetPoint);
    } else if (point.getAngle() - getAngle() < 0) {

      leftVelocityControl.setSetpoint(MACalculations.fromLinearSpeedToRPM(
          accelerationVelocitySetPoint(InitLinearSpeed, point.getVelocity(), point.getAcceleration()),
          ChassisConstants.KCHASSIS_GEAR));

      rightVelocityControl.setSetpoint(MACalculations.fromLinearSpeedToRPM(
          theOtherVelocitySetPoint(angleSetPoint, InitLinearSpeed, point.getVelocity(), point.getAcceleration()),
          ChassisConstants.KCHASSIS_GEAR));

      anglePidMApath.setSetpoint(angleSetPoint);
    } else {
      leftVelocityControl.setSetpoint(MACalculations.fromLinearSpeedToRPM(
          accelerationVelocitySetPoint(InitLinearSpeed, point.getVelocity(), point.getAcceleration()),
          ChassisConstants.KCHASSIS_GEAR));

      rightVelocityControl.setSetpoint(MACalculations.fromLinearSpeedToRPM(
          theOtherVelocitySetPoint(angleSetPoint, InitLinearSpeed, point.getVelocity(), point.getAcceleration()),
          ChassisConstants.KCHASSIS_GEAR));
      anglePidMApath.setSetpoint(angleSetPoint);
    }

  }

  public double angleEror() {
    return anglePidMApath.getPositionError();
  }

  public double velocityErorRight() {
    return rightVelocityControl.getPositionError();
  }

  public double velocityErorLeft() {
    return leftVelocityControl.getPositionError();
  }

  public boolean isPIDRightVelocityAtSetPoint() {
    return rightVelocityControl.atSetpoint();
  }

  public boolean isPIDLeftVelocityAtSetPoint() {
    return leftVelocityControl.atSetpoint();
  }

  public boolean isPIDAngleAtSetPoint() {
    return anglePidMApath.atSetpoint();
  }

  public double angleMApathPIDOutput() {
    return anglePidMApath.calculate(getAngle());
  }

  public double rightVelocityMApathPIDOutput() {
    return rightVelocityControl.calculate(rightRPM());
  }

  public double leftVelocityMApathPIDOutput() {
    return leftVelocityControl.calculate(leftRPM());
  }

  public double accelerationVelocitySetPoint(double LinearSpeed, double setPoint, double acceleration) {
    if (velocity + acceleration * RobotConstants.KDELTA_TIME >= setPoint) {
      return setPoint;
    } else {
      velocity = velocity + LinearSpeed + (acceleration * RobotConstants.KDELTA_TIME);
      return velocity;
    }

  }

  public double theOtherVelocitySetPoint(double tata, double LinearSpeed, double setPoint, double acceleration) {
    double V = accelerationVelocitySetPoint(LinearSpeed, setPoint, acceleration);
    if (tata > getAngle()) {
      return V - Math.sin(tata / 2) / ChassisConstants.KchassisLength;
    } else if (tata < getAngle()) {
      return V + Math.sin(tata / 2) / ChassisConstants.KchassisLength;
    }
    return V;
  }

  public double tataFromDistance(double maxDistacn, double maxTata, double initDistacn, double initTata) {
    double sight = Math.abs(maxTata) / maxTata;
    double a = maxDistacn * Math.cos(maxTata);
    if (Math.abs(initTata) + 10 > Math.abs(getAngle())) {
      return initTata + 10 * sight;
    }

    if (Math.abs(getAngle()) + 10 >= maxTata) {
      return maxTata;
    }

    if (maxTata - initTata > 0) {
      double H = rigthDistance() - initDistacn;
      return Math.toDegrees(Math.tan(H / a)) * -1;
    } else if (maxTata - initTata < 0) {
      double H = leftDistance() - initDistacn;
      return Math.toDegrees(Math.tan(H / a));
    }

    return 0;
  }

  public void pathMotorOutPut() {
    double leftPower = leftVelocityMApathPIDOutput();
    double rightPower = rightVelocityMApathPIDOutput();
    double anglePower = angleMApathPIDOutput();
    leftcontrol(leftPower - anglePower);
    rightcontrol(rightPower + anglePower);
  }

  public void leftcontrol(double voltage) {
    leftFrontMotor.setVoltage(voltage);
  }

  public void rightcontrol(double voltage) {
    rightFrontMotor.setVoltage(voltage);
  }

  public boolean atPoint(Point point) {
    if (Math.abs(point.getAngle()) > 0) {
      return Math.abs((point.getAngle() - getAngle())) < 2;
    } else {
      return Math.abs((point.getAngle() - getAngle())) < 2 && Math.abs(point.getDistance() - rigthDistance()) < 0.1
          && Math.abs(MACalculations.fromLinearSpeedToRPM(point.getVelocity(), ChassisConstants.KCHASSIS_GEAR)
              - rightRPM()) < 20
          && Math.abs(MACalculations.fromLinearSpeedToRPM(point.getVelocity(), ChassisConstants.KCHASSIS_GEAR)
              - leftRPM()) < 20;
    }
  }

  public static Chassis getinstance() {
    if (chassis == null) {
      chassis = new Chassis();
    }
    return chassis;
  }

  @Override
  public void periodic() {
    printValues();
  }

  public void printValues() {
    double angle = tataFromDistance(4, 180, 0, 0);
    chassisShuffleboard.addBoolean("isPIDVisionOnTargetAngle", isPIDVisionOnTargetAngle());
    chassisShuffleboard.addNum("Stage", MAPath.stage);
    chassisShuffleboard.addNum("fixedAngle", getAngle());
    chassisShuffleboard.addNum("angleSetPoint", anglePidMApath.getSetpoint());

    chassisShuffleboard.addNum("right distance", rigthDistance());
    chassisShuffleboard.addNum("left distance", leftDistance());
    chassisShuffleboard.addNum("right ticks", rightFrontMotor.getPosition());

    chassisShuffleboard.addNum("V1 setPoint",
        accelerationVelocitySetPoint(0, ChassisConstants.KMAX_SPEED, ChassisConstants.KMAX_ACCELERATION));
    chassisShuffleboard.addNum("V2 setPoint",
        theOtherVelocitySetPoint(angle, 0, ChassisConstants.KMAX_SPEED, ChassisConstants.KMAX_ACCELERATION));
    chassisShuffleboard.addNum("angle setPoint", angle);
  }
}
