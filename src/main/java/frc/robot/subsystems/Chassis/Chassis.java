
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
import frc.robot.utils.Autonomous.Autonomous;
import frc.robot.utils.Autonomous.Point;
import frc.robot.utils.Autonomous.Autonomous.autonomousState;
import frc.robot.utils.Calculation.MACalculations;
import frc.robot.Robot;
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
  public static autonomousState state;

  private double velocity = 0;
  private double timeInPoint = 0;
  private double rightCircelRadius = 0;
  private double leftCircelRadius = 0;
  private double timeInMaxSpeed = 0;

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
    return rightMotor.getVelocity();
  }

  public double leftRPM() {
    return leftMotor.getVelocity();
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

  public void reserValueForAutonomous() {
    velocity = 0;
    timeInPoint = 0;
    leftCircelRadius = 0;
    rightCircelRadius = 0;
 
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
    return anglePIDVision.atSetpoint();
  }

  public boolean isPIDVisionOnTargetDistance() {
    return distancePIDVision.atSetpoint();
  }

  public void setCircelRaduis() {
    if (state == autonomousState.RIGHT) {
      leftCircelRadius = 0;// TODO
      rightCircelRadius = leftCircelRadius + ChassisConstants.KchassisLength;
    } else if (state == autonomousState.LEFT) {
      rightCircelRadius = 0;// TODO
      leftCircelRadius = rightCircelRadius + ChassisConstants.KchassisLength;
    } else if (state == autonomousState.TURN_IN_PLACE) {
      leftCircelRadius = 0;
      rightCircelRadius = leftCircelRadius;
    } else {
      leftCircelRadius = 0;
      rightCircelRadius = leftCircelRadius;
    }
  }

  public void setpoint(Point point, double InitLinearSpeed, double initDistacn, double initTata) {

    timeInPoint = +RobotConstants.KDELTA_TIME;

    double angleSetPoint = tataFromDistance(point.getDistance(), point.getAngle(), initDistacn, initTata);

    double accelerationVelocitySetPoint = MACalculations
        .fromLinearSpeedToRPM(accelerationVelocitySetPoint(InitLinearSpeed, point), ChassisConstants.KCHASSIS_GEAR);

    double theOtherVelocitySetPoint = MACalculations.fromLinearSpeedToRPM(
        theOtherVelocitySetPoint(accelerationVelocitySetPoint, angleSetPoint), ChassisConstants.KCHASSIS_GEAR);

    if (state == autonomousState.RIGHT) {
      rightVelocityControl.setSetpoint(accelerationVelocitySetPoint);
      leftVelocityControl.setSetpoint(theOtherVelocitySetPoint);
      anglePidMApath.setSetpoint(angleSetPoint);

    } else if (state == autonomousState.LEFT) {
      leftVelocityControl.setSetpoint(accelerationVelocitySetPoint);
      rightVelocityControl.setSetpoint(theOtherVelocitySetPoint);
      anglePidMApath.setSetpoint(angleSetPoint);

    } else if (state == autonomousState.STRAIGHT_LINE) {
      leftVelocityControl.setSetpoint(accelerationVelocitySetPoint);
      rightVelocityControl.setSetpoint(accelerationVelocitySetPoint);
      anglePidMApath.setSetpoint(point.getAngle());

    } else {
      rightVelocityControl.setSetpoint(accelerationVelocitySetPoint);
      leftVelocityControl.setSetpoint(theOtherVelocitySetPoint);
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

  public double accelerationVelocitySetPoint(double LinearSpeed, Point point) {

    double accelerationTimeToSetPoint = Math
        .abs((point.getMaxVelocity() - point.getEndVelocity() / point.getAcceleration()));

    double accelerationTimeToMaxSpeed = Math.abs((point.getEndVelocity() - LinearSpeed) / point.getAcceleration());

    double distacnePassInAccelerationMove = LinearSpeed * accelerationTimeToMaxSpeed
        + (point.getAcceleration() / 2) * Math.pow(accelerationTimeToMaxSpeed, 2);

    double distacnePassInAccelerationToSetPoint = point.getEndVelocity() * accelerationTimeToSetPoint
        + (-point.getAcceleration() / 2) * Math.pow(accelerationTimeToSetPoint, 2);

    if (state == autonomousState.RIGHT || state == autonomousState.LEFT) {
      timeInMaxSpeed = Math.abs(point.getArcDistance()
          - (distacnePassInAccelerationToSetPoint + distacnePassInAccelerationMove) / point.getEndVelocity());
    } else {
      timeInMaxSpeed = Math.abs(point.getDistance()
          - (distacnePassInAccelerationToSetPoint + distacnePassInAccelerationMove) / point.getEndVelocity());
    }

    if (timeInPoint < accelerationTimeToMaxSpeed || Math
        .abs((velocity + point.getAcceleration() * RobotConstants.KDELTA_TIME)) < Math.abs(point.getMaxVelocity())) {
      velocity = velocity + LinearSpeed + (point.getAcceleration() * RobotConstants.KDELTA_TIME);
      return velocity;
    } else if (timeInPoint + RobotConstants.KDELTA_TIME < timeInMaxSpeed) {
      return point.getMaxVelocity();
    } else {
      if (Math.abs(velocity) <= Math.abs(point.getEndVelocity()))
        return point.getEndVelocity();
      else {
        velocity = velocity - (point.getAcceleration() * RobotConstants.KDELTA_TIME);
        return velocity;
      }
    }
  }

  public double theOtherVelocitySetPoint(double V, double tata) {
    double sight = Math.abs(V) / V;
    double tataToRadians = Math.toRadians(tata);
    if (state == autonomousState.RIGHT || state == autonomousState.LEFT)
      return V - ((2 * ChassisConstants.KchassisLength / Math.tan(tataToRadians)) * sight);

    if (state == autonomousState.TURN_IN_PLACE) {
      return -V;
    }

    return V;
  }

  public double tataFromDistance(double maxDistacn, double maxTata, double initDistacn, double initTata) {
    double sight = Math.abs(maxTata) / maxTata;
    double a = maxDistacn * Math.cos(maxTata);
    if (Math.abs(initTata) + 10 > Math.abs(getAngle())) {
      return initTata + 10 * sight;
    }

    if (Math.abs(getAngle()) + 10 >= Math.abs(maxTata)) {
      return maxTata;
    }

    if (state == autonomousState.RIGHT) {
      double H = rigthDistance() - initDistacn;
      return Math.toDegrees(Math.atan(H / a));
    } else if (state == autonomousState.LEFT) {
      double H = leftDistance() - initDistacn;
      return Math.toDegrees(Math.atan(H / a)) * -1;
    }

    return 0;
  }

  public void setAutonomousState(Point point) {
    state = Autonomous.getState(point.getAngle(), getAngle(), point.getDistance()); // TODO
  }

  public void pathMotorOutPut() {
    leftVelocityControl.setF((leftVelocityControl.getSetpoint() / RobotConstants.KMAX_RPM_NEO) * 12);
    rightVelocityControl.setF((rightVelocityControl.getSetpoint() / RobotConstants.KMAX_RPM_NEO) * 12);
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
    if (state == autonomousState.RIGHT) {
      return point.getAngle() == getAngle() && Math.abs(point.getArcDistance() - leftDistance()) < 0.2
          && Math.abs(point.getEndVelocity()
              - MACalculations.fromRPMToLinearSpeed(leftRPM(), ChassisConstants.KCHASSIS_GEAR)) < 0.1;
    } else if (state == autonomousState.LEFT) {
      return point.getAngle() == getAngle() && Math.abs(point.getArcDistance() - rigthDistance()) < 0.2
          && Math.abs(point.getEndVelocity()
              - MACalculations.fromRPMToLinearSpeed(rightRPM(), ChassisConstants.KCHASSIS_GEAR)) < 0.1;
    } else {
      return point.getAngle() == getAngle()
          && Math.abs(point.getDistance() - (rigthDistance() + leftDistance()) / 2) < 0.2
          && Math.abs(point.getEndVelocity() - MACalculations.fromRPMToLinearSpeed((rightRPM() + leftRPM()) / 2,
              ChassisConstants.KCHASSIS_GEAR)) < 0.1;
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
    chassisShuffleboard.addNum("Stage", MAPath.stage);
    chassisShuffleboard.addNum("fixedAngle", getAngle());
    chassisShuffleboard.addNum("right distance", rigthDistance());
    chassisShuffleboard.addNum("left distance", leftDistance());

  }
}
