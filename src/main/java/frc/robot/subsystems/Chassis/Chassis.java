
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
import frc.robot.utils.Autonomous.Path;
import frc.robot.utils.Autonomous.Point;
import frc.robot.utils.Autonomous.Point.pointState;
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

  private double distacne = 0;

  private static Chassis chassis;

  private Chassis() {
    chassisShuffleboard = new MAShuffleboard(ChassisConstants.KSUBSYSTEM_NAME);
    leftFrontMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless,ID1, false, 0, false,
        ENCODER.Encoder);
    leftMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless,ID2, false, 0, false, ENCODER.Encoder);

    rightFrontMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless,ID3, true, 0, false,
        ENCODER.Encoder);
    rightMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless,ID4, false, 0, false, ENCODER.Encoder);

    leftMotor.follow(leftFrontMotor);
    rightMotor.follow(rightFrontMotor);

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
    return ((leftFrontMotor.getPosition() + leftMotor.getPosition()) / 2) / ChassisConstants.KTICKS_PER_METER;
  }

  public double rigthDistance() {
    return ((rightFrontMotor.getPosition() + rightMotor.getPosition()) / 2) / ChassisConstants.KTICKS_PER_METER;
  }

  public double rightRPM() {
    return (rightMotor.getVelocity() + rightFrontMotor.getVelocity()) / 2;
  }

  public double leftRPM() {
    return (leftMotor.getVelocity() + leftFrontMotor.getVelocity()) / 2;
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
    rightMotor.resetEncoder();
    leftFrontMotor.resetEncoder();
    leftMotor.resetEncoder();
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

  public void setpoint(Point point) {

    double angleSetPoint = Autonomous.thetaFromDistance(distacne, point);
    double angleSetPointPID = point.getLastPoint().getAngle() + angleSetPoint;

    double accelerationVelocitySetPoint = MACalculations
        .fromLinearSpeedToRPM(Autonomous.accelerationVelocitySetPoint(point), ChassisConstants.KCHASSIS_GEAR);

    double theOtherVelocitySetPoint = MACalculations.fromLinearSpeedToRPM(
        Autonomous.theOtherVelocitySetPoint(accelerationVelocitySetPoint, point), ChassisConstants.KCHASSIS_GEAR);

    if (point.getState() == pointState.RIGHT) {
      distacne = leftDistance() - point.getLastPoint().getArcDistance();
      rightVelocityControl.setSetpoint(theOtherVelocitySetPoint);
      leftVelocityControl.setSetpoint(accelerationVelocitySetPoint);
      anglePidMApath.setSetpoint(angleSetPointPID);

    } else if (point.getState() == pointState.LEFT) {
      distacne = rigthDistance() - point.getLastPoint().getArcDistance();
      rightVelocityControl.setSetpoint(theOtherVelocitySetPoint);
      leftVelocityControl.setSetpoint(accelerationVelocitySetPoint);
      anglePidMApath.setSetpoint(angleSetPointPID);

    } else if (point.getState() == pointState.STRAIGHT_LINE) {
      leftVelocityControl.setSetpoint(accelerationVelocitySetPoint);
      rightVelocityControl.setSetpoint(accelerationVelocitySetPoint);
      anglePidMApath.setSetpoint(point.getAngle());
    } else {
      double sight = point.getAngle() / Math.abs(point.getAngle());
      rightVelocityControl.setSetpoint(accelerationVelocitySetPoint);
      leftVelocityControl.setSetpoint(accelerationVelocitySetPoint * sight);
      anglePidMApath.setSetpoint(point.getAngle());
    }

  }

  public boolean atPoint(Point point) {
    if (point.getState() == pointState.RIGHT) {
      return Math.abs(point.getAngle() - getAngle()) < 10 && Math.abs(point.getArcDistance() - leftDistance()) < 0.3
          && Math.abs(point.getEndVelocity()
              - MACalculations.fromRPMToLinearSpeed(leftRPM(), ChassisConstants.KCHASSIS_GEAR)) < 0.3;
    }

    if (point.getState() == pointState.LEFT) {
      return Math.abs(point.getAngle() - getAngle()) < 10 && Math.abs(point.getArcDistance() - rigthDistance()) < 0.3
          && Math.abs(point.getEndVelocity()
              - MACalculations.fromRPMToLinearSpeed(rightRPM(), ChassisConstants.KCHASSIS_GEAR)) < 0.3;
    }

    return Math.abs(point.getAngle() - getAngle()) < 5
        && Math.abs(point.getDistance() - (rigthDistance() + leftDistance()) / 2) < 0.3
        && Math.abs(point.getEndVelocity()
            - MACalculations.fromRPMToLinearSpeed((rightRPM() + leftRPM()) / 2, ChassisConstants.KCHASSIS_GEAR)) < 0.5;
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

  public void pathMotorOutPut(Point point) {
    leftVelocityControl.setF((leftVelocityControl.getSetpoint() / RobotConstants.KMAX_RPM_NEO) * 12);
    rightVelocityControl.setF((rightVelocityControl.getSetpoint() / RobotConstants.KMAX_RPM_NEO) * 12);
    double leftPower = leftVelocityMApathPIDOutput();
    double rightPower = rightVelocityMApathPIDOutput();
    double anglePower = angleMApathPIDOutput();

    if (point.getState() == pointState.RIGHT) {
      leftcontrol(leftPower + anglePower);
      rightcontrol(rightPower - anglePower);
    } else if (point.getState() == pointState.LEFT) {
      leftcontrol(leftPower + anglePower);
      rightcontrol(rightPower - anglePower);
    } else if (point.getState() == pointState.STRAIGHT_LINE) {
      leftcontrol(leftPower);
      rightcontrol(rightPower);
    } else {
      leftcontrol(-anglePower);
      rightcontrol(anglePower);
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

  @Override
  public void periodic() {

    printValues();
  }

  public void printValues() {
    chassisShuffleboard.addNum("Stage", MAPath.stage);
    chassisShuffleboard.addNum("fixedAngle", getAngle());
    chassisShuffleboard.addNum("right distance", rigthDistance());
    chassisShuffleboard.addNum("left distance", leftDistance());
    chassisShuffleboard.addString("state", MAPath.point.getState().toString());
    chassisShuffleboard.addBoolean("isAutoEnd", atPoint(Path.teast[Path.teast.length - 1]));

    chassisShuffleboard.addNum("lefttVelocityControlPositionError", leftVelocityControl.getPositionError());
    chassisShuffleboard.addNum("rightVelocityControlPositionError", rightVelocityControl.getPositionError());
    chassisShuffleboard.addNum("lefttVelocityControlSetPoint", leftVelocityControl.getSetpoint());
    chassisShuffleboard.addNum("rightVelocityControlSetPoint", rightVelocityControl.getSetpoint());

    chassisShuffleboard.addNum("angle setPoint", anglePidMApath.getSetpoint());

    chassisShuffleboard.addNum("right power", rightFrontMotor.getOutput());
    chassisShuffleboard.addNum("left power", leftFrontMotor.getOutput());

    chassisShuffleboard.addNum("right RPM", rightRPM());
    chassisShuffleboard.addNum("left RPM", leftRPM());

    chassisShuffleboard.addNum("right velocity",
        MACalculations.fromRPMToLinearSpeed(rightRPM(), ChassisConstants.KCHASSIS_GEAR));
    chassisShuffleboard.addNum("left velocity",
        MACalculations.fromRPMToLinearSpeed(leftRPM(), ChassisConstants.KCHASSIS_GEAR));

  }
}
