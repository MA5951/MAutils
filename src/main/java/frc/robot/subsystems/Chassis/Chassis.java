
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
import frc.robot.utils.controlers.MAPidController;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;
import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.Autonomous.Path;
import frc.robot.commands.Chassis.MAPath;

public class Chassis extends MASubsystem {
  private double angle;
  private double sign;
  private double modle;

  private MAMotorControler leftFrontMotor;
  private MAMotorControler leftMotor;

  private MAMotorControler rightFrontMotor;
  private MAMotorControler rightMotor;

  private AHRS navx;

  private MAPidController anglePidMApath;
  private MAPidController distancePidMApath;

  private MAPidController anglePIDVision;
  private MAPidController distancePIDVision;
  private static Chassis chassis;
  private MAShuffleboard Chassis;

  private Chassis() {
    Chassis = new MAShuffleboard(ChassisConstants.KSUBSYSTEM_NAME);
    leftFrontMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID1, true, 0, false,
        ENCODER.Encoder);
    leftMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID2, false, 0, false,
        ENCODER.Alternate_Encoder);
    rightFrontMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushled, IDMotor.ID3, false, 0, false,
        ENCODER.Encoder);
    rightMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushled, IDMotor.ID4, false, 0, false,
        ENCODER.Alternate_Encoder);

    rightMotor.PhaseSensor(true);
    leftMotor.follow(leftFrontMotor);
    rightMotor.follow(rightFrontMotor);

    navx = new AHRS(Port.kMXP);

    // the distance PID Pathfinder
    distancePidMApath = new MAPidController(ChassisConstants.KP_MAPATH_DISTANCE, ChassisConstants.KI_MAPATH_DISTANCE,
        ChassisConstants.KD_MAPATH_DISTANCE, 0, 0, -1, 1);

    // the angle PID pathfinder
    anglePidMApath = new MAPidController(ChassisConstants.KP_MAPATH_ANGLE, ChassisConstants.KI_MAPATH_ANGLE,
        ChassisConstants.KD_MAPATH_ANGLE, 0, 0, -1, 1);

    // the angle PID vison
    anglePIDVision = new MAPidController(ChassisConstants.KP_VISION_ANGLE, ChassisConstants.KI_VISION_ANGLE,
        ChassisConstants.KD_VISION_ANGLE, 0, 2, -1, 1);

    anglePIDVision.enableContinuousInput(-ChassisConstants.KANGLE_PID_VISION_SET_INPUTRANGE,
        ChassisConstants.KANGLE_PID_VISION_SET_INPUTRANGE);

    anglePidMApath.enableContinuousInput(-ChassisConstants.KANGLE_PID_MAPATH_SET_INPUTRANGE,
        ChassisConstants.KANGLE_PID_MAPATH_SET_INPUTRANGE);

    distancePIDVision = new MAPidController(ChassisConstants.KP_VISION_DISTANCE, ChassisConstants.KI_VISION_DISTANCE,
        ChassisConstants.KD_VISION_DISTANCE, 0, 2, -1, 1);

  }

  public void rampRate(double rampRate) {
    rightFrontMotor.configRampRate(rampRate);
    rightMotor.configRampRate(0);
    leftFrontMotor.configRampRate(rampRate);
    leftMotor.configRampRate(0);
  }

  // the average of the encoders
  public double average() {
    return ((rightMotor.getPosition() + leftMotor.getPosition()) / 2) / ChassisConstants.KTICKS_PER_METER;
  }

  public double averageRPM() {
    return ((rightMotor.getVelocity() + leftMotor.getVelocity()) / 2);
  }

  public double fixedAngle() {
    if (navx.getYaw() != 0) {
      angle = navx.getYaw();
      sign = angle / Math.abs(angle);
      modle = sign * (Math.abs(angle) % 360);
      return -((180 - modle) % 360) + 180;
    } else {
      return 0;
    }
  }

  public void setidilmodeBrake(boolean onOf) {
    leftFrontMotor.changeMood(onOf);
    leftMotor.changeMood(onOf);
    rightFrontMotor.changeMood(onOf);
    rightMotor.changeMood(onOf);
  }

  // set the left and the right motors powers
  public void tankDrive(double leftSpeed, double rightspped) {
    rightFrontMotor.set(rightspped);
    leftFrontMotor.set(leftSpeed);
  }

  // resat the value of the encoder and the navx
  public void resetValue() {
    navx.reset();
    leftMotor.resetEncoder();
    rightMotor.resetEncoder();
  }

  // pid vison distance
  public double visonDistance() {
    return (-1.3276 * Math.pow(10, 6) / (-2.43018 * Math.pow(limelight.y, 2) + -101.265 * limelight.y + -1854.19)); // TODO

  }

  // pid vosin
  public void reset() {
    anglePIDVision.reset();
  }

  public double anglePIDVisionOutput(double setpoint) {
    return anglePIDVision.calculate(limelight.x * -1, setpoint);
  }

  public double distancePIDVisionOutput(double setpoint) {
    return distancePIDVision.calculate(limelight.Tshort, setpoint);
  }

  public void ArcadeDrive(double angle, double distacne) {
    double w = (100 - Math.abs(angle * 100)) * (distacne) + distacne * 100;
    double v = (100 - Math.abs(distacne * 100)) * (angle) + angle * 100;
    tankDrive((-(v + w) / 200), ((v - w) / 200));
  }

  // the PIDvison
  public void PIDvisionAngle(double angleSetpoint) {
    double power = anglePIDVisionOutput(angleSetpoint);
    tankDrive(-power, power);
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
    distancePidMApath.setSetpoint(distancesetpoint);

    distancePidMApath.setP(ChassisConstants.KP_MAPATH_DISTANCE * Speedlimitdistance);
    distancePidMApath.setD(ChassisConstants.KD_MAPATH_DISTANCE * Speedlimitdistance);

    anglePidMApath.setP(ChassisConstants.KP_MAPATH_ANGLE * Speedlimitangle);
    anglePidMApath.setD(ChassisConstants.KD_MAPATH_ANGLE * Speedlimitangle);
  }

  public double angleEror() {
    return anglePidMApath.getPositionError();
  }

  public double distanceEror() {
    return distancePidMApath.getPositionError();
  }

  public double angleMApathPIDOutput() {
    return anglePidMApath.calculate(fixedAngle());
  }

  public double distanceMApathPIDOutput() {
    return distancePidMApath.calculate(average());
  }

  public void pathfinder() {
    try {
      double angle = chassis.angleMApathPIDOutput() * Path.mainPath[MAPath.stage][5];
      double distance = chassis.distanceMApathPIDOutput() * Path.mainPath[MAPath.stage][4];
      chassis.ArcadeDrive(angle, distance);
    } catch (Exception e) {
      chassis.tankDrive(0, 0);
    }
  }

  public void leftcontrol(double power) {
    leftFrontMotor.set(power);
  }

  public void rightcontrol(double power) {
    rightFrontMotor.set(power);
  }

  public static Chassis getinstance() {
    if (chassis == null) {
      chassis = new Chassis();
    }
    return chassis;
  }

  private double curentV = 0;
  private double prev_v = 0;

  public double DeltaV() {
    curentV = MACalculations.FromRPMToLinearSpeed(averageRPM(), ChassisConstants.KCHASSIS_GEAR)
        / RobotConstants.KDELTA_TIME;
    double delta = curentV - prev_v;
    prev_v = curentV;
    return delta;

  }

  public double acceleration() {
    return DeltaV() / RobotConstants.KDELTA_TIME;
  }

  private double VInit = 0;
  private double xInit = 0;

  public double getAccelerationSetPoint(double Distance, double Time) {
    if (xInit >= Distance) {
      xInit = Distance;
    } else {
      xInit = average();
    }

    if (VInit >= ChassisConstants.KMAX_SPEED) {
      VInit = ChassisConstants.KMAX_SPEED;
    } else {
      VInit = MACalculations.FromRPMToLinearSpeed(averageRPM(), ChassisConstants.KCHASSIS_GEAR) * Time;
    }
    double Acceleration = 2 * ((Distance - xInit) - VInit) / (Time * Time);
    if (Math.abs(Acceleration) >= ChassisConstants.KMAX_ACCELERATION) {
      return ChassisConstants.KMAX_ACCELERATION * (Math.abs(Acceleration) / Acceleration);
    } else if (MACalculations.FromRPMToLinearSpeed(averageRPM(),
        ChassisConstants.KCHASSIS_GEAR) >= ChassisConstants.KMAX_SPEED) {
      return 0;
    } else {
      return Acceleration;
    }

  }

  @Override
  public void periodic() {
    PrintValues();
  }

  public void PrintValues() {
    Chassis.addBoolean("isPIDVisionOnTargetAngle", isPIDVisionOnTargetAngle());
    Chassis.addNum("Stage", MAPath.stage);
    Chassis.addNum("fixedAngle", fixedAngle());
    Chassis.addNum("distacne", average());
    Chassis.addNum("angleSetPoint", anglePidMApath.getSetpoint());
    Chassis.addNum("distacenSetPoint", distancePidMApath.getSetpoint());
  }
}
