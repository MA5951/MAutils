
package frc.robot.subsystems;

/**
 * @author yuval rader
 */

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.utils.*;
import frc.robot.utils.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;
import frc.robot.Path.Path;
import frc.robot.commands.Chassis.MAPath;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Chassis extends SubsystemBase implements MASubsystem {

  private static final double KP_MApath_distance = 1;
  private static final double KI_MApath_distance = 0;
  private static final double KD_MApath_distance = 0;

  private static final double KP_MApath_angle = 1.5e-1;
  private static final double KI_MApath_angle = 0;
  private static final double KD_MApath_angle = 1e-2;

  private static final double KP_Vision_angle = 2.5e-2;
  private static final double KI_Vision_angle = 8e-4;
  private static final double KD_Vision_angle = 0.5e-3;

  private static final double KP_Vision_distance = 1.6e-2;
  private static final double KI_Vision_distance = 0;
  private static final double KD_Vision_distance = 0;

  private static final double anglePIDVisionSetInputRange = 44.5;
  private static final double anglePidMApathSetInputRange = 180;

  SimpleMotorFeedforward feed;
  private double angle;
  private double sign;
  private double modle = sign;

  private MAMotorControler leftFrontMotor;
  private MAMotorControler leftMotor;

  private MAMotorControler rightFrontMotor;
  private MAMotorControler rightMotor;

  private AHRS navx;

  // private MAPidController distancePidMApath; // PID controler of the distance
  // in the pathfinder
  private MAPidController anglePidMApath; // PID controler of the angel in the pathfinder
  private ProfiledPIDController distancePidMApath;

  private MAPidController anglePIDVision; // the angel PID in the vison PID
  private MAPidController distancePIDVision;
  private static Chassis chassis;
private MAShuffleboard Chassis;
  private Chassis() {
   Chassis = new MAShuffleboard("Chassis");
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
    distancePidMApath = new ProfiledPIDController(KP_MApath_distance, KI_MApath_distance, KD_MApath_distance,
        new TrapezoidProfile.Constraints(RobotConstants.Max_Speed, RobotConstants.Max_acceleration));

    // the angel PID pathfinder
    anglePidMApath = new MAPidController(KP_MApath_angle, KI_MApath_angle, KD_MApath_angle, 0, 0, -12, 12);

    // the angel PID vison
    anglePIDVision = new MAPidController(KP_Vision_angle, KI_Vision_angle, KD_Vision_angle, 0, 2, -1, 1);

    anglePIDVision.enableContinuousInput(-anglePIDVisionSetInputRange, anglePIDVisionSetInputRange);

    anglePidMApath.enableContinuousInput(-anglePidMApathSetInputRange, anglePidMApathSetInputRange);

    distancePIDVision = new MAPidController(KP_Vision_distance, KI_Vision_distance, KD_Vision_distance, 0, 2, -1, 1);

    feed = new SimpleMotorFeedforward(0.163, 7.31e-5, 8.18e-6);

  }

  public double lefttVelocityControlRPM() {
    return leftFrontMotor.getVelocity();
  }

  public double rightVelocityControlRPM() {
    return rightFrontMotor.getVelocity();
  }

  public void rampRate(double rampRate) {
    rightFrontMotor.configRampRate(rampRate);
    rightMotor.configRampRate(0);
    leftFrontMotor.configRampRate(rampRate);
    leftMotor.configRampRate(0);
  }

  // the average of the encoders
  public double average() {
    return ((rightMotor.getPosition() + leftMotor.getPosition()) / 2) / RobotConstants.ticksPerMeter;
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
    rightFrontMotor.setvoltage(rightspped);
    leftFrontMotor.setvoltage(leftSpeed);
  }

  // resat the value of the encoder and the navx
  public void resetValue() {
    distancePidMApath.reset(0);
    navx.reset();
    leftMotor.resetEncoder();
    rightMotor.resetEncoder();
  }

  // pid vison distance
  public double visonDistance() {
    return (-1.3276 * Math.pow(10, 6)
        / (-2.43018 * Math.pow(limelight.getinstance().y, 2) + -101.265 * limelight.getinstance().y + -1854.19)); // TODO

  }

  // pid vosin
  public void reset() {
    anglePIDVision.reset();
  }

  public double anglePIDVisionOutput(double setpoint) {
    return anglePIDVision.calculate(limelight.getinstance().x * -1, setpoint);
  }

  public double distancePIDVisionOutput(double setpoint) {
    return distancePIDVision.calculate(limelight.getinstance().Tshort, setpoint);
  }

  public void ArcadeDrive(double angel, double distacne) {
    double w = (100 - Math.abs(angel * 100)) * (distacne) + distacne * 100;
    double v = (100 - Math.abs(distacne * 100)) * (angel) + angel * 100;
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

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // MAPath
  public void setpoint(double distancesetpoint, double anglesetpoint, double Speedlimitdistance,
      double Speedlimitangle) {
    anglePidMApath.setSetpoint(anglesetpoint);
    distancePidMApath.setGoal(distancesetpoint);

    distancePidMApath.setP(KP_MApath_distance * Speedlimitdistance);
    distancePidMApath.setD(KD_MApath_distance * Speedlimitdistance);

    anglePidMApath.setP(KP_MApath_angle * Speedlimitangle);
    anglePidMApath.setD(KD_MApath_angle * Speedlimitangle);
  }

  public double angleEror() {
    return anglePidMApath.getPositionError();
  }

  public void resatPIDVA() {
    distancePidMApath.reset(0, 0);
  }

  public double distanceEror() {
    return distancePidMApath.getGoal().position - average();
  }

  public double angleMApathPIDOutput() {
    return anglePidMApath.calculate(fixedAngle());
  }

  public double distanceMApathPIDOutput() {
    return MathUtil.clamp(
        distancePidMApath.calculate(average())
            + feed.calculate(distancePidMApath.getSetpoint().position, distancePidMApath.getSetpoint().velocity),
        -12.0, 12.0);
  }

  // MApath
  public void pathfinder() {
    try {
      double angel = chassis.angleMApathPIDOutput() * Path.mainPath[MAPath.stage][5];
      double distance = chassis.distanceMApathPIDOutput() * Path.mainPath[MAPath.stage][4];

      chassis.ArcadeDrive(angel, distance);
    } catch (Exception e) {
      chassis.tankDrive(0, 0);
    }
  }

  public void leftcontrol(double power) {
    leftFrontMotor.setvoltage(power);
  }

  public void rightcontrol(double power) {
    rightFrontMotor.setvoltage(power);
  }

  public static Chassis getinstance() {
    if (chassis == null) {
      chassis = new Chassis();
    }
    return chassis;
  }

  private double curentX = 0;
  private double prev_x = 0;

  public double DeltaX() {
    curentX = average();
    double delta = curentX - prev_x;
    prev_x = curentX;
    return delta;
  }

  private double curentV = 0;
  private double prev_v = 0;

  public double DeltaV() {
    curentV = DeltaX() / 0.02;
    double delta = curentV - prev_v;
    prev_v = curentV;
    return delta;

  }

  public double acceleration() {
    return DeltaV() / 0.02;
  }

  private double VInit = 0;
  private double xInit = 0;

  public double getAccelerationSetPoint(double Distance, double Time) {
    if (xInit >= Distance) {
      xInit = Distance;
    } else {
      xInit = average();
    }

    if (VInit >= RobotConstants.Max_Speed) {
      VInit = RobotConstants.Max_Speed;
    } else {
      VInit = (DeltaX() / 0.02) * Time;
    }
    double Acceleration = 2 * ((Distance - xInit) - VInit) / (Time * Time);
    if (Math.abs(Acceleration) >= RobotConstants.Max_acceleration) {
      return RobotConstants.Max_acceleration * (Math.abs(Acceleration) / Acceleration);
    } else if (DeltaX() / 0.02 >= RobotConstants.Max_Speed) {
      return 0;
    } else {
      return Acceleration;
    }

  }

  @Override
  public void periodic() {
    PrintValues();
    tankDrive(1, 1);
  }

  @Override
  public Runnable setMotorPower(double power, int Indax) {
    return () -> angleEror(); // TODO
  }


  // updat the value in the smart dash bord
  @Override
  public void PrintValues() {
    Chassis.getNum("rightPower", BuiltInWidgets.kNumberSlider);
    Chassis.addBoolean("isPIDVisionOnTargetAngle",isPIDVisionOnTargetAngle());
    Chassis.addNum("Stage", MAPath.stage);
    Chassis.addNum("fixedAngle",fixedAngle());
    Chassis.addNum("distacne",average());
    Chassis.addNum("lefttVelocityControlRPM",lefttVelocityControlRPM());
    Chassis.addNum("angleSetPoint",anglePidMApath.getSetpoint());
    Chassis.addNum("distacenSetPoint",distancePidMApath.getSetpoint().position);
    Chassis.addNum("LeftMotorOutPut",leftMotor.getOutput());
    Chassis.addNum("RightMotorOutPut", rightMotor.getOutput());
    Chassis.addNum("rightVelocityControlRPM",rightVelocityControlRPM());
    Chassis.addString("title", "hi");
    Chassis.PID("anglePidMApath", anglePidMApath);
    Chassis.PID("leftvel", anglePIDVision);
  }
}
