package frc.robot.commands.Chassis;

import java.io.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Chassis.Chassis;
import frc.robot.utils.autonomous.OdometryHandler;
import frc.robot.utils.autonomous.Path;
import frc.robot.utils.autonomous.PathFollower;
import frc.robot.utils.autonomous.PathGenerator;
import frc.robot.utils.autonomous.Waypoint;

public class AutonomousCommand extends CommandBase {

  PathGenerator pathGenerator;
  PathFollower pathFollower;
  OdometryHandler odometry;
  double leftSetPointVelocity;
  double rightSetPointVelocity;

  double leftVelocity;
  double rightVelocity;

  double maxVelocity = 2.2; // 2.3
  double maxAcceleration = 0.33;

  private double sign;

  private Chassis chassis;
  private static ArrayList<Waypoint> autoPath;

  public AutonomousCommand(Path path) {
    chassis = Chassis.getinstance();
    odometry = chassis.getOdomoteryHandler();
    
    autoPath = new ArrayList<Waypoint>();
    autoPath.add(new Waypoint(0, 0));
    autoPath.add(new Waypoint(0, 3));
    autoPath.add(new Waypoint(0.25, 3.5));
    autoPath.add(new Waypoint(0.5, 4));
    autoPath.add(new Waypoint(1.5, 4));
    autoPath.add(new Waypoint(1.75, 3.5));
    autoPath.add(new Waypoint(2, 3));
    autoPath.add(new Waypoint(2, 2.5));
    autoPath.add(new Waypoint(2, 1.5));
    autoPath.add(new Waypoint(2, 0));


    pathGenerator = new PathGenerator(path.spacing, path.k, path.maxVelocity, path.maxAcceleration);
    List<Waypoint> waypoints = (List<Waypoint>) pathGenerator.calculate(autoPath);

    try {
      FileOutputStream fos = new FileOutputStream("generate path.tmp");
      ObjectOutputStream file = new ObjectOutputStream(fos);
      file.writeObject(path);
      file.close();
    } catch (Exception e){
      System.err.println("Error: " + e.getMessage());
    }
    pathFollower = new PathFollower(waypoints, odometry, path.lookaheadDistance, 
                                    path.maxRate, 0.75);    
    addRequirements(chassis);
  }


  @Override
  public void initialize()  {
    System.out.println("init");
    chassis.resetSensors();
    chassis.setIdleMode(NeutralMode.Brake);
  }

  @Override
  public void execute() {
    for (int i = 0; i < path.size(); i++) {
      chassis.chassisShuffleboard.addString("Point " + i, path.get(i).toString());
    }

    chassis.chassisShuffleboard.addString("closest point", pathFollower.getClosestPoint().toString());
    chassis.chassisShuffleboard.addString("lookahead point", pathFollower.getLookaheadPoint().toString());


    double[] speeds = pathFollower.getSpeeds();
    
    chassis.chassisShuffleboard.addNum("linear left speed", speeds[0]);
    chassis.chassisShuffleboard.addNum("linear right speed", speeds[1]);

    leftSetPointVelocity  = speeds[0];
    rightSetPointVelocity = speeds[1];

    chassis.chassisShuffleboard.addNum("left speed", leftSetPointVelocity);
    chassis.chassisShuffleboard.addNum("right speed", rightSetPointVelocity);

    // chassis.setLeftVelocitySetpoint(leftSetPointVelocity);
    // chassis.setRightVelocitySetpoint(-rightSetPointVelocity);

    sign = Math.signum(odometry.getCurrentPosition().y - pathFollower.getLookaheadPoint().y);

    chassis.setLeftVelocitySetpoint(leftSetPointVelocity);
    chassis.setRightVelocitySetpoint(rightSetPointVelocity);

    leftVelocity  = MathUtil.clamp(chassis.leftVelocityMApathPIDOutput()+chassis.getLeftF(), -1, 1);
    rightVelocity = MathUtil.clamp(chassis.rightVelocityMApathPIDOutput()+chassis.getRightF(), -1, 1);

    // System.out.println("left: " + []\
    // leftVelocity + "right: " + rightVelocity);

    chassis.setLeftPercent(leftVelocity + 0.05);
    chassis.setRightPercent(rightVelocity + 0.05);
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setLeftVoltage(0);
    chassis.setRightVoltage(0);
    System.out.println("end");
    System.out.println(chassis.getLeftEncoder());
    System.out.println(chassis.getRightEncoder());
    System.out.println(chassis.getAngle());
  }

  @Override
  public boolean isFinished() {
    //return false;
    return pathFollower.isDone();
  }
}
