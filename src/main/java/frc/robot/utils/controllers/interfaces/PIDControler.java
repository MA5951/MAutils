// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.controllers.interfaces;

/** Add your docs here. */
public interface PIDControler extends Controler {
    public void setOutputRange(double low, double high);

    public void setPID(double kp, double ki, double kd);

    public void setI(double ki);

    public void setP(double kp);

    public void setD(double kd);

    public void setF(double kf);

    public double getP();

    public double getI();

    public double getD();

    public double getF();

    public void enableContinuousInput(double minimumInput, double maximumInput);

    public void disableContinuousInput();

    public void setIntegratorRange(double minimumIntegral, double maximumIntegral);
}
