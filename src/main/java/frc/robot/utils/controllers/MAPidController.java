
package frc.robot.utils.controllers;

/**
 * @author yuval rader
 */


import edu.wpi.first.math.controller.PIDController;
import frc.robot.utils.controllers.interfaces.PIDControler;
import edu.wpi.first.math.MathUtil;

public class MAPidController implements PIDControler {
    private PIDController pidController;
    private double KP;
    private double KI;
    private double KD;
    private double KF;
    private double tolerance;
    private double low;
    private double high;
    /**
     * Allocates a PIDController with the given constants for Kp, Ki, Kd and KF and
     * a default period of 0.02 seconds.
     * 
     * @param KP        The proportional coefficient.
     * @param KI        The integral coefficient.
     * @param KD        The derivative coefficient.
     * @param KF        The feed forward.
     * @param tolerance Sets the error which is considered tolerable for use with
     *                  atSetpoint()
     * @param low       The lower boundary to which to clamp value.
     * @param high      The higher boundary to which to clamp value.
     * 
     */
    public MAPidController(double KP, double KI, double KD, double KF, double tolerance, double low, double high) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KF = KF;
        this.high = high;
        this.low = low;
        this.tolerance = tolerance;
        pidController = new PIDController(this.KP, this.KI, this.KD);
        pidController.setTolerance(this.tolerance);

    }

    /**
     * Allocates a PIDController with the given constants for Kp, Ki, and Kd and a
     * default period of 0.02 seconds.
     *
     * 
     * @param KP        The proportional coefficient.
     * @param KI        The integral coefficient.
     * @param KD        The derivative coefficient.
     * @param tolerance Sets the error which is considered tolerable for use with
     *                  atSetpoint()
     */
    public MAPidController(double KP, double KI, double KD, double tolerance) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.tolerance = tolerance;
        low = -1;
        high = 1;
        KF = 0;
        pidController = new PIDController(this.KP, this.KI, this.KD);
        pidController.setTolerance(this.tolerance);

    }

    /**
     * set the OutPutRange of the PIDController
     * 
     * @param low  The lower boundary to which to clamp value.
     * @param high The higher boundary to which to clamp value.
     * 
     */
    public void setOutputRange(double low, double high) {
        this.low = low;
        this.high = high;
    }

    /**
     * Sets the setpoint for the PIDController.
     *
     * @param setpoint The desired setpoint.
     */
    public void setSetpoint(double setPoint) {
        pidController.setSetpoint(setPoint);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint    The new setpoint of the controller.
     */

    public double calculate(double measurement, double setPoint) {
        return MathUtil.clamp(pidController.calculate(measurement, setPoint) + KF, low, high);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     */
    public double calculate(double measurement) {
        return MathUtil.clamp(pidController.calculate(measurement) + KF, low, high);
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>
     * Set the proportional, integral, and differential coefficients.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     */
    public void setPID(double kp, double ki, double kd) {
        pidController.setPID(kp, ki, kd);

    }

    /**
     * Sets the Proportional coefficient of the PID controller gain.
     *
     * @param kP proportional coefficient
     */
    public void setP(double kP) {
        pidController.setP(kP);
    }

    /**
     * Sets the Integral coefficient of the PID controller gain.
     *
     * @param ki integral coefficient
     */
    public void setI(double ki) {
        pidController.setI(ki);
    }

    /**
     * Sets the Differential coefficient of the PID controller gain.
     *
     * @param kd differential coefficient
     */
    public void setD(double kd) {
        pidController.setD(kd);
    }

    /**
     * Sets the feed forward of the PID controller.
     *
     * @param kf feed forward
     */
    public void setF(double kf) {
        KF = kf;
    }

    /**
     * Get the Proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return pidController.getP();
    }

    /**
     * Get the Integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return pidController.getI();
    }

    /**
     * Get the Differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return pidController.getD();
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    public double getPositionError() {
        return pidController.getPositionError();
    }

    /**
     * Returns the current setpoint of the PIDController.
     *
     * @return The current setpoint.
     */
    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    /**
     * Resets the previous error and the integral term.
     */
    public void reset() {
        pidController.reset();
    }

    /**
     * Enables continuous input.
     *
     * <p>
     * Rather then using the max and min input range as constraints, it considers
     * them to be the same point and automatically calculates the shortest route to
     * the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        pidController.enableContinuousInput(minimumInput, maximumInput);
    }

    /**
     * Disables continuous input.
     */
    public void disableContinuousInput() {
        pidController.disableContinuousInput();
    }

    /**
     * Sets the minimum and maximum values for the integrator.
     *
     * <p>
     * When the cap is reached, the integrator value is added to the controller
     * output rather than the integrator value times the integral gain.
     *
     * @param minimumIntegral The minimum value of the integrator.
     * @param maximumIntegral The maximum value of the integrator.
     */
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        pidController.setIntegratorRange(minimumIntegral, maximumIntegral);
    }

    @Override
    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    @Override
    public double getF() {
        return KF;
    }

}
