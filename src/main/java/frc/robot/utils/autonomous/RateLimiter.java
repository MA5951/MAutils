package frc.robot.utils.autonomous;

import edu.wpi.first.math.MathUtil;

public class RateLimiter {
    private double maxChange; 
    private double lastVelocity;

    public RateLimiter(double maxChange) {
        this.maxChange = maxChange;
    }


    public double calculate(double targetVelocity) {
        lastVelocity =  lastVelocity + MathUtil.clamp(targetVelocity - lastVelocity, -maxChange, maxChange);
        return lastVelocity;
    }
}
