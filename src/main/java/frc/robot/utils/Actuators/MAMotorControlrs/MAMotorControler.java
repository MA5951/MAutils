package frc.robot.utils.Actuators.MAMotorControlrs;

/**
 * @author yuval rader
 */
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.utils.MADriverStation;
import frc.robot.utils.MASubsystem.MASubsystem.ENCODER;
import frc.robot.utils.MASubsystem.MASubsystem.MOTOR_CONTROLL;

public class MAMotorControler {
    private MOTOR_CONTROLL MC;
    private ENCODER encoder;
    private MASpakrMax cansSpakrMax;
    private MATalonSRX talonSRX;
    private MAVictorSPX victorSPX;
    private boolean hasReverseLimitSwitch;
    private boolean hasForwardLimitSwitch;

    public MAMotorControler(MOTOR_CONTROLL MC, int id) {
        this.MC = MC;
        encoder = ENCODER.No_Encoder;
        hasForwardLimitSwitch = false;
        hasReverseLimitSwitch = false;
        setMAMotorControl(id, false, 0, false, false, false, ENCODER.No_Encoder);

    }

    public MAMotorControler(MOTOR_CONTROLL MC, int id, boolean inverted) {
        this.MC = MC;
        encoder = ENCODER.No_Encoder;
        hasForwardLimitSwitch = false;
        hasReverseLimitSwitch = false;
        setMAMotorControl(id, inverted, 0, false, false, false, ENCODER.No_Encoder);

    }

    public MAMotorControler(MOTOR_CONTROLL MC, int id, boolean inverted, double rampRate, boolean mod) {
        this.MC = MC;
        encoder = ENCODER.No_Encoder;
        hasForwardLimitSwitch = false;
        hasReverseLimitSwitch = false;
        setMAMotorControl(id, inverted, rampRate, mod, false, false, ENCODER.No_Encoder);

    }

    public MAMotorControler(MOTOR_CONTROLL MC, int id, boolean inverted, double rampRate, boolean mod,
            ENCODER encoder) {
        this.MC = MC;
        this.encoder = encoder;
        hasForwardLimitSwitch = false;
        hasReverseLimitSwitch = false;
        printWarningForMC("victor dont support", true, MOTOR_CONTROLL.VICTOR);
        setMAMotorControl(id, inverted, rampRate, mod, false, false, encoder);

    }

    public MAMotorControler(MOTOR_CONTROLL MC, int id, boolean inverted, double rampRate, boolean mod,
            boolean hasForwardLimitSwitch, boolean hasReverseLimitSwitch) {
        this.MC = MC;
        this.hasForwardLimitSwitch = hasForwardLimitSwitch;
        this.hasReverseLimitSwitch = hasReverseLimitSwitch;
        encoder = ENCODER.No_Encoder;
        printWarningForMC("victor dont support", true, MOTOR_CONTROLL.VICTOR);
        setMAMotorControl(id, inverted, rampRate, mod, hasForwardLimitSwitch, hasReverseLimitSwitch,
                ENCODER.No_Encoder);

    }

    public MAMotorControler(MOTOR_CONTROLL MC, int id, boolean inverted, double rampRate, boolean mod,
            boolean hasForwardLimitSwitch, boolean hasReverseLimitSwitch, ENCODER encoder) {
        this.MC = MC;
        this.encoder = encoder;
        this.hasForwardLimitSwitch = hasForwardLimitSwitch;
        this.hasReverseLimitSwitch = hasReverseLimitSwitch;
        printWarningForMC("victor dont support", true, MOTOR_CONTROLL.VICTOR);
        setMAMotorControl(id, inverted, rampRate, mod, hasForwardLimitSwitch, hasReverseLimitSwitch, encoder);

    }

    public void setVoltage(double voltage) {
        getMotorControll().setvoltage(voltage);
    }

    public void set(double power) {
        getMotorControll().set(power);
    }

    public void resetEncoder() {
        printWarningForMC("victor dont support", true, MOTOR_CONTROLL.VICTOR);
        if (encoder == ENCODER.Encoder || encoder == ENCODER.Alternate_Encoder)
            getMotorControll().resetEncoder();
    }

    public void resatOnLimitF(boolean limit) {
        printWarningForMC("victor dont support", true, MOTOR_CONTROLL.VICTOR);
        if (MC == MOTOR_CONTROLL.TALON && encoder == ENCODER.Encoder) {
            talonSRX.resatOnLimitF(limit);
        }
    }

    public void resatOnLimitR(boolean limit) {
        printWarningForMC("victor dont support", true, MOTOR_CONTROLL.VICTOR);
        printWarningForMC("sparkMax dont support", true, MOTOR_CONTROLL.SPARKMAXBrushled);
        printWarningForMC("sparkMax dont support", true, MOTOR_CONTROLL.SPARKMAXBrushless);
        if (MC == MOTOR_CONTROLL.TALON && encoder == ENCODER.Encoder) {
            talonSRX.resatOnLimitR(limit);
        }
    }

    /*
     * dont work with victor and inerEncoder neo
     */
    public void phaseSensor(boolean phaseSensor) {
        printWarningForMC("victor and spark max encoder dont support ", true, MOTOR_CONTROLL.VICTOR);
        if (encoder == ENCODER.Encoder || encoder == ENCODER.Alternate_Encoder)
            getMotorControll().phaseSensor(phaseSensor);
    }

    public double getPosition() {
        printWarningForMC("victor dont support", true, MOTOR_CONTROLL.VICTOR);
        if (encoder == ENCODER.Encoder || encoder == ENCODER.Alternate_Encoder) {
            return getMotorControll().getPosition();
        } else {
            return 0;
        }

    }

    public double getVelocity() {
        printWarningForMC("victor dont support", true, MOTOR_CONTROLL.VICTOR);
        if (encoder == ENCODER.Encoder || encoder == ENCODER.Alternate_Encoder) {
            return getMotorControll().getVelocity();
        } else {
            return 0;
        }
    }

    public boolean getForwardLimitSwitch() {
        printWarningForMC("victor dont support", true, MOTOR_CONTROLL.VICTOR);
        if (hasForwardLimitSwitch) {
            return getMotorControll().getForwardLimitSwitch();
        } else {
            return true;
        }
    }

    public boolean getReversLimitSwitch() {
        printWarningForMC("victor dont support", true, MOTOR_CONTROLL.VICTOR);
        if (hasReverseLimitSwitch) {
            return getMotorControll().getReversLimitSwitch();
        } else {
            return true;
        }
    }

    public void overrideLimitSwitches(boolean overrid) {
        printWarningForMC("victor dont support", true, MOTOR_CONTROLL.VICTOR);
        if (MC == MOTOR_CONTROLL.TALON && hasForwardLimitSwitch) {
            talonSRX.overrideLimitSwitches(overrid);
        } else if ((MC == MOTOR_CONTROLL.SPARKMAXBrushless || MC == MOTOR_CONTROLL.SPARKMAXBrushled)
                && hasForwardLimitSwitch || hasReverseLimitSwitch) {
            cansSpakrMax.enableLimitSwitchR(overrid);
            cansSpakrMax.enableLimitSwitchF(overrid);
        }
    }

    public MOTOR_CONTROLL getMotorControllType() {
        return MC;
    }

    public double getOutput() {
        return getMotorControll().getOutPut();
    }

    public double getStatorCurrent() {
        printWarningForMC("victor dont support", true, MOTOR_CONTROLL.VICTOR);
        return getMotorControll().getStatorCurrent();
    }

    public void configRampRate(double rampRate) {
        getMotorControll().configRampRate(rampRate);
    }

    public void setInverted(Boolean setInverted) {
        getMotorControll().setInverted(setInverted);
    }

    public void setCurrentLimit(int limit) {
        printWarningForMC("victor dont support", true, MOTOR_CONTROLL.VICTOR);
        if (MC != MOTOR_CONTROLL.VICTOR)
            getMotorControll().setCurrentLimit(limit);
    }

    public void DisableLimit(boolean onOff) {
        printWarningForMC("victor dont support", true, MOTOR_CONTROLL.VICTOR);
        printWarningForMC("sparkMax dont support", true, MOTOR_CONTROLL.SPARKMAXBrushled);
        printWarningForMC("sparkMax dont support", true, MOTOR_CONTROLL.SPARKMAXBrushless);
        if (MC == MOTOR_CONTROLL.TALON) {
            talonSRX.DisableLimit(onOff);
        }
    }

    public void changeMood(boolean onOff) {
        getMotorControll().changeMood(onOff);
    }

    private CANSparkMax getSparkMax() {
        return cansSpakrMax.getCanSparkMax();
    }

    private IMotorController getIMotorController() {
        if (MC == MOTOR_CONTROLL.TALON) {
            return talonSRX.getIMotorController();
        } else if (MC == MOTOR_CONTROLL.VICTOR) {
            return victorSPX.getIMotorController();
        }
        return null;
    }

    public void follow(MAMotorControler motor) {
        if (MC == MOTOR_CONTROLL.TALON) {
            talonSRX.follow(motor.getIMotorController());
        } else if (MC == MOTOR_CONTROLL.VICTOR) {
            victorSPX.follow(motor.getIMotorController());
        } else {
            cansSpakrMax.follow(motor.getSparkMax());
        }
    }

    public int getID() {
        return getMotorControll().getID();
    }

    private void setMAMotorControl(int id, boolean inverted, double rampRate, boolean mod,
            boolean hasForwardLimitSwitch, boolean hasReverseLimitSwitch, ENCODER encoder) {
        switch (MC) {
            case TALON:
                talonSRX = new MATalonSRX(id, inverted, rampRate, mod, hasForwardLimitSwitch, hasReverseLimitSwitch,
                        FeedbackDevice.QuadEncoder);
                break;
            case VICTOR:
                victorSPX = new MAVictorSPX(id, inverted, rampRate, mod);
                break;
            case SPARKMAXBrushless:
                cansSpakrMax = new MASpakrMax(id, inverted, rampRate, mod, hasForwardLimitSwitch, hasReverseLimitSwitch,
                        encoder, MotorType.kBrushless);
                break;
            case SPARKMAXBrushled:
                cansSpakrMax = new MASpakrMax(id, inverted, rampRate, mod, hasForwardLimitSwitch, hasReverseLimitSwitch,
                        encoder, MotorType.kBrushed);
                break;
        }
    }

    private MAMotorControlInterface getMotorControll() {
        switch (MC) {
            case TALON:
                return talonSRX;

            case VICTOR:
                return victorSPX;

            case SPARKMAXBrushless:
                return cansSpakrMax;

            case SPARKMAXBrushled:
                return cansSpakrMax;

            default:
                return null;

        }
    }

    public void printWarningForMC(String warning, boolean printTrace, MOTOR_CONTROLL MC) {
        if (this.MC == MC) {
            MADriverStation.printWarning(warning, printTrace);
        }
    }
}
