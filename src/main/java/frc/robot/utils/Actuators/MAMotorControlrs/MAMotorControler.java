package frc.robot.utils.Actuators.MAMotorControlrs;

/**
 * @author yuval rader
 */
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.utils.MASubsystem.ENCODER;
import frc.robot.utils.MASubsystem.IDMotor;
import frc.robot.utils.MASubsystem.MOTOR_CONTROLL;

public class MAMotorControler {
    private MOTOR_CONTROLL MC;
    private ENCODER encoder;
    private MASpakrMax cansSpakrMax;
    private MATalonSRX talonSRX;
    private MAVictorSPX victorSPX;
    private boolean hasReverseLimitSwitch;
    private boolean hasForwardLimitSwitch;

    public MAMotorControler(MOTOR_CONTROLL MC, IDMotor ID) {
        this.MC = MC;
        encoder = ENCODER.No_Encoder;
        hasForwardLimitSwitch = false;
        hasReverseLimitSwitch = false;
        setMAMotorControl(ID, false, 0, false, false, false, ENCODER.No_Encoder);

    }

    public MAMotorControler(MOTOR_CONTROLL MC, IDMotor ID, boolean Inverted) {
        this.MC = MC;
        encoder = ENCODER.No_Encoder;
        hasForwardLimitSwitch = false;
        hasReverseLimitSwitch = false;
        setMAMotorControl(ID, Inverted, 0, false, false, false, ENCODER.No_Encoder);

    }

    public MAMotorControler(MOTOR_CONTROLL MC, IDMotor ID, boolean Inverted, double rampRate, boolean mod) {
        this.MC = MC;
        encoder = ENCODER.No_Encoder;
        hasForwardLimitSwitch = false;
        hasReverseLimitSwitch = false;
        setMAMotorControl(ID, Inverted, rampRate, mod, false, false, ENCODER.No_Encoder);

    }

    public MAMotorControler(MOTOR_CONTROLL MC, IDMotor ID, boolean Inverted, double rampRate, boolean mod,
            ENCODER encoder) {
        this.MC = MC;
        this.encoder = encoder;
        hasForwardLimitSwitch = false;
        hasReverseLimitSwitch = false;
        setMAMotorControl(ID, Inverted, rampRate, mod, false, false, encoder);

    }

    public MAMotorControler(MOTOR_CONTROLL MC, IDMotor ID, boolean Inverted, double rampRate, boolean mod,
            boolean hasForwardLimitSwitch, boolean hasReverseLimitSwitch) {
        this.MC = MC;
        this.hasForwardLimitSwitch = hasForwardLimitSwitch;
        this.hasReverseLimitSwitch = hasReverseLimitSwitch;
        encoder = ENCODER.No_Encoder;
        setMAMotorControl(ID, Inverted, rampRate, mod, hasForwardLimitSwitch, hasReverseLimitSwitch,
                ENCODER.No_Encoder);

    }

    public MAMotorControler(MOTOR_CONTROLL MC, IDMotor ID, boolean Inverted, double rampRate, boolean mod,
            boolean hasForwardLimitSwitch, boolean hasReverseLimitSwitch, ENCODER encoder) {
        this.MC = MC;
        this.encoder = encoder;
        this.hasForwardLimitSwitch = hasForwardLimitSwitch;
        this.hasReverseLimitSwitch = hasReverseLimitSwitch;
        setMAMotorControl(ID, Inverted, rampRate, mod, hasForwardLimitSwitch, hasReverseLimitSwitch, encoder);

    }

    public void setVoltage(double voltage) {
        getMotorControll().setvoltage(voltage);
    }

    public void set(double Power) {
        getMotorControll().set(Power);
    }

    public void resetEncoder() {
        if (encoder == ENCODER.Encoder || encoder == ENCODER.Alternate_Encoder)
            getMotorControll().resetEncoder();
    }

    public void resatOnLimitF(boolean limit) {
        if (MC == MOTOR_CONTROLL.TALON && encoder == ENCODER.Encoder) {
            talonSRX.resatOnLimitF(limit);
        }
    }

    public void resatOnLimitR(boolean limit) {
        if (MC == MOTOR_CONTROLL.TALON && encoder == ENCODER.Encoder) {
            talonSRX.resatOnLimitR(limit);
        }
    }

    public void PhaseSensor(boolean PhaseSensor) {
        if (encoder == ENCODER.Encoder || encoder == ENCODER.Alternate_Encoder)
            getMotorControll().PhaseSensor(PhaseSensor);
    }

    public double getPosition() {
        if (encoder == ENCODER.Encoder || encoder == ENCODER.Alternate_Encoder) {
            return getMotorControll().getPosition();
        } else {
            return 0;
        }

    }

    public double getVelocity() {
        if (encoder == ENCODER.Encoder || encoder == ENCODER.Alternate_Encoder) {
            return getMotorControll().getVelocity();
        } else {
            return 0;
        }
    }

    public boolean getForwardLimitSwitch() {
        if (hasForwardLimitSwitch) {
            return getMotorControll().getForwardLimitSwitch();
        } else {
            return true;
        }
    }

    public boolean getReversLimitSwitch() {
        if (hasReverseLimitSwitch) {
            return getMotorControll().getReversLimitSwitch();
        } else {
            return true;
        }
    }

    public void overrideLimitSwitches(boolean overrid) {
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
        return getMotorControll().getStatorCurrent();
    }

    public void configRampRate(double rampRate) {
        getMotorControll().configRampRate(rampRate);
    }

    public void setInverted(Boolean setInverted) {
        getMotorControll().setInverted(setInverted);
    }

    public void setCurrentLimit(int limit) {
        if (MC != MOTOR_CONTROLL.VICTOR)
            getMotorControll().setCurrentLimit(limit);
    }

    public void DisableLimit(boolean onOff) {
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

    private int IDChooser(IDMotor myID) {
        String id = myID.toString();
        if (id.substring(id.length() - 3).charAt(0) == 'D')
            return Integer.parseInt(id.substring(id.length() - 2));
        else
            return Integer.parseInt(id.substring(id.length() - 1));
    }

    private void setMAMotorControl(IDMotor ID, boolean Inverted, double rampRate, boolean mod,
            boolean hasForwardLimitSwitch, boolean hasReverseLimitSwitch, ENCODER encoder) {
        switch (MC) {
            case TALON:
                talonSRX = new MATalonSRX(IDChooser(ID), Inverted, rampRate, mod, hasForwardLimitSwitch,
                        hasReverseLimitSwitch, FeedbackDevice.QuadEncoder);
                break;
            case VICTOR:
                victorSPX = new MAVictorSPX(IDChooser(ID), Inverted, rampRate, mod);
                System.err.println(""); // TODO
                break;
            case SPARKMAXBrushless:
                cansSpakrMax = new MASpakrMax(IDChooser(ID), Inverted, rampRate, mod, hasForwardLimitSwitch,
                        hasReverseLimitSwitch, encoder, MotorType.kBrushless);
                break;
            case SPARKMAXBrushled:
                cansSpakrMax = new MASpakrMax(IDChooser(ID), Inverted, rampRate, mod, hasForwardLimitSwitch,
                        hasReverseLimitSwitch, encoder, MotorType.kBrushed);
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

}
