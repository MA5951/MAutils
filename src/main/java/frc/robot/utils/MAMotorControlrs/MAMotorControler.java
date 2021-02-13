package frc.robot.utils.MAMotorControlrs;

/**
 * @author yuval rader
 */
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.revrobotics.CANSparkMax;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.MASubsystem.ENCODER;
import frc.robot.utils.MASubsystem.IDMotor;
import frc.robot.utils.MASubsystem.MOTOR_CONTROLL;
import java.util.HashMap;
import java.util.Map;

public class MAMotorControler {
    private MOTOR_CONTROLL MC;
    private ENCODER encoder;
    private MASpakrMax cansSpakrMax;
    private MATalonSRX talonSRX;
    private MAVictorSPX victorSPX;
    private boolean hasReverseLimitSwitch;
    private boolean hasForwardLimitSwitch;

    Map<MOTOR_CONTROLL, MAMotorControlInterface> map1;

    public MAMotorControler(MOTOR_CONTROLL MC, IDMotor ID) {
        this.MC = MC;
        setMAMotorControl(ID);
        setMAP();

    }

    public MAMotorControler(MOTOR_CONTROLL MC, IDMotor ID, boolean Inverted) {
        this.MC = MC;
        setMAMotorControl(ID, Inverted);
        setMAP();
    }

    public MAMotorControler(MOTOR_CONTROLL MC, IDMotor ID, boolean Inverted, double rampRate, boolean mod) {
        this.MC = MC;
        setMAMotorControl(ID, Inverted, rampRate, mod);
        setMAP();
    }

    public MAMotorControler(MOTOR_CONTROLL MC, IDMotor ID, boolean Inverted, double rampRate, boolean mod,
            ENCODER encoder) {
        this.MC = MC;
        this.encoder = encoder;
        setMAMotorControl(ID, Inverted, rampRate, mod, encoder);
        setMAP();
    }

    public MAMotorControler(MOTOR_CONTROLL MC, IDMotor ID, boolean Inverted, double rampRate, boolean mod,
            boolean hasForwardLimitSwitch, boolean hasReverseLimitSwitch) {
        this.MC = MC;
        this.hasForwardLimitSwitch = hasForwardLimitSwitch;
        this.hasReverseLimitSwitch = hasReverseLimitSwitch;
        setMAMotorControl(ID, Inverted, rampRate, mod, hasForwardLimitSwitch, hasReverseLimitSwitch);
        setMAP();

    }

    public MAMotorControler(MOTOR_CONTROLL MC, IDMotor ID, boolean Inverted, double rampRate, boolean mod,
            boolean hasForwardLimitSwitch, boolean hasReverseLimitSwitch, ENCODER encoder) {
        this.MC = MC;
        this.encoder = encoder;
        this.hasForwardLimitSwitch = hasForwardLimitSwitch;
        this.hasReverseLimitSwitch = hasReverseLimitSwitch;
        setMAMotorControl(ID, Inverted, rampRate, mod, hasForwardLimitSwitch, hasReverseLimitSwitch, encoder);
        setMAP();
    }

    public void setvoltage(double voltage) {
        map1.get(MC).setvoltage(voltage);
    }

    public void set(double Power) {
        map1.get(MC).set(Power);
    }

    public void resetEncoder() {
        if (encoder != null)
            map1.get(MC).resetEncoder();
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
        if (encoder != null)
            map1.get(MC).PhaseSensor(PhaseSensor);
    }

    public double getPosition() {
        if (encoder != null) {
            return map1.get(MC).getPosition();
        } else {
            return 0;
        }

    }

    public double getVelocity() {
        if (encoder != null) {
            return map1.get(MC).getVelocity();
        } else {
            return 0;
        }
    }

    public boolean getForwardLimitSwitch() {
        if (hasForwardLimitSwitch) {
            return map1.get(MC).getForwardLimitSwitch();
        } else {
            return true;
        }
    }

    public boolean getReversLimitSwitch() {
        if (hasReverseLimitSwitch) {
            return map1.get(MC).getReversLimitSwitch();
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
        return map1.get(MC).getOutPut();
    }

    public double getStatorCurrent() {
        return map1.get(MC).getStatorCurrent();
    }

    public void configRampRate(double rampRate) {
        map1.get(MC).configRampRate(rampRate);
    }

    public void setInverted(Boolean setInverted) {
        map1.get(MC).setInverted(setInverted);
    }

    public void setCurrentLimit(int limit) {
        if (MC != MOTOR_CONTROLL.VICTOR)
            map1.get(MC).setCurrentLimit(limit);
    }

    public void DisableLimit(boolean onOff) {
        if (this.MC == MOTOR_CONTROLL.TALON) {
            talonSRX.DisableLimit(onOff);
        }
    }

    public void changeMood(boolean onOff) {
        map1.get(MC).changeMood(onOff);
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
        return map1.get(MC).getID();
    }

    private int IDChooser(IDMotor myID) {

        switch (myID) {
            case ID1:
                return 1;

            case ID2:
                return 2;

            case ID3:
                return 3;

            case ID4:
                return 4;

            case ID5:
                return 5;

            case ID6:
                return 6;

            case ID7:
                return 7;

            case ID8:
                return 8;

            case ID9:
                return 9;

            case ID10:
                return 10;

            case ID11:
                return 11;

            case ID12:
                return 12;

            case ID13:
                return 13;

            case ID14:
                return 14;

            case ID15:
                return 15;

            case ID16:
                return 16;

        }
        return 0;
    }

    private void setMAMotorControl(IDMotor ID) {
        switch (MC) {
            case TALON:
                talonSRX = new MATalonSRX(IDChooser(ID));
                break;
            case VICTOR:
                victorSPX = new MAVictorSPX(IDChooser(ID));
                break;
            case SPARKMAXBrushless:
                cansSpakrMax = new MASpakrMax(IDChooser(ID), RobotConstants.kBrushless);
                break;
            case SPARKMAXBrushled:
                cansSpakrMax = new MASpakrMax(IDChooser(ID), RobotConstants.kBrushed);
                break;
        }
    }

    private void setMAMotorControl(IDMotor ID, boolean Inverted) {
        switch (MC) {
            case TALON:
                talonSRX = new MATalonSRX(IDChooser(ID), Inverted);
                break;
            case VICTOR:
                victorSPX = new MAVictorSPX(IDChooser(ID), Inverted);
                break;
            case SPARKMAXBrushless:
                cansSpakrMax = new MASpakrMax(IDChooser(ID), Inverted, RobotConstants.kBrushless);
                break;
            case SPARKMAXBrushled:
                cansSpakrMax = new MASpakrMax(IDChooser(ID), Inverted, RobotConstants.kBrushed);
                break;
        }
    }

    private void setMAMotorControl(IDMotor ID, boolean Inverted, double rampRate, boolean mod, ENCODER encoder) {
        switch (MC) {
            case TALON:
                talonSRX = new MATalonSRX(IDChooser(ID), Inverted, rampRate, mod, FeedbackDevice.QuadEncoder);
                break;
            case VICTOR:
                victorSPX = new MAVictorSPX(IDChooser(ID), Inverted, rampRate, mod);
                break;
            case SPARKMAXBrushless:
                cansSpakrMax = new MASpakrMax(IDChooser(ID), Inverted, rampRate, mod, encoder,
                        RobotConstants.kBrushless);
                break;
            case SPARKMAXBrushled:
                cansSpakrMax = new MASpakrMax(IDChooser(ID), Inverted, rampRate, mod, encoder, RobotConstants.kBrushed);
                break;
        }
    }

    private void setMAMotorControl(IDMotor ID, boolean Inverted, double rampRate, boolean mod) {
        switch (MC) {
            case TALON:
                talonSRX = new MATalonSRX(IDChooser(ID), Inverted, rampRate, mod);
                break;
            case VICTOR:
                victorSPX = new MAVictorSPX(IDChooser(ID), Inverted, rampRate, mod);
                break;
            case SPARKMAXBrushless:
                cansSpakrMax = new MASpakrMax(IDChooser(ID), Inverted, rampRate, mod, RobotConstants.kBrushless);
                break;
            case SPARKMAXBrushled:
                cansSpakrMax = new MASpakrMax(IDChooser(ID), Inverted, rampRate, mod, RobotConstants.kBrushed);
                break;
        }
    }

    private void setMAMotorControl(IDMotor ID, boolean Inverted, double rampRate, boolean mod,
            boolean hasForwardLimitSwitch, boolean hasReverseLimitSwitch) {
        switch (MC) {
            case TALON:
                talonSRX = new MATalonSRX(IDChooser(ID), Inverted, rampRate, mod, hasForwardLimitSwitch,
                        hasReverseLimitSwitch);
                break;
            case VICTOR:
                victorSPX = new MAVictorSPX(IDChooser(ID), Inverted, rampRate, mod);
                System.err.println(""); // TODO
                break;
            case SPARKMAXBrushless:
                cansSpakrMax = new MASpakrMax(IDChooser(ID), Inverted, rampRate, mod, hasForwardLimitSwitch,
                        hasReverseLimitSwitch, RobotConstants.kBrushless);
                break;
            case SPARKMAXBrushled:
                cansSpakrMax = new MASpakrMax(IDChooser(ID), Inverted, rampRate, mod, hasForwardLimitSwitch,
                        hasReverseLimitSwitch, RobotConstants.kBrushed);
                break;
        }
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
                        hasReverseLimitSwitch, encoder, RobotConstants.kBrushless);
                break;
            case SPARKMAXBrushled:
                cansSpakrMax = new MASpakrMax(IDChooser(ID), Inverted, rampRate, mod, hasForwardLimitSwitch,
                        hasReverseLimitSwitch, encoder, RobotConstants.kBrushed);
                break;
        }
    }

    private void setMAP() {
        map1 = new HashMap<MOTOR_CONTROLL, MAMotorControlInterface>();
        map1.put(MOTOR_CONTROLL.TALON, talonSRX);
        map1.put(MOTOR_CONTROLL.VICTOR, victorSPX);
        map1.put(MOTOR_CONTROLL.SPARKMAXBrushless, cansSpakrMax);
        map1.put(MOTOR_CONTROLL.SPARKMAXBrushled, cansSpakrMax);
    }

}
