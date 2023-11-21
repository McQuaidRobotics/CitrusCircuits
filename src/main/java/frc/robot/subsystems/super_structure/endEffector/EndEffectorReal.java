package frc.robot.subsystems.super_structure.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.kSuperStructure.kEndEffector;
import frc.robot.util.ShuffleboardApi.ShuffleEntryContainer;

public class EndEffectorReal implements EndEffector {

    private final TalonFX endEffectorMotor;

    private final StatusSignal<Double> endEffectorMotorAmps, endEffectorMotorVolts;

    private Double cachedEndEffectorVolts = 0.0, endEffectorCurrentLimit;

    public EndEffectorReal() {
        endEffectorMotor = new TalonFX(kEndEffector.MOTOR_ID);
        endEffectorMotor.getConfigurator().apply(getEndEffectorMotorConfig());

        endEffectorMotorAmps = endEffectorMotor.getStatorCurrent();
        endEffectorMotorVolts = endEffectorMotor.getSupplyVoltage();
    }

    /**
     * Constructs a TalonFXConfiguration object only from values
     * from {@link frc.robot.Constants.kEndEffector}
     * 
     * @return the TalonFXConfiguration object
     */
    private TalonFXConfiguration getEndEffectorMotorConfig() {
        TalonFXConfiguration eeMotorCfg = new TalonFXConfiguration();

        eeMotorCfg.MotorOutput.Inverted = kEndEffector.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        return eeMotorCfg;
    }

    @Override
    public void runEndEffector(Double volts) {
        var voltageControlRequest = new VoltageOut(volts);
        this.endEffectorMotor.setControl(voltageControlRequest);
    }

    @Override
    public Double getEndEffectorVoltage() {
        return cachedEndEffectorVolts;
    }

    @Override
    public void setEndEffectorCurrentLimits(Double limit) {
        if (limit != this.endEffectorCurrentLimit) {
            var cfg = new CurrentLimitsConfigs();
            cfg.SupplyCurrentLimitEnable = true;
            cfg.SupplyCurrentLimit = limit;
            cfg.SupplyCurrentThreshold = limit;
            cfg.SupplyTimeThreshold = 0.2;
            cfg.StatorCurrentLimit = limit;
            cfg.StatorCurrentLimitEnable = true;
            endEffectorMotor.getConfigurator().apply(cfg);
            this.endEffectorCurrentLimit = limit;
        }
    }

    @Override
    public void setupShuffleboard(ShuffleEntryContainer tab) {
        tab.addDouble("EndEffector Current", endEffectorMotorAmps::getValue);
        tab.addDouble("EndEffector Voltage", endEffectorMotorVolts::getValue);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(endEffectorMotorAmps, endEffectorMotorVolts);
        this.cachedEndEffectorVolts = endEffectorMotorVolts.getValue();
    }
}
