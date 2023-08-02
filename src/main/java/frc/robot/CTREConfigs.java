package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import frc.robot.Constants.Swerve;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Angle Motor Configuration */
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();
        angleSupplyLimit.SupplyCurrentLimitEnable = Swerve.ANGLE_ENABLE_CURRENT_LIMIT;
        angleSupplyLimit.SupplyCurrentLimit = Swerve.ANGLE_CONTINUOUS_CURRENT_LIMIT;
        angleSupplyLimit.SupplyCurrentThreshold = Swerve.ANGLE_PEAK_CURRENT_LIMIT;
        angleSupplyLimit.SupplyTimeThreshold = Swerve.ANGLE_PEAK_CURRENT_DURATION;

        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.ANGLE_KP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.ANGLE_KI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.ANGLE_KD;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = angleSupplyLimit.SupplyCurrentLimit;

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
        driveSupplyLimit.StatorCurrentLimitEnable = Swerve.DRIVE_ENABLE_CURRENT_LIMIT;
        driveSupplyLimit.SupplyCurrentLimit = Swerve.DRIVE_CONTINUOUS_CURRENT_LIMIT;
        driveSupplyLimit.SupplyCurrentThreshold = Swerve.DRIVE_PEAK_CURRENT_LIMIT;
        driveSupplyLimit.SupplyTimeThreshold = Swerve.DRIVE_PEAK_CURRENT_DURATION;

        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.DRIVE_KP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.DRIVE_KI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.DRIVE_KD;        
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = driveSupplyLimit.SupplyCurrentLimit;
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = Swerve.CANCODER_INVERT;
    }
}