package frc.robot.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import frc.robot.Constants;
import frc.robot.Constants.kSwerve;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Angle Motor Configuration */
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();
        angleSupplyLimit.SupplyCurrentLimitEnable = kSwerve.ANGLE_ENABLE_CURRENT_LIMIT;
        angleSupplyLimit.SupplyCurrentLimit = kSwerve.ANGLE_CONTINUOUS_CURRENT_LIMIT;
        angleSupplyLimit.SupplyCurrentThreshold = kSwerve.ANGLE_PEAK_CURRENT_LIMIT;
        angleSupplyLimit.SupplyTimeThreshold = kSwerve.ANGLE_PEAK_CURRENT_DURATION;

        swerveAngleFXConfig.Slot0.kP = Constants.kSwerve.ANGLE_KP;
        swerveAngleFXConfig.Slot0.kI = Constants.kSwerve.ANGLE_KI;
        swerveAngleFXConfig.Slot0.kD = Constants.kSwerve.ANGLE_KD;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = angleSupplyLimit.SupplyCurrentLimit;

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
        driveSupplyLimit.StatorCurrentLimitEnable = kSwerve.DRIVE_ENABLE_CURRENT_LIMIT;
        driveSupplyLimit.SupplyCurrentLimit = kSwerve.DRIVE_CONTINUOUS_CURRENT_LIMIT;
        driveSupplyLimit.SupplyCurrentThreshold = kSwerve.DRIVE_PEAK_CURRENT_LIMIT;
        driveSupplyLimit.SupplyTimeThreshold = kSwerve.DRIVE_PEAK_CURRENT_DURATION;

        swerveDriveFXConfig.Slot0.kP = Constants.kSwerve.DRIVE_KP;
        swerveDriveFXConfig.Slot0.kI = Constants.kSwerve.DRIVE_KI;
        swerveDriveFXConfig.Slot0.kD = Constants.kSwerve.DRIVE_KD;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = driveSupplyLimit.SupplyCurrentLimit;
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.kSwerve.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.kSwerve.CLOSED_LOOP_RAMP;

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = kSwerve.CANCODER_INVERT;
    }
}