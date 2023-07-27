package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import frc.robot.Constants.Swerve;

public final class CTREConfigs {
    public static TalonFXConfiguration swerveAngleFXConfig;
    public static TalonFXConfiguration swerveDriveFXConfig;
    public static CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Angle Motor Configuration */
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();
        angleSupplyLimit.SupplyCurrentLimitEnable = Swerve.angleEnableCurrentLimit;
        angleSupplyLimit.SupplyCurrentLimit = Swerve.angleContinuousCurrentLimit;
        angleSupplyLimit.SupplyCurrentThreshold = Swerve.anglePeakCurrentLimit;
        angleSupplyLimit.SupplyTimeThreshold = Swerve.anglePeakCurrentDuration;

        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = angleSupplyLimit.SupplyCurrentLimit;

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
        driveSupplyLimit.StatorCurrentLimitEnable = Swerve.driveEnableCurrentLimit;
        driveSupplyLimit.SupplyCurrentLimit = Swerve.driveContinuousCurrentLimit;
        driveSupplyLimit.SupplyCurrentThreshold = Swerve.drivePeakCurrentLimit;
        driveSupplyLimit.SupplyTimeThreshold = Swerve.drivePeakCurrentDuration;

        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;        
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = driveSupplyLimit.SupplyCurrentLimit;
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = Swerve.canCoderInvert;
    }
}