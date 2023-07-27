package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Swerve {
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private Pigeon2 gyro;

    public Swerve() {
        this.gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.canBus);
        var gyroEmptyConfig = new Pigeon2Configuration();
        gyro.getConfigurator().apply(gyroEmptyConfig);
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(Constants.Swerve.Mod0.constants, 0),
            new SwerveModule(Constants.Swerve.Mod1.constants, 1),
            new SwerveModule(Constants.Swerve.Mod2.constants, 2),
            new SwerveModule(Constants.Swerve.Mod3.constants, 3)
        };

        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void Drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] mSwerveModuleStates = 
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation, 
                    getYaw())
                : new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation
                ));

        SwerveDriveKinematics.desaturateWheelSpeeds(mSwerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule module : mSwerveMods) {
            module.setDesiredState(mSwerveModuleStates[module.moduleNumber], isOpenLoop);
        }
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule module : mSwerveMods) {
            module.resetToAbsolute();
        }
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValue()) : Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (SwerveModule module : mSwerveMods) {
            modulePositions[module.moduleNumber] = module.getPosition();
        }
        return getModulePositions();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule module : mSwerveMods){
            module.setDesiredState(desiredStates[module.moduleNumber], false);
        }
    } 

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    
}
