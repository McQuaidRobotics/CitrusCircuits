package frc.robot.subsystems.swerve;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.kSwerve;

public class SwerveModuleSim {
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0 / kSwerve.DRIVE_MECHANISM_RATIO, 0.025);
}
