package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;

public class commandBalanceSwerve extends CommandBase{
    private double pitchErrorDegrees;
    private Swerve swerve;
    private double forwardDriveStrength;
    private double backwardDriveStrength;

    /**
     * @param swerve The swerve subsystem to be used
     * @param pitchErrorDegrees The max error in degrees that is allowed
     * before being considered balanced
     * @param initalStrength The initial strength of the motors in percent from 0 to 1:
     *  EX: (1 = Max speed, 0.5 = Half of max speed)
     */

    public commandBalanceSwerve(Swerve swerve, Double pitchErrorDegrees, Double initialStrength) {
        this.swerve = swerve;
        this.pitchErrorDegrees = pitchErrorDegrees;
        this.forwardDriveStrength = initialStrength;
        this.backwardDriveStrength = initialStrength;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (swerve.getGyroPitch().getValue() > 0) {
            new TeleopSwerve(
                swerve, 
                () -> -1.0 * backwardDriveStrength, 
                () -> 0.0, 
                () -> 0.0);
        }
        else {
            new TeleopSwerve(
                swerve, 
                () -> 1.0 * forwardDriveStrength, 
                () -> 0.0, 
                () -> 0.0);
        }

        /*
        *TODO {Maddox} May want some kind of system in place which will decrease
        *the drive strength each time the robot switches direction while teetering
        *on the charge station
        */
    }

    @Override
    public void end(boolean interrupted) {
        swerve.commandPerpendicularDrives();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerve.getGyroPitch().getValue()) <= pitchErrorDegrees;
    }
}
