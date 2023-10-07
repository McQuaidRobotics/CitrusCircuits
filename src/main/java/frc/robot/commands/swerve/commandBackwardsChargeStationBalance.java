package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Helpers;
import frc.robot.commands.swerve.commandDriveTimedDirection.Direction;
import frc.robot.subsystems.swerve.Swerve;

public class commandBackwardsChargeStationBalance extends CommandBase{
    /*
     * TODO {Maddox} It looks like CitrisCircuits just punches it over the
     * charge station and stops at just the right time so that the robot
     * balances.
     * 
     * Was thinking of doing the same thing, either using a commandDriveTimedDirection
     * which should drive the robot in a direction for a certain amount of time, or 
     * creating a path in pathplanner that drives the robot in a direction towards the
     * charge station and ends at just the right time.
     * 
     * However, looking at CitrisCircuits, its balance sometimes fails because their brute
     * force up the charge station does not end at the right time, probably because every charge station
     * is slightly different in terms of give and felxability and stuff.
     * 
     * So I'm going to see if after brute forcing, then going into a state which teeters the robot until its
     * in the center of the charge station is better, combining what Scorpion does with what
     * Citris does.
     * 
     * Brute force most of the way up the charge station, then balance with whats left over.
     */
    
    private Swerve swerve;
    private double driveStrengthPercent;
    private double driveClimbTimeSeconds;
    private boolean isFinished;

    /**
     * @param swerve The swerve subsystem to be used
     * @param driveStrengthPercent The percentage of max speed to drive between 0 and 1:
     * EX: (1 = Max speed, 0.5 = Half of max speed)
     * @param driveClimbTimeSeconds The time in seconds for the swerve to climb up the
     * charge station before stopping
     */
    public commandBackwardsChargeStationBalance(Swerve swerve, double driveStrengthPercent, double driveClimbTimeSeconds) {
        this.swerve = swerve;
        this.driveStrengthPercent = Helpers.clamp(driveStrengthPercent, 0.0, 1.0);
        this.driveClimbTimeSeconds = driveClimbTimeSeconds;
        this.isFinished = false;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        new commandDriveTimedDirection(
            swerve, 
            Direction.BACKWARD, 
            driveStrengthPercent, 
            driveClimbTimeSeconds, 
            true);
        new commandBalanceSwerve(
            swerve, 
            5.0, //TODO {Maddox} Magic number needs to be tuned
            0.3); //TODO {Maddox} Magic number needs to tuned
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
