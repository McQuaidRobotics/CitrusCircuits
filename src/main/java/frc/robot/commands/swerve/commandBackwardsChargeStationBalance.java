package frc.robot.commands.swerve;

public class commandBackwardsChargeStationBalance {
    /*
     * TODO {Maddox} It looks like CitrisCircuits just punches it over the
     * charge station and stops at just the right time so that the robot
     * balances.
     * 
     * Was thinking of doing the same thing, either using a commandDriveTimedDirection
     * which should drive the robot in a direction for a certain amount of time, or 
     * creating a path in pathplanner than drives the robot in a direction towards the
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
}
