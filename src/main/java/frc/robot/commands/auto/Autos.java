package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Autos {
    public static SendableChooser<Command> autoChooser;

    public static void createSendableChooser() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public static Command getAutonomousCommand() {
        if (autoChooser != null) return autoChooser.getSelected();
        return new InstantCommand().withName("Nothing -> Auto Chooser Not Created!");
    }

    public static String getSelectedAutoName() {
        if (autoChooser != null) return autoChooser.getSelected().getName();
        return "Nothing -> Auto Chooser Not Created!";
    }
}
