package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;


public class NTpreferences {
    private static HashMap<String, Double> universalOffsets = new HashMap<String, Double>();
    private static boolean loadedCorrectly = true;

    public static enum Module{
        u3,
        u0,
        u2,
        u1
    }

    public static void loadPreferences() {
        loadedCorrectly = true;
        
        universalOffsets.put("3U", Rotation2d.fromRotations(Preferences.getDouble("Module 3 Offset Universal", 0)).getDegrees());
        universalOffsets.put("0U", Rotation2d.fromRotations(Preferences.getDouble("Module 0 Offset Universal", 0)).getDegrees());
        universalOffsets.put("2U", Rotation2d.fromRotations(Preferences.getDouble("Module 2 Offset Universal", 0)).getDegrees());
        universalOffsets.put("1U", Rotation2d.fromRotations(Preferences.getDouble("Module 1 Offset Universal", 0)).getDegrees());

        SmartDashboard.putNumber("Mod 3", Rotation2d.fromDegrees(universalOffsets.get("3U")).getRotations());
        SmartDashboard.putNumber("Mod 0", Rotation2d.fromDegrees(universalOffsets.get("0U")).getRotations());
        SmartDashboard.putNumber("Mod 2", Rotation2d.fromDegrees(universalOffsets.get("2U")).getRotations());
        SmartDashboard.putNumber("Mod 1", Rotation2d.fromDegrees(universalOffsets.get("1U")).getRotations());

        for (Double angle : universalOffsets.values()) {
            if ((angle <= 0 || angle >= 360) && Robot.isReal()) loadedCorrectly = false;
        }
    }

    public static boolean isOk() {
        return loadedCorrectly;
    }

    public static Double getRotationOffset(Module module) {
        if (module == Module.u3) return Rotation2d.fromDegrees(findCoterminalAngle(universalOffsets.get("3U") + 225)).getRotations();
        else if (module == Module.u0) return Rotation2d.fromDegrees(findCoterminalAngle(universalOffsets.get("0U") + 315)).getRotations();
        else if (module == Module.u2) return Rotation2d.fromDegrees(findCoterminalAngle(universalOffsets.get("2U") + 125)).getRotations();
        else if (module == Module.u1) return Rotation2d.fromDegrees(findCoterminalAngle(universalOffsets.get("1U") + 45)).getRotations();
        return 0.0;
    }

    public static double findCoterminalAngle(double angleOffset) {
        return (angleOffset > 360) ? angleOffset % 360 : angleOffset;
    }
}
