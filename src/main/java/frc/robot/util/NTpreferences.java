package frc.robot.util;

import java.util.HashMap;
import edu.wpi.first.wpilibj.Preferences;
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
        
        universalOffsets.put("3U", Preferences.getDouble("Module 3 Offset Universal", 0));
        universalOffsets.put("0U", Preferences.getDouble("Module 0 Offset Universal", 0));
        universalOffsets.put("2U", Preferences.getDouble("Module 2 Offset Universal", 0));
        universalOffsets.put("1U", Preferences.getDouble("Module 1 Offset Universal", 0));

        for (Double angle : universalOffsets.values()) {
            if ((angle <= 0 || angle >= 360) && Robot.isReal()) loadedCorrectly = false;
        }
    }

    public static boolean isOk() {
        return loadedCorrectly;
    }

    public static Double getAngleOffset(Module module) {
        if (module == Module.u3) return findCoterminalAngle(universalOffsets.get("3U") + 225);
        else if (module == Module.u0) return findCoterminalAngle(universalOffsets.get("0U") + 315);
        else if (module == Module.u2) return findCoterminalAngle(universalOffsets.get("2U") + 125);
        else if (module == Module.u1) return findCoterminalAngle(universalOffsets.get("1U") + 45);
        return 0.0;
    }

    public static double findCoterminalAngle(double angleOffset) {
        return (angleOffset > 360) ? angleOffset % 360 : angleOffset;
    }
}
