package frc.robot.subsystems;

public final class Globals{

    public  static boolean gotCoral = false;

    private static LEDmode ledMode  = LEDmode.ALLIANCE;

    public static void setLEDMode(LEDmode mode) {
        ledMode = mode;
    }

    public static LEDmode getLEDMode() {
        return ledMode;
    }

 }
