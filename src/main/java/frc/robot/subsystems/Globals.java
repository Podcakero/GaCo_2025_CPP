package frc.robot.subsystems;

public final class Globals{

    public  static boolean GOT_CORAL = false;
    public  static boolean GOT_ALGAE = false;
    public  static boolean WRIST_IN_POSITION = false;
    public  static boolean ELEVATOR_IN_POSITION = false;
    public  static ApproachTarget IDENTIFIED_TARGET = ApproachTarget.UNKNOWN;    
    public  static boolean HIGH_CAM_ENABLED = false;


    private static LEDmode ledMode  = LEDmode.ALLIANCE;

    public static void setLEDMode(LEDmode mode) {
        ledMode = mode;
    }

    public static LEDmode getLEDMode() {
        return ledMode;
    }

    public static void enableHighCam(){
        HIGH_CAM_ENABLED = true;
    }

    public static void disableHighCam(){
        HIGH_CAM_ENABLED = false;
    }

 }
