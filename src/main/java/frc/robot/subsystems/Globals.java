package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Globals extends SubsystemBase {

    public  static boolean      GOT_CORAL;
    public  static boolean      GOT_ALGAE;
    public  static boolean      WRIST_IN_POSITION;
    public  static boolean      ELEVATOR_IN_POSITION;
    public  static ApproachTarget IDENTIFIED_TARGET;    
    public  static boolean      UPPER_CAM_ENABLED;
    private static LEDmode      LED_MODE;

    public Globals(){
        GOT_CORAL = false;
        GOT_ALGAE = false;
        WRIST_IN_POSITION = false;
        ELEVATOR_IN_POSITION = false;
        IDENTIFIED_TARGET = ApproachTarget.UNKNOWN;    
        UPPER_CAM_ENABLED = false;
        LED_MODE  = LEDmode.ALLIANCE;
    }

    @Override
	public void periodic() {
        SmartDashboard.putBoolean("Wrist In Position", WRIST_IN_POSITION);
        SmartDashboard.putBoolean("Elevator In Position", ELEVATOR_IN_POSITION);
        SmartDashboard.putString("Approach Target", IDENTIFIED_TARGET.toString());
        SmartDashboard.putBoolean("Upper Cam Enabled", UPPER_CAM_ENABLED);   
        SmartDashboard.putString("LED Mode", Globals.LED_MODE.toString());
    }

    public static void setLEDMode(LEDmode mode) {
        LED_MODE = mode;
    }

    public static LEDmode getLEDMode() {
        return LED_MODE;
    }

    public static void enableUpperCam(){
        UPPER_CAM_ENABLED = true;
    }

    public static void disableUpperCam(){
        UPPER_CAM_ENABLED = false;
    }

 }
