package frc.robot.subsystems;

public enum LEDmode {
    NONE,
    ALLIANCE,           // Display Alliance color
    RAINBOW,            // Show a pretty Rainbow
    MANUAL,             // Seeking a speaker to score
    WINDING_UP,         // We want to shoot but we aren't ready yet.
    APPROACH,           // Shooting even if not ready
    SPEEDOMETER,        // Displaying robot speed on power meter.
    LOWERING,           // Baton is still returning from raised position
    ERROR,              // Displaying system error code
    DEFAULT
}
