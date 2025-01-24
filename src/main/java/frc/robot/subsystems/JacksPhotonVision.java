
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class JacksPhotonVision extends SubsystemBase{

    PhotonPipelineResult result;
    PhotonCamera camera;

    public JacksPhotonVision(){
        camera = new PhotonCamera("PC_Camera R");
    }

    

    public void periodic(){
        result = camera.getLatestResult();
        SmartDashboard.putBoolean("cameraData" , result.hasTargets());
        
    }
    
}
