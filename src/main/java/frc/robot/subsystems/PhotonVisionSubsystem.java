package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class PhotonVisionSubsystem extends SubsystemBase {
    private PhotonCamera camera;

    public PhotonVisionSubsystem() {
        camera = new PhotonCamera("photonvision");
    }

    public PhotonPipelineResult getResult() {
        return camera.getLatestResult();
    }

    public PhotonTrackedTarget getTarget() {
        return getResult().getBestTarget();
    }

    public PhotonTrackedTarget getSpecificTarget(int index) {
        return getResult().getTargets().get(index);
    }

    public void setPipeline(int index){
        camera.setPipelineIndex(index);
    }

}
