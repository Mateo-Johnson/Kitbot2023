package frc.robot.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;

public class photonvisionsetup {

    public static void main(String[] args){

        
        PortForwarder.add(5800, "photonvision.local", 5800);

        PhotonCamera camera = new PhotonCamera("Metrobots3324");

        var result = camera.getLatestResult();

        boolean hasTargets = result.hasTargets();

        List<PhotonTrackedTarget> targets = result.getTargets();

        PhotonTrackedTarget target = result.getBestTarget();

        int targetID = target.getFiducialId();
        double poseAmbiguity = target.getPoseAmbiguity();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

        
    }    

    
}



