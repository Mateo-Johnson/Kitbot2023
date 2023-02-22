package frc.robot.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;







public class visionmovement {


    public static void vision(){

        PortForwarder.add(5800, "photonvision.local", 5800);

        PhotonCamera camera = new PhotonCamera("Metrobots3324");


        var result = camera.getLatestResult();


        boolean hasTargets = result.hasTargets();


        PhotonTrackedTarget target = result.getBestTarget();




        if (hasTargets) {
            
            int targetID = target.getFiducialId();

            SmartDashboard.putNumber("TargetID", targetID); 


        } else {

            SmartDashboard.putNumber("TargetID", 0); 
        
        }

    }
}