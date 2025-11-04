package frc.robot.SubSystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class Vision extends SubsystemBase {

        private double tx,ta,ty;
        private boolean tv;
        private final PhotonCamera camera;
        private final VisionSystemSim visionSim;
        private final SimCameraProperties cameraProps;
    

        
     
        private final NetworkTable limelight;           
                    
                    public Vision(){
                    limelight = NetworkTableInstance.getDefault().getTable("limelight-front");
                      camera = new PhotonCamera("limelight-front");

        SimCameraProperties camProps = new SimCameraProperties();
        camProps.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        camProps.setFPS(20);
        camProps.setAvgLatencyMs(30);

        cameraSim = new PhotonCameraSim(camera, camProps);

        visionSim = new VisionSystemSim("limelightSim");

        // Define a posição da câmera no robô
        Transform3d robotToCamera = new Transform3d(
            new Translation3d(0.2, 0.0, 0.3),
            new Rotation3d()
        );
        visionSim.addCamera(cameraSim, robotToCamera);

        // Adicionar um alvo simulado no campo
        Pose3d targetPose = new Pose3d(3.0, 0.0, 1.0, new Rotation3d());
        VisionTargetSim targetSim = new VisionTargetSim(targetPose, TargetModel.kAprilTag36h11);
        visionSim.addVisionTargets(targetSim);
    }
        }   
    
        public boolean hasTarget() {
         return limelight.getEntry("tv").getDouble(0.0) == 1.0;
        }
    
        public double getTx() {
         return limelight.getEntry("tx").getDouble(0.0);
        }
    
        public double getTa() {
         return limelight.getEntry("ta").getDouble(0.0);
        }
    
        @Override
        public void periodic() {
            SmartDashboard.putBoolean("Limelight Target", hasTarget());
            SmartDashboard.putNumber("Limelight TX", getTx());
            SmartDashboard.putNumber("Limelight TA", getTa());
          
        }
    
        @Override
        public void simulationPeriodic(){

            var result = camera.getLatestResult();
            boolean hasTargets = result.hasTargets();
    
            List<PhotonTrackedTarget> targets = result.getTargets();
    
            PhotonTrackedTarget target = result.getBestTarget();
    
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getArea();
            double skew = target.getSkew();
            Transform3d pose = target.getAlternateCameraToTarget();
            List<TargetCorner> corners = target.getDetectedCorners();
    
            int targetID = target.getFiducialId();
            double poseAmbiguity = target.getPoseAmbiguity();
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
        
          camera.takeInputSnapshot();
          camera.takeOutputSnapshot();
             
          Logger.recordOutput("Limelight/tx", tx);
          Logger.recordOutput("Limelight/ty", ty);
          Logger.recordOutput("Limelight/ta", ta);
          Logger.recordOutput("Limelight/tv", tv);
    }
 }

