package frc.robot.SubSystem;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calcs.DriveSpeeds;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


public class Vision extends SubsystemBase {

        private double tx,ta,ty;
        private boolean tv;
        private final double targetArea;
  private boolean finished;
  public double RightSpeed;
  private Drive drive;
  
    public double LeftSpeed;
    private Timer timer = new Timer();
  
    private final double hysteresis = 0.5; 
    private boolean holdingPosition = false;
  
  
          
          private final PhotonCamera camera;
         private PhotonCameraSim cameraSim;
         private VisionSystemSim visionSim;
  
          private final NetworkTable limelight;           
  
        public Vision(Drive drive, double targetArea) {
          this.targetArea = targetArea;
          this.drive = drive;
                 timer.reset();
                drive.getPose();
                finished = false;
                holdingPosition = false;
                camera = new PhotonCamera("camera");

limelight = NetworkTableInstance.getDefault().getTable("limelight-front");
PhotonCamera camera = new PhotonCamera("camera");

AprilTagFieldLayout tagLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();

if (RobotBase.isSimulation()) {

  SimCameraProperties props = new SimCameraProperties();
  props.setCalibration(960, 720, Rotation2d.fromDegrees(90));  //resolução e FOV
  props.setFPS(30);
  props.setAvgLatencyMs(30);
  props.setLatencyStdDevMs(5);
      
                   
  visionSim = new VisionSystemSim("MainVisionSim");
  visionSim.addAprilTags(tagLayout);

  Transform3d robotToCamera = new Transform3d(
    new Translation3d(0.2, 0.0, 0.6),
    new Rotation3d(0, 0, 0)
                );
    
                
                cameraSim = new PhotonCameraSim(camera, props);
                visionSim.addCamera(cameraSim, robotToCamera);
    
      
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

        public DriveSpeeds Perseguir(){
          timer.start();
          if (hasTarget()) {
        double tx = getTx();
        double ta = getTa();
        
        double kP_turn = 0.03;
        double kP_forward = 0.1;
  
        double turn = tx * kP_turn; 
        double forward = (targetArea - ta) * kP_forward;
  
        forward = Math.max(-0.5, Math.min(0.5, forward));
  
        double left = forward + turn;
        double right = forward - turn;
  
        left = Math.max(-0.5, Math.min(0.5, left));
        right = Math.max(-0.5, Math.min(0.5, right));
  
        RightSpeed = right;
        LeftSpeed = left;
  
        if (ta >= (targetArea - hysteresis)) {
            setDriveSpeeds(0, 0);
        } else {
            setDriveSpeeds(left, right);
        }
    } else if (!hasTarget()) {
      if (getTime() < 2.0) {
        setDriveSpeeds(-0.2, 0.2); 
        LeftSpeed = -0.2; RightSpeed = 0.2;
    } else if (getTime() < 4.0) {
        setDriveSpeeds(0.2, -0.2); 
        LeftSpeed = 0.2; RightSpeed = -0.2;
    } else {
        setDriveSpeeds(0, 0);
        LeftSpeed = 0; RightSpeed = 0; 
    }
    }
              return new DriveSpeeds(LeftSpeed, RightSpeed);
 }
        private void setDriveSpeeds(double left, double right) {
          drive.rawTank(left, right);
        }

        public double getTime(){
          return timer.get();
        }
        public void resetTime(){
          timer.reset();
        }
    
        @Override
        public void simulationPeriodic(){

          if (visionSim != null) {
            // Atualiza a posição da simulação de visão com base no drive (pose estimada do robô)
            visionSim.update(drive.getPose());
        }
    
        // Obtém o resultado da câmera simulada
        if (camera != null) {
            try {
                var result = camera.getLatestResult();
                if (result.hasTargets()) {
                    var target = result.getBestTarget();
                    tx = target.getYaw();
                    ty = target.getPitch();
                    ta = target.getArea();
                    tv = true;
                } else {
                    tv = false;
                    tx = ty = ta = 0;
                }
            } catch (Exception e) {
                System.out.println("[Vision] Erro getLatestResult: " + e.getMessage());
            }
        }
    
        SmartDashboard.putBoolean("Sim Target", tv);
        SmartDashboard.putNumber("Sim TX", tx);
        SmartDashboard.putNumber("Sim TY", ty);
        SmartDashboard.putNumber("Sim TA", ta);
    }
 }

