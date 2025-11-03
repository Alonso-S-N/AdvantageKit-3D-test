package frc.robot.SubSystem;

import edu.wpi.first.math.geometry.*;
import org.littletonrobotics.junction.Logger;

public class Arm3d {
    // Comprimentos (em metros)
    private static final double ARM_LENGTH = 0.5;
    private static final double EXTENSOR_LENGTH = 0.3;

    // Estados
    private double armAngleDeg = 90;   // ângulo em relação ao chão
    private double extensorMeters = 0; // extensão linear
    private Pose3d robotPose = new Pose3d(); // posição base do robô

    public void setRobotPose(Pose3d pose) {
        this.robotPose = pose;
    }

    public void setArmAngle(double degrees) {
        this.armAngleDeg = degrees;
    }

    public void setExtensor(double meters) {
        this.extensorMeters = meters;
    }

    public void update() {
        // Braço — sai da base do robô
        Rotation3d armRotation = new Rotation3d(0, Math.toRadians(armAngleDeg), 0);
        Transform3d baseToArm = new Transform3d(
            new Translation3d(0.1, 0.1, 0.2),
            armRotation
        );

        Pose3d armPose = robotPose.plus(baseToArm);

        // Extensor — sai da ponta do braço
        Transform3d armToExtensor = new Transform3d(
            new Translation3d(ARM_LENGTH, 0.1, 0.2),
            new Rotation3d(0, 0, 0)
        );

        Pose3d extensorBase = armPose.plus(armToExtensor);

        // Ponta do intake — extensão variável
        Transform3d extensorToIntake = new Transform3d(
            new Translation3d(EXTENSOR_LENGTH + extensorMeters, 0.1, 0.2),
            new Rotation3d(0, 0, 0)
        );

        Pose3d intakePose = extensorBase.plus(extensorToIntake);

        Logger.recordOutput("Binga/Base", robotPose);
        Logger.recordOutput("Binga/Arm", armPose);
        Logger.recordOutput("Binga/Extensor", extensorBase);
        Logger.recordOutput("Binga/Intake", intakePose);
        Logger.recordOutput("Binga/Array", new Pose3d[] { armPose, extensorBase, intakePose }); // opcional

    }
}
