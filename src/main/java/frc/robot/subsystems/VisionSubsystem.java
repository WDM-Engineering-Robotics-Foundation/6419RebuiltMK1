package frc.robot.subsystems;

import org.ejml.ops.IPredicateBinary;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionConstants;

import java.util.List;

public class VisionSubsystem extends SubsystemBase {

    private PhotonCamera frontLocalCamera, frontElementCamera;

    private PhotonPoseEstimator poseEstimator;

    private Pose2d latestEstimate = new Pose2d();

    private double latestEstimateTime = 0;

    VisionSubsystem() {
        frontLocalCamera = new PhotonCamera(VisionConstants.FRONT_LOCALIZATION_CAMERA_NAME);
        frontElementCamera = new PhotonCamera(VisionConstants.FRONT_ELEMENT_CAMERA_NAME);
        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark), VisionConstants.FRONT_LOCALIZATION_CAMERA_OFFSET);

    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> latest = frontLocalCamera.getAllUnreadResults();
        
        for (PhotonPipelineResult result : latest) {
            EstimatedRobotPose poseEstimate;
            var optionalEstimate = poseEstimator.estimateCoprocMultiTagPose(result);
            if (optionalEstimate.isPresent()) poseEstimate = optionalEstimate.get();
            else {
                optionalEstimate = poseEstimator.estimateLowestAmbiguityPose(result);
                if (optionalEstimate.isPresent()) poseEstimate = optionalEstimate.get();
                else continue; // no valid estimate
            }
        
            Subsystems.drivetrain().addVisionMeasurement(latestEstimate = poseEstimate.estimatedPose.toPose2d(), latestEstimateTime = poseEstimate.timestampSeconds);
        }
        
    }

    public Pose2d getLatestPoseEstimate() {
        return latestEstimate;
    }

    public double getLatestEstimateTime() {
        return latestEstimateTime;
    }
    
}
