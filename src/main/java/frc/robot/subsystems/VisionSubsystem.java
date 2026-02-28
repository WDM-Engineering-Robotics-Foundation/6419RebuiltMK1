package frc.robot.subsystems;

import org.ejml.ops.IPredicateBinary;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionConstants;

import java.util.List;

public class VisionSubsystem extends SubsystemBase {

    private PhotonCamera frontLocalCamera, frontElementCamera;

    private PhotonPoseEstimator poseEstimator;

    VisionSubsystem() {
        frontLocalCamera = new PhotonCamera(VisionConstants.FRONT_LOCALIZATION_CAMERA_NAME);
        frontElementCamera = new PhotonCamera(VisionConstants.FRONT_ELEMENT_CAMERA_NAME);
        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark), VisionConstants.FRONT_LOCALIZATION_CAMERA_OFFSET);

    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> latest = frontLocalCamera.getAllUnreadResults();
        EstimatedRobotPose poseEstimate;
        if (latest.isEmpty()) return;
        var optionalEstimate = poseEstimator.estimateCoprocMultiTagPose(latest.get(latest.size()-1));
        if (optionalEstimate.isPresent()) poseEstimate = optionalEstimate.get();
        else {
            optionalEstimate = poseEstimator.estimateLowestAmbiguityPose(latest.get(latest.size()-1));
            if (optionalEstimate.isPresent()) poseEstimate = optionalEstimate.get();
            else return; // no valid estimate
        }
        Subsystems.drivetrain().addVisionMeasurement(poseEstimate.estimatedPose.toPose2d(), poseEstimate.timestampSeconds);
    }
    
}
