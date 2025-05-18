package frc.robot.subsystems.apriltagvision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.apriltagvision.AprilTagVision.Pose2dWithTimestamp;
public class AprilTagVisionIOPhotonVisionSim implements AprilTagVision.AprilTagVisionIO {
    private final PhotonCameraSim cameraSim;
    private static VisionSystemSim visionSystemSim = null;
    private PhotonPoseEstimator poseEstimator;
    public AprilTagVisionIOPhotonVisionSim(String cameraName, SimCameraProperties cameraProperties, Transform3d robotToCamera) {
        cameraSim = new PhotonCameraSim(new PhotonCamera(cameraName), cameraProperties);
        try {
            poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile), PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, robotToCamera);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            poseEstimator = null;
        }
        getVisionSystemSim().addCamera(cameraSim, robotToCamera);
    }

    public List<Pose2dWithTimestamp> getPose() {
        List<PhotonPipelineResult> poseResults = cameraSim.getCamera().getAllUnreadResults();
        List<Pose2dWithTimestamp> poses = new ArrayList<>();
        for (PhotonPipelineResult result : poseResults) {
            if (result.hasTargets()) {
                var pose = poseEstimator.update(result);
                if (pose.isPresent())
                    poses.add(new Pose2dWithTimestamp(pose.get().estimatedPose.toPose2d(), result.getTimestampSeconds()));
            }
        }

        return poses;
    }

    public static VisionSystemSim getVisionSystemSim() {
        if (visionSystemSim == null) {
            visionSystemSim = new VisionSystemSim("main");
            AprilTagFieldLayout fieldLayout;
            try {
                fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
                fieldLayout = null;
            }
            visionSystemSim.addAprilTags(fieldLayout);
        }
        return visionSystemSim;
    }
}
