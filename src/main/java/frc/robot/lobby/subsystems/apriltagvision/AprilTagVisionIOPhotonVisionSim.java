package frc.robot.lobby.subsystems.apriltagvision;

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

/**
 * This class is used to interface with PhotonVision for AprilTag detection to
 * enable simulations.
 */
public class AprilTagVisionIOPhotonVisionSim implements AprilTagVisionIO
{
    private final PhotonCameraSim cameraSim;
    private static VisionSystemSim visionSystemSim = null;
    private PhotonPoseEstimator poseEstimator;

    /**
     * Constructor for the AprilTagVisionIOPhotonVisionSim class.
     * 
     * @param cameraName       The name of the PhotonVision camera to use for
     *                         AprilTag detection.
     * @param cameraProperties The properties of the camera to simulate.
     * @param robotToCamera    The transform from the robot to the camera.
     */
    public AprilTagVisionIOPhotonVisionSim(String cameraName, SimCameraProperties cameraProperties,
            Transform3d robotToCamera)
    {
        cameraSim = new PhotonCameraSim(new PhotonCamera(cameraName), cameraProperties);
        try
        {
            poseEstimator = new PhotonPoseEstimator(
                    AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile),
                    PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, robotToCamera);
        } catch (IOException e)
        {
            throw new RuntimeException("Unable to initialize PhotonPoseEstimator due to missing field layout.", e);
        }
        getVisionSystemSim().addCamera(cameraSim, robotToCamera);
    }

    /**
     * Get the list of poses of the robot relative to the AprilTag.
     */

    @Override
    public List<Pose2dWithTimestamp> getPoses()
    {
        List<PhotonPipelineResult> poseResults = cameraSim.getCamera().getAllUnreadResults();
        List<Pose2dWithTimestamp> poses = new ArrayList<>();
        for (PhotonPipelineResult result : poseResults)
        {
            if (result.hasTargets())
            {
                var pose = poseEstimator.update(result);
                if (pose.isPresent())
                    poses.add(
                            new Pose2dWithTimestamp(pose.get().estimatedPose.toPose2d(), result.getTimestampSeconds()));
            }
        }

        return poses;
    }

    /**
     * Retrieves and sets the camera simulation object for PhotonVision.
     * 
     * @return the camera simulation object.
     */
    public static VisionSystemSim getVisionSystemSim()
    {
        if (visionSystemSim == null)
        {
            visionSystemSim = new VisionSystemSim("main");
            AprilTagFieldLayout fieldLayout;
            try
            {
                fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
                visionSystemSim.addAprilTags(fieldLayout);
            } catch (IOException e)
            {
                throw new RuntimeException("Unable to initialize VisionSystemSim due to missing field layout.", e);
            }
        }
        return visionSystemSim;
    }
}
