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

/**
 * This class is used to interface with PhotonVision for AprilTag detection to
 * enable simulations.
 */
public class AprilTagVisionIOPhotonVisionSim implements AprilTagVision.AprilTagVisionIO
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
            // TODO Auto-generated catch block
            e.printStackTrace();
            poseEstimator = null;
        }
        getVisionSystemSim().addCamera(cameraSim, robotToCamera);
    }

    /**
     * Get the list of poses of the robot relative to the AprilTag.
     */
    public List<Pose2dWithTimestamp> getPose()
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
            } catch (IOException e)
            {
                // TODO Auto-generated catch block
                e.printStackTrace();
                fieldLayout = null;
            }
            visionSystemSim.addAprilTags(fieldLayout);
        }
        return visionSystemSim;
    }
}
