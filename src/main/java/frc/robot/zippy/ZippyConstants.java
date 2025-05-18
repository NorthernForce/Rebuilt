package frc.robot.zippy;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ZippyConstants
{
    public class VisionConstants
    {
        public class LimeLightConstants
        {
            public static final String kLimeLightName = "limelight";
            public static final int[] kValidIds =
            { 13, 12, 16, 14, 15, 4, 5, 3, 2, 1 };
        }

        public class PhotonVisionConstants
        {
            public static final Transform3d kRobotToCamera = new Transform3d(new Translation3d(0.0, 0.0, 0.0),
                    new Rotation3d(0.0, 0.0, 0.0));
        }
    }
}
