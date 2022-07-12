package frc3512.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.NetworkTableUtil;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  public static String kVisionCamName = "mmal_service_16.1";

  public static double kTargetHeight = 2.6;
  public static double kCameraHeight = 0.76835;
  public static double kCameraPitch = 44.6;
  public static double kCameraDiagnoalFOV = 74.8;
  public static double kYawOffset = 3.5;

  PhotonCamera m_visionCamera = new PhotonCamera(kVisionCamName);

  boolean m_haveTargets = false;

  public class VisionMeasurements {
    public double m_yaw;
    public double m_range;
    public double m_pitch;
  }

  List<VisionMeasurements> m_visionQueue = new ArrayList<>();

  NetworkTableEntry m_yawEntry = NetworkTableUtil.MakeDoubleEntry("/Diagnostics/Vision/Yaw");
  NetworkTableEntry m_rangeEntry = NetworkTableUtil.MakeDoubleEntry("/Diagnostics/Vision/Range");
  NetworkTableEntry m_pitchEntry = NetworkTableUtil.MakeDoubleEntry("/Diagnostics/Vision/Pitch");
  NetworkTableEntry m_timestampEntry =
      NetworkTableUtil.MakeDoubleEntry("/Diagnostics/Vision/Timestamp");

  public double getLatestYaw() {
    return m_visionQueue.get(m_visionQueue.size() - 1).m_yaw;
  }

  public double getLatestPitch() {
    return m_visionQueue.get(m_visionQueue.size() - 1).m_pitch;
  }

  public double getLatestRange() {
    return m_visionQueue.get(m_visionQueue.size() - 1).m_range;
  }

  public boolean hasTargets() {
    return m_haveTargets;
  }

  public void updateVision() {
    if (!RobotBase.isSimulation()) {
      var result = m_visionCamera.getLatestResult();

      if (result.getTargets().size() == 0 || DriverStation.isDisabled()) {
        m_haveTargets = false;
        return;
      }

      m_haveTargets = true;
      var timestamp = Timer.getFPGATimestamp() - result.getLatencyMillis();
      m_timestampEntry.setDouble(timestamp);

      VisionMeasurements currentMeasurement = new VisionMeasurements();
      PhotonTrackedTarget target = result.getBestTarget();

      var pitch = target.getPitch();
      currentMeasurement.m_pitch = pitch;

      if (target.getYaw() < 0.0) {
        currentMeasurement.m_yaw = target.getYaw() - kYawOffset;
      } else if (target.getYaw() > 0.0) {
        currentMeasurement.m_yaw = target.getYaw() + kYawOffset;
      } else {
        currentMeasurement.m_yaw = target.getYaw();
      }

      m_yawEntry.setDouble(currentMeasurement.m_yaw);

      currentMeasurement.m_range =
          PhotonUtils.calculateDistanceToTargetMeters(kCameraHeight, 2.606, kCameraHeight, pitch);

      m_rangeEntry.setDouble(currentMeasurement.m_range);

      m_visionQueue.add(currentMeasurement);
    }
  }

  @Override
  public void periodic() {
    updateVision();
  }
}
