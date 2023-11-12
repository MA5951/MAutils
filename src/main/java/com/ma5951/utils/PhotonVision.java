package com.ma5951.utils;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class PhotonVision {
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    private double cameraHeightMeters;
    private double cameraPitchRadians;
    private AprilTagFieldLayout layout;
    private PhotonPoseEstimator photonPoseEstimator;
    private int pipline = 0;

    public PhotonVision(String cameraName,
            Transform3d robotToCam,
            AprilTagFieldLayout layout) {
        camera = new PhotonCamera(cameraName);
        this.cameraHeightMeters = robotToCam.getZ();
        this.cameraPitchRadians = robotToCam.getRotation().getY();
        this.layout = layout;
        photonPoseEstimator = new PhotonPoseEstimator(
                layout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, robotToCam);
        update();
    }

    /**
     * @return The yaw of the target in degrees (positive right).
     */
    public double getYaw() {
        return target.getYaw();
    }

    /**
     * @return The pitch of the target in degrees (positive up).
     */
    public double getPich() {
        return target.getPitch();
    }

    /**
     * @return The area (how much of the camera feed the bounding box takes up) as a
     *         percent (0-100).
     */
    public double getArea() {
        return target.getArea();
    }

    /**
     * @return The skew of the target in degrees (counter-clockwise positive).
     */
    public double getSkew() {
        return target.getSkew();
    }

    /**
     * @return Get the transform that maps camera space (X = forward, Y = left, Z =
     *         up)
     *         to object/fiducial tag space (X forward, Y left, Z up)
     *         with the lowest reprojection error.
     */
    public Transform3d getBestCameraToTarget() {
        return target.getBestCameraToTarget();
    }

    /**
     * @return Get the transform that maps camera space (X = forward, Y = left, Z =
     *         up)
     *         to object/fiducial tag space (X forward, Y left, Z up)
     *         with the highest reprojection error.
     */
    public Transform3d getAlternateCameraToTarget() {
        return target.getAlternateCameraToTarget();
    }

    /**
     * @return How ambiguous the pose of the target is (see below).
     */
    public double getPoseAmbiguity() {
        return target.getPoseAmbiguity();
    }

    /**
     * @return The ID of the detected fiducial marker.
     */
    public int getTargetID() {
        return target.getFiducialId();
    }

    /**
     * Capture pre-process camera stream image
     */
    public void takeInputSnapshot() {
        camera.takeInputSnapshot();
    }

    /**
     * Capture post-process camera stream image
     */
    public void takeOutputSnapshot() {
        camera.takeOutputSnapshot();
    }

    /**
     * @return distance from the target in meters
    */
    public double getDistanceToTargetMeters() {
        return PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeightMeters,
                layout.getTagPose(getTargetID()).get().getZ(),
                cameraPitchRadians,
                Units.degreesToRadians(getPich())) - 0.1;
    }

    /**
     * @return distance from the target in meters
     */
    public double getDistanceToTargetMeters(double targetHaight) {
        return PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeightMeters,
                targetHaight,
                cameraPitchRadians,
                Units.degreesToRadians(getPich())) - 0.1;
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    /**
     * change pipelines
     * 
     * @param pipeLine pipeline index
     */
    public void changePipeline(int PipelineIndex) {
        camera.setPipelineIndex(PipelineIndex);
        pipline = PipelineIndex;
    }

    public int getPipeline() {
        return pipline;
    }

    /**
     * driver mode is an unfiltered / normal view of the camera to be used while
     * driving the robot.
     * 
     * @param value is driver mode sould be on
     */
    public void setDriverMode(boolean value) {
        camera.setDriverMode(value);
    }

    /**
     * @return Get the pipeline latency.
     */
    public double getLatencySeconds() {
        return result.getLatencyMillis() / 1000.0;
    }

    /**
     * @param mode enum class is provided to choose values from.
     *             These values include, kOff, kOn, kBlink, and kDefault.
     *             kDefault uses the default LED value from the selected pipeline.
     */
    public void setLED(VisionLEDMode mode) {
        camera.setLED(mode);
    }
 
    public boolean hasResult() {
        return result != null;
    }

    /**
     * @return if a targt was detected
     */
    public boolean hasTarget() {
        if (!result.hasTargets()) {
            return false;
        }else if ( getPipeline() == 0 && getTargetID() <= 8) {
            return true;
        } else if ( getPipeline() == 1) {
            return true;
        } else {
            return false;
        }

    }

    public void update() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
        }
    }
}
