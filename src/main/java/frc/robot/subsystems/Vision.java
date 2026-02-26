/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase {
    private final PhotonCamera frontCam = new PhotonCamera(VisionConstants.ROBOT_TO_FRONT_CAM_NAME);
    private final PhotonCamera backCam = new PhotonCamera(VisionConstants.ROBOT_TO_BACK_CAM_NAME);
    private final PhotonPoseEstimator frontCamPhotonEstimator = new PhotonPoseEstimator(VisionConstants.TAG_LAYOUT,
            VisionConstants.ROBOT_TO_FRONT_CAM);
    private final PhotonPoseEstimator backCamPhotonEstimator = new PhotonPoseEstimator(VisionConstants.TAG_LAYOUT,
            VisionConstants.ROBOT_TO_BACK_CAM);
    private Matrix<N3, N1> frontCurStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
    private Matrix<N3, N1> backCurStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;

    private static Vision instance;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    private Vision() {
        backCam.setPipelineIndex(1);
    }

    private Optional<EstimatedRobotPose> estimateCameraPose(
            PhotonCamera camera,
            PhotonPoseEstimator estimator,
            Matrix<N3, N1> curStdDevs,
            Consumer<Matrix<N3, N1>> stdDevSetter) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (PhotonPipelineResult change : camera.getAllUnreadResults()) {
            visionEst = estimator.estimateCoprocMultiTagPose(change);
            if (visionEst.isEmpty()) {
                visionEst = estimator.estimateLowestAmbiguityPose(change);
            }
            curStdDevs = updateEstimationStdDevs(estimator, visionEst, change.getTargets());
        }
        stdDevSetter.accept(curStdDevs);
        return visionEst;
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets
     *         used for estimation.
     */
    public Optional<EstimatedRobotPose> frontCamGetEstimatedGlobalPose() {
        return estimateCameraPose(frontCam, frontCamPhotonEstimator, frontCurStdDevs,
                stdDevs -> frontCurStdDevs = stdDevs);
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets
     *         used for estimation.
     */
    public Optional<EstimatedRobotPose> backCamGetEstimatedGlobalPose() {
        return estimateCameraPose(backCam, backCamPhotonEstimator, backCurStdDevs, stdDevs -> backCurStdDevs = stdDevs);
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from
     * the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private Matrix<N3, N1> updateEstimationStdDevs(
            PhotonPoseEstimator estimator,
            Optional<EstimatedRobotPose> estimatedPose,
            List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            return VisionConstants.SINGLE_TAG_STD_DEVS;
        } else {
            Matrix<N3, N1> estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
            int numTags = 0;
            double avgDist = 0;

            for (PhotonTrackedTarget tgt : targets) {
                Optional<Pose3d> tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) {
                    continue;
                }
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                return estStdDevs;
            } else {
                if (numTags > 1) {
                    avgDist /= numTags;
                    estStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;
                } else if (numTags == 1 && avgDist > 4) {
                    return estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                }

                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

                return estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from
     * {@link #frontCamGetEstimatedGlobalPose}.
     * For use with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     */
    public Matrix<N3, N1> getFrontEstimationStdDevs() {
        return frontCurStdDevs;
    }

    /**
     * Returns the latest standard deviations of the estimated pose from
     * {@link #backCamGetEstimatedGlobalPose}.
     * For use with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     */
    public Matrix<N3, N1> getBackEstimationStdDevs() {
        return backCurStdDevs;
    }
}