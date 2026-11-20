package gtsam;

import org.junit.jupiter.api.Test;

import gtsam.noiseModel.Isotropic;

/**
 * The GeneralSFMFactor test in cpp doesn't test anything interesting.
 */
public class GeneralSFMFactorExtraTest {

    /**
     * See gtsam/slam/doc/GeneralSFMFactor.ipynb
     */
    @Test
    void test0() {

        Point2 measured_pt = new Point2(320, 240);// Measurement in pixels
        shared_ptr<Isotropic> sfm_noise = Isotropic.Sigma(2, 1.0);
        Key landmark_key = Key.L(0);
        Key pose_key = Key.X(0);
        Key calib_key = Key.K(0);

        shared_ptr<GeneralSFMFactor2<Cal3DS2>> factor2 = GeneralSFMFactor2.GeneralSFMFactor2Cal3DS2(
                measured_pt, sfm_noise, pose_key, landmark_key, calib_key);

        // should print
        // keys = { x0 l0 k0 }
        // noise model: unit (2)
        // GeneralSFMFactor2: .z[
        // 320;
        // 240
        // ]
        factor2.get().print();

        Values values2 = new Values();
        Pose3 camera_pose = new Pose3();
        Cal3DS2 calibration = new Cal3DS2(500, 500, 0, 320, 240, 0, 0, 0, 0); // fx, fy, s, u0, v0
        Point3 landmark = new Point3(0, 0, 5); // Point 5m in front of camera
        values2.insert(pose_key, camera_pose);
        values2.insert(landmark_key, landmark);
        values2.insert(calib_key, calibration);

        // should be zero
        double error2 = factor2.get().error(values2);
        System.out.printf("Error for GeneralSFMFactor2: %f\n", error2);

        Vector2 errV = factor2.get().evaluateError(
                camera_pose, landmark, calibration, new Matrix(), new Matrix(), new Matrix());

        // should be (0, 0)
        System.out.printf("errV %f %f\n", errV.at(0), errV.at(1));
    }

}
