package gtsam;

import java.util.List;

import org.junit.jupiter.api.Test;

import gtsam.FixedLagSmoother.Result;
import gtsam.noiseModel.Diagonal;

/** These tests are for investigating the smoother problem. */
public class SmootherExtraTest {
    // static double TAG_SIZE_M = 0.1651;
    static double TAG_SIZE_M = 2;
    static double HALF = TAG_SIZE_M / 2;

    @Test
    void testBatch() throws Throwable {
        var new_factors = new NonlinearFactorGraph();
        var new_values = new Values();
        var new_timestamps = new FixedLagSmoother.KeyTimestampMap();
        var lag = 100000;
        var smoother = new BatchFixedLagSmoother(lag);

        new_values.insert(Key.X(0), new Pose2());
        new_timestamps.put(Key.X(0), 0);

        prior(0, 100, new_factors);

        vision(new_factors);

        Result updateResult = smoother.update(new_factors, new_values, new_timestamps);
        updateResult.print();
        Values result = smoother.calculateEstimate();
        result.print("result");
    }

    @Test
    void testLM() throws Throwable {
        var new_factors = new NonlinearFactorGraph();
        var new_values = new Values();

        new_values.insert(Key.X(0), new Pose2());

        prior(0, 100, new_factors);

        vision(new_factors);

        var optimizer = new LevenbergMarquardtOptimizer(new_factors, new_values);
        Values result = optimizer.optimize();
        result.print("result");
    }

    @Test
    void testGN() throws Throwable {
        var new_factors = new NonlinearFactorGraph();
        var new_values = new Values();

        new_values.insert(Key.X(0), new Pose2());

        prior(0, 100, new_factors);

        vision(new_factors);

        var optimizer = new GaussNewtonOptimizer(new_factors, new_values, new GaussNewtonParams());
        Values result = optimizer.optimize();
        result.print("result");
    }

    @Test
    void testGNBetween() throws Throwable {
        var new_factors = new NonlinearFactorGraph();
        var new_values = new Values();

        new_values.insert(Key.X(0), new Pose2());
        new_values.insert(Key.X(1), new Pose2());

        prior(0, 100, new_factors);
        // like "landmark"
        prior(1, 0.001, new_factors);

        shared_ptr<BetweenFactorPose2> f = BetweenFactorPose2.newBetweenFactorPose2(
                Key.X(0), Key.X(1), new Pose2(1, 0, 0), Diagonal.Sigmas(new Vector3(0.001, 0.001, 0.001)));
        new_factors.add(f);
        var optimizer = new GaussNewtonOptimizer(new_factors, new_values, new GaussNewtonParams());
        Values result = optimizer.optimize();
        result.print("result");
    }

    @Test
    void testGNBearingRange() throws Throwable {
        var new_factors = new NonlinearFactorGraph();
        var new_values = new Values();

        new_values.insert(Key.X(0), new Pose2());
        new_values.insert(Key.L(1), new Point2());
        new_values.insert(Key.L(2), new Point2());

        new_factors.add(PriorFactor.PriorFactorPose2(
                Key.X(0),
                new Pose2(0, 0, 0),
                Diagonal.Sigmas(new Vector3(100, 100, 100))));
        // "landmarks"
        new_factors.add(PriorFactor.PriorFactorPoint2(
                Key.L(1),
                new Point2(2, 1),
                Diagonal.Sigmas(new Vector2(0.001, 0.001))));
        new_factors.add(PriorFactor.PriorFactorPoint2(
                Key.L(2),
                new Point2(1, 2),
                Diagonal.Sigmas(new Vector2(0.001, 0.001))));

        shared_ptr<BearingRangeFactorPose2Point2> f = BearingRangeFactorPose2Point2.newBearingRangeFactorPose2Point2(
                Key.X(0), Key.L(1), new Rot2(-Math.PI / 4), 1, Diagonal.Sigmas(new Vector2(0.001, 0.001)));
        new_factors.add(f);
        shared_ptr<BearingRangeFactorPose2Point2> f2 = BearingRangeFactorPose2Point2.newBearingRangeFactorPose2Point2(
                Key.X(0), Key.L(2), new Rot2(Math.PI / 4), 1, Diagonal.Sigmas(new Vector2(0.001, 0.001)));
        new_factors.add(f2);

        var optimizer = new GaussNewtonOptimizer(new_factors, new_values, new GaussNewtonParams());
        Values result = optimizer.optimize();
        result.print("result");
    }

    @Test
    void testIncremental() throws Throwable {
        var new_factors = new NonlinearFactorGraph();
        var new_values = new Values();
        var new_timestamps = new FixedLagSmoother.KeyTimestampMap();
        var lag = 100000;
        ISAM2Params isam2Params = new ISAM2Params();
        isam2Params.findUnusedFactorSlots(true);
        var smoother = new IncrementalFixedLagSmoother(lag, isam2Params);

        new_values.insert(Key.X(0), new Pose2());
        new_timestamps.put(Key.X(0), 0);

        prior(0, 100, new_factors);

        vision(new_factors);

        Result updateResult = smoother.update(new_factors, new_values, new_timestamps);
        updateResult.print();
        Values result = smoother.calculateEstimate();
        result.print("result");
    }

    @Test
    void testISAM2() throws Throwable {
        var new_factors = new NonlinearFactorGraph();
        var new_values = new Values();

        new_values.insert(Key.X(0), new Pose2());

        prior(0, 100, new_factors);

        vision(new_factors);

        ISAM2Params isam2Params = new ISAM2Params();
        ISAM2 isam2 = new ISAM2(isam2Params);
        ISAM2Result isam2Result = isam2.update(new_factors, new_values);
        isam2Result.print();
        Values result = isam2.calculateEstimate();
        result.print("result");
    }

    @Test
    void testISAM2Between() throws Throwable {
        var new_factors = new NonlinearFactorGraph();
        var new_values = new Values();

        new_values.insert(Key.X(0), new Pose2());
        new_values.insert(Key.X(1), new Pose2());

        prior(0, 100, new_factors);
        // like "landmark"
        prior(1, 0.001, new_factors);

        shared_ptr<BetweenFactorPose2> f = BetweenFactorPose2.newBetweenFactorPose2(
                Key.X(0), Key.X(1), new Pose2(1, 0, 0), Diagonal.Sigmas(new Vector3(0.001, 0.001, 0.001)));
        new_factors.add(f);

        ISAM2Params isam2Params = new ISAM2Params();
        ISAM2 isam2 = new ISAM2(isam2Params);
        ISAM2Result isam2Result = isam2.update(new_factors, new_values);
        isam2Result.print();
        Values result = isam2.calculateEstimate();
        result.print("result");
    }

    @Test
    void testISAM2BearingRange() throws Throwable {
        var new_factors = new NonlinearFactorGraph();
        var new_values = new Values();

        new_values.insert(Key.X(0), new Pose2());
        new_values.insert(Key.L(1), new Point2());
        new_values.insert(Key.L(2), new Point2());

        new_factors.add(PriorFactor.PriorFactorPose2(
                Key.X(0),
                new Pose2(0, 0, 0),
                Diagonal.Sigmas(new Vector3(100, 100, 100))));
        // "landmarks"
        new_factors.add(PriorFactor.PriorFactorPoint2(
                Key.L(1),
                new Point2(2, 1),
                Diagonal.Sigmas(new Vector2(0.001, 0.001))));
        new_factors.add(PriorFactor.PriorFactorPoint2(
                Key.L(2),
                new Point2(1, 2),
                Diagonal.Sigmas(new Vector2(0.001, 0.001))));

        shared_ptr<BearingRangeFactorPose2Point2> f = BearingRangeFactorPose2Point2.newBearingRangeFactorPose2Point2(
                Key.X(0), Key.L(1), new Rot2(-Math.PI / 4), 1, Diagonal.Sigmas(new Vector2(0.001, 0.001)));
        new_factors.add(f);
        shared_ptr<BearingRangeFactorPose2Point2> f2 = BearingRangeFactorPose2Point2.newBearingRangeFactorPose2Point2(
                Key.X(0), Key.L(2), new Rot2(Math.PI / 4), 1, Diagonal.Sigmas(new Vector2(0.001, 0.001)));
        new_factors.add(f2);

        ISAM2Params isam2Params = new ISAM2Params();
        ISAM2 isam2 = new ISAM2(isam2Params);
        ISAM2Result isam2Result = isam2.update(new_factors, new_values);
        isam2Result.print();
        Values result = isam2.calculateEstimate();
        result.print("result");
    }

    void prior(long j, double sigma, NonlinearFactorGraph new_factors) throws Throwable {
        new_factors.add(PriorFactor.PriorFactorPose2(
                Key.X(j),
                new Pose2(0, 0, 0),
                Diagonal.Sigmas(new Vector3(sigma, sigma, sigma))));
    }

    void vision(NonlinearFactorGraph new_factors) throws Throwable {
        List<Point3> landmarks = make_tag(8, 4, 1, 0);
        // this is what the solver should produce
        Pose2 ground_truth = new Pose2(2, 2, 0);
        List<Point2> measurements = pixels(landmarks, ground_truth);
        if (landmarks.size() != measurements.size())
            throw new RuntimeException();
        // try one point only, for debugging.
        // m_vision.add(t1_us, m_landmarks.get(0), measurements.get(0));
        for (int i = 0; i < landmarks.size(); ++i) {
            Point3 landmark = landmarks.get(i);
            Point2 measurement = measurements.get(i);
            shared_ptr<PlanarProjectionFactor1> f = //
                    PlanarProjectionFactor1.newPlanarProjectionFactor1(
                            Key.X(0),
                            landmark,
                            measurement,
                            camera_offset(),
                            calib(),
                            Diagonal.Sigmas(new Vector2(2, 2)));
            new_factors.add(f);
        }
    }

    List<Point3> make_tag(double x, double y, double z, double yaw) throws Throwable {
        double s = HALF * Math.sin(yaw);
        double c = HALF * Math.cos(yaw);
        Point3 ll = new Point3(x - s, y + c, z - HALF);
        Point3 lr = new Point3(x + s, y - c, z - HALF);
        Point3 ur = new Point3(x + s, y - c, z + HALF);
        Point3 ul = new Point3(x - s, y + c, z + HALF);
        return List.of(ll, lr, ur, ul);
    }

    Point2 pixel(Point3 landmark, Pose2 robot_pose) throws Throwable {
        Pose3 camera_offset = camera_offset();
        Cal3DS2 calib = calib();
        Pose3 camera_pose = new Pose3(robot_pose).compose(camera_offset);
        PinholeCamera<Cal3DS2> camera = PinholeCamera.PinholeCameraCal3DS2(
                camera_pose, calib);
        return camera.project(landmark);
    }

    Cal3DS2 calib() throws Throwable {
        return new Cal3DS2(200.0, 200.0, 0.0, 400.0, 300.0, 0.0, 0.0);
    }

    Pose3 camera_offset() throws Throwable {
        return new Pose3(
                new Rot3(//
                        0, 0, 1, //
                        -1, 0, 0, //
                        0, -1, 0),
                new Point3(0, 0, 0.5));
    }

    List<Point2> pixels(List<Point3> landmarks, Pose2 robot_pose) throws Throwable {
        // lower left
        Point2 p0 = pixel(landmarks.get(0), robot_pose);
        // lower right
        Point2 p1 = pixel(landmarks.get(1), robot_pose);
        // upper right
        Point2 p2 = pixel(landmarks.get(2), robot_pose);
        // upper left
        Point2 p3 = pixel(landmarks.get(3), robot_pose);
        List<Point2> gt_pixels = List.of(p0, p1, p2, p3);
        // Omit out-of-frame tags.
        for (Point2 p : gt_pixels) {
            double x = p.x();
            double y = p.y();
            if (x < 0 || y < 0 || x > 800 || y > 600) {
                // any corner out of frame means the whole tag is not seen
                return List.of();
            }
        }
        return gt_pixels;
    }
}
