package gtsam;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;
import java.util.function.DoubleSupplier;

import org.junit.jupiter.api.Test;

/**
 * See gtsam/slam/tests/testPlanarProjectionFactor.cpp.
 */
public class PlanarProjectionFactorTest {
    /**
     * Example: center projection and Jacobian.
     */
    @Test
    void testError1_1() throws Throwable {
        Point3 landmark = new Point3(1, 0, 0);
        Point2 measured = new Point2(200, 200);
        Pose3 offset = new Pose3(
                new Rot3(0, 0, 1, //
                        -1, 0, 0, //
                        0, -1, 0),
                new Point3(0, 0, 0));
        Cal3DS2 calib = new Cal3DS2(200, 200, 0, 200, 200, 0, 0);
        SharedNoiseModel model = SharedNoiseModel.Sigmas(new Vector2(1, 1));
        shared_ptr<PlanarProjectionFactor1> factor = PlanarProjectionFactor1.newPlanarProjectionFactor1(
                Key.X(0), landmark, measured, offset, calib, model);
        Pose2 pose = new Pose2(0, 0, 0);
        Matrix H = new Matrix();
        Vector2 err = factor.get().evaluateError(pose, H);
        assertEquals(0, err.at(0), 1e-6);
        assertEquals(0, err.at(1), 1e-6);
        assertEquals(0, H.at(0, 0), 1e-6);
        assertEquals(200, H.at(0, 1), 1e-6);
        assertEquals(200, H.at(0, 2), 1e-6);
        assertEquals(0, H.at(1, 0), 1e-6);
        assertEquals(0, H.at(1, 1), 1e-6);
        assertEquals(0, H.at(1, 2), 1e-6);
    }

    /**
     * Example: upper left corner projection and Jacobian.
     */
    @Test
    void Error1_2() throws Throwable {
        Point3 landmark = new Point3(1, 1, 1);
        Point2 measured = new Point2(0, 0);
        Pose3 offset = new Pose3(
                new Rot3(0, 0, 1, //
                        -1, 0, 0, //
                        0, -1, 0),
                new Point3(0, 0, 0));
        Cal3DS2 calib = new Cal3DS2(200, 200, 0, 200, 200, 0, 0);
        SharedNoiseModel model = SharedNoiseModel.Sigmas(new Vector2(1, 1));
        shared_ptr<PlanarProjectionFactor1> factor = PlanarProjectionFactor1.newPlanarProjectionFactor1(
                Key.X(0), landmark, measured, offset, calib, model);
        Pose2 pose = new Pose2(0, 0, 0);
        Matrix H = new Matrix();
        Vector2 err = factor.get().evaluateError(pose, H);
        assertEquals(0, err.at(0), 1e-6);
        assertEquals(0, err.at(1), 1e-6);
        assertEquals(-200, H.at(0, 0), 1e-6);
        assertEquals(200, H.at(0, 1), 1e-6);
        assertEquals(400, H.at(0, 2), 1e-6);
        assertEquals(-200, H.at(1, 0), 1e-6);
        assertEquals(0, H.at(1, 1), 1e-6);
        assertEquals(200, H.at(1, 2), 1e-6);
    }

    /**
     * Example: upper left corner projection and Jacobian with distortion.
     */
    @Test
    void Error1_3() throws Throwable {
        Point3 landmark = new Point3(1, 1, 1);
        Point2 measured = new Point2(0, 0);
        Pose3 offset = new Pose3(
                new Rot3(0, 0, 1, //
                        -1, 0, 0, //
                        0, -1, 0),
                new Point3(0, 0, 0));
        Cal3DS2 calib = new Cal3DS2(200, 200, 0, 200, 200, -0.2, 0.1); // note distortion
        SharedNoiseModel model = SharedNoiseModel.Sigmas(new Vector2(1, 1));
        shared_ptr<PlanarProjectionFactor1> factor = PlanarProjectionFactor1.newPlanarProjectionFactor1(
                Key.X(0), landmark, measured, offset, calib, model);
        Pose2 pose = new Pose2(0, 0, 0);
        Matrix H = new Matrix();
        Vector2 err = factor.get().evaluateError(pose, H);
        assertEquals(0, err.at(0), 1e-6);
        assertEquals(0, err.at(1), 1e-6);
        assertEquals(-360, H.at(0, 0), 1e-6);
        assertEquals(280, H.at(0, 1), 1e-6);
        assertEquals(640, H.at(0, 2), 1e-6);
        assertEquals(-360, H.at(1, 0), 1e-6);
        assertEquals(80, H.at(1, 1), 1e-6);
        assertEquals(440, H.at(1, 2), 1e-6);
    }

    /**
     * Verify Jacobians with numeric derivative.
     */
    @Test
    void Jacobian() throws Throwable {
        Random rng = new Random(42);
        DoubleSupplier dist = () -> rng.nextDouble(-0.3, 0.3);
        SharedNoiseModel model = SharedNoiseModel.Sigmas(new Vector2(1, 1));
        // center of the random camera poses
        Pose3 centerOffset = new Pose3(
                new Rot3(0, 0, 1, //
                        -1, 0, 0, //
                        0, -1, 0),
                new Point3(0, 0, 0));

        for (int i = 0; i < 1000; ++i) {
            Point3 landmark = new Point3(2 + dist.getAsDouble(), dist.getAsDouble(), dist.getAsDouble());
            Point2 measured = new Point2(200 + 100 * dist.getAsDouble(), 200 + 100 * dist.getAsDouble());
            Pose3 offset = centerOffset.compose(
                    new Pose3(
                            Rot3.Ypr(dist.getAsDouble(), dist.getAsDouble(), dist.getAsDouble()),
                            new Point3(dist.getAsDouble(), dist.getAsDouble(), dist.getAsDouble())));
            Cal3DS2 calib = new Cal3DS2(200, 200, 0, 200, 200, -0.2, 0.1);
            shared_ptr<PlanarProjectionFactor1> factor = PlanarProjectionFactor1.newPlanarProjectionFactor1(
                    Key.X(0), landmark, measured, offset, calib, model);
            Pose2 pose = new Pose2(dist.getAsDouble(), dist.getAsDouble(), dist.getAsDouble());
            Matrix H1 = new Matrix();
            factor.get().evaluateError(pose, H1);
            var expectedH1 = NumericalDerivative.<Vector2, Pose2>numericalDerivative11(
                    (p) -> factor.get().evaluateError(p, new Matrix()),
                    pose);
            assertEquals(expectedH1, H1);
        }
    }

    /*
     * *************************************************************************
     *
     *
     *
     *
     * @Test
     * void Solve() {
     * // Example localization
     * 
     * SharedNoiseModel pxModel = noiseModel::Diagonal::Sigmas(Vector2(1, 1));
     * // pose model is wide, so the solver finds the right answer.
     * SharedNoiseModel xNoise = noiseModel::Diagonal::Sigmas(Vector3(10, 10, 10));
     * 
     * // landmarks
     * Point3 l0 = new Point3(1, 0.1, 1);
     * Point3 l1 = new Point3(1, -0.1, 1);
     * 
     * // camera pixels
     * Point2 p0 = new Point2(180, 0);
     * Point2 p1 = new Point2(220, 0);
     * 
     * // body
     * Pose2 x0 = new Pose2(0, 0, 0);
     * 
     * // camera z looking at +x with (xy) antiparallel to (yz)
     * Pose3 c0 = new Pose3(
     * new Rot3(0, 0, 1, //
     * -1, 0, 0, //
     * 0, -1, 0), //
     * Vector3(0, 0, 0));
     * Cal3DS2 calib(200, 200, 0, 200, 200, 0, 0);
     * 
     * NonlinearFactorGraph graph;
     * graph.add(PlanarProjectionFactor1(X(0), l0, p0, c0, calib, pxModel));
     * graph.add(PlanarProjectionFactor1(X(0), l1, p1, c0, calib, pxModel));
     * graph.add(PriorFactor<Pose2>(X(0), x0, xNoise));
     * 
     * Values initialEstimate;
     * initialEstimate.insert(X(0), x0);
     * 
     * // run the optimizer
     * LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
     * Values result = optimizer.optimize();
     * 
     * // verify that the optimizer found the right pose.
     * CHECK(assert_equal(x0, result.at<Pose2>(X(0)), 2e-3));
     * 
     * // covariance
     * Marginals marginals(graph, result);
     * Matrix cov = marginals.marginalCovariance(X(0));
     * CHECK(assert_equal((Matrix33() << //
     * 0.000012, 0.000000, 0.000000, //
     * 0.000000, 0.001287, -.001262, //
     * 0.000000, -.001262, 0.001250).finished(), cov, 3e-6));
     * 
     * // pose stddev
     * Vector3 sigma = cov.diagonal().cwiseSqrt();
     * CHECK(assert_equal((Vector3() << //
     * 0.0035,
     * 0.0359,
     * 0.0354
     * ).finished(), sigma, 1e-4));
     * 
     * }
     * 
     * @Test
     * void Error3_1() {
     * // Example: center projection and Jacobian
     * Point3 landmark = new Point3(1, 0, 0);
     * Point2 measured = new Point2(200, 200);
     * SharedNoiseModel model = SharedNoiseModel.Sigmas(Vector2(1, 1));
     * PlanarProjectionFactor3 factor =
     * PlanarProjectionFactor3.newPlanarProjectionFactor3(X(0), C(0), K(0),
     * landmark, measured, model);
     * Pose2 pose = new Pose2(0, 0, 0);
     * Pose3 offset = new Pose3(
     * Rot3(0, 0, 1,//
     * -1, 0, 0, //
     * 0, -1, 0),
     * Vector3(0, 0, 0)
     * );
     * Cal3DS2 calib = new Cal3DS2(200, 200, 0, 200, 200, 0, 0);
     * Matrix H1;
     * Matrix H2;
     * Matrix H3;
     * CHECK(assert_equal(Vector2(0, 0), factor.evaluateError(pose, offset, calib,
     * H1, H2, H3), 1e-6));
     * CHECK(assert_equal((Matrix23() <<//
     * 0, 200, 200,//
     * 0, 0, 0).finished(), H1, 1e-6));
     * CHECK(assert_equal((Matrix26() <<//
     * 0, -200, 0, -200, 0, 0,//
     * 200, -0, 0, 0, -200, 0).finished(), H2, 1e-6));
     * CHECK(assert_equal((Matrix29() <<//
     * 0, 0, 0, 1, 0, 0, 0, 0, 0,//
     * 0, 0, 0, 0, 1, 0, 0, 0, 0).finished(), H3, 1e-6));
     * }
     * 
     * @Test
     * void Error3_2() {
     * Point3 landmark = new Point3(1, 1, 1);
     * Point2 measured = new Point2(0, 0);
     * SharedNoiseModel model = SharedNoiseModel.Sigmas(Vector2(1, 1));
     * PlanarProjectionFactor3 factor(X(0), C(0), K(0), landmark, measured, model);
     * Pose2 pose(0, 0, 0);
     * Pose3 offset(
     * Rot3(0, 0, 1,//
     * -1, 0, 0, //
     * 0, -1, 0),
     * Vector3(0, 0, 0)
     * );
     * Cal3DS2 calib(200, 200, 0, 200, 200, 0, 0);
     * Matrix H1;
     * Matrix H2;
     * Matrix H3;
     * gtsam::Vector actual = factor.evaluateError(pose, offset, calib, H1, H2, H3);
     * CHECK(assert_equal(Vector2(0, 0), actual));
     * CHECK(assert_equal((Matrix23() <<//
     * -200, 200, 400,//
     * -200, 0, 200).finished(), H1, 1e-6));
     * CHECK(assert_equal((Matrix26() <<//
     * 200, -400, -200, -200, 0, -200,//
     * 400, -200, 200, 0, -200, -200).finished(), H2, 1e-6));
     * CHECK(assert_equal((Matrix29() <<//
     * -1, 0, -1, 1, 0, -400, -800, 400, 800,//
     * 0, -1, 0, 0, 1, -400, -800, 800, 400).finished(), H3, 1e-6));
     * }
     * 
     * @Test
     * void Error3_3() {
     * Point3 landmark = new Point3(1, 1, 1);
     * Point2 measured = new Point2(0, 0);
     * SharedNoiseModel model = SharedNoiseModel.Sigmas(Vector2(1, 1));
     * PlanarProjectionFactor3 factor(X(0), C(0), K(0), landmark, measured, model);
     * Pose2 pose(0, 0, 0);
     * Pose3 offset(
     * Rot3(0, 0, 1,//
     * -1, 0, 0, //
     * 0, -1, 0),
     * Vector3(0, 0, 0)
     * );
     * Cal3DS2 calib(200, 200, 0, 200, 200, -0.2, 0.1);
     * Matrix H1;
     * Matrix H2;
     * Matrix H3;
     * CHECK(assert_equal(Vector2(0, 0), factor.evaluateError(pose, offset, calib,
     * H1, H2, H3), 1e-6));
     * CHECK(assert_equal((Matrix23() <<//
     * -360, 280, 640,//
     * -360, 80, 440).finished(), H1, 1e-6));
     * CHECK(assert_equal((Matrix26() <<//
     * 440, -640, -200, -280, -80, -360,//
     * 640, -440, 200, -80, -280, -360).finished(), H2, 1e-6));
     * CHECK(assert_equal((Matrix29() <<//
     * -1, 0, -1, 1, 0, -400, -800, 400, 800,//
     * 0, -1, 0, 0, 1, -400, -800, 800, 400).finished(), H3, 1e-6));
     * }
     * 
     * @Test
     * void Jacobian3() {
     * // Verify Jacobians with numeric derivative
     * 
     * std::default_random_engine rng(42);
     * std::uniform_real_distribution<double> dist(-0.3, 0.3);
     * SharedNoiseModel model = noiseModel::Diagonal::Sigmas(Vector2(1, 1));
     * // center of the random camera poses
     * Pose3 centerOffset(
     * Rot3(0, 0, 1,//
     * -1, 0, 0, //
     * 0, -1, 0),
     * Vector3(0, 0, 0)
     * );
     * 
     * for (int i = 0; i < 1000; ++i) {
     * Point3 landmark(2 + dist(rng), dist(rng), dist(rng));
     * Point2 measured(200 + 100 * dist(rng), 200 + 100 * dist(rng));
     * Pose3 offset = centerOffset.compose(
     * Pose3(
     * Rot3::Ypr(dist(rng), dist(rng), dist(rng)),
     * Point3(dist(rng), dist(rng), dist(rng))));
     * Cal3DS2 calib(200, 200, 0, 200, 200, -0.2, 0.1);
     * 
     * PlanarProjectionFactor3 factor(X(0), C(0), K(0), landmark, measured, model);
     * 
     * Pose2 pose(dist(rng), dist(rng), dist(rng));
     * 
     * // actual H
     * Matrix H1, H2, H3;
     * factor.evaluateError(pose, offset, calib, H1, H2, H3);
     * 
     * Matrix expectedH1 = numericalDerivative31<Vector, Pose2, Pose3, Cal3DS2>(
     * [&factor](const Pose2& p, const Pose3& o, const Cal3DS2& c) {
     * return factor.evaluateError(p, o, c, {}, {}, {});},
     * pose, offset, calib);
     * Matrix expectedH2 = numericalDerivative32<Vector, Pose2, Pose3, Cal3DS2>(
     * [&factor](const Pose2& p, const Pose3& o, const Cal3DS2& c) {
     * return factor.evaluateError(p, o, c, {}, {}, {});},
     * pose, offset, calib);
     * Matrix expectedH3 = numericalDerivative33<Vector, Pose2, Pose3, Cal3DS2>(
     * [&factor](const Pose2& p, const Pose3& o, const Cal3DS2& c) {
     * return factor.evaluateError(p, o, c, {}, {}, {});},
     * pose, offset, calib);
     * CHECK(assert_equal(expectedH1, H1, 5e-6));
     * CHECK(assert_equal(expectedH2, H2, 5e-6));
     * CHECK(assert_equal(expectedH3, H3, 5e-6));
     * }
     * }
     * 
     * @Test
     * void SolveOffset() {
     * // Example localization
     * SharedNoiseModel pxModel = noiseModel::Diagonal::Sigmas(Vector2(1, 1));
     * SharedNoiseModel xNoise = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01,
     * 0.01));
     * // offset model is wide, so the solver finds the right answer.
     * SharedNoiseModel cNoise = noiseModel::Diagonal::Sigmas(Vector6(10, 10, 10,
     * 10, 10, 10));
     * SharedNoiseModel kNoise = noiseModel::Diagonal::Sigmas(Vector9(0.001, 0.001,
     * 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001));
     * 
     * // landmarks
     * Point3 l0 = new Point3(1, 0, 1);
     * Point3 l1 = new Point3(1, 0, 0);
     * Point3 l2 = new Point3(1, -1, 1);
     * Point3 l3 = new Point3(2, 2, 1);
     * 
     * // camera pixels
     * Point2 p0 = new Point2(200, 200);
     * Point2 p1 = new Point2(200, 400);
     * Point2 p2 = new Point2(400, 200);
     * Point2 p3 = new Point2(0, 200);
     * 
     * // body
     * Pose2 x0(0, 0, 0);
     * 
     * // camera z looking at +x with (xy) antiparallel to (yz)
     * Pose3 c0(
     * Rot3(0, 0, 1, //
     * -1, 0, 0, //
     * 0, -1, 0), //
     * Vector3(0, 0, 1)); // note z offset
     * Cal3DS2 calib(200, 200, 0, 200, 200, 0, 0);
     * 
     * NonlinearFactorGraph graph;
     * graph.add(PlanarProjectionFactor3(X(0), C(0), K(0), l0, p0, pxModel));
     * graph.add(PlanarProjectionFactor3(X(0), C(0), K(0), l1, p1, pxModel));
     * graph.add(PlanarProjectionFactor3(X(0), C(0), K(0), l2, p2, pxModel));
     * graph.add(PlanarProjectionFactor3(X(0), C(0), K(0), l3, p3, pxModel));
     * graph.add(PriorFactor<Pose2>(X(0), x0, xNoise));
     * graph.add(PriorFactor<Pose3>(C(0), c0, cNoise));
     * graph.add(PriorFactor<Cal3DS2>(K(0), calib, kNoise));
     * 
     * Values initialEstimate;
     * initialEstimate.insert(X(0), x0);
     * initialEstimate.insert(C(0), c0);
     * initialEstimate.insert(K(0), calib);
     * 
     * // run the optimizer
     * LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
     * Values result = optimizer.optimize();
     * 
     * // verify that the optimizer found the right pose.
     * CHECK(assert_equal(x0, result.at<Pose2>(X(0)), 2e-3));
     * 
     * // verify the camera is pointing at +x
     * Pose3 cc0 = result.at<Pose3>(C(0));
     * CHECK(assert_equal(c0, cc0, 5e-3));
     * 
     * // verify the calibration
     * CHECK(assert_equal(calib, result.at<Cal3DS2>(K(0)), 2e-3));
     * 
     * Marginals marginals(graph, result);
     * Matrix x0cov = marginals.marginalCovariance(X(0));
     * 
     * // narrow prior => ~zero cov
     * CHECK(assert_equal(Matrix33::Zero(), x0cov, 1e-4));
     * 
     * Matrix c0cov = marginals.marginalCovariance(C(0));
     * 
     * // invert the camera offset to get covariance in body coordinates
     * Matrix66 HcTb = cc0.inverse().AdjointMap().inverse();
     * Matrix c0cov2 = HcTb * c0cov * HcTb.transpose();
     * 
     * // camera-frame stddev
     * Vector6 c0sigma = c0cov.diagonal().cwiseSqrt();
     * CHECK(assert_equal((Vector6() << //
     * 0.009,
     * 0.011,
     * 0.004,
     * 0.012,
     * 0.012,
     * 0.011
     * ).finished(), c0sigma, 1e-3));
     * 
     * // body frame stddev
     * Vector6 bTcSigma = c0cov2.diagonal().cwiseSqrt();
     * CHECK(assert_equal((Vector6() << //
     * 0.004,
     * 0.009,
     * 0.011,
     * 0.012,
     * 0.012,
     * 0.012
     * ).finished(), bTcSigma, 1e-3));
     * 
     * // narrow prior => ~zero cov
     * CHECK(assert_equal(Matrix99::Zero(), marginals.marginalCovariance(K(0)),
     * 3e-3));
     * }
     */

}
