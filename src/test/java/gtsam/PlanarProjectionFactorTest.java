package gtsam;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import java.util.function.DoubleSupplier;

import org.junit.jupiter.api.Test;

import gtsam.NumericalDerivative.ThrowingFunction;
import gtsam.NumericalDerivative.ThrowingFunction3;

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

        assertTrue(err.equals(new double[] { 0.0, 0.0 }, 1e-6));

        assertTrue(H.equals(new double[][] { //
                { 0, 200, 200 }, //
                { 0, 0, 0 } //
        }, 1e-6));
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

        assertTrue(err.equals(new double[] { 0.0, 0.0 }, 1e-6));

        assertTrue(H.equals(new double[][] { //
                { -200, 200, 400 }, //
                { -200, 0, 200 } //
        }, 1e-6));
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

        assertTrue(err.equals(new double[] { 0.0, 0.0 }, 1e-6));

        assertTrue(H.equals(new double[][] { //
                { -360, 280, 640 }, //
                { -360, 80, 440 } //
        }, 1e-6));
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
            ThrowingFunction<Pose2, Vector2> h = (p) -> factor.get().evaluateError(p, new Matrix());
            var expectedH1 = NumericalDerivative.<Vector2, Vector2, Pose2, Vector3>numericalDerivative11(
                    h, pose, 1e-5);
            assertEquals(expectedH1, H1);
        }
    }

    /**
     * Example localization.
     */
    @Test
    void Solve() throws Throwable {
        SharedNoiseModel pxModel = SharedNoiseModel.Sigmas(new Vector2(1, 1));
        // pose model is wide, so the solver finds the right answer.
        SharedNoiseModel xNoise = SharedNoiseModel.Sigmas(new Vector3(10, 10, 10));

        // landmarks
        Point3 l0 = new Point3(1, 0.1, 1);
        Point3 l1 = new Point3(1, -0.1, 1);

        // camera pixels
        Point2 p0 = new Point2(180, 0);
        Point2 p1 = new Point2(220, 0);

        // body
        Pose2 x0 = new Pose2(0, 0, 0);

        // camera z looking at +x with (xy) antiparallel to (yz)
        Pose3 c0 = new Pose3(
                new Rot3(0, 0, 1, //
                        -1, 0, 0, //
                        0, -1, 0), //
                new Point3(0, 0, 0));
        Cal3DS2 calib = new Cal3DS2(200, 200, 0, 200, 200, 0, 0);

        NonlinearFactorGraph graph = new NonlinearFactorGraph();
        graph.add(PlanarProjectionFactor1.newPlanarProjectionFactor1(Key.X(0), l0, p0, c0, calib, pxModel));
        graph.add(PlanarProjectionFactor1.newPlanarProjectionFactor1(Key.X(0), l1, p1, c0, calib, pxModel));
        graph.add(PriorFactor.PriorFactorPose2(Key.X(0), x0, xNoise));

        Values initialEstimate = new Values();
        initialEstimate.insert(Key.X(0), x0);

        // run the optimizer
        LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer(graph, initialEstimate);
        Values result = optimizer.optimize();

        // verify that the optimizer found the right pose.
        Pose2 xhat = result.atPose2(Key.X(0));
        assertEquals(0, xhat.x(), 2e-3);
        assertEquals(0, xhat.y(), 2e-3);
        assertEquals(0, xhat.theta(), 2e-3);

        // covariance
        Marginals marginals = new Marginals(graph, result);
        Matrix cov = marginals.marginalCovariance(Key.X(0));

        assertTrue(cov.equals(new double[][] { //
                { 0.000012, 0.000000, 0.000000 }, //
                { 0.000000, 0.001287, -0.001262 }, //
                { 0.000000, -0.001262, 0.001250 } //
        }, 1e-6));

        // pose stddev
        Vector sigma = cov.diagonal_cwiseSqrt();

        assertTrue(sigma.equals(new double[] { 0.0035, 0.0359, 0.0354 }, 1e-4));

    }

    /**
     * Example: center projection and Jacobian
     */
    @Test
    void Error3_1() throws Throwable {
        Point3 landmark = new Point3(1, 0, 0);
        Point2 measured = new Point2(200, 200);
        SharedNoiseModel model = SharedNoiseModel.Sigmas(new Vector2(1, 1));
        shared_ptr<PlanarProjectionFactor3> factor = PlanarProjectionFactor3.newPlanarProjectionFactor3(
                Key.X(0), Key.C(0), Key.K(0), landmark, measured, model);
        Pose2 pose = new Pose2(0, 0, 0);
        Pose3 offset = new Pose3(
                new Rot3(0, 0, 1, //
                        -1, 0, 0, //
                        0, -1, 0),
                new Point3(0, 0, 0));
        Cal3DS2 calib = new Cal3DS2(200, 200, 0, 200, 200, 0, 0);
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        Matrix H3 = new Matrix();
        Vector err = factor.get().evaluateError(pose, offset, calib, H1, H2, H3);

        assertTrue(err.equals(new double[] { 0, 0 }, 1e-6));
        assertTrue(H1.equals(new double[][] { //
                { 0, 200, 200 }, //
                { 0, 0, 0 } //
        }, 1e-6));

        assertTrue(H2.equals(new double[][] { //
                { 0, -200, 0, -200, 0, 0 }, //
                { 200, 0, 0, 0, -200, 0 } //
        }, 1e-6));

        assertTrue(H3.equals(new double[][] { //
                { 0, 0, 0, 1, 0, 0, 0, 0, 0 }, //
                { 0, 0, 0, 0, 1, 0, 0, 0, 0 } //
        }, 1e-6));
    }

    @Test
    void Error3_2() throws Throwable {
        Point3 landmark = new Point3(1, 1, 1);
        Point2 measured = new Point2(0, 0);
        SharedNoiseModel model = SharedNoiseModel.Sigmas(new Vector2(1, 1));
        shared_ptr<PlanarProjectionFactor3> factor = PlanarProjectionFactor3.newPlanarProjectionFactor3(
                Key.X(0), Key.C(0), Key.K(0), landmark, measured, model);
        Pose2 pose = new Pose2(0, 0, 0);
        Pose3 offset = new Pose3(
                new Rot3(0, 0, 1, //
                        -1, 0, 0, //
                        0, -1, 0),
                new Point3(0, 0, 0));
        Cal3DS2 calib = new Cal3DS2(200, 200, 0, 200, 200, 0, 0);
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        Matrix H3 = new Matrix();
        Vector actual = factor.get().evaluateError(pose, offset, calib, H1, H2, H3);

        assertTrue(actual.equals(new double[] { 0.0, 0.0 }, 1e-6));

        assertTrue(H1.equals(new double[][] { //
                { -200, 200, 400 }, //
                { -200, 0, 200 } //
        }, 1e-6));

        assertTrue(H2.equals(new double[][] { //
                { 200, -400, -200, -200, 0, -200 }, //
                { 400, -200, 200, 0, -200, -200 } //
        }, 1e-6));

        assertTrue(H3.equals(new double[][] { //
                { -1, 0, -1, 1, 0, -400, -800, 400, 800 }, //
                { 0, -1, 0, 0, 1, -400, -800, 800, 400 } //
        }, 1e-6));
    }

    @Test
    void Error3_3() throws Throwable {
        Point3 landmark = new Point3(1, 1, 1);
        Point2 measured = new Point2(0, 0);
        SharedNoiseModel model = SharedNoiseModel.Sigmas(new Vector2(1, 1));
        shared_ptr<PlanarProjectionFactor3> factor = PlanarProjectionFactor3.newPlanarProjectionFactor3(
                Key.X(0), Key.C(0), Key.K(0), landmark, measured, model);
        Pose2 pose = new Pose2(0, 0, 0);
        Pose3 offset = new Pose3(
                new Rot3(0, 0, 1, //
                        -1, 0, 0, //
                        0, -1, 0),
                new Point3(0, 0, 0));
        Cal3DS2 calib = new Cal3DS2(200, 200, 0, 200, 200, -0.2, 0.1);
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        Matrix H3 = new Matrix();
        Vector err = factor.get().evaluateError(pose, offset, calib,
                H1, H2, H3);
        assertTrue(err.equals(new double[] { 0, 0 }, 1e-6));
        assertTrue(H1.equals(new double[][] {
                { -360, 280, 640 },
                { -360, 80, 440 } }, 1e-6));
        assertTrue(H2.equals(new double[][] {
                { 440, -640, -200, -280, -80, -360 },
                { 640, -440, 200, -80, -280, -360 } }, 1e-6));
        assertTrue(H3.equals(new double[][] {
                { -1, 0, -1, 1, 0, -400, -800, 400, 800 },
                { 0, -1, 0, 0, 1, -400, -800, 800, 400 } }, 1e-6));
    }

    /**
     * Verify Jacobians with numeric derivative
     */
    @Test
    void Jacobian3() throws Throwable {
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

            shared_ptr<PlanarProjectionFactor3> factor = PlanarProjectionFactor3.newPlanarProjectionFactor3(
                    Key.X(0), Key.C(0), Key.K(0), landmark, measured, model);

            Pose2 pose = new Pose2(dist.getAsDouble(), dist.getAsDouble(), dist.getAsDouble());

            // actual H
            Matrix H1 = new Matrix();
            Matrix H2 = new Matrix();
            Matrix H3 = new Matrix();
            factor.get().evaluateError(pose, offset, calib, H1, H2, H3);

            // Ugh. NoiseModelFactorN (the superclass of most of the factors we use) is a
            // template with the "error" type filed in as a dynamic-dimension "Vector" type,
            // not a specific length.
            ThrowingFunction3<Pose2, Pose3, Cal3DS2, Vector> h = (Pose2 p, Pose3 o, Cal3DS2 c) -> factor.get()
                    .evaluateError(
                            p, o, c, new Matrix(), new Matrix(), new Matrix());

            Matrix expectedH1 = NumericalDerivative
                    .<Vector, Vector, //
                            Pose2, Vector3, //
                            Pose3, Vector6, //
                            Cal3DS2, Vector9>numericalDerivative31(
                                    h, pose, offset, calib, 1e-5);

            Matrix expectedH2 = NumericalDerivative
                    .<Vector, Vector, //
                            Pose2, Vector3, //
                            Pose3, Vector6, //
                            Cal3DS2, Vector9>numericalDerivative32(
                                    h, pose, offset, calib, 1e-5);

            Matrix expectedH3 = NumericalDerivative
                    .<Vector, Vector, //
                            Pose2, Vector3, //
                            Pose3, Vector6, //
                            Cal3DS2, Vector9>numericalDerivative33(
                                    h, pose, offset, calib, 1e-5);

            assertEquals(expectedH1, H1);
            assertEquals(expectedH2, H2);
            assertEquals(expectedH3, H3);
        }
    }

    /**
     * Example localization.
     */
    @Test
    void SolveOffset() throws Throwable {
        SharedNoiseModel pxModel = SharedNoiseModel.Sigmas(new Vector2(1, 1));
        SharedNoiseModel xNoise = SharedNoiseModel.Sigmas(new Vector3(0.01, 0.01,
                0.01));
        // offset model is wide, so the solver finds the right answer.
        SharedNoiseModel cNoise = SharedNoiseModel.Sigmas(
                new Vector(new double[] { 10, 10, 10, 10, 10, 10 }));
        SharedNoiseModel kNoise = SharedNoiseModel.Sigmas(
                new Vector(new double[] { 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 }));

        // landmarks
        Point3 l0 = new Point3(1, 0, 1);
        Point3 l1 = new Point3(1, 0, 0);
        Point3 l2 = new Point3(1, -1, 1);
        Point3 l3 = new Point3(2, 2, 1);

        // camera pixels
        Point2 p0 = new Point2(200, 200);
        Point2 p1 = new Point2(200, 400);
        Point2 p2 = new Point2(400, 200);
        Point2 p3 = new Point2(0, 200);

        // body
        Pose2 x0 = new Pose2(0, 0, 0);

        // camera z looking at +x with (xy) antiparallel to (yz)
        Pose3 c0 = new Pose3(
                new Rot3(0, 0, 1, //
                        -1, 0, 0, //
                        0, -1, 0), //
                new Point3(0, 0, 1)); // note z offset
        Cal3DS2 calib = new Cal3DS2(200, 200, 0, 200, 200, 0, 0);

        NonlinearFactorGraph graph = new NonlinearFactorGraph();
        graph.add(PlanarProjectionFactor3.newPlanarProjectionFactor3(Key.X(0), Key.C(0), Key.K(0), l0, p0, pxModel));
        graph.add(PlanarProjectionFactor3.newPlanarProjectionFactor3(Key.X(0), Key.C(0), Key.K(0), l1, p1, pxModel));
        graph.add(PlanarProjectionFactor3.newPlanarProjectionFactor3(Key.X(0), Key.C(0), Key.K(0), l2, p2, pxModel));
        graph.add(PlanarProjectionFactor3.newPlanarProjectionFactor3(Key.X(0), Key.C(0), Key.K(0), l3, p3, pxModel));
        graph.add(PriorFactor.PriorFactorPose2(Key.X(0), x0, xNoise));
        graph.add(PriorFactor.PriorFactorPose3(Key.C(0), c0, cNoise));
        graph.add(PriorFactor.PriorFactorCal3DS2(Key.K(0), calib, kNoise));

        Values initialEstimate = new Values();
        initialEstimate.insert(Key.X(0), x0);
        initialEstimate.insert(Key.C(0), c0);
        initialEstimate.insert(Key.K(0), calib);

        // run the optimizer
        LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer(graph, initialEstimate);
        Values result = optimizer.optimize();

        // verify that the optimizer found the right pose.
        assertTrue(x0.equals(result.atPose2(Key.X(0)), 2e-3));

        // verify the camera is pointing at +x
        Pose3 cc0 = result.atPose3(Key.C(0));
        assertTrue(c0.equals(cc0, 5e-3));

        // verify the calibration
        assertTrue(calib.equals(result.atCal3DS2(Key.K(0)), 2e-3));

        Marginals marginals = new Marginals(graph, result);
        Matrix x0cov = marginals.marginalCovariance(Key.X(0));

        // narrow prior => ~zero cov
        assertTrue(x0cov.equals(new double[][] {
                { 0, 0, 0 }, //
                { 0, 0, 0 }, //
                { 0, 0, 0 }//
        }, 1e-4));

        Matrix c0cov = marginals.marginalCovariance(Key.C(0));

        // invert the camera offset to get covariance in body coordinates
        Matrix HcTb = cc0.inverse().AdjointMap().inverse();
        Matrix c0cov2 = HcTb.compose(c0cov).compose(HcTb.transpose());

        // camera-frame stddev
        Vector c0sigma = c0cov.diagonal_cwiseSqrt();
        assertTrue(c0sigma.equals(new double[] {
                0.009, 0.011, 0.004, 0.012, 0.012, 0.011
        }, 1e-3));

        // body frame stddev
        Vector bTcSigma = c0cov2.diagonal_cwiseSqrt();
        assertTrue(bTcSigma.equals(new double[] {
                0.004, 0.009, 0.011, 0.012, 0.012, 0.012
        }, 1e-3));

        // narrow prior => ~zero cov
        Matrix k0cov = marginals.marginalCovariance(Key.K(0));
        assertTrue(k0cov.equals(new double[][] {
                { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        }, 3e-3));
    }

}
