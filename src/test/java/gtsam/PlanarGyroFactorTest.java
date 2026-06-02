package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import gtsam.NumericalDerivative.ThrowingFunction;
import gtsam.NumericalDerivative.ThrowingFunction2;
import gtsam.NumericalDerivative.ThrowingFunction3;
import gtsam.PlanarGyroFactor.PlanarGyroBiasFactor;
import gtsam.PlanarGyroFactor.PlanarGyroParams;
import gtsam.noiseModel.Diagonal;

/**
 * See gtsam/navigation/tests/testPlanarGyroFactor.cpp.
 */
public class PlanarGyroFactorTest {

    @Test
    void testfromRate() throws Throwable {
        double arwSigma = 1.0;
        double omega = 0.1;
        double dt = 0.5;
        double biasInstabilitySigma = 3e-4;

        shared_ptr<PlanarGyroParams> p = PlanarGyroParams.makeSharedPlanarGyroParams(//
                arwSigma, biasInstabilitySigma);

        shared_ptr<PlanarGyroFactor> x = PlanarGyroFactor.FromRate(//
                Key.P(0), Key.P(1), Key.B(0), p, omega, dt);

        // // Check the effect of bias.
        double bias = 0.05;
        Matrix H = new Matrix();
        Rot2 corrected = x.get().deltaR(bias, H);
        assertTrue(assert_equal(0.025, corrected.theta(), 1e-9));
        assertTrue(assert_equal(-0.5, H.at(0, 0), 1e-9));

        // Numeric derivative matches.
        ThrowingFunction<Vector1, Rot2> f = (b) -> x.get().deltaR(b.at(0), new Matrix());
        Matrix numericH = NumericalDerivative.<Rot2, Vector1, //
                Vector1, Vector1>numericalDerivative11(f, new Vector1(bias), 1e-4);
        assertTrue(assert_equal(-0.5, numericH.at(0, 0), 1e-9));
    }

    @Test
    void testfromRotation() throws Throwable {
        double arwSigma = 1.0;
        Rot2 dr = new Rot2(0.05);
        double dt = 0.5;
        double biasInstabilitySigma = 3e-4;

        shared_ptr<PlanarGyroParams> p = PlanarGyroParams.makeSharedPlanarGyroParams(//
                arwSigma, biasInstabilitySigma);

        shared_ptr<PlanarGyroFactor> x = PlanarGyroFactor.FromRotation(//
                Key.P(0), Key.P(1), Key.B(0), p, dr, dt);
        double bias = 0.05;
        Matrix H = new Matrix();
        Rot2 corrected = x.get().deltaR(bias, H);
        assertTrue(assert_equal(0.025, corrected.theta(), 1e-9));
        assertTrue(assert_equal(-0.5, H.at(0, 0), 1e-9));
    }

    @Test
    void testvariance() throws Throwable {
        double arwSigma = 1.0;
        double dt = 0.5;
        double biasInstabilitySigma = 3e-4;
        shared_ptr<PlanarGyroParams> p = PlanarGyroParams.makeSharedPlanarGyroParams(//
                arwSigma, biasInstabilitySigma);
        // 1.0 * 0.5 = 0.5
        assertTrue(assert_equal(0.707107, p.get().arwSigma(dt), 1e-6));
    }

    @Test
    void testpredict() throws Throwable {
        double arwSigma = 1.0;
        double omega = 0.1;
        double dt = 0.5;
        double biasInstabilitySigma = 3e-4;

        shared_ptr<PlanarGyroParams> p = PlanarGyroParams.makeSharedPlanarGyroParams(//
                arwSigma, biasInstabilitySigma);

        shared_ptr<PlanarGyroFactor> x = PlanarGyroFactor.FromRate(//
                Key.P(0), Key.P(1), Key.B(0), p, omega, dt);

        // Check prediction.
        Rot2 Ri = Rot2.fromAngle(1);
        double bias = 0.05;
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        Rot2 predictedRj = x.get().predict(Ri, bias, H1, H2);

        // 1 + 0.025 = 1.025
        assertTrue(assert_equal(1.025, predictedRj.theta(), 1e-9));
        // Ri adds to prediction.
        assertTrue(assert_equal(1.0, H1.at(0, 0), 1e-9));
        // Bias * dt subtracts from prediction.
        assertTrue(assert_equal(-0.5, H2.at(0, 0), 1e-9));

        // Numeric derivative matches.
        ThrowingFunction2<Rot2, Vector1, Rot2> f = (r, b) -> x.get().predict(//
                r, b.at(0), new Matrix(), new Matrix());
        Matrix nH1 = NumericalDerivative.<Rot2, Vector1, Rot2, Vector1, Vector1, Vector1>numericalDerivative21(//
                f, Ri, new Vector1(bias), 1e-4);
        Matrix nH2 = NumericalDerivative.<Rot2, Vector1, Rot2, Vector1, Vector1, Vector1>numericalDerivative22(f, Ri,
                new Vector1(bias), 1e-4);
        assertTrue(assert_equal(1.0, nH1.at(0, 0), 1e-9));
        assertTrue(assert_equal(-0.5, nH2.at(0, 0), 1e-9));
    }

    @Test
    void testcomputeError() throws Throwable {
        double arwSigma = 1.0;
        double omega = 0.1;
        double dt = 0.5;
        double biasInstabilitySigma = 3e-4;

        shared_ptr<PlanarGyroParams> p = PlanarGyroParams.makeSharedPlanarGyroParams(//
                arwSigma, biasInstabilitySigma);

        shared_ptr<PlanarGyroFactor> x = PlanarGyroFactor.FromRate(//
                Key.P(0), Key.P(1), Key.B(0), p, omega, dt);

        // Check error.
        Rot2 Ri = Rot2.fromAngle(1);
        Rot2 Rj = Rot2.fromAngle(2);
        double bias = 0.05;
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        Matrix H3 = new Matrix();
        double err = x.get().computeError(Ri, Rj, bias, H1, H2, H3);

        // estimate - prediction = 2 - 1.025 = -0.975
        assertTrue(assert_equal(-0.975, err, 1e-9));
        // Ri up => error up (less negative)
        assertTrue(assert_equal(1.0, H1.at(0, 0), 1e-9));
        // Rj up -> error down (more negative)
        assertTrue(assert_equal(-1.0, H2.at(0, 0), 1e-9));
        // bias up -> error down (more negative), scaled by dt
        assertTrue(assert_equal(-0.5, H3.at(0, 0), 1e-9));

        // Numeric derivative matches
        ThrowingFunction3<Rot2, Rot2, Vector1, Vector1> f = (r1, r2, b) -> new Vector1(x.get().computeError(//
                r1, r2, b.at(0), new Matrix(), new Matrix(), new Matrix()));
        Matrix nH1 = NumericalDerivative.<//
                Vector1, Vector1, //
                Rot2, Vector1, //
                Rot2, Vector1, //
                Vector1, Vector1>numericalDerivative31(f, Ri, Rj,
                        new Vector1(bias), 1e-4);
        Matrix nH2 = NumericalDerivative.<//
                Vector1, Vector1, //
                Rot2, Vector1, //
                Rot2, Vector1, //
                Vector1, Vector1>numericalDerivative32(f, Ri, Rj, new Vector1(bias), 1e-4);
        Matrix nH3 = NumericalDerivative.<//
                Vector1, Vector1, //
                Rot2, Vector1, //
                Rot2, Vector1, //
                Vector1, Vector1>numericalDerivative33(f, Ri, Rj, new Vector1(bias), 1e-4);
        assertTrue(assert_equal(1.0, nH1.at(0, 0), 1e-9));
        assertTrue(assert_equal(-1.0, nH2.at(0, 0), 1e-9));
        assertTrue(assert_equal(-0.5, nH3.at(0, 0), 1e-9));
    }

    @Test
    void testevaluateError() throws Throwable {
        double arwSigma = 0.1;
        double trueOmega = Math.PI / 10.0;
        double B1 = 0.3;
        // // Measurement includes bias.
        double measuredOmega = trueOmega + B1;
        double deltaT = 1.0;
        double biasInstabilitySigma = 3e-4;

        shared_ptr<PlanarGyroParams> p = PlanarGyroParams.makeSharedPlanarGyroParams(//
                arwSigma, biasInstabilitySigma);

        shared_ptr<PlanarGyroFactor> factor = PlanarGyroFactor.FromRate(//
                Key.P(1), Key.P(2), Key.B(1), p, measuredOmega, deltaT);

        double initialRotation = Math.PI / 4.0;
        Pose2 P1 = new Pose2(0.0, 0.0, initialRotation);
        double error = 0.1;
        Pose2 P2 = new Pose2(0.0, 0.0, initialRotation + trueOmega * deltaT - error);

        assertTrue(assert_equal(new Vector(new double[] { 0, 0, error }), factor.get().evaluateError(//
                P1, P2, B1, new Matrix(), new Matrix(), new Matrix()), 1e-6));
    }

    @Test
    void testoptimize() throws Throwable {
        // using noiseModel::Diagonal;

        NonlinearFactorGraph graph = new NonlinearFactorGraph();

        // Starting pose is known.
        graph.add(PriorFactor.PriorFactorPose2(Key.P(0), new Pose2(),
                Diagonal.Sigmas(new Vector3(0.001, 0.001, 0.001))));

        // BetweenFactors that simulate odometry.
        Pose2 p0 = new Pose2(0, 0, 0);
        Pose2 p1 = new Pose2(0, 0, 0.1);
        Pose2 p2 = new Pose2(0.1, 0, 0.2);
        Pose2 p3 = new Pose2(0.2, 0, 0.3);
        Pose2 p4 = new Pose2(0.3, 0, 0.4);
        // Add error in the "between" rotation, so the gyro factor can fix it.
        Pose2 pErr = new Pose2(0, 0, 0.1);
        // When motionless, the rotation is known.
        // This is how we learn the bias.
        shared_ptr<Diagonal> lowRotationNoise = Diagonal.Sigmas(new Vector3(1e-3, 1e-3, 1e-3));
        graph.add(BetweenFactorPose2.newBetweenFactorPose2(//
                Key.P(0), Key.P(1), p0.between(p1), lowRotationNoise));

        // When moving, rotation is much less certain.
        shared_ptr<Diagonal> highRotationNoise = Diagonal.Sigmas(new Vector3(1e-3, 1e-3, 1));
        graph.add(BetweenFactorPose2.newBetweenFactorPose2(//
                Key.P(1), Key.P(2), p1.between(p2).compose(pErr), highRotationNoise));
        graph.add(BetweenFactorPose2.newBetweenFactorPose2(//
                Key.P(2), Key.P(3), p2.between(p3).compose(pErr), highRotationNoise));
        graph.add(BetweenFactorPose2.newBetweenFactorPose2(//
                Key.P(3), Key.P(4), p3.between(p4).compose(pErr), highRotationNoise));

        // Bias priorr: very uncertain.
        graph.add(PriorFactor.PriorFactorDouble(Key.B(0), 1.0, Diagonal.Sigmas(new Vector1(1))));

        // // Gyro measurements affect rotation only.
        double trueOmega = 0.1;
        double bias = 1; // large !
        double measuredOmega = trueOmega + bias;
        double dt = 1.0;
        double arwSigma = 1e-4;
        double biasInstabilitySigma = 3e-4;

        // Bias evolution. Bias stability is an important parameter.
        shared_ptr<PlanarGyroParams> p = PlanarGyroParams.makeSharedPlanarGyroParams(//
                arwSigma, biasInstabilitySigma);
        graph.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(Key.B(0), Key.B(1), p));
        graph.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(Key.B(1), Key.B(2), p));
        graph.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(Key.B(2), Key.B(3), p));
        graph.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(Key.B(3), Key.B(4), p));

        graph.add(PlanarGyroFactor.FromRate(Key.P(0), Key.P(1), Key.B(0), p, measuredOmega, dt));
        graph.add(PlanarGyroFactor.FromRate(Key.P(1), Key.P(2), Key.B(1), p, measuredOmega, dt));
        graph.add(PlanarGyroFactor.FromRate(Key.P(2), Key.P(3), Key.B(2), p, measuredOmega, dt));
        graph.add(PlanarGyroFactor.FromRate(Key.P(3), Key.P(4), Key.B(3), p, measuredOmega, dt));

        // // Initial values should not matter.
        Values values = new Values();
        values.insert(Key.B(0), 0.0);
        values.insert(Key.B(1), 0.0);
        values.insert(Key.B(2), 0.0);
        values.insert(Key.B(3), 0.0);
        values.insert(Key.B(4), 0.0);
        values.insert(Key.P(0), new Pose2());
        values.insert(Key.P(1), new Pose2());
        values.insert(Key.P(2), new Pose2());
        values.insert(Key.P(3), new Pose2());
        values.insert(Key.P(4), new Pose2());

        LevenbergMarquardtParams params = new LevenbergMarquardtParams();
        LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer(graph, values, params);
        Values result = optimizer.optimize();

        // Rotation increments are what the more-certain gyro factor said, overriding
        // what the less-certain "between" factor said.
        assertTrue(assert_equal(new Pose2(0.0, 0.0, 0.0), result.atPose2(Key.P(0)), 1e-5));
        assertTrue(assert_equal(new Pose2(0.0, 0.0, 0.1), result.atPose2(Key.P(1)), 1e-5));
        assertTrue(assert_equal(new Pose2(0.1, 0.0, 0.2), result.atPose2(Key.P(2)), 1e-5));
        assertTrue(assert_equal(new Pose2(0.2, 0.0, 0.3), result.atPose2(Key.P(3)), 1e-5));
        assertTrue(assert_equal(new Pose2(0.3, 0.0, 0.4), result.atPose2(Key.P(4)), 1e-5));

        // Bias is correctly learned.
        assertTrue(assert_equal(1.0, result.atDouble(Key.B(0)), 1e-6));
        assertTrue(assert_equal(1.0, result.atDouble(Key.B(1)), 1e-6));
        assertTrue(assert_equal(1.0, result.atDouble(Key.B(2)), 1e-6));
        assertTrue(assert_equal(1.0, result.atDouble(Key.B(3)), 1e-6));
        assertTrue(assert_equal(1.0, result.atDouble(Key.B(4)), 1e-6));

        Marginals marginals = new Marginals(graph, result);

        // Look at std dev because it's not so tiny.
        assertTrue(assert_equal(
                new Vector3(0.001000, 0.001000, 0.001000),
                new Vector3(marginals.marginalCovariance(Key.P(0)).diagonal_cwiseSqrt()), 1e-6));
        assertTrue(assert_equal(
                new Vector3(0.001414, 0.001414, 0.001414),
                new Vector3(marginals.marginalCovariance(Key.P(1)).diagonal_cwiseSqrt()), 1e-6));
        assertTrue(assert_equal(
                new Vector3(0.001732, 0.001738, 0.002261),
                new Vector3(marginals.marginalCovariance(Key.P(2)).diagonal_cwiseSqrt()), 1e-6));
        assertTrue(assert_equal(
                new Vector3(0.002003, 0.002030, 0.003242),
                new Vector3(marginals.marginalCovariance(Key.P(3)).diagonal_cwiseSqrt()), 1e-6));
        assertTrue(assert_equal(
                new Vector3(0.002252, 0.002322, 0.004287),
                new Vector3(marginals.marginalCovariance(Key.P(4)).diagonal_cwiseSqrt()), 1e-6));

        // Bias variance is roughly constant.
        assertTrue(assert_equal(0.001005,
                Math.sqrt(marginals.marginalCovariance(Key.B(0)).at(0, 0)), 1e-6));
        assertTrue(assert_equal(0.001049,
                Math.sqrt(marginals.marginalCovariance(Key.B(1)).at(0, 0)), 1e-6));
        assertTrue(assert_equal(0.001091,
                Math.sqrt(marginals.marginalCovariance(Key.B(2)).at(0, 0)), 1e-6));
        assertTrue(assert_equal(0.001131,
                Math.sqrt(marginals.marginalCovariance(Key.B(3)).at(0, 0)), 1e-6));
        assertTrue(assert_equal(0.001170,
                Math.sqrt(marginals.marginalCovariance(Key.B(4)).at(0, 0)), 1e-6));
    }

}
