package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import gtsam.noiseModel.Constrained;
import gtsam.noiseModel.Diagonal;
import gtsam.noiseModel.Gaussian;
import gtsam.noiseModel.Isotropic;
import gtsam.noiseModel.Robust;
import gtsam.noiseModel.Unit;
import gtsam.noiseModel.Util;
import gtsam.noiseModel.mEstimator.AsymmetricCauchy;
import gtsam.noiseModel.mEstimator.AsymmetricTukey;
import gtsam.noiseModel.mEstimator.Cauchy;
import gtsam.noiseModel.mEstimator.DCS;
import gtsam.noiseModel.mEstimator.Fair;
import gtsam.noiseModel.mEstimator.GemanMcClure;
import gtsam.noiseModel.mEstimator.Huber;
import gtsam.noiseModel.mEstimator.L2WithDeadZone;
import gtsam.noiseModel.mEstimator.TruncatedLeastSquares;
import gtsam.noiseModel.mEstimator.Tukey;
import gtsam.noiseModel.mEstimator.Welsch;

/**
 * See gtsam/linear/tests/testNoiseModel.cpp
 * 
 * I skipped the custom loss function.
 */
public class NoiseModelTest {

    static final double kSigma;
    static final double kInverseSigma;
    static final double kVariance;
    static final double prc;
    static final Matrix R;
    static final Matrix kCovariance;
    static final Vector3 kSigmas;

    static {
        try {
            kSigma = 2;
            kInverseSigma = 1.0 / kSigma;
            kVariance = kSigma * kSigma;
            prc = 1.0 / kVariance;
            R = Matrix.I_3x3().times(kInverseSigma);
            kCovariance = Matrix.I_3x3().times(kVariance);
            kSigmas = new Vector3(kSigma, kSigma, kSigma);
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }
    }

    @Test
    void testconstructors() throws Throwable {
        Vector3 whitened = new Vector3(5.0, 10.0, 15.0);
        Vector3 unwhitened = new Vector3(10.0, 20.0, 30.0);

        // Construct noise models
        List<shared_ptr<? extends Gaussian>> m = new ArrayList<>();

        m.add(Gaussian.SqrtInformation(R, false));
        m.add(Gaussian.Covariance(kCovariance, false));
        m.add(Gaussian.Information(kCovariance.inverse(), false));
        m.add(Diagonal.Sigmas(kSigmas, false));
        m.add(Diagonal.Variances(new Vector3(kVariance, kVariance, kVariance), false));
        m.add(Diagonal.Precisions(new Vector3(prc, prc, prc), false));
        m.add(Isotropic.Sigma(3, kSigma, false));
        m.add(Isotropic.Variance(3, kVariance, false));
        m.add(Isotropic.Precision(3, prc, false));

        // test kSigmas
        for (shared_ptr<? extends Gaussian> mi : m)
            assertTrue(assert_equal(kSigmas, new Vector3(mi.get().sigmas())));

        // test whiten
        for (shared_ptr<? extends Gaussian> mi : m)
            assertTrue(assert_equal(whitened, new Vector3(mi.get().whiten(new Vector(unwhitened)))));

        // test unwhiten
        for (shared_ptr<? extends Gaussian> mi : m)
            assertTrue(assert_equal(unwhitened, new Vector3(mi.get().unwhiten(new Vector(whitened)))));

        // test squared Mahalanobis distance
        double distance = 5 * 5 + 10 * 10 + 15 * 15;
        for (shared_ptr<? extends Gaussian> mi : m)
            assertEquals(distance, mi.get().squaredMahalanobisDistance(new Vector(unwhitened)), 1e-9);

        // test R matrix
        for (shared_ptr<? extends Gaussian> mi : m)
            assertTrue(assert_equal(R, mi.get().R()));

        // test covariance
        for (shared_ptr<? extends Gaussian> mi : m)
            assertTrue(assert_equal(kCovariance, mi.get().covariance()));

        // test covariance
        for (shared_ptr<? extends Gaussian> mi : m)
            assertTrue(assert_equal(kCovariance.inverse(), mi.get().information()));

        // test Whiten operator
        Matrix H = new Matrix(new double[][] {
                { 0.0, 0.0, 1.0, 1.0 },
                { 0.0, 1.0, 0.0, 1.0 },
                { 1.0, 0.0, 0.0, 1.0 } });
        Matrix expected = H.times(kInverseSigma);
        for (shared_ptr<? extends Gaussian> mi : m)
            assertTrue(assert_equal(expected, mi.get().Whiten(H)));

        // can only test inplace version once :-)
        m.get(0).get().WhitenInPlace(H);
        assertTrue(assert_equal(expected, H));
    }

    @Test
    void testUnit() throws Throwable {
        Vector3 v = new Vector3(5.0, 10.0, 15.0);
        shared_ptr<? extends Gaussian> u = Unit.Create(3);
        assertTrue(assert_equal(v, new Vector3(u.get().whiten(new Vector(v)))));
    }

    @Test
    void testUnitCreateMeasured() throws Throwable {
        Matrix fixed = Matrix.I_2x2();
        shared_ptr<Unit> fixedModel = Unit.Create(fixed);
        assertEquals(4, fixedModel.get().dim());
        assertTrue(assert_equal(fixedModel.get(), Unit.Create(fixed).get()));

        Matrix dynamic = new Matrix(new double[][] { { 0, 0, 0 }, { 0, 0, 0 } });
        assertEquals(6, Unit.Create(dynamic).get().dim());
        assertEquals(2, Unit.Create(new Vector(new double[] { 1.0, 2.0 })).get().dim());
        assertEquals(1, Unit.Create(1).get().dim());
    }

    @Test
    void testMatchesDimension() throws Throwable {
        Matrix fixed = Matrix.I_2x2();
        assertTrue(Util.matchesDimension(Unit.Create(4).get(), fixed));
        assertFalse(Util.matchesDimension(Unit.Create(3).get(), fixed));

        Matrix dynamic = new Matrix(new double[][] { { 0, 0, 0 }, { 0, 0, 0 } });
        assertTrue(Util.matchesDimension(Unit.Create(6).get(), dynamic));
        assertTrue(Util.matchesDimension(Unit.Create(2).get(), new Vector(new double[] { 1.0, 2.0 })));
        assertTrue(Util.matchesDimension(Unit.Create(1).get(), 1));
    }

    @Test
    void testequals() throws Throwable {
        shared_ptr<Gaussian> g1 = Gaussian.SqrtInformation(R, true);
        shared_ptr<Gaussian> g2 = Gaussian.SqrtInformation(Matrix.I_3x3(), true);
        shared_ptr<Diagonal> d1 = Diagonal.Sigmas(new Vector3(kSigma, kSigma, kSigma));
        shared_ptr<Diagonal> d2 = Diagonal.Sigmas(new Vector3(0.1, 0.2, 0.3));
        shared_ptr<Isotropic> i1 = Isotropic.Sigma(3, kSigma, true);
        shared_ptr<Isotropic> i2 = Isotropic.Sigma(3, 0.7, true);

        assertTrue(assert_equal(g1.get(), g1.get()));
        assertFalse(assert_equal(g1.get(), g2.get()));

        assertTrue(assert_equal(d1.get(), d1.get()));
        assertFalse(assert_equal(d1.get(), d2.get()));

        assertTrue(assert_equal(i1.get(), i1.get()));
        assertFalse(assert_equal(i1.get(), i2.get()));
    }

    @Test
    void testConstrainedConstructors() throws Throwable {
        int d = 3;
        double m = 100.0;
        final double kInfinity = Double.POSITIVE_INFINITY;
        Vector3 sigmas = new Vector3(kSigma, 0.0, 0.0);
        Vector3 mu = new Vector3(200.0, 300.0, 400.0);
        shared_ptr<Constrained> actual = Constrained.All(d);
        // TODO: why should this be a thousand ??? Dummy variable?
        assertTrue(assert_equal(Vector.Constant(d, 1000.0), actual.get().mu()));
        assertTrue(assert_equal(Vector.Constant(d, 0), actual.get().sigmas()));
        assertTrue(assert_equal(Vector.Constant(d, 0), actual.get().invsigmas()));
        //        Actually zero as dummy value
        assertTrue(assert_equal(Vector.Constant(d, kInfinity),
        actual.get().precisions()));
        // Infinite precision for hard constraints

        actual = Constrained.All(d, m);
        assertTrue(assert_equal(Vector.Constant(d, m), actual.get().mu()));

        actual = Constrained.All(d, mu);
        assertTrue(assert_equal(mu, actual.get().mu()));

        actual = Constrained.MixedSigmas(mu, sigmas);
        assertTrue(assert_equal(mu, actual.get().mu()));

        actual = Constrained.MixedSigmas(m, sigmas);
        assertTrue(assert_equal(Vector::Constant(d, m), actual.get().mu()));
    }

    @Test
    void testConstrainedMixed() throws Throwable {
        Vector3 feasible = new Vector3(1.0, 0.0, 1.0);
        Vector3 infeasible = new Vector3(1.0, 1.0, 1.0);
        shared_ptr<Diagonal> d = Constrained.MixedSigmas(Vector3(kSigma, 0.0,
                kSigma));
        // NOTE: we catch constrained variables elsewhere, so whitening does nothing
        assertTrue(assert_equal(new Vector3(0.5, 1.0, 0.5), d.get().whiten(infeasible)));
        assertTrue(assert_equal(new Vector3(0.5, 0.0, 0.5), d.get().whiten(feasible)));

        assertEquals(0.5 * (1000.0 + 0.25 +
                0.25), d.get().loss(d.get().squaredMahalanobisDistance(infeasible)), 1e-9);
        assertEquals(0.5, d.get().squaredMahalanobisDistance(feasible), 1e-9);
        assertEquals(0.5 * 0.5, d.get().loss(0.5), 1e-9);
    }

    @Test
    void testConstrainedAll() throws Throwable {
        Vector3 feasible = new Vector3(0.0, 0.0, 0.0);
        Vector3 infeasible = new Vector3(1.0, 1.0, 1.0);

        shared_ptr<Constrained> i = Constrained.All(3);
        // NOTE: we catch constrained variables elsewhere, so whitening does nothing
        assertTrue(assert_equal(new Vector(new double[] { 1.0, 1.0, 1.0 }), i.get().whiten(new Vector(infeasible))));
        assertTrue(assert_equal(new Vector(new double[] { 0.0, 0.0, 0.0 }), i.get().whiten(new Vector(feasible))));

        assertEquals(0.5 * 1000.0 * 3.0,
                i.get().loss(i.get().squaredMahalanobisDistance(new Vector(infeasible))),
                1e-9);
        assertEquals(0.0, i.get().squaredMahalanobisDistance(new Vector(feasible)), 1e-9);
        assertEquals(0.0, i.get().loss(0.0), 1e-9);
    }

    @Test
    void testConstrainedInformationFromA() throws Throwable {
        // Use one constrained row and one finite-precision row.
        Vector2 sigmas = new Vector2(0.0, 2.0);
        shared_ptr<Constrained> model = Constrained.MixedSigmas(new Vector(sigmas));

        Matrix A = new Matrix(new double[][] { { 1.0, 0.0 }, { 0.0, 2.0 } });

        Matrix info = model.get().informationFromA(A);

        assertTrue(Double.isInfinite(info(0, 0)));
        assertEquals(0.0, info.at(0, 1), 1e-12);
        assertEquals(0.0, info.at(1, 0), 1e-12);
        assertEquals(1.0, info.at(1, 1), 1e-12);

        // Constrained row with support in multiple columns should mark cross-terms.
        Matrix A_dense = new Matrix(new double[][] { { 1.0, 1.0 }, { 0.0, 2.0 } });

        Matrix info_dense = model.get().informationFromA(A_dense);

        assertTrue(Double.isInfinite(info_dense(0, 0)));
        assertTrue(Double.isInfinite(info_dense(0, 1)));
        assertTrue(Double.isInfinite(info_dense(1, 0)));
        assertTrue(Double.isInfinite(info_dense(1, 1)));
    }

    static class exampleQR {
        // create a matrix to eliminate
        static Matrix Ab;
        static Vector sigmas;
        static Matrix Rd;
        static shared_ptr<Diagonal> diagonal;
        static {
            try {
                Ab = new Matrix(new double[][] {
                        { -1., 0., 1., 0., 0., 0., -0.2 },
                        { 0., -1., 0., 1., 0., 0., 0.3 },
                        { 1., 0., 0., 0., -1., 0., 0.2 },
                        { 0., 1., 0., 0., 0., -1., -0.1 } });
                sigmas = new Vector(new double[] { 0.2, 0.2, 0.1, 0.1 });

                // the matrix AB yields the following factorized version:
                Rd = new Matrix(new double[][] {
                        { 11.1803, 0.0, -2.23607, 0.0, -8.94427, 0.0, 2.23607 },
                        { 0.0, 11.1803, 0.0, -2.23607, 0.0, -8.94427, -1.56525 },
                        { 0.0, 0.0, 4.47214, 0.0, -4.47214, 0.0, 0.0 },
                        { 0.0, 0.0, 0.0, 4.47214, 0.0, -4.47214, 0.894427 } });

                diagonal = Diagonal.Sigmas(sigmas);
            } catch (Throwable t) {
                throw new RuntimeException(t);
            }
        }
    }

    @Test
    void testQR() throws Throwable {
        Matrix Ab1 = exampleQR.Ab;
        // make a copy. otherwise overwritten !
        Matrix Ab2 = exampleQR.Ab.times(1.0);

        // Call Gaussian version
        shared_ptr<Diagonal> actual1 = exampleQR.diagonal.get().QR(Ab1);
        assertTrue(actual1.get().isUnit());
        // Ab was modified in place !!!
        assertTrue(Matrix.linear_dependent(exampleQR.Rd, Ab1, 1e-4));

        // // Expected result for constrained version
        Vector expectedSigmas = new Vector(new double[] {
                0.0894427, 0.0894427, 0.223607, 0.223607 });
        shared_ptr<Diagonal> expectedModel = Diagonal.Sigmas(expectedSigmas);
        Matrix expectedRd2 = new Matrix(new double[][] {
                { 1., 0., -0.2, 0., -0.8, 0., 0.2 },
                { 0., 1., 0., -0.2, 0., -0.8, -0.14 },
                { 0., 0., 1., 0., -1., 0., 0.0 },
                { 0., 0., 0., 1., 0., -1., 0.2 } });

        // Call Constrained version
        shared_ptr<? extends Diagonal> constrained = Constrained.MixedSigmas(exampleQR.sigmas);
        shared_ptr<Diagonal> actual2 = constrained.get().QR(Ab2);
        assertTrue(assert_equal(expectedModel.get(), actual2.get(), 1e-6));
        // Ab was modified in place !!!
        assertTrue(Matrix.linear_dependent(expectedRd2, Ab2, 1e-6));
    }

    @Test
    void testOverdeterminedQR() {
        // Matrix Ab1(9, 4);
        // Ab1 << 0, 1, 0, 0, //
        // 0, 0, 1, 0, //
        // Matrix74::Ones();
        // Matrix Ab2 = Ab1; // otherwise overwritten !

        // Call Gaussian version
        // Vector9 sigmas = Vector9::Ones() ;
        // shared_ptr<Diagonal> diagonal = noiseModel::Diagonal::Sigmas(sigmas);
        // shared_ptr<Diagonal> actual1 = diagonal.get().QR(Ab1);
        // assertTrue(actual1.get().isUnit());
        // Matrix expectedRd(9,4);
        // expectedRd << -2.64575131, -2.64575131, -2.64575131, -2.64575131, //
        // 0.0, -1, 0, 0, //
        // 0.0, 0.0, -1, 0, //
        // Matrix64::Zero();
        // assertTrue(assert_equal(expectedRd, Ab1, 1e-4)); // Ab was modified in place
        // !!!

        // // Expected result for constrained version
        // Vector3 expectedSigmas(0.377964473, 1, 1);
        // shared_ptr<Diagonal> expectedModel =
        // noiseModel::Diagonal::Sigmas(expectedSigmas);

        // // Call Constrained version
        // shared_ptr<Diagonal> constrained =
        // noiseModel::Constrained::MixedSigmas(sigmas);
        // shared_ptr<Diagonal> actual2 = constrained.get().QR(Ab2);
        // assertTrue(assert_equal(*expectedModel, *actual2, 1e-6));
        // expectedRd.row(0) *= 0.377964473; // not divided by sigma!
        // assertTrue(assert_equal(-expectedRd, Ab2, 1e-6)); // Ab was modified in place
        // !!!
    }

    @Test
    void testMixedQR() {
        // // Call Constrained version, with first and third row treated as constraints
        // // Naming the 6 variables u,v,w,x,y,z, we have
        // // u = -z
        // // w = -x
        // // And let's have simple priors on variables
        // Matrix Ab(5,6+1);
        // Ab <<
        // 1,0,0,0,0,1, 0, // u+z = 0
        // 0,0,0,0,1,0, 0, // y^2
        // 0,0,1,1,0,0, 0, // w+x = 0
        // 0,1,0,0,0,0, 0, // v^2
        // 0,0,0,0,0,1, 0; // z^2
        // Vector mixed_sigmas = (Vector(5) << 0, 1, 0, 1, 1).finished();
        // shared_ptr<Diagonal> constrained =
        // noiseModel::Constrained::MixedSigmas(mixed_sigmas);

        // // Expected result
        // Vector expectedSigmas = (Vector(5) << 0, 1, 0, 1, 1).finished();
        // shared_ptr<Diagonal> expectedModel =
        // noiseModel::Diagonal::Sigmas(expectedSigmas);
        // Matrix expectedRd(5, 6+1);
        // expectedRd << 1, 0, 0, 0, 0, 1, 0, //
        // 0, 1, 0, 0, 0, 0, 0, //
        // 0, 0, 1, 1, 0, 0, 0, //
        // 0, 0, 0, 0, 1, 0, 0, //
        // 0, 0, 0, 0, 0, 1, 0; //

        // shared_ptr<Diagonal> actual = constrained.get().QR(Ab);
        // assertTrue(assert_equal(*expectedModel,*actual,1e-6));
        // assertTrue(linear_dependent(expectedRd,Ab,1e-6)); // Ab was modified in place
        // !!!
    }

    @Test
    void testMixedQR2() {
        // Let's have three variables x,y,z, but x=z and y=z
        // Hence, all non-constraints are really measurements on z
        // Matrix Ab(11,3+1);
        // Ab <<
        // 1,0,0, 0, //
        // 0,1,0, 0, //
        // 0,0,1, 0, //
        // -1,0,1, 0, // x=z
        // 1,0,0, 0, //
        // 0,1,0, 0, //
        // 0,0,1, 0, //
        // 0,-1,1, 0, // y=z
        // 1,0,0, 0, //
        // 0,1,0, 0, //
        // 0,0,1, 0; //

        // Vector sigmas(11);
        // sigmas.setOnes();
        // sigmas[3] = 0;
        // sigmas[7] = 0;
        // shared_ptr<Diagonal> constrained =
        // noiseModel::Constrained::MixedSigmas(sigmas);

        // // Expected result
        // Vector3 expectedSigmas(0,0,1.0/3);
        // shared_ptr<Diagonal> expectedModel =
        // noiseModel::Constrained::MixedSigmas(expectedSigmas);
        // Matrix expectedRd(11, 3+1);
        // expectedRd.setZero();
        // expectedRd.row(0) << -1, 0, 1, 0; // x=z
        // expectedRd.row(1) << 0, -1, 1, 0; // y=z
        // expectedRd.row(2) << 0, 0, 1, 0; // z=0 +/- 1/3

        // shared_ptr<Diagonal> actual = constrained.get().QR(Ab);
        // assertTrue(assert_equal(*expectedModel,*actual,1e-6));
        // assertTrue(assert_equal(expectedRd,Ab,1e-6)); // Ab was modified in place !!!
    }

    @Test
    void testFullyConstrained() {
        // Matrix Ab(3,7);
        // Ab <<
        // 1,0,0,0,0,1, 2, // u+z = 2
        // 0,0,1,1,0,0, 4, // w+x = 4
        // 0,1,0,1,1,1, 8; // v+x+y+z=8
        shared_ptr<? extends Diagonal> constrained = Constrained.All(3);

        // // Expected result
        // shared_ptr<Diagonal> expectedModel = noiseModel::Diagonal::Sigmas(Vector3
        // (0,0,0));
        // Matrix expectedRd(3, 7);
        // expectedRd << 1, 0, 0, 0, 0, 1, 2, //
        // 0, 1, 0, 1, 1, 1, 8, //
        // 0, 0, 1, 1, 0, 0, 4; //

        shared_ptr<Diagonal> actual = constrained.get().QR(Ab);
        assertTrue(assert_equal(*expectedModel,*actual,1e-6));
        assertTrue(linear_dependent(expectedRd,Ab,1e-6));
         // Ab was modified in place !!!
    }

    // This matches constraint_eliminate2 in testJacobianFactor
    @Test
    void testQRNan() throws Throwable {
        shared_ptr<? extends Diagonal> constrained = Constrained.All(2);
        Matrix Ab = new Matrix(new double[][] {
                { 2, 4, 2, 4, 6 },
                { 2, 1, 2, 4, 4 }
        });

        shared_ptr<? extends Diagonal> expected = Constrained.All(2);
        Matrix expectedAb = new Matrix(new double[][] {
                { 1, 2, 1, 2, 3 },
                { 0, 1, 0, 0, 2.0 / 3 }
        });

        shared_ptr<Diagonal> actual = constrained.get().QR(Ab);
        assertTrue(assert_equal(expected.get(), actual.get()));
        assertTrue(Matrix.linear_dependent(expectedAb, Ab));
    }

    @Test
    void testSmartSqrtInformation() throws Throwable {
        boolean smart = true;
        shared_ptr<? extends Gaussian> expected = Unit.Create(3);
        shared_ptr<Gaussian> actual = Gaussian.SqrtInformation(Matrix.I_3x3(), smart);
        assertTrue(assert_equal(expected.get(), actual.get()));
    }

    @Test
    void testSmartSqrtInformation2() throws Throwable {
        boolean smart = true;
        shared_ptr<? extends Gaussian> expected = Isotropic.Sigma(3, 2, true);
        shared_ptr<Gaussian> actual = Gaussian.SqrtInformation(Matrix.I_3x3().times(0.5), smart);
        assertTrue(assert_equal(expected.get(), actual.get()));
    }

    @Test
    void testSmartInformation() throws Throwable {
        boolean smart = true;
        shared_ptr<? extends Gaussian> expected = Isotropic.Variance(3, 2, true);
        Matrix M = Matrix.I_3x3().times(0.5);
        // TODO: implement checkIfDiagonal
        // assertTrue(checkIfDiagonal(M));
        shared_ptr<Gaussian> actual = Gaussian.Information(M, smart);
        assertTrue(assert_equal(expected.get(), actual.get()));
    }

    @Test
    void testSmartCovariance() throws Throwable {
        boolean smart = true;
        shared_ptr<? extends Gaussian> expected = Unit.Create(3);
        shared_ptr<Gaussian> actual = Gaussian.Covariance(Matrix.I_3x3(), smart);
        assertTrue(assert_equal(expected.get(), actual.get()));
    }

    @Test
    void testScalarOrVector() throws Throwable {
        boolean smart = true;
        shared_ptr<? extends Gaussian> expected = Unit.Create(3);
        shared_ptr<Gaussian> actual = Gaussian.Covariance(Matrix.I_3x3(), smart);
        assertTrue(assert_equal(expected.get(), actual.get()));
    }

    @Test
    void testWhitenInPlace() throws Throwable {
        Vector3 sigmas = new Vector3(0.1, 0.1, 0.1);
        shared_ptr<Diagonal> model = Diagonal.Sigmas(sigmas);
        Matrix A = Matrix.I_3x3();
        model.get().WhitenInPlace(A);
        Matrix expected = Matrix.I_3x3().times(10);
        assertTrue(assert_equal(expected, A));
    }

    @Test
    void testInPlaceVectorOperations() throws Throwable {
        shared_ptr<Gaussian> gaussian = Gaussian.SqrtInformation(R, false);
        Vector3 v = new Vector3(5.0, 10.0, 15.0);
        Vector expected = gaussian.get().unwhiten(new Vector(v));
        Vector actual = new Vector(v);
        gaussian.get().unwhitenInPlace(actual);
        assertTrue(assert_equal(expected, actual));

        shared_ptr<Diagonal> diagonal = Diagonal.Sigmas(kSigmas, false);
        v = new Vector3(10.0, 20.0, 30.0);
        expected = diagonal.get().whiten(new Vector(v));
        actual = new Vector(v);
        diagonal.get().whitenInPlace(actual);
        assertTrue(assert_equal(expected, actual));

        v = new Vector3(1.0, 2.0, 3.0);
        expected = diagonal.get().unwhiten(new Vector(v));
        actual = new Vector(v);
        diagonal.get().unwhitenInPlace(actual);
        assertTrue(assert_equal(expected, actual));

        shared_ptr<Isotropic> isotropic = Isotropic.Sigma(3, kSigma, false);
        v = new Vector3(1.0, 2.0, 3.0);
        expected = isotropic.get().unwhiten(new Vector(v));
        actual = new Vector(v);
        isotropic.get().unwhitenInPlace(actual);
        assertTrue(assert_equal(expected, actual));

        shared_ptr<Constrained> constrained = Constrained.MixedSigmas(
                new Vector(new double[] { kSigma, 0.0, kSigma }));
        v = new Vector3(2.0, 3.0, 4.0);
        expected = constrained.get().whiten(new Vector(v));
        actual = new Vector(v);
        constrained.get().whitenInPlace(actual);
        assertTrue(assert_equal(expected, actual));
        assertEquals(v.at(1), actual.at(1), 1e-12);

        shared_ptr<Robust> robust = Robust.Create(Huber.Create(1.345),
                diagonal);
        v = new Vector3(1.0, 2.0, 3.0);
        expected = robust.get().whiten(new Vector(v));
        actual = new Vector(v);
        robust.get().whitenInPlace(actual);
        assertTrue(assert_equal(expected, actual));
    }

    @Test
    void testInPlaceVectorBlockOperations() throws Throwable {
        shared_ptr<Gaussian> gaussian = Gaussian.SqrtInformation(R, false);
        shared_ptr<Diagonal> diagonal = Diagonal.Sigmas(kSigmas, false);
        shared_ptr<Isotropic> isotropic = Isotropic.Sigma(3, kSigma, false);
        shared_ptr<Constrained> constrained = Constrained.MixedSigmas(
                new Vector(new double[] { kSigma, 0.0, kSigma }));

        Vector v = Vector.LinSpaced(5, 1.0, 5.0);
        Eigen::Block<Vector> block(v, 1, 0, 3, 1);
        Vector expected = diagonal.get().whiten(Vector(block));
        diagonal.get().whitenInPlace(block);
        assertTrue(assert_equal(expected, Vector(block)));

        v = Vector.LinSpaced(5, 1.0, 5.0);
        Eigen::Block<Vector> block_unwhiten(v, 1, 0, 3, 1);
        expected = diagonal.get().unwhiten(Vector(block_unwhiten));
        diagonal.get().unwhitenInPlace(block_unwhiten);
        assertTrue(assert_equal(expected, Vector(block_unwhiten)));

        v = Vector.LinSpaced(5, 1.0, 5.0);
        Eigen::Block<Vector> block_constrained(v, 1, 0, 3, 1);
        expected = constrained.get().whiten(Vector(block_constrained));
        constrained.get().whitenInPlace(block_constrained);
        assertTrue(assert_equal(expected, Vector(block_constrained)));

        v = Vector.LinSpaced(5, 1.0, 5.0);
        Eigen::Block<Vector> block_iso(v, 1, 0, 3, 1);
        expected = isotropic.get().unwhiten(Vector(block_iso));
        isotropic.get().unwhitenInPlace(block_iso);
        assertTrue(assert_equal(expected, Vector(block_iso)));

        v = Vector.LinSpaced(5, 1.0, 5.0);
        Eigen::Block<Vector> block_gauss(v, 1, 0, 3, 1);
        expected = gaussian.get().unwhiten(Vector(block_gauss));
        gaussian.get().unwhitenInPlace(block_gauss);
        assertTrue(assert_equal(expected, Vector(block_gauss)));
    }

    // /*
    // * These tests are responsible for testing the weight functions for the
    // m-estimators in GTSAM.
    // * The weight function is related to the analytic derivative of the loss
    // function. See
    // *
    // https://members.loria.fr/MOBerger/Enseignement/Master2/Documents/ZhangIVC-97-01.pdf
    // * for details. This weight function is required when optimizing cost
    // functions with robust
    // * penalties using iteratively re-weighted least squares.
    // */

    @Test
    void testrobustFunctionFair() {
        final double k = 5.0;
        final double error1 = 1.0;
        final double error2 = 10.0;
        final double error3 = -10.0;
        final double error4 = -1.0;
        final shared_ptr<Fair> fair = Fair.Create(k);
        assertEquals(0.8333333333333333, fair.get().weight(error1), 1e-8);
        assertEquals(0.3333333333333333, fair.get().weight(error2), 1e-8);
        // Test negative value to ensure we take absolute value of error.
        assertEquals(0.3333333333333333, fair.get().weight(error3), 1e-8);
        assertEquals(0.8333333333333333, fair.get().weight(error4), 1e-8);

        assertEquals(0.441961080151135, fair.get().loss(error1), 1e-8);
        assertEquals(22.534692783297260, fair.get().loss(error2), 1e-8);
        assertEquals(22.534692783297260, fair.get().loss(error3), 1e-8);
        assertEquals(0.441961080151135, fair.get().loss(error4), 1e-8);
    }

    @Test
    void testrobustFunctionHuber() {
        final double k = 5.0;
        final double error1 = 1.0;
        final double error2 = 10.0;
        final double error3 = -10.0;
        final double error4 = -1.0;
        final shared_ptr<Huber> huber = Huber.Create(k);
        assertEquals(1.0, huber.get().weight(error1), 1e-8);
        assertEquals(0.5, huber.get().weight(error2), 1e-8);
        // Test negative value to ensure we take absolute value of error.
        assertEquals(0.5, huber.get().weight(error3), 1e-8);
        assertEquals(1.0, huber.get().weight(error4), 1e-8);

        assertEquals(0.5000, huber.get().loss(error1), 1e-8);
        assertEquals(37.5000, huber.get().loss(error2), 1e-8);
        assertEquals(37.5000, huber.get().loss(error3), 1e-8);
        assertEquals(0.5000, huber.get().loss(error4), 1e-8);
    }

    @Test
    void testrobustFunctionCauchy() {
        final double k = 5.0;
        final double error1 = 1.0;
        final double error2 = 10.0;
        final double error3 = -10.0;
        final double error4 = -1.0;
        final shared_ptr<Cauchy> cauchy = Cauchy.Create(k);
        assertEquals(0.961538461538461, cauchy.get().weight(error1), 1e-8);
        assertEquals(0.2000, cauchy.get().weight(error2), 1e-8);
        // Test negative value to ensure we take absolute value of error.
        assertEquals(0.2000, cauchy.get().weight(error3), 1e-8);
        assertEquals(0.961538461538461, cauchy.get().weight(error4), 1e-8);

        assertEquals(0.490258914416017, cauchy.get().loss(error1), 1e-8);
        assertEquals(20.117973905426254, cauchy.get().loss(error2), 1e-8);
        assertEquals(20.117973905426254, cauchy.get().loss(error3), 1e-8);
        assertEquals(0.490258914416017, cauchy.get().loss(error4), 1e-8);
    }

    @Test
    void testrobustFunctionAsymmetricCauchy() {
        final double k = 5.0;
        final double error1 = 1.0;
        final double error2 = 10.0;
        final double error3 = -10.0;
        final double error4 = -1.0;
        final shared_ptr<AsymmetricCauchy> cauchy = AsymmetricCauchy.Create(k);
        assertEquals(0.961538461538461, cauchy.get().weight(error1), 1e-8);
        assertEquals(0.2000, cauchy.get().weight(error2), 1e-8);
        // Test negative value to ensure we take absolute value of error.
        assertEquals(1.0, cauchy.get().weight(error3), 1e-8);
        assertEquals(1.0, cauchy.get().weight(error4), 1e-8);

        assertEquals(0.490258914416017, cauchy.get().loss(error1), 1e-8);
        assertEquals(20.117973905426254, cauchy.get().loss(error2), 1e-8);
        assertEquals(50.0, cauchy.get().loss(error3), 1e-8);
        assertEquals(0.5, cauchy.get().loss(error4), 1e-8);
    }

    @Test
    void testrobustFunctionGemanMcClure() {
        final double k = 1.0;
        final double error1 = 1.0;
        final double error2 = 10.0;
        final double error3 = -10.0;
        final double error4 = -1.0;
        final shared_ptr<GemanMcClure> gmc = GemanMcClure.Create(k);
        assertEquals(0.25, gmc.get().weight(error1), 1e-8);
        assertEquals(9.80296e-5, gmc.get().weight(error2), 1e-8);
        assertEquals(9.80296e-5, gmc.get().weight(error3), 1e-8);
        assertEquals(0.25, gmc.get().weight(error4), 1e-8);

        assertEquals(0.2500, gmc.get().loss(error1), 1e-8);
        assertEquals(0.495049504950495, gmc.get().loss(error2), 1e-8);
        assertEquals(0.495049504950495, gmc.get().loss(error3), 1e-8);
        assertEquals(0.2500, gmc.get().loss(error4), 1e-8);
    }

    @Test
    void testrobustFunctionTLS() {
        final double k = 4.0;
        final double error1 = 0.5;
        final double error2 = 10.0;
        final double error3 = -10.0;
        final double error4 = -0.5;
        final shared_ptr<TruncatedLeastSquares> tls = TruncatedLeastSquares.Create(k);
        assertEquals(1.0, tls.get().weight(error1), 1e-8);
        assertEquals(0.0, tls.get().weight(error2), 1e-8);
        assertEquals(0.0, tls.get().weight(error3), 1e-8);
        assertEquals(1.0, tls.get().weight(error4), 1e-8);

        assertEquals(0.1250, tls.get().loss(error1), 1e-8);
        assertEquals(8.0, tls.get().loss(error2), 1e-8);
        assertEquals(8.0, tls.get().loss(error3), 1e-8);
        assertEquals(0.1250, tls.get().loss(error4), 1e-8);
    }

    @Test
    void testrobustFunctionWelsch() {
        final double k = 5.0;
        final double error1 = 1.0;
        final double error2 = 10.0;
        final double error3 = -10.0;
        final double error4 = -1.0;
        final shared_ptr<Welsch> welsch = Welsch.Create(k);
        assertEquals(0.960789439152323, welsch.get().weight(error1), 1e-8);
        assertEquals(0.018315638888734, welsch.get().weight(error2), 1e-8);
        // Test negative value to ensure we take absolute value of error.
        assertEquals(0.018315638888734, welsch.get().weight(error3), 1e-8);
        assertEquals(0.960789439152323, welsch.get().weight(error4), 1e-8);

        assertEquals(0.490132010595960, welsch.get().loss(error1), 1e-8);
        assertEquals(12.271054513890823, welsch.get().loss(error2), 1e-8);
        assertEquals(12.271054513890823, welsch.get().loss(error3), 1e-8);
        assertEquals(0.490132010595960, welsch.get().loss(error4), 1e-8);
    }

    @Test
    void testrobustFunctionTukey() {
        final double k = 5.0;
        final double error1 = 1.0;
        final double error2 = 10.0;
        final double error3 = -10.0;
        final double error4 = -1.0;
        final shared_ptr<Tukey> tukey = Tukey.Create(k);
        assertEquals(0.9216, tukey.get().weight(error1), 1e-8);
        assertEquals(0.0, tukey.get().weight(error2), 1e-8);
        // Test negative value to ensure we take absolute value of error.
        assertEquals(0.0, tukey.get().weight(error3), 1e-8);
        assertEquals(0.9216, tukey.get().weight(error4), 1e-8);

        assertEquals(0.480266666666667, tukey.get().loss(error1), 1e-8);
        assertEquals(4.166666666666667, tukey.get().loss(error2), 1e-8);
        assertEquals(4.166666666666667, tukey.get().loss(error3), 1e-8);
        assertEquals(0.480266666666667, tukey.get().loss(error4), 1e-8);
    }

    @Test
    void testrobustFunctionAsymmetricTukey() {
        final double k = 5.0;
        final double error1 = 1.0;
        final double error2 = 10.0;
        final double error3 = -10.0;
        final double error4 = -1.0;
        final shared_ptr<AsymmetricTukey> tukey = AsymmetricTukey.Create(k);
        assertEquals(0.9216, tukey.get().weight(error1), 1e-8);
        assertEquals(0.0, tukey.get().weight(error2), 1e-8);
        // Test negative value to ensure we take absolute value of error.
        assertEquals(1.0, tukey.get().weight(error3), 1e-8);
        assertEquals(1.0, tukey.get().weight(error4), 1e-8);

        assertEquals(0.480266666666667, tukey.get().loss(error1), 1e-8);
        assertEquals(4.166666666666667, tukey.get().loss(error2), 1e-8);
        assertEquals(50.0, tukey.get().loss(error3), 1e-8);
        assertEquals(0.5, tukey.get().loss(error4), 1e-8);
    }

    @Test
    void testrobustFunctionDCS() {
        final double k = 1.0;
        final double error1 = 1.0;
        final double error2 = 10.0;
        final shared_ptr<DCS> dcs = DCS.Create(k);

        assertEquals(1.0, dcs.get().weight(error1), 1e-8);
        assertEquals(0.00039211, dcs.get().weight(error2), 1e-8);

        assertEquals(0.5, dcs.get().loss(error1), 1e-8);
        assertEquals(0.9900990099, dcs.get().loss(error2), 1e-8);
    }

    @Test
    void testrobustFunctionL2WithDeadZone() {
        final double k = 1.0;
        final double e0 = -10.0;
        final double e1 = -1.01;
        final double e2 = -0.99;
        final double e3 = 0.99;
        final double e4 = 1.01;
        final double e5 = 10.0;
        final shared_ptr<L2WithDeadZone> lsdz = L2WithDeadZone.Create(k);

        assertEquals(0.9, lsdz.get().weight(e0), 1e-8);
        assertEquals(0.00990099009, lsdz.get().weight(e1), 1e-8);
        assertEquals(0.0, lsdz.get().weight(e2), 1e-8);
        assertEquals(0.0, lsdz.get().weight(e3), 1e-8);
        assertEquals(0.00990099009, lsdz.get().weight(e4), 1e-8);
        assertEquals(0.9, lsdz.get().weight(e5), 1e-8);

        assertEquals(40.5, lsdz.get().loss(e0), 1e-8);
        assertEquals(0.00005, lsdz.get().loss(e1), 1e-8);
        assertEquals(0.0, lsdz.get().loss(e2), 1e-8);
        assertEquals(0.0, lsdz.get().loss(e3), 1e-8);
        assertEquals(0.00005, lsdz.get().loss(e4), 1e-8);
        assertEquals(40.5, lsdz.get().loss(e5), 1e-8);
    }

    @Test
    void testrobustNoiseHuber() throws Throwable {
        final double k = 10.0;
        final double error1 = 1.0;
        final double error2 = 100.0;
        Matrix A = new Matrix(new double[][] { { 1.0, 10.0 }, { 100.0, 1000.0 } });
        Vector b = Vector2(error1, error2);
        final shared_ptr<Robust> robust = Robust.Create(
        Huber.Create(k, mEstimator::Huber::Scalar),
        Unit::Create(2));

        robust.get().WhitenSystem(A, b);

        assertEquals(error1, b(0), 1e-8);
        assertEquals(sqrt(k*error2), b(1), 1e-8);

        assertEquals(1.0, A(0,0), 1e-8);
        assertEquals(10.0, A(0,1), 1e-8);
        assertEquals(sqrt(k*100.0), A(1,0), 1e-8);
        assertEquals(sqrt(k/100.0)*1000.0, A(1,1), 1e-8);
    }

    @Test
    void testrobustNoiseGemanMcClure() throws Throwable {
        final double k = 1.0;
        final double error1 = 1.0;
        final double error2 = 100.0;
        final double a00 = 1.0;
        final double a01 = 10.0;
        final double a10 = 100.0;
        final double a11 = 1000.0;
        Matrix A = new Matrix(new double[][] { { a00, a01 }, { a10, a11 } });
        Vector b = new Vector(new double[] { error1, error2 });
        final shared_ptr<Robust> robust = Robust.Create(
        GemanMcClure.Create(k, GemanMcClure::Scalar),
        Unit::Create(2));

        robust.get().WhitenSystem(A, b);

        final double k2 = k*k;
        final double k4 = k2*k2;
        final double k2error = k2 + error2*error2;

        final double sqrt_weight_error1 = sqrt(0.25);
        final double sqrt_weight_error2 = sqrt(k4/(k2error*k2error));

        assertEquals(sqrt_weight_error1*error1, b(0), 1e-8);
        assertEquals(sqrt_weight_error2*error2, b(1), 1e-8);

        assertEquals(sqrt_weight_error1*a00, A(0,0), 1e-8);
        assertEquals(sqrt_weight_error1*a01, A(0,1), 1e-8);
        assertEquals(sqrt_weight_error2*a10, A(1,0), 1e-8);
        assertEquals(sqrt_weight_error2*a11, A(1,1), 1e-8);
    }

    @Test
    void testrobustNoiseTLS() throws Throwable {
        final double k = 1.0;
        final double error1 = 1.0;
        final double error2 = 100.0;
        final double a00 = 1.0;
        final double a01 = 10.0;
        final double a10 = 100.0;
        final double a11 = 1000.0;
        Matrix A = new Matrix(new double[][] { { a00, a01 }, { a10, a11 } });
        Vector b = new Vector(new double[] { error1, error2 });
        final shared_ptr<Robust> robust = Robust.Create(
        TruncatedLeastSquares::Create(k,
        TruncatedLeastSquares::Scalar),
        Unit::Create(2));

        robust.get().WhitenSystem(A, b);

        final double sqrt_weight_error1 = 1.0;
        final double sqrt_weight_error2 = 0.0;

        assertEquals(sqrt_weight_error1*error1, b(0), 1e-8);
        assertEquals(sqrt_weight_error2*error2, b(1), 1e-8);

        assertEquals(sqrt_weight_error1*a00, A(0,0), 1e-8);
        assertEquals(sqrt_weight_error1*a01, A(0,1), 1e-8);
        assertEquals(sqrt_weight_error2*a10, A(1,0), 1e-8);
        assertEquals(sqrt_weight_error2*a11, A(1,1), 1e-8);
    }

    @Test
    void testrobustNoiseDCS() throws Throwable {
        final double k = 1.0;
        final double error1 = 1.0;
        final double error2 = 100.0;
        final double a00 = 1.0;
        final double a01 = 10.0;
        final double a10 = 100.0;
        final double a11 = 1000.0;
        Matrix A = new Matrix(new double[][] { { a00, a01 }, { a10, a11 } });
        Vector b = new Vector(new double[] { error1, error2 });
        final shared_ptr<Robust> robust = Robust.Create(
                DCS.Create(k, DCS::Scalar),
                Unit.Create(2));

        robust.get().WhitenSystem(A, b);

        final double sqrt_weight = 2.0 * k / (k + error2 * error2);

        assertEquals(error1, b(0), 1e-8);
        assertEquals(sqrt_weight * error2, b(1), 1e-8);

        assertEquals(a00, A(0, 0), 1e-8);
        assertEquals(a01, A(0, 1), 1e-8);
        assertEquals(sqrt_weight * a10, A(1, 0), 1e-8);
        assertEquals(sqrt_weight * a11, A(1, 1), 1e-8);
    }

    @Test
    void testrobustNoiseL2WithDeadZone() {
        double dead_zone_size = 1.0;
        var robust = Robust.Create(
        L2WithDeadZone.Create(dead_zone_size),
        Unit::Create(3));

        for (int i = 0; i < 5; i++) {
            Vector error = Vector3(i, 0, 0);
            robust.get().WhitenSystem(error);
            assertEquals(std::fmax(0, i - dead_zone_size) * i,
            robust.get().squaredMahalanobisDistance(error), 1e-8);
        }
    }

    @Test
    void testlossFunctionAtZero() {
        final double k = 5.0;
        var fair = mEstimator::Fair::Create(k);
        assertEquals(fair.get().loss(0), 0, 1e-8);
        assertEquals(fair.get().weight(0), 1, 1e-8);
        var huber = mEstimator::Huber::Create(k);
        assertEquals(huber.get().loss(0), 0, 1e-8);
        assertEquals(huber.get().weight(0), 1, 1e-8);
        var cauchy = mEstimator::Cauchy::Create(k);
        assertEquals(cauchy.get().loss(0), 0, 1e-8);
        assertEquals(cauchy.get().weight(0), 1, 1e-8);
        var gmc = mEstimator::GemanMcClure::Create(k);
        assertEquals(gmc.get().loss(0), 0, 1e-8);
        assertEquals(gmc.get().weight(0), 1, 1e-8);
        var welsch = mEstimator::Welsch::Create(k);
        assertEquals(welsch.get().loss(0), 0, 1e-8);
        assertEquals(welsch.get().weight(0), 1, 1e-8);
        var tukey = mEstimator::Tukey::Create(k);
        assertEquals(tukey.get().loss(0), 0, 1e-8);
        assertEquals(tukey.get().weight(0), 1, 1e-8);
        var dcs = mEstimator::DCS::Create(k);
        assertEquals(dcs.get().loss(0), 0, 1e-8);
        assertEquals(dcs.get().weight(0), 1, 1e-8);
        var lsdz = mEstimator::L2WithDeadZone::Create(k);
        assertEquals(lsdz.get().loss(0), 0, 1e-8);
        assertEquals(lsdz.get().weight(0), 0, 1e-8);
        var assy_cauchy = mEstimator::AsymmetricCauchy::Create(k);
        assertEquals(lsdz.get().loss(0), 0, 1e-8);
        assertEquals(lsdz.get().weight(0), 0, 1e-8);
        var assy_tukey = mEstimator::AsymmetricTukey::Create(k);
        assertEquals(lsdz.get().loss(0), 0, 1e-8);
        assertEquals(lsdz.get().weight(0), 0, 1e-8);
    }

    static void TEST_GAUSSIAN(shared_ptr<Gaussian> gaussian) throws Throwable {
        Matrix R = new Matrix(new double[][] {
                { 6, 5, 4 },
                { 0, 3, 2 },
                { 0, 0, 1 }
        });
        Matrix info = R.transpose().compose(R);
        Matrix cov = info.inverse();
        Vector e = new Vector(new double[] { 1, 1, 1 });
        Vector white = R.times(e);
        assertTrue(assert_equal(info, gaussian.get().information()));
        assertTrue(assert_equal(cov, gaussian.get().covariance()));
        assertTrue(assert_equal(white, gaussian.get().whiten(e)));
        assertTrue(assert_equal(e, gaussian.get().unwhiten(white)));
        assertEquals(251.0, gaussian.get().squaredMahalanobisDistance(e), 1e-9);
        assertEquals(0.5 * 251.0, gaussian.get().loss(251.0), 1e-9);
        Matrix A = R.inverse();
        Vector b = e;
        gaussian.get().WhitenSystem(A, b);
        Matrix I = Matrix.I_3x3();
        assertTrue(assert_equal(I, A));
        assertTrue(assert_equal(white, b));
    }

    @Test
    void testNonDiagonalGaussian() throws Throwable {
        Matrix R = new Matrix(new double[][] {
                { 6, 5, 4 },
                { 0, 3, 2 },
                { 0, 0, 1 }
        });
        Matrix info = R.transpose().compose(R);
        Matrix cov = info.inverse();

        {
            shared_ptr<Gaussian> gaussian = Gaussian.SqrtInformation(R, true);
            TEST_GAUSSIAN(gaussian);
        }

        {
            shared_ptr<Gaussian> gaussian = Gaussian.Information(info, true);
            TEST_GAUSSIAN(gaussian);
        }

        {
            shared_ptr<Gaussian> gaussian = Gaussian.Covariance(cov, true);
            TEST_GAUSSIAN(gaussian);
        }
    }

    @Test
    void testNegLogNormalizationConstant1D() throws Throwable {
        // Very simple 1D noise model, which we can compute by hand.
        double sigma = 0.1;
        // For expected values, we compute -log(1/sqrt(|2πΣ|)) by hand.
        // = 0.5*(log(2π) - log(Σ)) (since it is 1D)
        double expected_value = 0.5 * Math.log(2 * Math.PI * sigma * sigma);

        // Gaussian
        {
            Matrix R = new Matrix(new double[][] { { 1 / sigma } });
            var noise_model = Gaussian.SqrtInformation(R, true);
            double actual_value = noise_model.get().negLogConstant();
            assertEquals(expected_value, actual_value, 1e-9);
        }
        // Diagonal
        {
            shared_ptr<Diagonal> noise_model = Diagonal.Sigmas(new Vector(new double[] { sigma }));
            double actual_value = noise_model.get().negLogConstant();
            assertEquals(expected_value, actual_value, 1e-9);
        }
        // Isotropic
        {
            shared_ptr<Isotropic> noise_model = Isotropic.Sigma(1, sigma, true);
            double actual_value = noise_model.get().negLogConstant();
            assertEquals(expected_value, actual_value, 1e-9);
        }
        // Unit
        {
            shared_ptr<Unit> noise_model = Unit.Create(1);
            double actual_value = noise_model.get().negLogConstant();
            double sigma2 = 1.0;
            expected_value = 0.5 * Math.log(2 * Math.PI * sigma2 * sigma2);
            assertEquals(expected_value, actual_value, 1e-9);
        }
    }

    @Test
    void testNegLogNormalizationConstant3D() throws Throwable {
        // Simple 3D noise model, which we can compute by hand.
        double sigma = 0.1;
        int n = 3;
        // We compute the expected values just like in the NegLogNormalizationConstant1D
        // test, but we multiply by 3 due to the determinant.
        double expected_value = 0.5 * n * Math.log(2 * Math.PI * sigma * sigma);

        // Gaussian
        {
            Matrix R = new Matrix(new double[][] {
                    { 1 / sigma, 2, 3 }, //
                    { 0, 1 / sigma, 4 }, //
                    { 0, 0, 1 / sigma } });
            shared_ptr<Gaussian> noise_model = Gaussian.SqrtInformation(R, true);
            double actual_value = noise_model.get().negLogConstant();
            assertEquals(expected_value, actual_value, 1e-9);
        }
        // Diagonal
        {
            var noise_model = Diagonal.Sigmas(new Vector3(sigma, sigma, sigma));
            double actual_value = noise_model.get().negLogConstant();
            assertEquals(expected_value, actual_value, 1e-9);
        }
        // Isotropic
        {
            shared_ptr<Isotropic> noise_model = Isotropic.Sigma(n, sigma, true);
            double actual_value = noise_model.get().negLogConstant();
            assertEquals(expected_value, actual_value, 1e-9);
        }
        // Unit
        {
            shared_ptr<Unit> noise_model = Unit.Create(3);
            double actual_value = noise_model.get().negLogConstant();
            double sigma2 = 1.0;
            expected_value = 0.5 * n * Math.log(2 * Math.PI * sigma2 * sigma2);
            assertEquals(expected_value, actual_value, 1e-9);
        }
    }

    // Negative sigma values should throw (#695)
    @Test
    void testNegativeSigmaThrows() {
        assertThrows(IllegalArgumentException.class, () -> Isotropic.Sigma(2, -2.0, true));
        assertThrows(IllegalArgumentException.class, () -> Isotropic.Variance(2, -1.0, true));
        assertThrows(IllegalArgumentException.class, () -> Diagonal.Sigmas(new Vector3(-1.0, 2.0, 3.0)));
    }

}
