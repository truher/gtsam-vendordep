package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import gtsam.NumericalDerivative.ThrowingFunction2;
import gtsam.noiseModel.Isotropic;

/**
 * I removed the actual jacobian testing since it was kinda complicated.
 */
public class BetweenFactorTest {


    @Test
    void testRot3() throws Throwable {
        Rot3 R1 = Rot3.Rodrigues(0.1, 0.2, 0.3);
        Rot3 R2 = Rot3.Rodrigues(0.4, 0.5, 0.6);
        Rot3 noise = new Rot3();
        Rot3 measured = R1.between(R2).compose(noise);

        shared_ptr<BetweenFactorRot3> factor = BetweenFactorRot3.newBetweenFactorRot3(
                Key.R(1), Key.R(2), measured, Isotropic.Sigma(3, 0.05, true));
        Matrix actualH1 = new Matrix();
        Matrix actualH2 = new Matrix();
        Vector3 actual = factor.get().evaluateError(R1, R2, actualH1, actualH2);

        Vector3 expected = Rot3.companion.Logmap(measured.inverse().compose(R1.between(R2)));
        // Uncomment to make unit test fail
        assertTrue(assert_equal(expected, actual/* , 1e-100 */));

        ThrowingFunction2<Rot3, Rot3, Vector3> f = (a, b) -> factor.get().evaluateError(a, b);
        Matrix numericalH1 = NumericalDerivative.<//
                Vector3, Vector3, //
                Rot3, Vector3, //
                Rot3, Vector3>numericalDerivative21(f, R1, R2, 1e-5);
        assertTrue(assert_equal(numericalH1, actualH1, 1E-5));

        Matrix numericalH2 = NumericalDerivative.<//
                Vector3, Vector3, //
                Rot3, Vector3, //
                Rot3, Vector3>numericalDerivative22(f, R1, R2, 1e-5);
        assertTrue(assert_equal(numericalH2, actualH2, 1E-5));
    }

    @Test
    void testScalar() throws Throwable {
        shared_ptr<Isotropic> model = Isotropic.Sigma(1, 1, true);
        double measured = 0.0;
        shared_ptr<BetweenFactorDouble> factor = BetweenFactorDouble.newBetweenFactorDouble(
                new Key(1), new Key(2), measured, model);
        Vector1 error = factor.get().evaluateError(0, 1);
        assertTrue(assert_equal(new Vector1(1), error, 1e-9));
    }

    @Test
    void testVector3() throws Throwable {
        shared_ptr<Isotropic> model = Isotropic.Sigma(3, 1.0, true);
        Vector3 measured = new Vector3(1, 2, 3);
        shared_ptr<BetweenFactorVector3> factor = BetweenFactorVector3.newBetweenFactorVector3(
                new Key(1), new Key(2), measured, model);
        Vector3 error = factor.get().evaluateError(
                new Vector3(0, 0, 0), new Vector3(1, 2, 3));
        assertTrue(assert_equal(new Vector3(0, 0, 0), error, 1e-9));
    }

    /** Note this throws in C++ if the sizes are wrong, so be careful. */
    @Test
    void testDynamicSizeVector() throws Throwable {
        shared_ptr<Isotropic> model = Isotropic.Sigma(5, 1.0, true);
        Vector measured = new Vector(new double[] { 1, 2, 3, 4, 5 });
        shared_ptr<BetweenFactorVector> factor = BetweenFactorVector.newBetweenFactorVector(
                new Key(1), new Key(2), measured, model);
        Vector error = factor.get().evaluateError(
                new Vector(new double[] { 0, 0, 0, 0, 0 }), new Vector(new double[] { 1, 2, 3, 4, 5 }));
        assertTrue(assert_equal(new Vector(new double[] { 0, 0, 0, 0, 0 }), error, 1e-9));
    }

    @Test
    void testPoint3() throws Throwable {
        shared_ptr<Isotropic> model = Isotropic.Sigma(3, 1.0, true);
        Point3 measured = new Point3(1, 2, 3);
        shared_ptr<BetweenFactorPoint3> factor = BetweenFactorPoint3.newBetweenFactorPoint3(//
                new Key(1), new Key(2), measured, model);
        Vector3 error = factor.get().evaluateError(
                new Point3(0, 0, 0), new Point3(1, 2, 3));
        assertTrue(assert_equal(new Vector3(0, 0, 0), error, 1e-9));
    }

    @Test
    void testRot3Again() throws Throwable {
        shared_ptr<Isotropic> model = Isotropic.Sigma(3, 1.0, true);
        Rot3 measured = Rot3.Ry(Math.PI / 2);
        shared_ptr<BetweenFactorRot3> factor = BetweenFactorRot3.newBetweenFactorRot3(
                new Key(1), new Key(2), measured, model);
        Vector3 error = factor.get().evaluateError(
                new Rot3(), Rot3.Ry(Math.PI / 2));
        assertTrue(assert_equal(new Vector3(0, 0, 0), error, 1e-9));
    }

    @Test
    void testPose3() throws Throwable {
        shared_ptr<Isotropic> model = Isotropic.Sigma(6, 1.0, true);
        Pose3 measured = new Pose3(new Rot3(), new Point3(1, 2, 3));
        shared_ptr<BetweenFactorPose3> factor = BetweenFactorPose3.newBetweenFactorPose3(
                new Key(1), new Key(2), measured, model);
        Vector6 error = factor.get().evaluateError(
                new Pose3(), new Pose3(new Rot3(), new Point3(1, 2, 3)));
        assertTrue(assert_equal(new Vector6(0, 0, 0, 0, 0, 0), error, 1e-9));
    }

}
