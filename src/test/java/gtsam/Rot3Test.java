package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.List;

import org.junit.jupiter.api.Test;

import gtsam.NumericalDerivative.ThrowingFunction;
import gtsam.NumericalDerivative.ThrowingFunction2;

/**
 * See gtsam/geometry/tests/testRot3.cpp
 * 
 * I skipped the SO3 parts.
 */
public class Rot3Test {

    static Rot3 R;
    static Point3 P;
    static double error;
    static double epsilon;

    static {
        try {
            R = Rot3.Rodrigues(0.1, 0.4, 0.2);
            P = new Point3(0.2, 0.7, -2.0);
            error = 1e-9;
            epsilon = 0.001;
        } catch (Throwable e) {
            throw new RuntimeException(e);
        }
    }

    @Test
    void testchart() throws Throwable {
        Matrix3 R = new Matrix3(//
                0, 1, 0, //
                1, 0, 0, //
                0, 0, -1);
        new Rot3(R);
    }

    @Test
    void testfinalructor() throws Throwable {
        Rot3 expected = new Rot3(Matrix3.identity());
        Point3 r1 = new Point3(1, 0, 0);
        Point3 r2 = new Point3(0, 1, 0);
        Point3 r3 = new Point3(0, 0, 1);
        Rot3 actual = new Rot3(r1, r2, r3);
        assertTrue(assert_equal(actual, expected));
    }

    @Test
    void testfinalructor2() throws Throwable {
        Matrix3 R = new Matrix3(//
                0, 1, 0, //
                1, 0, 0, //
                0, 0, -1);
        Rot3 actual = new Rot3(R);
        Rot3 expected = new Rot3(//
                0, 1, 0, //
                1, 0, 0, //
                0, 0, -1);
        assertTrue(assert_equal(actual, expected));
    }

    @Test
    void testfinalructor3() throws Throwable {
        Rot3 expected = new Rot3(//
                0, 1, 0, //
                1, 0, 0, //
                0, 0, -1);
        Point3 r1 = new Point3(0, 1, 0);
        Point3 r2 = new Point3(1, 0, 0);
        Point3 r3 = new Point3(0, 0, -1);
        assertTrue(assert_equal(expected, new Rot3(r1, r2, r3)));
    }

    @Test
    void testtranspose() throws Throwable {
        Point3 r1 = new Point3(0, 1, 0);
        Point3 r2 = new Point3(1, 0, 0);
        Point3 r3 = new Point3(0, 0, -1);
        Rot3 R = new Rot3(//
                0, 1, 0, //
                1, 0, 0, //
                0, 0, -1);
        assertTrue(assert_equal(R.inverse(), new Rot3(r1, r2, r3)));
    }

    @Test
    void testequals() throws Throwable {
        assertTrue(R.equals(R));
        Rot3 zero = new Rot3();
        assertTrue(!R.equals(zero));
    }

    @Test
    void testAxisAngle() throws Throwable {
        // rotation around Y
        Point3 axis = new Point3(0., 1., 0.);
        double angle = 3.14 / 4.0;
        Rot3 expected = new Rot3(//
                0.707388, 0, 0.706825, //
                0, 1, 0, //
                -0.706825, 0, 0.707388);
        Rot3 actual = Rot3.AxisAngle(axis, angle);
        assertTrue(assert_equal(expected, actual, 1e-5));
        Rot3 actual2 = Rot3.AxisAngle(axis, angle - 2 * Math.PI);
        assertTrue(assert_equal(expected, actual2, 1e-5));

        axis = new Point3(0, 50, 0);
        Rot3 actual3 = Rot3.AxisAngle(axis, angle);
        assertTrue(assert_equal(expected, actual3, 1e-5));
    }

    @Test
    void testAxisAngle2() throws Throwable {
        // finalructor from a rotation matrix, as doubles in *row-major* order.
        Rot3 R1 = new Rot3(//
                -0.999957, 0.00922903, 0.00203116, //
                0.00926964, 0.999739, 0.0208927, //
                -0.0018374, 0.0209105, -0.999781);

        // convert Rot3 to quaternion using GTSAM
        Pair<Unit3, Double> aa = R1.axisAngle();
        double actualAngle = aa.second;
        double expectedAngle = 3.1396582;
        assertTrue(assert_equal(expectedAngle, actualAngle, 1e-5));
    }

    // Notice this uses J^2 whereas fast uses w*w', and has cos(t)*I + ....
    Rot3 slow_but_correct_Rodrigues(Point3 p) throws Throwable {
        Vector3 w = new Vector3(p);
        double t = w.norm();
        Matrix3 J = Matrix3.skewSymmetric(w.times(1 / t));
        if (t < 1e-5)
            return new Rot3();
        Matrix3 R = Matrix3.identity()
                .plus(J.times(Math.sin(t))
                        .plus((J.compose(J)).times(1 - Math.cos(t))));
        return new Rot3(R);
    }

    @Test
    void testRodrigues() throws Throwable {
        Rot3 R1 = Rot3.Rodrigues(epsilon, 0, 0);
        Point3 w = new Point3(epsilon, 0., 0.);
        Rot3 R2 = slow_but_correct_Rodrigues(w);
        assertTrue(assert_equal(R2, R1));
    }

    @Test
    void testRodrigues2() throws Throwable {
        // rotation around Y
        Point3 axis = new Point3(0., 1., 0.);
        double angle = 3.14 / 4.0;
        Rot3 expected = new Rot3(//
                0.707388, 0, 0.706825, //
                0, 1, 0, //
                -0.706825, 0, 0.707388);
        Rot3 actual = Rot3.AxisAngle(axis, angle);
        assertTrue(assert_equal(expected, actual, 1e-5));
        Rot3 actual2 = Rot3.Rodrigues(axis.times(angle));
        assertTrue(assert_equal(expected, actual2, 1e-5));
    }

    @Test
    void testRodrigues3() throws Throwable {
        Vector3 w = new Vector3(0.1, 0.2, 0.3);
        Rot3 R1 = Rot3.AxisAngle(new Point3(w.times(1 / w.norm())), w.norm());
        Rot3 R2 = slow_but_correct_Rodrigues(new Point3(w));
        assertTrue(assert_equal(R2, R1));
    }

    @Test
    void testRodrigues4() throws Throwable {
        // rotation around Z
        Point3 axis = new Point3(0., 0., 1.);
        double angle = Math.PI / 2.0;
        Rot3 actual = Rot3.AxisAngle(axis, angle);
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        Rot3 expected = new Rot3(
                c, -s, 0,
                s, c, 0,
                0, 0, 1);
        assertTrue(assert_equal(expected, actual));
        assertTrue(assert_equal(slow_but_correct_Rodrigues(axis.times(angle)), actual));
    }

    @Test
    void testretract() throws Throwable {
        Vector3 v = new Vector3();
        assertTrue(assert_equal(R, R.retract(v)));
    }

    record RetractNormalizationMetrics(
            double quaternionNormError,
            double orthogonalityError,
            double determinantError) {
    }

    RetractNormalizationMetrics measureRetractNormalization(
            final Rot3 base,
            final Vector3 omega) throws Throwable {
        final Rot3 retracted = base.retract(omega);
        final Matrix3 matrix = retracted.matrix();
        return new RetractNormalizationMetrics(
                Math.abs(retracted.toQuaternion().norm() - 1.0),
                (matrix.transpose().compose(matrix)
                        .plus(Matrix3.identity().times(-1))).norm(),
                Math.abs(matrix.determinant() - 1.0));
    }

    @Test
    void testretractNormalizationAcrossMagnitudes() throws Throwable {
        final Vector3 direction = new Vector3(1.0, -2.0, 3.0).normalized();
        List<Rot3> bases = List.of(
                new Rot3(),
                Rot3.RzRyRx(0.3, -0.2, 0.5));
        List<Double> magnitudes = List.of(
                0.0, 1e-16, 1e-14, 1e-12, 1e-10, 1e-8, 1e-6, 1e-4, 1e-2,
                1.0, 1e2, 1e4, 1e6, 1e8, 1e10, 1e12, 1e14);
        double quaternionNormTolerance = 1e-12;
        double orthogonalityTolerance = 1e-10;
        double determinantTolerance = 1e-10;

        for (int baseIndex = 0; baseIndex < bases.size(); ++baseIndex) {
            for (double magnitude : magnitudes) {
                var metrics = measureRetractNormalization(
                        bases.get(baseIndex), direction.times(magnitude));
                boolean quaternionOk = metrics.quaternionNormError <= quaternionNormTolerance;
                boolean orthogonalityOk = metrics.orthogonalityError <= orthogonalityTolerance;
                boolean determinantOk = metrics.determinantError <= determinantTolerance;
                if (quaternionOk && orthogonalityOk && determinantOk)
                    continue;

                StringBuilder b = new StringBuilder();
                b.append("Retract normalization broke down for base[");
                b.append(baseIndex);
                b.append("] at |omega|=");
                b.append(magnitude);
                b.append(" with quaternion norm error=");
                b.append(metrics.quaternionNormError);
                b.append(", orthogonality error=");
                b.append(metrics.orthogonalityError);
                b.append(", determinant error=");
                b.append(metrics.determinantError);
                fail(b.toString());
            }
        }
    }

    void CHECK_OMEGA(double X, double Y, double Z) throws Throwable {
        Point3 w = new Point3(X, Y, Z);
        Rot3 R = Rot3.Rodrigues(w);
        assertTrue(assert_equal(w, new Point3(new Rot3().logmap(R)), 1e-12));
    }

    void CHECK_OMEGA_ZERO(double X, double Y, double Z) throws Throwable {
        Point3 w = new Point3(X, Y, Z);
        Rot3 R = Rot3.Rodrigues(w);
        assertTrue(assert_equal(new Vector3(), new Rot3().logmap(R)));
    }

    @Test
    void testlog() throws Throwable {
        final double PI = Math.acos(-1.0);

        // Check zero
        CHECK_OMEGA(0, 0, 0);

        // create a random direction:
        double norm = Math.sqrt(1.0 + 16.0 + 4.0);
        double x = 1.0 / norm;
        double y = 4.0 / norm;
        double z = 2.0 / norm;

        // Check very small rotation for Taylor expansion
        // Note that tolerance above is 1e-12, so Taylor is pretty good !
        double d = 0.0001;
        CHECK_OMEGA(d, 0, 0);
        CHECK_OMEGA(0, d, 0);
        CHECK_OMEGA(0, 0, d);
        CHECK_OMEGA(x * d, y * d, z * d);

        // check normal rotation
        d = 0.1;
        CHECK_OMEGA(d, 0, 0);
        CHECK_OMEGA(0, d, 0);
        CHECK_OMEGA(0, 0, d);
        CHECK_OMEGA(x * d, y * d, z * d);

        // Check 180 degree rotations
        CHECK_OMEGA(PI, 0, 0);
        CHECK_OMEGA(0, PI, 0);
        CHECK_OMEGA(0, 0, PI);

        // Windows and Linux have flipped sign in quaternion mode
        // #if !defined(__APPLE__) && defined(GTSAM_USE_QUATERNIONS)
        Point3 w = new Point3(x * PI, y * PI, z * PI);
        Rot3 R = Rot3.Rodrigues(w);
        assertTrue(assert_equal(new Vector3(w.times(-1)), new Rot3().logmap(R), 1e-12));
        // #else
        // CHECK_OMEGA(x * PI, y * PI, z * PI);
        // #endif

        // Check 360 degree rotations

        CHECK_OMEGA_ZERO(2.0 * PI, 0, 0);
        CHECK_OMEGA_ZERO(0, 2.0 * PI, 0);
        CHECK_OMEGA_ZERO(0, 0, 2.0 * PI);
        CHECK_OMEGA_ZERO(x * 2. * PI, y * 2. * PI, z * 2. * PI);

        // Check problematic case from Lund dataset vercingetorix.g2o
        // This is an almost rotation with determinant not *quite* 1.
        Rot3 Rlund = new Rot3(-0.98582676, -0.03958746, -0.16303092, //
                -0.03997006, -0.88835923, 0.45740671, //
                -0.16293753, 0.45743998, 0.87418537);

        // Rot3's Logmap returns different, but equivalent compacted
        // axis-angle vectors depending on whether Rot3 is implemented
        // by Quaternions or SO3.
        // Note, quaternions are off by default
        // #if defined(GTSAM_USE_QUATERNIONS)
        // Quaternion bounds angle to [-pi, pi] resulting in ~179.9 degrees
        // assertTrue(assert_equal(Vector3(0.264451979, -0.742197651, -3.04098211),
        // (Vector)Rot3::Logmap(Rlund), 1e-8));
        // #else
        // SO3 will be approximate because of the non-orthogonality
        assertTrue(assert_equal(new Vector3(0.264452, -0.742197708, -3.04098184),
                new Rot3().logmap(Rlund), 1e-8));
        // #endif
    }

    @Test
    void testretract_localCoordinates() throws Throwable {
        Vector3 d12 = new Vector3(0.1, 0.1, 0.1);
        Rot3 R2 = R.retract(d12);
        assertTrue(assert_equal(d12, R.local(R2)));
    }

    @Test
    void testexpmap_logmap() throws Throwable {
        Vector3 d12 = new Vector3(0.1, 0.1, 0.1);
        Rot3 R2 = R.expmap(d12);
        assertTrue(assert_equal(d12, R.logmap(R2)));
    }

    @Test
    void testretract_localCoordinates2() throws Throwable {
        Rot3 t1 = R;
        Rot3 t2 = R.compose(R);
        Vector3 d12 = t1.local(t2);
        assertTrue(assert_equal(t2, t1.retract(d12)));
        Vector3 d21 = t2.local(t1);
        assertTrue(assert_equal(t1, t2.retract(d21)));
        assertTrue(assert_equal(d12, d21.times(-1.0)));
    }

    @Test
    void testmanifold_expmap() throws Throwable {
        Rot3 gR1 = Rot3.Rodrigues(0.1, 0.4, 0.2);
        Rot3 gR2 = Rot3.Rodrigues(0.3, 0.1, 0.7);

        // log behaves correctly
        Vector3 d12 = new Rot3().logmap(gR1.between(gR2));
        Vector3 d21 = new Rot3().logmap(gR2.between(gR1));

        // Check expmap
        assertTrue(assert_equal(gR2, gR1.compose(new Rot3().expmap(d12))));
        assertTrue(assert_equal(gR1, gR2.compose(new Rot3().expmap(d21))));

        // Check that log(t1,t2)=-log(t2,t1)
        assertTrue(assert_equal(d12, d21.times(-1)));

        // lines in canonical coordinates correspond to Abelian subgroups in SO(3)
        Vector3 d = new Vector3(0.1, 0.2, 0.3);
        // exp(-d)=inverse(exp(d))
        assertTrue(assert_equal(new Rot3().expmap(d.times(-1.0)),
                new Rot3().expmap(d).inverse()));
        // exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
        Rot3 R2 = new Rot3().expmap(d.times(2));
        Rot3 R3 = new Rot3().expmap(d.times(3));
        Rot3 R5 = new Rot3().expmap(d.times(5));
        assertTrue(assert_equal(R5, R2.compose(R3)));
        assertTrue(assert_equal(R5, R3.compose(R2)));
    }

    @Test
    void testrotate_derivatives() throws Throwable {
        Matrix actualDrotate1a = new Matrix();
        Matrix actualDrotate1b = new Matrix();
        Matrix actualDrotate2 = new Matrix();
        R.rotate(P, actualDrotate1a, actualDrotate2);
        R.inverse().rotate(P, actualDrotate1b, new Matrix());
        ThrowingFunction2<Rot3, Point3, Point3> rotate = (a, b) -> a.rotate(b);
        Matrix numerical1 = NumericalDerivative.<//
                Point3, Vector3, //
                Rot3, Vector3, //
                Point3, Vector3>numericalDerivative21(rotate, R, P, 1e-5);
        Matrix numerical2 = NumericalDerivative.<//
                Point3, Vector3, //
                Rot3, Vector3, //
                Point3, Vector3>numericalDerivative21(rotate, R.inverse(), P, 1e-5);
        Matrix numerical3 = NumericalDerivative.<//
                Point3, Vector3, //
                Rot3, Vector3, //
                Point3, Vector3>numericalDerivative22(rotate, R, P, 1e-5);
        assertTrue(assert_equal(numerical1, actualDrotate1a, error));
        assertTrue(assert_equal(numerical2, actualDrotate1b, error));
        assertTrue(assert_equal(numerical3, actualDrotate2, error));
    }

    @Test
    void testunrotate() throws Throwable {
        Point3 w = R.rotate(P);
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        Point3 actual = R.unrotate(w, H1, H2);
        assertTrue(assert_equal(P, actual));

        ThrowingFunction2<Rot3, Point3, Point3> unrotate = (a, b) -> a.unrotate(b);
        Matrix numerical1 = NumericalDerivative.<//
                Point3, Vector3, //
                Rot3, Vector3, //
                Point3, Vector3>numericalDerivative21(unrotate, R, w, 1e-5);
        assertTrue(assert_equal(numerical1, H1, error));

        Matrix numerical2 = NumericalDerivative.<//
                Point3, Vector3, //
                Rot3, Vector3, //
                Point3, Vector3>numericalDerivative22(unrotate, R, w, 1e-5);
        assertTrue(assert_equal(numerical2, H2, error));
    }

    @Test
    void testcompose() throws Throwable {
        Rot3 R1 = Rot3.Rodrigues(0.1, 0.2, 0.3);
        Rot3 R2 = Rot3.Rodrigues(0.2, 0.3, 0.5);

        Rot3 expected = R1.compose(R2);
        Matrix actualH1 = new Matrix();
        Matrix actualH2 = new Matrix();
        Rot3 actual = R1.compose(R2, actualH1, actualH2);
        assertTrue(assert_equal(expected, actual));

        ThrowingFunction2<Rot3, Rot3, Rot3> compose = (a, b) -> a.compose(b);
        Matrix numericalH1 = NumericalDerivative.<//
                Rot3, Vector3, //
                Rot3, Vector3, //
                Rot3, Vector3>numericalDerivative21(compose, R1,
                        R2, 1e-2);
        assertTrue(assert_equal(numericalH1, actualH1));

        Matrix numericalH2 = NumericalDerivative.<//
                Rot3, Vector3, //
                Rot3, Vector3, //
                Rot3, Vector3>numericalDerivative22(compose, R1,
                        R2, 1e-2);
        assertTrue(assert_equal(numericalH2, actualH2));
    }

    @Test
    void testinverse() throws Throwable {
        Rot3 R = Rot3.Rodrigues(0.1, 0.2, 0.3);
        Rot3 I = new Rot3();
        Matrix actualH = Matrix.I_3x3();
        Rot3 actual = R.inverse(actualH);
        assertTrue(assert_equal(I, R.compose(actual)));
        assertTrue(assert_equal(I, actual.compose(R)));
        assertTrue(assert_equal(actual.matrix(), R.transpose()));

        ThrowingFunction<Rot3, Rot3> h = (r) -> r.inverse();
        Matrix numericalH = NumericalDerivative.<//
                Rot3, Vector3, //
                Rot3, Vector3//
        >numericalDerivative11(h, R, 1e-3);
        assertTrue(assert_equal(numericalH, actualH));
    }

    @Test
    void testbetween() throws Throwable {
        Rot3 r1 = Rot3.Rz(Math.PI / 3.0);
        // this seems not to be used in the C++ test too.
        // Rot3 r2 = Rot3.Rz(2.0 * Math.PI / 3.0);

        Matrix3 expectedr1 = new Matrix3(
                0.5, -Math.sqrt(3.0) / 2.0, 0.0, //
                Math.sqrt(3.0) / 2.0, 0.5, 0.0, //
                0.0, 0.0, 1.0);
        assertTrue(assert_equal(expectedr1, r1.matrix()));

        Rot3 R = Rot3.Rodrigues(0.1, 0.4, 0.2);
        Rot3 origin = new Rot3();
        assertTrue(assert_equal(R, origin.between(R)));
        assertTrue(assert_equal(R.inverse(), R.between(origin)));

        Rot3 R1 = Rot3.Rodrigues(0.1, 0.2, 0.3);
        Rot3 R2 = Rot3.Rodrigues(0.2, 0.3, 0.5);

        Rot3 expected = R1.inverse().compose(R2);
        Matrix actualH1 = new Matrix();
        Matrix actualH2 = new Matrix();
        Rot3 actual = R1.between(R2, actualH1, actualH2);
        assertTrue(assert_equal(expected, actual));

        ThrowingFunction2<Rot3, Rot3, Rot3> between = (a, b) -> a.between(b);

        Matrix numericalH1 = NumericalDerivative.<//
                Rot3, Vector3, //
                Rot3, Vector3, //
                Rot3, Vector3>numericalDerivative21(between, R1, R2, 1e-5);
        assertTrue(assert_equal(numericalH1, actualH1));

        Matrix numericalH2 = NumericalDerivative.<//
                Rot3, Vector3, //
                Rot3, Vector3, //
                Rot3, Vector3>numericalDerivative22(between, R1, R2, 1e-5);
        assertTrue(assert_equal(numericalH2, actualH2));
    }

    @Test
    void testxyz() throws Throwable {
        double t = 0.1;
        double st = Math.sin(t);
        double ct = Math.cos(t);

        // Make sure all counterclockwise
        // Diagrams below are all from from unchanging axis

        // z
        // | * Y=(ct,st)
        // x----y
        Rot3 expected1 = new Rot3(1, 0, 0, 0, ct, -st, 0, st, ct);
        assertTrue(assert_equal(expected1, Rot3.Rx(t)));

        // x
        // | * Z=(ct,st)
        // y----z
        Rot3 expected2 = new Rot3(ct, 0, st, 0, 1, 0, -st, 0, ct);
        assertTrue(assert_equal(expected2, Rot3.Ry(t)));

        // y
        // | X=* (ct,st)
        // z----x
        Rot3 expected3 = new Rot3(ct, -st, 0, st, ct, 0, 0, 0, 1);
        assertTrue(assert_equal(expected3, Rot3.Rz(t)));

        // Check compound rotation
        Rot3 expected = Rot3.Rz(0.3).compose(Rot3.Ry(0.2)).compose(Rot3.Rx(0.1));
        assertTrue(assert_equal(expected, Rot3.RzRyRx(0.1, 0.2, 0.3)));
    }

    @Test
    void testyaw_pitch_roll() throws Throwable {
        double t = 0.1;

        // yaw is around z axis
        assertTrue(assert_equal(Rot3.Rz(t), Rot3.Yaw(t)));

        // pitch is around y axis
        assertTrue(assert_equal(Rot3.Ry(t), Rot3.Pitch(t)));

        // roll is around x axis
        assertTrue(assert_equal(Rot3.Rx(t), Rot3.Roll(t)));

        // Check compound rotation
        Rot3 expected = Rot3.Yaw(0.1).compose(Rot3.Pitch(0.2)).compose(Rot3.Roll(0.3));
        assertTrue(assert_equal(expected, Rot3.Ypr(0.1, 0.2, 0.3)));

        assertTrue(assert_equal(new Vector3(0.1, 0.2, 0.3), expected.ypr()));
    }

    @Test
    void testRQ() throws Throwable {
        // Try RQ on a pure rotation
        Pair<Matrix3, Vector3> actualPair = Rot3.RQ(R.matrix());
        Matrix3 actualK = actualPair.first;
        Vector3 actual = actualPair.second;
        Vector3 expected = new Vector3(0.14715, 0.385821, 0.231671);
        assertTrue(assert_equal(Matrix3.identity(), actualK));
        assertTrue(assert_equal(expected, actual, 1e-6));

        // Try using xyz call, asserting that Rot3::RzRyRx(x,y,z).xyz()==[x;y;z]
        assertTrue(assert_equal(expected, R.xyz(), 1e-6));
        assertTrue(assert_equal(new Vector3(0.1, 0.2, 0.3), Rot3.RzRyRx(0.1, 0.2, 0.3).xyz()));

        // Try using ypr call, asserting that Rot3::Ypr(y,p,r).ypr()==[y;p;r]
        assertTrue(assert_equal(new Vector3(0.1, 0.2, 0.3), Rot3.Ypr(0.1, 0.2, 0.3).ypr()));
        assertTrue(assert_equal(new Vector3(0.3, 0.2, 0.1), Rot3.Ypr(0.1, 0.2, 0.3).rpy()));

        // Try ypr for pure yaw-pitch-roll matrices
        assertTrue(assert_equal(new Vector3(0.1, 0.0, 0.0), Rot3.Yaw(0.1).ypr()));
        assertTrue(assert_equal(new Vector3(0.0, 0.1, 0.0), Rot3.Pitch(0.1).ypr()));
        assertTrue(assert_equal(new Vector3(0.0, 0.0, 0.1), Rot3.Roll(0.1).ypr()));

        // Try RQ to recover calibration from 3*3 sub-block of projection matrix
        Matrix3 K = new Matrix3(//
                500.0, 0.0, 320.0, //
                0.0, 500.0, 240.0, //
                0.0, 0.0, 1.0);
        Matrix3 A = K.compose(R.matrix());
        Pair<Matrix3, Vector3> actualPair2 = Rot3.RQ(A);
        Matrix3 actualK2 = actualPair2.first;
        Vector3 actual2 = actualPair2.second;
        assertTrue(assert_equal(K, actualK2));
        assertTrue(assert_equal(expected, actual2, 1e-6));
    }

    @Test
    void testexpmapStability() throws Throwable {
        Vector3 w = new Vector3(78e-9, 5e-8, 97e-7);
        double theta = w.norm();
        double theta2 = theta * theta;
        Rot3 actualR = new Rot3().expmap(w);
        Matrix3 W = new Matrix3(//
                0.0, -w.at(2), w.at(1), //
                w.at(2), 0.0, -w.at(0), //
                -w.at(1), w.at(0), 0.0);
        Matrix3 W2 = W.compose(W);
        Matrix3 Rmat = Matrix3.identity().plus(
                W.times(1.0 - theta2 / 6.0 + theta2 * theta2 / 120.0 - theta2 * theta2 * theta2 / 5040.0))
                .plus(W2.times(0.5 - theta2 / 24.0 + theta2 * theta2 / 720.0));
        Rot3 expectedR = new Rot3(Rmat);
        assertTrue(assert_equal(expectedR, actualR, 1e-10));
    }

    @Test
    void testlogmapStability() throws Throwable {
        Vector3 w = new Vector3(1e-8, 0.0, 0.0);
        Rot3 R = new Rot3().expmap(w);
        Vector3 actualw = new Rot3().logmap(R);
        assertTrue(assert_equal(w, actualw, 1e-15));
    }

    @Test
    void testquaternion() throws Throwable {
        // NOTE: This is also verifying the ability to convert Vector to Quaternion
        Quaternion q1 = new Quaternion(//
                0.710997408193224, 0.360544029310185, 0.594459869568306, 0.105395217842782);
        Rot3 R1 = new Rot3(//
                0.271018623057411, 0.278786459830371, 0.921318086098018, //
                0.578529366719085, 0.717799701969298, -0.387385285854279, //
                -0.769319620053772, 0.637998195662053, 0.033250932803219);

        Quaternion q2 = new Quaternion(//
                0.263360579192421, 0.571813128030932, 0.494678363680335, 0.599136268678053);
        Rot3 R2 = new Rot3(
                -0.207341903877828, 0.250149415542075, 0.945745528564780, //
                0.881304914479026, -0.371869043667957, 0.291573424846290, //
                0.424630407073532, 0.893945571198514, -0.143353873763946);

        // Check creating Rot3 from quaternion
        assertTrue(assert_equal(R1, new Rot3(q1)));
        assertTrue(assert_equal(R1, Rot3.Quaternion(q1.w(), q1.x(), q1.y(), q1.z())));
        assertTrue(assert_equal(R2, new Rot3(q2)));
        assertTrue(assert_equal(R2, Rot3.Quaternion(q2.w(), q2.x(), q2.y(), q2.z())));

        // Check converting Rot3 to quaterion
        assertTrue(assert_equal(R1.toQuaternion().coeffs(), q1.coeffs()));
        assertTrue(assert_equal(R2.toQuaternion().coeffs(), q2.coeffs()));

        // Check that quaternion and Rot3 represent the same rotation
        Point3 p1 = new Point3(1.0, 2.0, 3.0);
        Point3 p2 = new Point3(8.0, 7.0, 9.0);

        Point3 expected1 = R1.rotate(p1);
        Point3 expected2 = R2.rotate(p2);

        Point3 actual1 = q1.rotate(p1);
        Point3 actual2 = q2.rotate(p2);

        assertTrue(assert_equal(expected1, actual1));
        assertTrue(assert_equal(expected2, actual2));
    }

    @Test
    void testConvertQuaternion() throws Throwable {
        Quaternion q = new Quaternion(1, 2, 3, 4);
        assertEquals(1, q.w(), 1e-9);
        assertEquals(2, q.x(), 1e-9);
        assertEquals(3, q.y(), 1e-9);
        assertEquals(4, q.z(), 1e-9);

        Rot3 R = new Rot3(q);
        assertEquals(1, R.toQuaternion().w(), 1e-9);
        assertEquals(2, R.toQuaternion().x(), 1e-9);
        assertEquals(3, R.toQuaternion().y(), 1e-9);
        assertEquals(4, R.toQuaternion().z(), 1e-9);
    }

    Matrix3 Cayley(final Matrix3 A) throws Throwable {
        Matrix3 I = Matrix3.identity();
        return I.minus(A).compose(I.plus(A).inverse());
    }

    @Test
    void testCayley() throws Throwable {
        Matrix3 A = Matrix3.skewSymmetric(new Vector3(1, 2, -3));
        Matrix3 Q = Cayley(A);
        assertTrue(assert_equal(Matrix3.identity(), Q.transpose().compose(Q)));
        assertTrue(assert_equal(A, Cayley(Q)));
    }

    @Test
    void testslerp() throws Throwable {
        // A first simple test
        Rot3 R1 = Rot3.Rz(1);
        Rot3 R2 = Rot3.Rz(2);
        Rot3 R3 = Rot3.Rz(1.5);
        assertTrue(assert_equal(R1, R1.slerp(0.0, R2)));
        assertTrue(assert_equal(R2, R1.slerp(1.0, R2)));
        assertTrue(assert_equal(R3, R1.slerp(0.5, R2)));
        // Make sure other can be *this
        assertTrue(assert_equal(R1, R1.slerp(0.5, R1)));
    }

    static Rot3 id;
    static Rot3 T1;
    static Rot3 T2;

    static {
        try {
            id = new Rot3();
            T1 = Rot3.AxisAngle(new Point3(0, 0, 1), 1);
            T2 = Rot3.AxisAngle(new Point3(0, 1, 0), 2);
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }
    }

    @Test
    void testInvariants() throws Throwable {
        assertTrue(Rot3.check_group_invariants(id, id));
        assertTrue(Rot3.check_group_invariants(id, T1));
        assertTrue(Rot3.check_group_invariants(T2, id));
        assertTrue(Rot3.check_group_invariants(T2, T1));
        assertTrue(Rot3.check_group_invariants(T1, T2));

        assertTrue(Rot3.check_manifold_invariants(id, id));
        assertTrue(Rot3.check_manifold_invariants(id, T1));
        assertTrue(Rot3.check_manifold_invariants(T2, id));
        assertTrue(Rot3.check_manifold_invariants(T2, T1));
        assertTrue(Rot3.check_manifold_invariants(T1, T2));
    }

    @Test
    void testClosestTo() throws Throwable {
        Matrix3 M = new Matrix3(
                0.79067393, 0.6051136, -0.0930814, //
                0.4155925, -0.64214347, -0.64324489, //
                -0.44948549, 0.47046326, -0.75917576);
        Matrix3 expected = new Matrix3(
                0.790687, 0.605096, -0.0931312, //
                0.415746, -0.642355, -0.643844, //
                -0.449411, 0.47036, -0.759468);
        Rot3 actual = Rot3.ClosestTo(M.times(3.0));
        assertTrue(assert_equal(expected, actual.matrix(), 1e-6));
    }

    void CHECK_AXIS_ANGLE(Unit3 expectedAxis, double expectedAngle, Rot3 rotation) throws Throwable {
        Pair<Unit3, Double> aa = rotation.axisAngle();
        Unit3 actualAxis = aa.first;
        double actualAngle = aa.second;
        assertTrue(assert_equal(expectedAxis, actualAxis, 1e-9));
        assertEquals(expectedAngle, actualAngle, 1e-9);
        assertTrue(assert_equal(rotation, Rot3.AxisAngle(expectedAxis, expectedAngle)));
    }

    @Test
    void testaxisAngle() throws Throwable {
        // CHECK R defined at top = Rot3::Rodrigues(0.1, 0.4, 0.2)
        Vector3 omega = new Vector3(0.1, 0.4, 0.2);
        Unit3 axis = new Unit3(omega);
        Unit3 _axis = new Unit3(omega.times(-1));
        CHECK_AXIS_ANGLE(axis, omega.norm(), R);

        // rotate by 90
        CHECK_AXIS_ANGLE(new Unit3(1, 0, 0), Math.PI / 2, Rot3.Ypr(0, 0, Math.PI / 2));
        CHECK_AXIS_ANGLE(new Unit3(0, 1, 0), Math.PI / 2, Rot3.Ypr(0, Math.PI / 2, 0));
        CHECK_AXIS_ANGLE(new Unit3(0, 0, 1), Math.PI / 2, Rot3.Ypr(Math.PI / 2, 0, 0));
        CHECK_AXIS_ANGLE(axis, Math.PI / 2, Rot3.AxisAngle(axis, Math.PI / 2));

        // rotate by -90
        CHECK_AXIS_ANGLE(new Unit3(-1, 0, 0), Math.PI / 2, Rot3.Ypr(0, 0, -Math.PI / 2));
        CHECK_AXIS_ANGLE(new Unit3(0, -1, 0), Math.PI / 2, Rot3.Ypr(0, -Math.PI / 2, 0));
        CHECK_AXIS_ANGLE(new Unit3(0, 0, -1), Math.PI / 2, Rot3.Ypr(-Math.PI / 2, 0, 0));
        CHECK_AXIS_ANGLE(_axis, Math.PI / 2, Rot3.AxisAngle(axis, -Math.PI / 2));

        // rotate by 270
        final double theta270 = Math.PI + Math.PI / 2;
        CHECK_AXIS_ANGLE(new Unit3(-1, 0, 0), Math.PI / 2, Rot3.Ypr(0, 0, theta270));
        CHECK_AXIS_ANGLE(new Unit3(0, -1, 0), Math.PI / 2, Rot3.Ypr(0, theta270, 0));
        CHECK_AXIS_ANGLE(new Unit3(0, 0, -1), Math.PI / 2, Rot3.Ypr(theta270, 0, 0));
        CHECK_AXIS_ANGLE(_axis, Math.PI / 2, Rot3.AxisAngle(axis, theta270));

        // rotate by -270
        final double theta_270 = -(Math.PI + Math.PI / 2); // 90 (or -270) degrees
        CHECK_AXIS_ANGLE(new Unit3(1, 0, 0), Math.PI / 2, Rot3.Ypr(0, 0, theta_270));
        CHECK_AXIS_ANGLE(new Unit3(0, 1, 0), Math.PI / 2, Rot3.Ypr(0, theta_270, 0));
        CHECK_AXIS_ANGLE(new Unit3(0, 0, 1), Math.PI / 2, Rot3.Ypr(theta_270, 0, 0));
        CHECK_AXIS_ANGLE(axis, Math.PI / 2, Rot3.AxisAngle(axis, theta_270));

        final double theta195 = 195 * Math.PI / 180;
        final double theta165 = 165 * Math.PI / 180;

        /// Non-trivial angle 165
        CHECK_AXIS_ANGLE(new Unit3(1, 0, 0), theta165, Rot3.Ypr(0, 0, theta165));
        CHECK_AXIS_ANGLE(new Unit3(0, 1, 0), theta165, Rot3.Ypr(0, theta165, 0));
        CHECK_AXIS_ANGLE(new Unit3(0, 0, 1), theta165, Rot3.Ypr(theta165, 0, 0));
        CHECK_AXIS_ANGLE(axis, theta165, Rot3.AxisAngle(axis, theta165));

        /// Non-trivial angle 195
        CHECK_AXIS_ANGLE(new Unit3(-1, 0, 0), theta165, Rot3.Ypr(0, 0, theta195));
        CHECK_AXIS_ANGLE(new Unit3(0, -1, 0), theta165, Rot3.Ypr(0, theta195, 0));
        CHECK_AXIS_ANGLE(new Unit3(0, 0, -1), theta165, Rot3.Ypr(theta195, 0, 0));
        CHECK_AXIS_ANGLE(_axis, theta165, Rot3.AxisAngle(axis, theta195));
    }

    Rot3 RzRyRx_proxy(Vector1 a, Vector1 b, Vector1 c) throws Throwable {
        return Rot3.RzRyRx(a.at(0), b.at(0), c.at(0));
    }

    @Test
    void testRzRyRx_scalars_derivative() throws Throwable {
        final Vector1 x = new Vector1(0.1);
        final Vector1 y = new Vector1(0.4);
        final Vector1 z = new Vector1(0.2);
        Matrix num_x = NumericalDerivative.<//
                Rot3, Vector3, //
                Vector1, Vector1, //
                Vector1, Vector1, //
                Vector1, Vector1>numericalDerivative31(this::RzRyRx_proxy, x, y, z, 1e-5);
        Matrix num_y = NumericalDerivative.<//
                Rot3, Vector3, //
                Vector1, Vector1, //
                Vector1, Vector1, //
                Vector1, Vector1>numericalDerivative32(this::RzRyRx_proxy, x, y, z, 1e-5);
        Matrix num_z = NumericalDerivative.<//
                Rot3, Vector3, //
                Vector1, Vector1, //
                Vector1, Vector1, //
                Vector1, Vector1>numericalDerivative33(this::RzRyRx_proxy, x, y, z, 1e-5);
        Matrix act_x = new Matrix();
        Matrix act_y = new Matrix();
        Matrix act_z = new Matrix();
        Rot3.RzRyRx(x.at(0), y.at(0), z.at(0), act_x, act_y, act_z);
        assertTrue(assert_equal(num_x, act_x));
        assertTrue(assert_equal(num_y, act_y));
        assertTrue(assert_equal(num_z, act_z));
    }

    Rot3 RzRyRx_proxy(Vector3 xyz) throws Throwable {
        return Rot3.RzRyRx(xyz);
    }

    @Test
    void testRzRyRx_vector_derivative() throws Throwable {
        final Vector3 xyz = new Vector3(-0.3, 0.1, 0.7);
        Matrix num = NumericalDerivative.<Rot3, Vector3, //
                Vector3, Vector3>numericalDerivative11(this::RzRyRx_proxy, xyz, 1e-5);

        Matrix act = new Matrix(new double[][] { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } });
        Rot3.RzRyRx(xyz, act);

        assertTrue(assert_equal(num, act));
    }

    Rot3 Ypr_proxy(Vector1 y, Vector1 p, Vector1 r) throws Throwable {
        return Rot3.Ypr(y.at(0), p.at(0), r.at(0));
    }

    @Test
    void testYpr_derivative() throws Throwable {
        final Vector1 y = new Vector1(0.7);
        final Vector1 p = new Vector1(-0.3);
        final Vector1 r = new Vector1(0.1);
        Matrix num_y = NumericalDerivative.<//
                Rot3, Vector3, //
                Vector1, Vector1, //
                Vector1, Vector1, //
                Vector1, Vector1>numericalDerivative31(this::Ypr_proxy, y, p, r, 1e-5);
        Matrix num_p = NumericalDerivative.<//
                Rot3, Vector3, //
                Vector1, Vector1, //
                Vector1, Vector1, //
                Vector1, Vector1>numericalDerivative32(this::Ypr_proxy, y, p, r, 1e-5);
        Matrix num_r = NumericalDerivative.<//
                Rot3, Vector3, //
                Vector1, Vector1, //
                Vector1, Vector1, //
                Vector1, Vector1>numericalDerivative33(this::Ypr_proxy, y, p, r, 1e-5);
        Matrix act_y = new Matrix();
        Matrix act_p = new Matrix();
        Matrix act_r = new Matrix();
        Rot3.Ypr(y.at(0), p.at(0), r.at(0), act_y, act_p, act_r);
        assertTrue(assert_equal(num_y, act_y));
        assertTrue(assert_equal(num_p, act_p));
        assertTrue(assert_equal(num_r, act_r));
    }

    Vector3 xyz_proxy(Rot3 R) throws Throwable {
        return R.xyz();
    }

    @Test
    void testxyz_derivative() throws Throwable {
        final Vector3 aa = new Vector3(-0.6, 0.3, 0.2);
        final Rot3 R = new Rot3().expmap(aa);
        Matrix num = NumericalDerivative.<Vector3, Vector3, //
                Rot3, Vector3>numericalDerivative11(this::xyz_proxy, R, 1e-5);
        Matrix calc = new Matrix();
        R.xyz(calc);
        assertTrue(assert_equal(num, calc));
    }

    Vector3 ypr_proxy(Rot3 R) throws Throwable {
        return R.ypr();
    }

    @Test
    void testypr_derivative() throws Throwable {
        final Vector3 aa = new Vector3(0.1, -0.3, -0.2);
        final Rot3 R = new Rot3().expmap(aa);
        Matrix num = NumericalDerivative.<Vector3, Vector3, //
                Rot3, Vector3>numericalDerivative11(this::ypr_proxy, R, 1e-5);
        Matrix calc = new Matrix();
        R.ypr(calc);
        assertTrue(assert_equal(num, calc));
    }

    Vector3 rpy_proxy(Rot3 R) throws Throwable {
        return R.rpy();
    }

    @Test
    void testrpy_derivative() throws Throwable {
        final Vector3 aa = new Vector3(1.2, 0.3, -0.9);
        final Rot3 R = new Rot3().expmap(aa);
        Matrix num = NumericalDerivative.<Vector3, Vector3, //
                Rot3, Vector3>numericalDerivative11(this::rpy_proxy, R, 1e-5);
        Matrix calc = new Matrix();
        R.rpy(calc);
        assertTrue(assert_equal(num, calc));
    }

    Vector1 roll_proxy(Rot3 R) throws Throwable {
        return new Vector1(R.roll());
    }

    @Test
    void testroll_derivative() throws Throwable {
        Vector3 aa = new Vector3(0.8, -0.8, 0.8);
        Rot3 R = new Rot3().expmap(aa);
        Matrix num = NumericalDerivative.<Vector1, Vector1, //
                Rot3, Vector3>numericalDerivative11(this::roll_proxy, R, 1e-5);
        Matrix calc = new Matrix();
        R.roll(calc);
        assertTrue(assert_equal(num, calc));
    }

    Vector1 pitch_proxy(Rot3 R) throws Throwable {
        return new Vector1(R.pitch());
    }

    @Test
    void testpitch_derivative() throws Throwable {
        final Vector3 aa = new Vector3(0.01, 0.1, 0.0);
        final Rot3 R = new Rot3().expmap(aa);
        final Matrix num = NumericalDerivative.<Vector1, Vector1, //
                Rot3, Vector3>numericalDerivative11(this::pitch_proxy, R, 1e-5);
        Matrix calc = new Matrix();
        R.pitch(calc);
        assertTrue(assert_equal(num, calc));
    }

    Vector1 yaw_proxy(Rot3 R) throws Throwable {
        return new Vector1(R.yaw());
    }

    @Test
    void testyaw_derivative() throws Throwable {
        final Vector3 aa = new Vector3(0.0, 0.1, 0.6);
        final Rot3 R = new Rot3().expmap(aa);
        Matrix num = NumericalDerivative.<Vector1, Vector1, //
                Rot3, Vector3>numericalDerivative11(this::yaw_proxy, R, 1e-5);
        Matrix calc = new Matrix();
        R.yaw(calc);
        assertTrue(assert_equal(num, calc));
    }

    @Test
    void testdeterminant() throws Throwable {
        int degree = 1;
        Rot3 R_w0 = new Rot3(); // Zero rotation
        Rot3 R_w1 = Rot3.Ry(degree * Math.PI / 180);
        Rot3 R_01 = new Rot3();
        Rot3 R_w2 = new Rot3();
        double actual = 0;
        double expected = 1.0;
        for (int i = 2; i < 360; ++i) {
            R_01 = R_w0.between(R_w1);
            R_w2 = R_w1.compose(R_01);
            R_w0 = R_w1;
            R_w1 = R_w2.normalized();
            actual = R_w2.matrix().determinant();
            assertEquals(expected, actual, 1e-7);
        }
    }

    @Test
    void testExpmapChainRule() throws Throwable {
        // Multiply with an arbitrary matrix and exponentiate
        Matrix3 M = new Matrix3( //
                1, 2, 3, //
                4, 5, 6, //
                7, 8, 9);
        ThrowingFunction<Vector3, Rot3> g = (omega) -> new Rot3().expmap(M.times(omega));

        // Test the derivatives at zero
        final Matrix expected = NumericalDerivative.<//
                Rot3, Vector3, //
                Vector3, Vector3>numericalDerivative11(g, new Vector3(0, 0, 0), 1e-5);
        // SO3::ExpmapDerivative(Z_3x1) is identity
        assertTrue(assert_equal(expected, new Matrix(M), 1e-5));
    }

    @Test
    void testexpmapChainRule() throws Throwable {
        // Multiply an arbitrary rotation with exp(M*x)
        // Perhaps counter-intuitively, this has the same derivatives as above
        Matrix3 M = new Matrix3( //
                1, 2, 3, //
                4, 5, 6, //
                7, 8, 9);
        final Rot3 R = new Rot3().expmap(new Vector3(1, 2, 3));

        ThrowingFunction<Vector3, Rot3> g = (omega) -> R.expmap(M.times(omega));

        // Test the derivatives at zero
        Matrix expected = NumericalDerivative.<//
                Rot3, Vector3, //
                Vector3, Vector3>numericalDerivative11(g, new Vector3(), 1e-5);
        assertTrue(assert_equal(expected, new Matrix(M), 1e-5));
    }

}
