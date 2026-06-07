package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import gtsam.NumericalDerivative.ThrowingFunction;
import gtsam.NumericalDerivative.ThrowingFunction2;

/**
 * See gtsam/geometry/tests/testPose2.cpp.
 */
public class Pose2Test {

    @Test
    void testConstructors() throws Throwable {
        Point2 p = new Point2(0, 0);
        Pose2 pose = new Pose2(0, p);
        Pose2 origin = new Pose2();
        assertTrue(assert_equal(pose, origin, 1e-6));
        Pose2 t = new Pose2(Math.PI / 2.0 + 0.018, new Point2(1.015, 2.01));
        assertTrue(assert_equal(t, new Pose2(t.matrix()), 1e-6));
    }

    @Test
    void testManifold() throws Throwable {
        Pose2 t1 = new Pose2(Math.PI / 2.0, new Point2(1, 2));
        Pose2 t2 = new Pose2(Math.PI / 2.0 + 0.018, new Point2(1.015, 2.01));
        Pose2 origin = new Pose2();
        Vector3 d12 = t1.local(t2);
        assertTrue(assert_equal(t2, t1.retract(d12), 1e-6));
        assertTrue(assert_equal(t2, t1.compose(origin.retract(d12)), 1e-6));
        Vector3 d21 = t2.local(t1);
        assertTrue(assert_equal(t1, t2.retract(d21), 1e-6));
        assertTrue(assert_equal(t1, t2.compose(origin.retract(d21)), 1e-6));
    }

    @Test
    void testRetract() throws Throwable {
        Pose2 pose = new Pose2(Math.PI / 2.0, new Point2(1, 2));
        // expected if GTSAM_SLOW_BUT_CORRECT_EXPMAP is set
        // Pose2 expected = new Pose2(1.00811, 2.01528, 2.5608);
        // what we actually do,
        Pose2 expected = new Pose2(Math.PI / 2.0 + 0.99, new Point2(1.015, 2.01));

        Pose2 actual = pose.retract(new Vector3(0.01, -0.015, 0.99));
        assertTrue(assert_equal(expected, actual, 1e-5));
    }

    @Test
    void testRetractJacobian() throws Throwable {
        Pose2 pose = new Pose2(Math.PI / 2.0, new Point2(1, 2));
        Vector3 v = new Vector3(0.01, -0.015, 0.99);

        Matrix actualH = new Matrix();
        Pose2.Retract(pose, v, new Matrix(), actualH);

        ThrowingFunction<Vector3, Pose2> retract_from_pose = (Vector3 delta) -> pose.retract(delta);
        Matrix expectedH = NumericalDerivative.<Pose2, Vector3, Vector3, Vector3>numericalDerivative11(
                retract_from_pose, v, 1e-6);

        assertTrue(assert_equal(expectedH, actualH, 1e-5));
    }

    @Test
    void testExpmap() throws Throwable {
        Pose2 pose = new Pose2(Math.PI / 2.0, new Point2(1, 2));
        Pose2 expected = new Pose2(1.00811, 2.01528, 2.5608);
        Pose2 actual = Pose2.expmap_default(pose, new Vector3(0.01, -0.015, 0.99));
        assertTrue(assert_equal(expected, actual, 1e-5));
    }

    @Test
    void testExpmap2() throws Throwable {
        Pose2 pose = new Pose2(Math.PI / 2.0, new Point2(1, 2));
        Pose2 expected = new Pose2(1.00811, 2.01528, 2.5608);
        Pose2 actual = Pose2.expmap_default(pose, new Vector3(0.01, -0.015, 0.99));
        assertTrue(assert_equal(expected, actual, 1e-5));
    }

    @Test
    void testExpmap3() throws Throwable {
        // do an actual series exponential map
        // see e.g. http://www.cis.upenn.edu/~cis610/cis610lie1.ps
        Matrix3 A = new Matrix3(
                0.0, -0.99, 0.01, //
                0.99, 0.0, -0.015, //
                0.0, 0.0, 0.0);
        Matrix3 A2 = A.compose(A).times(1.0 / 2.0);
        Matrix3 A3 = A2.compose(A).times(1.0 / 3.0);
        Matrix3 A4 = A3.compose(A).times(1.0 / 4.0);
        Matrix3 expected = Matrix3.identity().plus(A).plus(A2).plus(A3).plus(A4);

        Vector3 v = new Vector3(0.01, -0.015, 0.99);
        Pose2 pose = new Pose2().expmap(v, new Matrix(), new Matrix(3, 3));
        Pose2 pose2 = new Pose2(v);
        assertTrue(assert_equal(pose, pose2, 1e-6),
                String.format("expected %s actual %s\n", pose, pose2));
        Matrix3 actual = pose.matrix();
        // TODO: check that this inexactness is expected
        assertTrue(assert_equal(expected, actual, 1e-2));
    }

    @Test
    void testExpmap0a() throws Throwable {
        Pose2 expected = new Pose2(0.0101345, -0.0149092, 0.018);
        Pose2 actual = new Pose2().expmap(new Vector3(0.01, -0.015, 0.018), new Matrix(), new Matrix(3, 3));
        assertTrue(assert_equal(expected, actual, 1e-5));
    }

    @Test
    void testExpmap0b() throws Throwable {
        // a quarter turn
        Pose2 expected = new Pose2(1.0, 1.0, Math.PI / 2);
        Pose2 actual = new Pose2().expmap(new Vector3(Math.PI / 2, 0.0, Math.PI / 2), new Matrix(), new Matrix(3, 3));
        assertTrue(assert_equal(expected, actual, 1e-5));
    }

    @Test
    void testExpmap0c() throws Throwable {
        // a half turn
        Pose2 expected = new Pose2(0.0, 2.0, Math.PI);
        Pose2 actual = new Pose2().expmap(new Vector3(Math.PI, 0.0, Math.PI), new Matrix(), new Matrix(3, 3));
        assertTrue(assert_equal(expected, actual, 1e-5));
    }

    @Test
    void testExpmap0d() throws Throwable {
        // a full turn
        Pose2 expected = new Pose2(0, 0, 0);
        Pose2 actual = new Pose2().expmap(new Vector3(2 * Math.PI, 0.0, 2 * Math.PI), new Matrix(), new Matrix(3, 3));
        assertTrue(assert_equal(expected, actual, 1e-5));
    }

    // test case for screw motion in the plane
    class screwPose2 {
        static double w;
        static Vector3 xi;
        static Rot2 expectedR;
        static Point2 expectedT;
        static Pose2 expected;
        static {
            try {
                w = 0.3;
                xi = new Vector3(0.0, w, w);
                expectedR = Rot2.fromAngle(w);
                expectedT = new Point2(-0.0446635, 0.29552);
                expected = new Pose2(expectedR, expectedT);
            } catch (Throwable t) {
                throw new RuntimeException(t);
            }
        }
    }

    @Test
    void testexpmap_c() throws Throwable {
        Pose2 actual2 = new Pose2().expmap(screwPose2.xi);
        assertTrue(assert_equal(screwPose2.expected, actual2, 1e-6),
                String.format("expected %s actual %s", screwPose2.expected, actual2));
        Vector3 actual3 = new Pose2().logmap(screwPose2.expected);
        assertTrue(assert_equal(screwPose2.xi, actual3, 1e-6));
    }

    @Test
    void testexpmap_c_full() throws Throwable {
        double w = 0.3;
        Vector3 xi = new Vector3(0.0, w, w);
        Rot2 expectedR = Rot2.fromAngle(w);
        Point2 expectedT = new Point2(-0.0446635, 0.29552);
        Pose2 expected = new Pose2(expectedR, expectedT);
        Pose2 actual2 = new Pose2().expmap(xi);
        assertTrue(assert_equal(expected, actual2, 1e-6));
        Vector3 actual3 = new Pose2().logmap(expected);
        assertTrue(assert_equal(xi, actual3, 1e-6));
    }

    // assert that T*exp(xi)*T^-1 is equal to exp(Ad_T(xi))
    @Test
    void testAdjoint_full() throws Throwable {
        Pose2 T = new Pose2(1, 2, 3);
        Pose2 expected = T.compose(new Pose2().expmap(screwPose2.xi)).compose(T.inverse());
        Vector3 xiprime = T.Adjoint(screwPose2.xi);
        assertTrue(assert_equal(expected, new Pose2().expmap(xiprime), 1e-6));

        Vector3 xi2 = new Vector3(4, 5, 6);
        Pose2 expected2 = T.compose(new Pose2().expmap(xi2)).compose(T.inverse());
        Vector3 xiprime2 = T.Adjoint(xi2);
        assertTrue(assert_equal(expected2, new Pose2().expmap(xiprime2), 1e-6));
    }

    @Test
    void testlogmap() throws Throwable {
        Pose2 pose0 = new Pose2(Math.PI / 2.0, new Point2(1, 2));
        Pose2 pose = new Pose2(Math.PI / 2.0 + 0.018, new Point2(1.015, 2.01));
        // expected if GTSAM_SLOW_BUT_CORRECT_EXPMAP is set
        // Vector3 expected(0.00986473, -0.0150896, 0.018);
        // What we actually do
        Vector3 expected = new Vector3(0.01, -0.015, 0.018);
        Vector3 actual = pose0.local(pose);
        assertTrue(assert_equal(expected, actual, 1e-5));
    }

    @Test
    void testlogmap_full() throws Throwable {
        Pose2 pose0 = new Pose2(Math.PI / 2.0, new Point2(1, 2));
        Pose2 pose = new Pose2(Math.PI / 2.0 + 0.018, new Point2(1.015, 2.01));
        Vector3 expected = new Vector3(0.00986473, -0.0150896, 0.018);
        Vector3 actual = Pose2.logmap_default(pose0, pose);
        assertTrue(assert_equal(expected, actual, 1e-5));
    }

    @Test
    void testExpmapDerivative1() throws Throwable {
        Matrix actualH = new Matrix(3, 3);
        Vector3 w = new Vector3(0.1, 0.27, -0.3);
        new Pose2().expmap(w, new Matrix(), actualH);
        ThrowingFunction<Vector3, Pose2> h = (v) -> new Pose2().expmap(v, new Matrix(), new Matrix(3, 3));
        Matrix expectedH = NumericalDerivative.<//
                Pose2, Vector3, //
                Vector3, Vector3>numericalDerivative11(
                        h, w, 1e-2);
        assertTrue(assert_equal(expectedH, actualH, 1e-5));
    }

    @Test
    void testExpmapDerivative2() throws Throwable {
        Matrix actualH = new Matrix(3, 3);
        // alpha = 0
        Vector3 w0 = new Vector3(0.1, 0.27, 0.0);
        new Pose2().expmap(w0, new Matrix(), actualH);
        ThrowingFunction<Vector3, Pose2> h = (v) -> new Pose2().expmap(v, new Matrix(), new Matrix(3, 3));
        Matrix expectedH = NumericalDerivative.<//
                Pose2, Vector3, //
                Vector3, Vector3>numericalDerivative11(h, w0, 1e-3);
        assertTrue(assert_equal(expectedH, actualH, 1e-5));
    }

    @Test
    void testLogmapDerivative1() throws Throwable {
        Matrix actualH = new Matrix(3, 3);
        Vector3 w = new Vector3(0.1, 0.27, -0.3);
        Pose2 p = new Pose2().expmap(w, new Matrix(), new Matrix(3, 3));
        Vector3 other = new Pose2().logmap(p, new Matrix(), actualH);
        assertTrue(assert_equal(w, other, 1e-5));

        ThrowingFunction<Pose2, Vector3> h = (pp) -> new Pose2().logmap(pp, new Matrix(), new Matrix(3, 3));
        Matrix expectedH = NumericalDerivative.<//
                Vector3, Vector3, //
                Pose2, Vector3>numericalDerivative11(h, p, 1e-2);
        assertTrue(assert_equal(expectedH, actualH, 1e-5));
    }

    @Test
    void testLogmapDerivative2() throws Throwable {
        Matrix actualH = new Matrix(3, 3);
        Vector3 w0 = new Vector3(0.1, 0.27, 0.0); // alpha = 0
        Pose2 p = new Pose2().expmap(w0, new Matrix(), new Matrix(3, 3));
        assertTrue(assert_equal(w0, new Pose2().logmap(p, new Matrix(), actualH), 1e-5));
        ThrowingFunction<Pose2, Vector3> h = (pp) -> new Pose2().logmap(pp, new Matrix(), new Matrix(3, 3));
        Matrix expectedH = NumericalDerivative.<//
                Vector3, Vector3, //
                Pose2, Vector3>numericalDerivative11(h, p, 1e-2);
        assertTrue(assert_equal(expectedH, actualH, 1e-5));
    }

    @Test
    void testtransformTo() throws Throwable {
        // robot at (1,2) looking towards y
        Pose2 pose = new Pose2(Math.PI / 2.0, new Point2(1, 2));
        // landmark at (-1,4)
        Point2 point = new Point2(-1, 4);

        Point2 expected = new Point2(2, 2);
        Matrix expectedH1 = new Matrix(new double[][] { //
                { -1.0, 0.0, 2.0 }, //
                { 0.0, -1.0, -2.0 }//
        });
        Matrix expectedH2 = new Matrix(new double[][] {
                { 0.0, 1.0 }, //
                { -1.0, 0.0 } //
        });

        Matrix actualH1 = new Matrix();
        Matrix actualH2 = new Matrix();
        Point2 actual = pose.transformTo(point, actualH1, actualH2);
        assertTrue(assert_equal(expected, actual, 1e-6));

        assertTrue(assert_equal(expectedH1, actualH1, 1e-6));
        ThrowingFunction2<Pose2, Point2, Point2> transformTo_ = (p, p0) -> p.transformTo(p0, new Matrix(),
                new Matrix());
        Matrix numericalH1 = NumericalDerivative
                .<Point2, Vector2, Pose2, Vector3, Point2, Vector2>numericalDerivative21(
                        transformTo_, pose, point, 1e-3);
        assertTrue(assert_equal(numericalH1, actualH1, 1e-6));

        assertTrue(assert_equal(expectedH2, actualH2, 1e-6));
        Matrix numericalH2 = NumericalDerivative
                .<Point2, Vector2, Pose2, Vector3, Point2, Vector2>numericalDerivative22(
                        transformTo_, pose, point, 1e-3);
        assertTrue(assert_equal(numericalH2, actualH2, 1e-6));
    }

    @Test
    void testtransformFrom() throws Throwable {
        Pose2 pose = new Pose2(1., 0., Math.PI / 2.0);
        Point2 pt = new Point2(2., 1.);
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        Point2 actual = pose.transformFrom(pt, H1, H2);

        Point2 expected = new Point2(0., 2.);
        assertTrue(assert_equal(expected, actual, 1e-6));

        Matrix H1_expected = new Matrix(new double[][] {
                { 0., -1., -2. }, //
                { 1., 0., -1. } });
        Matrix H2_expected = new Matrix(new double[][] {
                { 0., -1. }, //
                { 1., 0. } });

        ThrowingFunction2<Pose2, Point2, Point2> transformFrom_ = (p, p0) -> p.transformFrom(
                p0, new Matrix(), new Matrix());
        Matrix numericalH1 = NumericalDerivative
                .<Point2, Vector2, Pose2, Vector3, Point2, Vector2>numericalDerivative21(
                        transformFrom_, pose, pt, 1e-3);
        assertTrue(assert_equal(H1_expected, H1, 1e-6));
        assertTrue(assert_equal(H1_expected, numericalH1, 1e-6));
        Matrix numericalH2 = NumericalDerivative
                .<Point2, Vector2, Pose2, Vector3, Point2, Vector2>numericalDerivative22(
                        transformFrom_, pose, pt, 1e-3);
        assertTrue(assert_equal(H2_expected, H2, 1e-6));
        assertTrue(assert_equal(H2_expected, numericalH2, 1e-6));
    }

    @Test
    void testcompose_a() throws Throwable {
        Pose2 pose1 = new Pose2(Math.PI / 4.0, new Point2(Math.sqrt(0.5), Math.sqrt(0.5)));
        Pose2 pose2 = new Pose2(Math.PI / 2.0, new Point2(0.0, 2.0));

        Matrix actualDcompose1 = new Matrix();
        Matrix actualDcompose2 = new Matrix();
        Pose2 actual = pose1.compose(pose2, actualDcompose1, actualDcompose2);

        Pose2 expected = new Pose2(3.0 * Math.PI / 4.0, new Point2(-Math.sqrt(0.5), 3.0 * Math.sqrt(0.5)));
        assertTrue(assert_equal(expected, actual, 1e-6));

        Matrix expectedH1 = new Matrix(new double[][] {
                { 0.0, 1.0, 0.0 },
                { -1.0, 0.0, 2.0 },
                { 0.0, 0.0, 1.0 } });

        Matrix expectedH2 = Matrix.I_3x3();
        Matrix numericalH1 = NumericalDerivative.<//
                Pose2, Vector3, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative21(
                        (a, b) -> a.compose(b), pose1, pose2, 1e-3);
        Matrix numericalH2 = NumericalDerivative.<//
                Pose2, Vector3, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative22(
                        (a, b) -> a.compose(b), pose1, pose2, 1e-3);
        assertTrue(assert_equal(expectedH1, actualDcompose1, 1e-6));
        assertTrue(assert_equal(numericalH1, actualDcompose1, 1e-6));
        assertTrue(assert_equal(expectedH2, actualDcompose2, 1e-6));
        assertTrue(assert_equal(numericalH2, actualDcompose2, 1e-6));

        Point2 point = new Point2(Math.sqrt(0.5), 3.0 * Math.sqrt(0.5));
        Point2 expected_point = new Point2(-1.0, -1.0);
        Point2 actual_point1 = (pose1.compose(pose2)).transformTo(point);
        Point2 actual_point2 = pose2.transformTo(pose1.transformTo(point));
        assertTrue(assert_equal(expected_point, actual_point1, 1e-6));
        assertTrue(assert_equal(expected_point, actual_point2, 1e-6));
    }

    @Test
    void testcompose_b() throws Throwable {
        Pose2 pose1 = new Pose2(Rot2.fromAngle(Math.PI / 10.0), new Point2(.75, .5));
        Pose2 pose2 = new Pose2(Rot2.fromAngle(Math.PI / 4.0 - Math.PI / 10.0),
                new Point2(0.701289620636, 1.34933052585));

        Pose2 pose_expected = new Pose2(Rot2.fromAngle(Math.PI / 4.0), new Point2(1.0, 2.0));

        Pose2 pose_actual_op = pose1.compose(pose2);
        Matrix actualDcompose1 = new Matrix();
        Matrix actualDcompose2 = new Matrix();
        Pose2 pose_actual_fcn = pose1.compose(pose2, actualDcompose1, actualDcompose2);

        Matrix numericalH1 = NumericalDerivative.<//
                Pose2, Vector3, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative21(
                        (a, b) -> a.compose(b), pose1, pose2, 1e-3);
        Matrix numericalH2 = NumericalDerivative.<//
                Pose2, Vector3, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative22(
                        (a, b) -> a.compose(b), pose1, pose2, 1e-3);
        assertTrue(assert_equal(numericalH1, actualDcompose1, 1e-5));
        assertTrue(assert_equal(numericalH2, actualDcompose2, 1e-6));

        assertTrue(assert_equal(pose_expected, pose_actual_op, 1e-6));
        assertTrue(assert_equal(pose_expected, pose_actual_fcn, 1e-6));
    }

    @Test
    void testcompose_c() throws Throwable {
        Pose2 pose1 = new Pose2(Rot2.fromAngle(Math.PI / 4.0), new Point2(1.0, 1.0));
        Pose2 pose2 = new Pose2(Rot2.fromAngle(Math.PI / 4.0), new Point2(Math.sqrt(.5), Math.sqrt(.5)));

        Pose2 pose_expected = new Pose2(Rot2.fromAngle(Math.PI / 2.0), new Point2(1.0, 2.0));

        Pose2 pose_actual_op = pose1.compose(pose2);
        Matrix actualDcompose1 = new Matrix();
        Matrix actualDcompose2 = new Matrix();

        Pose2 pose_actual_fcn = pose1.compose(pose2, actualDcompose1, actualDcompose2);

        Matrix numericalH1 = NumericalDerivative.<//
                Pose2, Vector3, Pose2, Vector3, Pose2, Vector3>numericalDerivative21(
                        (a, b) -> a.compose(b), pose1, pose2, 1e-3);
        Matrix numericalH2 = NumericalDerivative.<//
                Pose2, Vector3, Pose2, Vector3, Pose2, Vector3>numericalDerivative22(
                        (a, b) -> a.compose(b), pose1, pose2, 1e-3);
        assertTrue(assert_equal(numericalH1, actualDcompose1, 1e-5));
        assertTrue(assert_equal(numericalH2, actualDcompose2, 1e-6));

        assertTrue(assert_equal(pose_expected, pose_actual_op, 1e-6));
        assertTrue(assert_equal(pose_expected, pose_actual_fcn, 1e-6));
    }

    @Test
    void testinverse() throws Throwable {
        Point2 t = new Point2(1, 2);
        // robot at (1,2) looking towards y
        Pose2 gTl = new Pose2(Math.PI / 2.0, t);

        Pose2 identity = new Pose2();
        Pose2 lTg = gTl.inverse();
        assertTrue(assert_equal(identity, lTg.compose(gTl), 1e-6));
        assertTrue(assert_equal(identity, gTl.compose(lTg), 1e-6));

        Point2 l = new Point2(4, 5);
        Point2 g = new Point2(-4, 6);
        assertTrue(assert_equal(g, gTl.transformFrom(l), 1e-6));
        assertTrue(assert_equal(l, lTg.transformFrom(g), 1e-6));

        // Check derivative
        Matrix numericalH = NumericalDerivative.<//
                Pose2, Vector3, Pose2, Vector3>numericalDerivative11(
                        (a) -> a.inverse(), lTg, 1e-3);
        Matrix actualDinverse = new Matrix();
        lTg.inverse(actualDinverse);
        assertTrue(assert_equal(numericalH, actualDinverse, 1e-6));
    }

    Vector3 homogeneous(Point2 p) throws Throwable {
        return new Vector3(p.x(), p.y(), 1.0);
    }

    Matrix matrix(Pose2 gTl) throws Throwable {
        Matrix2 gRl = gTl.r().matrix();
        Point2 gt = gTl.t();
        return new Matrix(new double[][] {
                { gRl.at(0, 0), gRl.at(0, 1), gt.x() },
                { gRl.at(1, 0), gRl.at(1, 1), gt.y() },
                { 0.0, 0.0, 1.0 } });
    }

    @Test
    void testmatrix() throws Throwable {
        Point2 origin = new Point2(0, 0);
        Point2 t = new Point2(1, 2);
        // robot at (1,2) looking towards y
        Pose2 gTl = new Pose2(Math.PI / 2.0, t);
        Matrix gMl = matrix(gTl);
        assertTrue(assert_equal(new Matrix(new double[][] {
                { 0.0, -1.0, 1.0 },
                { 1.0, 0.0, 2.0 },
                { 0.0, 0.0, 1.0 } }), gMl, 1e-6));
        Rot2 gR1 = gTl.r();
        assertTrue(assert_equal(homogeneous(t), gMl.times(homogeneous(origin)), 1e-6));
        Point2 x_axis = new Point2(1, 0);
        Point2 y_axis = new Point2(0, 1);
        assertTrue(assert_equal(new Matrix2(
                0.0, -1.0, //
                1.0, 0.0), gR1.matrix(), 1e-6));
        assertTrue(assert_equal(new Point2(0, 1), gR1.rotate(x_axis), 1e-6));
        assertTrue(assert_equal(new Point2(-1, 0), gR1.rotate(y_axis), 1e-6));
        assertTrue(assert_equal(homogeneous(new Point2(1 + 0, 2 + 1)), gMl.times(homogeneous(x_axis)), 1e-6));
        assertTrue(assert_equal(homogeneous(new Point2(1 - 1, 2 + 0)), gMl.times(homogeneous(y_axis)), 1e-6));

        // check inverse pose
        Matrix lMg = matrix(gTl.inverse());
        assertTrue(assert_equal(new Matrix(new double[][] {
                { 0.0, 1.0, -2.0 },
                { -1.0, 0.0, 1.0 },
                { 0.0, 0.0, 1.0 } }), lMg, 1e-6));
    }

    @Test
    void testcompose_matrix() throws Throwable {
        // robot at (1,2) facing +y
        Pose2 gT1 = new Pose2(Math.PI / 2.0, new Point2(1, 2));
        // local robot at (-1,4) facing -x
        Pose2 _1T2 = new Pose2(Math.PI, new Point2(-1, 4));
        Matrix gM1 = matrix(gT1);
        Matrix _1M2 = matrix(_1T2);
        assertTrue(assert_equal(gM1.compose(_1M2),
                matrix(gT1.compose(_1T2)), 1e-6));
    }

    @Test
    void testtranslation() throws Throwable {
        Pose2 pose = new Pose2(3.5, -8.2, 4.2);

        Matrix actualH = new Matrix();
        assertTrue(assert_equal(new Point2(3.5, -8.2),
                pose.translation(actualH), 1e-8));

        ThrowingFunction<Pose2, Point2> f = (T) -> T.t();
        Matrix numericalH = NumericalDerivative.<//
                Point2, Vector2, //
                Pose2, Vector3>numericalDerivative11(f, pose, 1e-3);
        assertTrue(assert_equal(numericalH, actualH, 1e-6));
    }

    @Test
    void testrotation() throws Throwable {
        Pose2 pose = new Pose2(3.5, -8.2, 4.2);

        Matrix actualH = new Matrix();
        assertTrue(assert_equal(new Rot2(4.2), pose.rotation(actualH), 1e-8));

        ThrowingFunction<Pose2, Rot2> f = (t) -> t.rotation();
        Matrix numericalH = NumericalDerivative.<Rot2, Vector1, Pose2, Vector3>numericalDerivative11(f, pose, 1e-5);
        assertTrue(assert_equal(numericalH, actualH, 1e-6));
    }

    /**
     * <pre>
     * <
     *
     *       ^
     *
     * *--0--*--*
     * </pre>
     */
    @Test
    void testbetween() throws Throwable {
        // robot at (1,2) looking towards y
        Pose2 gT1 = new Pose2(Math.PI / 2.0, new Point2(1, 2));
        // robot at (-1,4) looking at negative x
        Pose2 gT2 = new Pose2(Math.PI, new Point2(-1, 4));

        Matrix actualH1 = new Matrix();
        Matrix actualH2 = new Matrix();
        Pose2 expected = new Pose2(Math.PI / 2.0, new Point2(2, 2));
        Pose2 actual1 = gT1.between(gT2);
        Pose2 actual2 = gT1.between(gT2, actualH1, actualH2);
        assertTrue(assert_equal(expected, actual1, 1e-6));
        assertTrue(assert_equal(expected, actual2, 1e-6));

        Matrix expectedH1 = new Matrix(new double[][] {
                { 0.0, -1.0, -2.0 },
                { 1.0, 0.0, -2.0 },
                { 0.0, 0.0, -1.0 } });
        Matrix numericalH1 = NumericalDerivative.<//
                Pose2, Vector3, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative21((a, b) -> a.between(b), gT1, gT2, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(numericalH1, actualH1));

        Matrix expectedH2 = new Matrix(new double[][] {
                { 1.0, 0.0, 0.0 },
                { 0.0, 1.0, 0.0 },
                { 0.0, 0.0, 1.0 } });
        Matrix numericalH2 = NumericalDerivative.<//
                Pose2, Vector3, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative22((a, b) -> a.between(b), gT1, gT2, 1e-5);
        assertTrue(assert_equal(expectedH2, actualH2));
        assertTrue(assert_equal(numericalH2, actualH2));
    }

    // reverse situation for extra test
    @Test
    void testbetween2() throws Throwable {
        // robot at (1,2) looking towards y
        Pose2 p2 = new Pose2(Math.PI / 2.0, new Point2(1, 2));
        // robot at (-1,4) loooking at negative x
        Pose2 p1 = new Pose2(Math.PI, new Point2(-1, 4));

        Matrix actualH1 = new Matrix();
        Matrix actualH2 = new Matrix();
        p1.between(p2, actualH1, actualH2);
        Matrix numericalH1 = NumericalDerivative.<//
                Pose2, Vector3, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative21((a, b) -> a.between(b), p1, p2, 1e-5);
        assertTrue(assert_equal(numericalH1, actualH1));
        Matrix numericalH2 = NumericalDerivative.<//
                Pose2, Vector3, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative22((a, b) -> a.between(b), p1, p2, 1e-5);
        assertTrue(assert_equal(numericalH2, actualH2));
    }

    // arbitrary, non perpendicular angles to be extra safe
    @Test
    void testbetween3() throws Throwable {
        Pose2 p2 = new Pose2(Math.PI / 3.0, new Point2(1, 2));
        Pose2 p1 = new Pose2(Math.PI / 6.0, new Point2(-1, 4));

        Matrix actualH1 = new Matrix();
        Matrix actualH2 = new Matrix();
        p1.between(p2, actualH1, actualH2);
        Matrix numericalH1 = NumericalDerivative.<//
                Pose2, Vector3, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative21((a, b) -> a.between(b), p1, p2, 1e-5);
        assertTrue(assert_equal(numericalH1, actualH1));
        Matrix numericalH2 = NumericalDerivative.<//
                Pose2, Vector3, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative22((a, b) -> a.between(b), p1, p2, 1e-5);
        assertTrue(assert_equal(numericalH2, actualH2));
    }

    @Test
    void testround_trip() throws Throwable {
        Pose2 p1 = new Pose2(1.23, 2.30, 0.2);
        Pose2 odo = new Pose2(0.53, 0.39, 0.15);
        Pose2 p2 = p1.compose(odo);
        assertTrue(assert_equal(odo, p1.between(p2), 1e-6));
    }

    // some shared test values
    static Pose2 x1;
    static Pose2 x2;
    static Pose2 x3;
    static Point2 l1;
    static Point2 l2;
    static Point2 l3;
    static Point2 l4;
    static {
        try {
            x1 = new Pose2();
            x2 = new Pose2(1, 1, 0);
            x3 = new Pose2(1, 1, Math.PI / 4.0);
            l1 = new Point2(1, 0);
            l2 = new Point2(1, 1);
            l3 = new Point2(2, 2);
            l4 = new Point2(1, 3);
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }
    }

    Rot2 bearing_proxy(Pose2 pose, Point2 pt) throws Throwable {
        return pose.bearing(pt);
    }

    @Test
    void testbearing() throws Throwable {

        Matrix expectedH1 = new Matrix();
        Matrix actualH1 = new Matrix();
        Matrix expectedH2 = new Matrix();
        Matrix actualH2 = new Matrix();

        // establish bearing is indeed zero
        assertTrue(assert_equal(new Rot2(), x1.bearing(l1)));

        // establish bearing is indeed 45 degrees
        assertTrue(assert_equal(Rot2.fromAngle(Math.PI / 4.0), x1.bearing(l2)));

        // establish bearing is indeed 45 degrees even if shifted
        Rot2 actual23 = x2.bearing(l3, actualH1, actualH2);
        assertTrue(assert_equal(Rot2.fromAngle(Math.PI / 4.0), actual23));

        // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Rot2, Vector1, //
                Pose2, Vector3, //
                Point2, Vector2>numericalDerivative21(this::bearing_proxy, x2, l3, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        expectedH2 = NumericalDerivative.<//
                Rot2, Vector1, //
                Pose2, Vector3, //
                Point2, Vector2>numericalDerivative22(this::bearing_proxy, x2, l3, 1e-5);
        assertTrue(assert_equal(expectedH2, actualH2));

        // establish bearing is indeed 45 degrees even if rotated
        Rot2 actual34 = x3.bearing(l4, actualH1, actualH2);
        assertTrue(assert_equal(Rot2.fromAngle(Math.PI / 4.0), actual34));

        // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Rot2, Vector1, //
                Pose2, Vector3, //
                Point2, Vector2>numericalDerivative21(this::bearing_proxy, x3, l4, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Rot2, Vector1, //
                Pose2, Vector3, //
                Point2, Vector2>numericalDerivative22(this::bearing_proxy, x3, l4, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));
    }

    Rot2 bearing_pose_proxy(Pose2 pose, Pose2 pt) throws Throwable {
        return pose.bearing(pt);
    }

    @Test
    void testbearing_pose() throws Throwable {
        Pose2 xl1 = new Pose2(1, 0, Math.PI / 2.0);
        Pose2 xl2 = new Pose2(1, 1, Math.PI);
        Pose2 xl3 = new Pose2(2.0, 2.0, -Math.PI / 2.0);
        Pose2 xl4 = new Pose2(1, 3, 0);

        Matrix expectedH1 = new Matrix();
        Matrix actualH1 = new Matrix();
        Matrix expectedH2 = new Matrix();
        Matrix actualH2 = new Matrix();

        // establish bearing is indeed zero
        assertTrue(assert_equal(new Rot2(), x1.bearing(xl1)));

        // establish bearing is indeed 45 degrees
        assertTrue(assert_equal(Rot2.fromAngle(Math.PI / 4.0), x1.bearing(xl2)));

        // establish bearing is indeed 45 degrees even if shifted
        Rot2 actual23 = x2.bearing(xl3, actualH1, actualH2);
        assertTrue(assert_equal(Rot2.fromAngle(Math.PI / 4.0), actual23));

        // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Rot2, Vector1, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative21(this::bearing_pose_proxy, x2, xl3, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Rot2, Vector1, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative22(this::bearing_pose_proxy, x2, xl3, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));

        // establish bearing is indeed 45 degrees even if rotated
        Rot2 actual34 = x3.bearing(xl4, actualH1, actualH2);
        assertTrue(assert_equal(Rot2.fromAngle(Math.PI / 4.0), actual34));

        // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Rot2, Vector1, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative21(this::bearing_pose_proxy, x3, xl4, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Rot2, Vector1, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative22(this::bearing_pose_proxy, x3, xl4, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));
    }

    Vector1 range_proxy(Pose2 pose, Point2 point) throws Throwable {
        return new Vector1(pose.range(point));
    }

    @Test
    void testrange() throws Throwable {

        Matrix expectedH1 = new Matrix();
        Matrix actualH1 = new Matrix();
        Matrix expectedH2 = new Matrix();
        Matrix actualH2 = new Matrix();

        // establish range is indeed zero
        assertEquals(1, x1.range(l1), 1e-9);

        // establish range is indeed 45 degrees
        assertEquals(Math.sqrt(2.0), x1.range(l2), 1e-9);

        // Another pair
        double actual23 = x2.range(l3, actualH1, actualH2);
        assertEquals(Math.sqrt(2.0), actual23, 1e-9);

        // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose2, Vector3, //
                Point2, Vector2>numericalDerivative21(this::range_proxy, x2, l3, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose2, Vector3, //
                Point2, Vector2>numericalDerivative22(this::range_proxy, x2, l3, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));

        // Another test
        double actual34 = x3.range(l4, actualH1, actualH2);
        assertEquals(2, actual34, 1e-9);

        // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose2, Vector3, //
                Point2, Vector2>numericalDerivative21(this::range_proxy, x3, l4, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose2, Vector3, //
                Point2, Vector2>numericalDerivative22(this::range_proxy, x3, l4, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));
    }

    Vector1 range_pose_proxy(Pose2 pose, Pose2 point) throws Throwable {
        return new Vector1(pose.range(point));
    }

    @Test
    void testrange_pose() throws Throwable {

        Pose2 xl1 = new Pose2(1, 0, Math.PI / 2.0);
        Pose2 xl2 = new Pose2(1, 1, Math.PI);
        Pose2 xl3 = new Pose2(2.0, 2.0, -Math.PI / 2.0);
        Pose2 xl4 = new Pose2(1, 3, 0);

        Matrix expectedH1 = new Matrix();
        Matrix actualH1 = new Matrix();
        Matrix expectedH2 = new Matrix();
        Matrix actualH2 = new Matrix();

        // establish range is indeed zero
        assertEquals(1, x1.range(xl1), 1e-9);

        // establish range is indeed 45 degrees
        assertEquals(Math.sqrt(2.0), x1.range(xl2), 1e-9);

        // Another pair
        double actual23 = x2.range(xl3, actualH1, actualH2);
        assertEquals(Math.sqrt(2.0), actual23, 1e-9);

        // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative21(this::range_pose_proxy, x2, xl3, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative22(this::range_pose_proxy, x2, xl3, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));

        // Another test
        double actual34 = x3.range(xl4, actualH1, actualH2);
        assertEquals(2, actual34, 1e-9);

        // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative21(this::range_pose_proxy, x3, xl4, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose2, Vector3, //
                Pose2, Vector3>numericalDerivative22(this::range_pose_proxy, x3, xl4, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));
    }

    static Pose2 id;
    static Pose2 T1;
    static Pose2 T2;

    static {
        try {
            id = new Pose2();
            T1 = new Pose2(Math.PI / 4.0, new Point2(Math.sqrt(0.5), Math.sqrt(0.5)));
            T2 = new Pose2(Math.PI / 2.0, new Point2(0.0, 2.0));
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }
    }

    @Test
    void testInvariants() throws Throwable {
        assertTrue(Pose2.check_group_invariants(id, id));
        assertTrue(Pose2.check_group_invariants(id, T1));
        assertTrue(Pose2.check_group_invariants(T2, id));
        assertTrue(Pose2.check_group_invariants(T2, T1));

        assertTrue(Pose2.check_manifold_invariants(id, id));
        assertTrue(Pose2.check_manifold_invariants(id, T1));
        assertTrue(Pose2.check_manifold_invariants(T2, id));
        assertTrue(Pose2.check_manifold_invariants(T2, T1));
    }
}
