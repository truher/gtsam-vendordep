package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.function.Function;

import org.junit.jupiter.api.Test;

import gtsam.NumericalDerivative.ThrowingFunction;
import gtsam.NumericalDerivative.ThrowingFunction2;

/**
 * See gtsam/geometry/tests/testPose3.cpp
 */
public class Pose3Test {

    static Point3 P;
    static Rot3 R;
    static Point3 P2;
    static Pose3 T;
    static Pose3 T2;
    static Pose3 T3;
    static double tol;

    static {
        try {
            P = new Point3(0.2, 0.7, -2);
            R = Rot3.Rodrigues(0.3, 0, 0);
            P2 = new Point3(3.5, -8.2, 4.2);
            T = new Pose3(R, P2);
            T2 = new Pose3(Rot3.Rodrigues(0.3, 0.2, 0.1), P2);
            T3 = new Pose3(Rot3.Rodrigues(-90, 0, 0), new Point3(1, 2, 3));
            tol = 1e-5;
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }
    }

    @Test
    void testequals() throws Throwable {
        Pose3 pose2 = T3;
        assertTrue(assert_equal(T3, pose2));
        Pose3 origin = new Pose3();
        assertTrue(!assert_equal(T3, origin));
    }

    @Test
    void testfinalructors() throws Throwable {
        Pose3 expected = new Pose3(Rot3.Rodrigues(0, 0, 3), new Point3(1, 2, 0));
        Pose2 pose2 = new Pose2(1, 2, 3);
        assertTrue(assert_equal(expected, new Pose3(pose2)));
    }

    // note this flag is not set
    // #ifndef GTSAM_POSE3_EXPMAP
    @Test
    void testretract_first_order() throws Throwable {
        Pose3 id = new Pose3();
        Vector6 v = new Vector6(0, 0, 0, 0, 0, 0);
        v.set(0, 0.3);
        assertTrue(assert_equal(new Pose3(R, new Point3(0, 0, 0)), id.retract(v), 1e-2));
        v.set(3, 0.2);
        v.set(4, 0.7);
        v.set(5, -2);
        // note ridiculously coarse tolerance
        assertTrue(assert_equal(new Pose3(R, P), id.retract(v), 0.3));
        // this is true if the flag is set.
        // assertTrue(assert_equal(new Pose3(R, P), id.retract(v), 1e-2));
    }
    // #endif

    @Test
    void testretract_expmap() throws Throwable {
        Vector6 v = new Vector6(0, 0, 0, 0, 0, 0);
        v.set(0, 0.3);
        Pose3 pose = Pose3.statics.Expmap(v);
        assertTrue(assert_equal(new Pose3(R, new Point3(0, 0, 0)), pose, 1e-2));
        assertTrue(assert_equal(v, Pose3.statics.Logmap(pose), 1e-2));
    }

    @Test
    void testexpmap_a_full() throws Throwable {
        Pose3 id = new Pose3();
        Vector6 v = new Vector6(0.3, 0, 0, 0, 0, 0);
        assertTrue(assert_equal(Pose3.expmap_default(id, v), new Pose3(R, new Point3(0, 0, 0))));
        v.set(3, 0.2);
        v.set(4, 0.394742);
        v.set(5, -2.08998);
        assertTrue(assert_equal(new Pose3(R, P), Pose3.expmap_default(id, v), 1e-5));
    }

    @Test
    void testexpmap_a_full2() throws Throwable {
        Pose3 id = new Pose3();
        Vector6 v = new Vector6(0.3, 0, 0, 0, 0, 0);
        assertTrue(assert_equal(Pose3.expmap_default(id, v), new Pose3(R, new Point3(0, 0, 0))));
        v.set(3, 0.2);
        v.set(4, 0.394742);
        v.set(5, -2.08998);
        assertTrue(assert_equal(new Pose3(R, P), Pose3.expmap_default(id, v), 1e-5));
    }

    @Test
    void testexpmap_b() throws Throwable {
        Pose3 p1 = new Pose3(new Rot3(), new Point3(100, 0, 0));
        Pose3 p2 = p1.retract(new Vector6(0.0, 0.0, 0.1, 0.0, 0.0, 0.0));
        Pose3 expected = new Pose3(Rot3.Rodrigues(0.0, 0.0, 0.1), new Point3(100.0, 0.0, 0.0));
        assertTrue(assert_equal(expected, p2, 1e-2));
    }

    // test case for screw motion in the plane
    class screwPose3 {
        static double a = 0.3;
        static double c = Math.cos(a);
        static double s = Math.sin(a);
        static double w = 0.3;
        static Vector6 xi;
        static Rot3 expectedR;
        static Point3 expectedT;
        static Pose3 expected;

        static {
            try {
                xi = new Vector6(0.0, 0.0, w, w, 0.0, 1.0);
                expectedR = new Rot3(c, -s, 0, s, c, 0, 0, 0, 1);
                expectedT = new Point3(0.29552, 0.0446635, 1);
                expected = new Pose3(expectedR, expectedT);
            } catch (Throwable t) {
                throw new RuntimeException(t);
            }
        }
    }

    // assert that T*exp(xi)*T^-1 is equal to exp(Ad_T(xi))
    @Test
    void testAdjoint_full() throws Throwable {
        Pose3 expected = T.compose(Pose3.statics.Expmap(screwPose3.xi)).compose(T.inverse());
        Vector6 xiprime = T.Adjoint(screwPose3.xi);
        assertTrue(assert_equal(expected, Pose3.statics.Expmap(xiprime), 1e-6));

        Pose3 expected2 = T2.compose(Pose3.statics.Expmap(screwPose3.xi)).compose(T2.inverse());
        Vector6 xiprime2 = T2.Adjoint(screwPose3.xi);
        assertTrue(assert_equal(expected2, Pose3.statics.Expmap(xiprime2), 1e-6));

        Pose3 expected3 = T3.compose(Pose3.statics.Expmap(screwPose3.xi)).compose(T3.inverse());
        Vector6 xiprime3 = T3.Adjoint(screwPose3.xi);
        assertTrue(assert_equal(expected3, Pose3.statics.Expmap(xiprime3), 1e-6));
    }

    // Check Adjoint numerical derivatives
    @Test
    void testAdjoint_jacobians() throws Throwable {
        Vector6 xi = new Vector6(0.1, 1.2, 2.3, 3.1, 1.4, 4.5);

        // Check evaluation sanity check
        assertTrue(assert_equal(T.AdjointMap().times(new Vector(xi)), new Vector(T.Adjoint(xi))));
        assertTrue(assert_equal(T2.AdjointMap().times(new Vector(xi)), new Vector(T2.Adjoint(xi))));
        assertTrue(assert_equal(T3.AdjointMap().times(new Vector(xi)), new Vector(T3.Adjoint(xi))));

        // Check jacobians
        Matrix actualH1 = new Matrix();
        Matrix actualH2 = new Matrix();
        Matrix expectedH1 = new Matrix();
        Matrix expectedH2 = new Matrix();
        ThrowingFunction2<Pose3, Vector6, Vector6> Ad = (TT, xxi) -> TT.Adjoint(xxi);

        T.Adjoint(xi, actualH1, actualH2);
        expectedH1 = NumericalDerivative.<//
                Vector6, Vector6, //
                Pose3, Vector6, //
                Vector6, Vector6>numericalDerivative21(Ad, T, xi, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Vector6, Vector6, //
                Pose3, Vector6, //
                Vector6, Vector6>numericalDerivative22(Ad, T, xi, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));

        T2.Adjoint(xi, actualH1, actualH2);
        expectedH1 = NumericalDerivative.<//
                Vector6, Vector6, //
                Pose3, Vector6, //
                Vector6, Vector6>numericalDerivative21(Ad, T2, xi, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Vector6, Vector6, //
                Pose3, Vector6, //
                Vector6, Vector6>numericalDerivative22(Ad, T2, xi, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));

        T3.Adjoint(xi, actualH1, actualH2);
        expectedH1 = NumericalDerivative.<//
                Vector6, Vector6, //
                Pose3, Vector6, //
                Vector6, Vector6>numericalDerivative21(Ad, T3, xi, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Vector6, Vector6, //
                Pose3, Vector6, //
                Vector6, Vector6>numericalDerivative22(Ad, T3, xi, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));
    }

    // Check translation and its pushforward
    @Test
    void testtranslation() throws Throwable {
        Matrix actualH = new Matrix();
        assertTrue(assert_equal(new Point3(3.5, -8.2, 4.2), T.translation(actualH), 1e-8));

        ThrowingFunction<Pose3, Point3> f = (TT) -> TT.translation();
        Matrix numericalH = NumericalDerivative.<//
                Point3, Vector3, //
                Pose3, Vector6>numericalDerivative11(f, T, 1e-5);
        assertTrue(assert_equal(numericalH, actualH, 1e-6));
    }

    // // Check rotation and its pushforward
    @Test
    void testrotation() throws Throwable {
        Matrix actualH = new Matrix();
        assertTrue(assert_equal(R, T.rotation(actualH), 1e-8));

        ThrowingFunction<Pose3, Rot3> f = (TT) -> TT.rotation();
        Matrix numericalH = NumericalDerivative.<//
                Rot3, Vector3, //
                Pose3, Vector6>numericalDerivative11(f, T, 1e-5);
        assertTrue(assert_equal(numericalH, actualH, 1e-6));
    }

    @Test
    void testAdjoint_compose_full() throws Throwable {
        // To debug derivatives of compose, assert that
        // T1*T2*exp(Adjoint(inv(T2),x) = T1*exp(x)*T2
        Pose3 T1 = T;
        Vector6 x = new Vector6(0.1, 0.1, 0.1, 0.4, 0.2, 0.8);
        Pose3 expected = T1.compose(Pose3.statics.Expmap(x)).compose(T2);
        Vector6 y = T2.inverse().Adjoint(x);
        Pose3 actual = T1.compose(T2).compose(Pose3.statics.Expmap(y));
        assertTrue(assert_equal(expected, actual, 1e-6));
    }

    // Check compose and its pushforward
    // NOTE: testing::compose<Pose3>(t1,t2) = t1.compose(t2) (see lieProxies.h)
    @Test
    void testcompose() throws Throwable {
        Matrix actual = (T2.compose(T2)).matrix();
        Matrix expected = T2.matrix().compose(T2.matrix());
        assertTrue(assert_equal(actual, expected, 1e-8));

        Matrix actualDcompose1 = new Matrix();
        Matrix actualDcompose2 = new Matrix();
        T2.compose(T2, actualDcompose1, actualDcompose2);

        Matrix numericalH1 = NumericalDerivative.<//
                Pose3, Vector6, //
                Pose3, Vector6, //
                Pose3, Vector6>numericalDerivative21((a, b) -> a.compose(b), T2, T2, 1e-5);
        assertTrue(assert_equal(numericalH1, actualDcompose1, 5e-3));
        assertTrue(assert_equal(T2.inverse().AdjointMap(), actualDcompose1, 5e-3));

        Matrix numericalH2 = NumericalDerivative.<//
                Pose3, Vector6, //
                Pose3, Vector6, //
                Pose3, Vector6>numericalDerivative22((a, b) -> a.compose(b), T2, T2, 1e-5);
        assertTrue(assert_equal(numericalH2, actualDcompose2, 1e-4));
    }

    // Check compose and its pushforward, another case
    @Test
    void testcompose2() throws Throwable {
        Pose3 T1 = T;
        Matrix actual = (T1.compose(T2)).matrix();
        Matrix expected = T1.matrix().compose(T2.matrix());
        assertTrue(assert_equal(actual, expected, 1e-8));

        Matrix actualDcompose1 = new Matrix();
        Matrix actualDcompose2 = new Matrix();
        T1.compose(T2, actualDcompose1, actualDcompose2);

        Matrix numericalH1 = NumericalDerivative.<//
                Pose3, Vector6, //
                Pose3, Vector6, //
                Pose3, Vector6>numericalDerivative21((a, b) -> a.compose(b), T1, T2, 1e-5);
        assertTrue(assert_equal(numericalH1, actualDcompose1, 5e-3));
        assertTrue(assert_equal(T2.inverse().AdjointMap(), actualDcompose1, 5e-3));

        Matrix numericalH2 = NumericalDerivative.<//
                Pose3, Vector6, //
                Pose3, Vector6, //
                Pose3, Vector6>numericalDerivative22((a, b) -> a.compose(b), T1, T2, 1e-5);
        assertTrue(assert_equal(numericalH2, actualDcompose2, 1e-5));
    }

    @Test
    void testinverse() throws Throwable {
        Matrix actualDinverse = new Matrix();
        Matrix actual = T.inverse(actualDinverse).matrix();
        Matrix expected = T.matrix().inverse();
        assertTrue(assert_equal(actual, expected, 1e-8));

        Matrix numericalH = NumericalDerivative.<//
                Pose3, Vector6, //
                Pose3, Vector6>numericalDerivative11((a) -> a.inverse(), T, 1e-5);
        assertTrue(assert_equal(numericalH, actualDinverse, 5e-3));
        assertTrue(assert_equal(T.AdjointMap().times(-1), actualDinverse, 5e-3));
    }

    @Test
    void testinverseDerivatives2() throws Throwable {
        Rot3 R = Rot3.Rodrigues(0.3, 0.4, -0.5);
        Point3 t = new Point3(3.5, -8.2, 4.2);
        Pose3 T = new Pose3(R, t);

        Matrix numericalH = NumericalDerivative.<//
                Pose3, Vector6, //
                Pose3, Vector6>numericalDerivative11((a) -> a.inverse(), T, 1e-5);
        Matrix actualDinverse = new Matrix();
        T.inverse(actualDinverse);
        assertTrue(assert_equal(numericalH, actualDinverse, 5e-3));
        assertTrue(assert_equal(T.AdjointMap().times(-1), actualDinverse, 5e-3));
    }

    @Test
    void testcompose_inverse() throws Throwable {
        Matrix actual = (T.compose(T.inverse())).matrix();
        Matrix expected = Matrix.I_4x4();
        assertTrue(assert_equal(actual, expected, 1e-8));
    }

    // Point3 transformFrom_(final Pose3& pose, final Point3& point) {
    // return pose.transformFrom(point);
    // }

    @Test
    void testDtransform_from1_a() throws Throwable {
        Matrix actualDtransform_from1 = new Matrix();
        T.transformFrom(P, actualDtransform_from1, new Matrix());
        // Matrix numerical = numericalDerivative21(transformFrom_, T, P);
        // assertTrue(assert_equal(numerical, actualDtransform_from1, 1e-8));
    }

    @Test
    void testDtransform_from1_b() throws Throwable {
        Pose3 origin = new Pose3();
        Matrix actualDtransform_from1 = new Matrix();
        origin.transformFrom(P, actualDtransform_from1, new Matrix());
        // Matrix numerical = numericalDerivative21(transformFrom_, origin, P);
        // assertTrue(assert_equal(numerical, actualDtransform_from1, 1e-8));
    }

    @Test
    void testDtransform_from1_c() throws Throwable {
        Point3 origin = new Point3(0, 0, 0);
        Pose3 T0 = new Pose3(R, origin);
        Matrix actualDtransform_from1 = new Matrix();
        T0.transformFrom(P, actualDtransform_from1, new Matrix());
        // Matrix numerical = numericalDerivative21(transformFrom_, T0, P);
        // assertTrue(assert_equal(numerical, actualDtransform_from1, 1e-8));
    }

    @Test
    void testDtransform_from1_d() throws Throwable {
        Rot3 I = new Rot3();
        Point3 t0 = new Point3(100, 0, 0);
        Pose3 T0 = new Pose3(I, t0);
        Matrix actualDtransform_from1 = new Matrix();
        T0.transformFrom(P, actualDtransform_from1, new Matrix());
        // Matrix numerical = numericalDerivative21(transformFrom_, T0, P);
        // assertTrue(assert_equal(numerical, actualDtransform_from1, 1e-8));
    }

    @Test
    void testDtransform_from2() throws Throwable {
        Matrix actualDtransform_from2 = new Matrix();
        T.transformFrom(P, new Matrix(), actualDtransform_from2);
        // Matrix numerical = numericalDerivative22(transformFrom_, T, P);
        // assertTrue(assert_equal(numerical, actualDtransform_from2, 1e-8));
    }

    // Point3 transform_to_(final Pose3& pose, final Point3& point) {
    // return pose.transformTo(point);
    // }
    @Test
    void testDtransform_to1() throws Throwable {
        Matrix computed = new Matrix();
        T.transformTo(P, computed, new Matrix());
        // Matrix numerical = numericalDerivative21(transform_to_, T, P);
        // assertTrue(assert_equal(numerical, computed, 1e-8));
    }

    @Test
    void testDtransform_to2() throws Throwable {
        Matrix computed = new Matrix();
        T.transformTo(P, new Matrix(), computed);
        // Matrix numerical = numericalDerivative22(transform_to_, T, P);
        // assertTrue(assert_equal(numerical, computed, 1e-8));
    }

    @Test
    void testtransform_to_with_derivatives() throws Throwable {
        Matrix actH1 = new Matrix();
        Matrix actH2 = new Matrix();
        T.transformTo(P, actH1, actH2);
        // Matrix expH1 = numericalDerivative21(transform_to_, T, P),
        // expH2 = numericalDerivative22(transform_to_, T, P);
        // assertTrue(assert_equal(expH1, actH1, 1e-8));
        // assertTrue(assert_equal(expH2, actH2, 1e-8));
    }

    @Test
    void testtransform_from_with_derivatives() throws Throwable {
        Matrix actH1 = new Matrix();
        Matrix actH2 = new Matrix();
        T.transformFrom(P, actH1, actH2);
        // Matrix expH1 = numericalDerivative21(transformFrom_, T, P),
        // expH2 = numericalDerivative22(transformFrom_, T, P);
        // assertTrue(assert_equal(expH1, actH1, 1e-8));
        // assertTrue(assert_equal(expH2, actH2, 1e-8));
    }

    @Test
    void testtransform_to_translate() throws Throwable {
        Point3 actual = new Pose3(new Rot3(), new Point3(1, 2, 3))
                .transformTo(new Point3(10., 20., 30.));
        Point3 expected = new Point3(9., 18., 27.);
        assertTrue(assert_equal(expected, actual));
    }

    @Test
    void testtransform_to_rotate() throws Throwable {
        Pose3 transform = new Pose3(Rot3.Rodrigues(0, 0, -1.570796), new Point3(0, 0, 0));
        Point3 actual = transform.transformTo(new Point3(2, 1, 10));
        Point3 expected = new Point3(-1, 2, 10);
        assertTrue(assert_equal(expected, actual, 0.001));
    }

    // // Check transformPoseFrom and its pushforward
    // Pose3 transformPoseFrom_(final Pose3& wTa, final Pose3& aTb) {
    // return wTa.transformPoseFrom(aTb);
    // }

    @Test
    void testtransformPoseFrom() throws Throwable {
        Matrix actual = (T2.compose(T2)).matrix();
        Matrix expected = T2.matrix().compose(T2.matrix());
        assertTrue(assert_equal(actual, expected, 1e-8));

        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        // T2.transformPoseFrom(T2, H1, H2);

        // Matrix numericalH1 = numericalDerivative21(transformPoseFrom_, T2, T2);
        // assertTrue(assert_equal(numericalH1, H1, 5e-3));
        // assertTrue(assert_equal(T2.inverse().AdjointMap(), H1, 5e-3));

        // Matrix numericalH2 = numericalDerivative22(transformPoseFrom_, T2, T2);
        // assertTrue(assert_equal(numericalH2, H2, 1e-4));
    }

    @Test
    void testtransformTo() {
        // Pose3 transform(Rot3::Rodrigues(0, 0, -1.570796), Point3(2, 4, 0));
        // Point3 actual = transform.transformTo(Point3(3, 2, 10));
        // Point3 expected(2, 1, 10);
        // assertTrue(assert_equal(expected, actual, 0.001));
    }

    // Pose3 transformPoseTo_(final Pose3& pose, final Pose3& pose2) {
    // return pose.transformPoseTo(pose2);
    // }

    @Test
    void testtransformPoseTo() {
        // Pose3 origin = T.transformPoseTo(T);
        // assertTrue(assert_equal(Pose3{}, origin));
    }

    @Test
    void testtransformPoseTo_with_derivatives() throws Throwable {
        Matrix actH1 = new Matrix();
        Matrix actH2 = new Matrix();
        // Pose3 res = T.transformPoseTo(T2, actH1, actH2);
        // assertTrue(assert_equal(res, T.inverse().compose(T2)));

        // Matrix expH1 = numericalDerivative21(transformPoseTo_, T, T2),
        // expH2 = numericalDerivative22(transformPoseTo_, T, T2);
        // assertTrue(assert_equal(expH1, actH1, 1e-8));
        // assertTrue(assert_equal(expH2, actH2, 1e-8));
    }

    @Test
    void testtransformPoseTo_with_derivatives2() throws Throwable {
        Matrix actH1 = new Matrix();
        Matrix actH2 = new Matrix();
        // Pose3 res = T.transformPoseTo(T3, actH1, actH2);
        // assertTrue(assert_equal(res, T.inverse().compose(T3)));

        // Matrix expH1 = numericalDerivative21(transformPoseTo_, T, T3),
        // expH2 = numericalDerivative22(transformPoseTo_, T, T3);
        // assertTrue(assert_equal(expH1, actH1, 1e-8));
        // assertTrue(assert_equal(expH2, actH2, 1e-8));
    }

    @Test
    void testtransformFrom() throws Throwable {
        Point3 actual = T3.transformFrom(new Point3(0, 0, 0));
        Point3 expected = new Point3(1., 2., 3.);
        assertTrue(assert_equal(expected, actual));
    }

    @Test
    void testtransform_roundtrip() throws Throwable {
        Point3 actual = T3.transformFrom(T3.transformTo(new Point3(12., -0.11, 7.0)));
        Point3 expected = new Point3(12., -0.11, 7.0);
        assertTrue(assert_equal(expected, actual));
    }

    @Test
    void testRetract_LocalCoordinates() throws Throwable {
        Vector6 d = new Vector6(1, 2, 3, 4, 5, 6);
        d = d.times(0.1);
        // final Rot3 R = Rot3::Retract(d.head<3>());
        // Pose3 t = Pose3::Retract(d);
        // assertTrue(assert_equal(d, Pose3::LocalCoordinates(t)));
    }

    @Test
    void testretract_localCoordinates() throws Throwable {
        Vector6 d12 = new Vector6(1, 2, 3, 4, 5, 6);
        d12 = d12.times(0.1);
        Pose3 t1 = T;
        Pose3 t2 = t1.retract(d12);
        assertTrue(assert_equal(d12, t1.localCoordinates(t2)));
    }

    @Test
    void testexpmap_logmap() throws Throwable {
        Vector6 d12 = new Vector6(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
        Pose3 t1 = T;
        Pose3 t2 = t1.expmap(d12);
        assertTrue(assert_equal(d12, t1.logmap(t2)));
    }

    @Test
    void testretract_localCoordinates2() throws Throwable {
        Pose3 t1 = T;
        Pose3 t2 = T3;
        Vector6 d12 = t1.localCoordinates(t2);
        assertTrue(assert_equal(t2, t1.retract(d12)));
        Vector6 d21 = t2.localCoordinates(t1);
        assertTrue(assert_equal(t1, t2.retract(d21)));
        // TODO(hayk): This currently fails!
        // assertTrue(assert_equal(d12, -d21));
    }

    @Test
    void testmanifold_expmap() throws Throwable {
        Pose3 t1 = T;
        Pose3 t2 = T3;
        Vector6 d12 = t1.logmap(t2);
        assertTrue(assert_equal(t2, t1.expmap(d12)));
        Vector6 d21 = t2.logmap(t1);
        assertTrue(assert_equal(t1, t2.expmap(d21)));

        // Check that log(t1,t2)=-log(t2,t1)
        assertTrue(assert_equal(d12, d21.times(-1)));
    }

    @Test
    void testsubgroups() throws Throwable {
        // Frank - Below only works for correct "Agrawal06iros style expmap
        // lines in canonical coordinates correspond to Abelian subgroups in SE(3)
        Vector6 d = new Vector6(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
        // exp(-d)=inverse(exp(d))
        assertTrue(assert_equal(Pose3.statics.Expmap(d.times(-1)), Pose3.statics.Expmap(d).inverse()));
        // exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
        Pose3 T2 = Pose3.statics.Expmap(d.times(2));
        Pose3 T3 = Pose3.statics.Expmap(d.times(3));
        Pose3 T5 = Pose3.statics.Expmap(d.times(5));
        assertTrue(assert_equal(T5, T2.compose(T3)));
        assertTrue(assert_equal(T5, T3.compose(T2)));
    }

    @Test
    void testbetween() throws Throwable {
        Pose3 expected = T2.inverse().compose(T3);
        Matrix actualDBetween1 = new Matrix();
        Matrix actualDBetween2 = new Matrix();
        Pose3 actual = T2.between(T3, actualDBetween1, actualDBetween2);
        assertTrue(assert_equal(expected, actual));

        Matrix numericalH1 = NumericalDerivative.<//
                Pose3, Vector6, //
                Pose3, Vector6, //
                Pose3, Vector6>numericalDerivative21((a, b) -> a.between(b), T2, T3, 1e-5);
        assertTrue(assert_equal(numericalH1, actualDBetween1, 5e-3));

        Matrix numericalH2 = NumericalDerivative.<//
                Pose3, Vector6, //
                Pose3, Vector6, //
                Pose3, Vector6>numericalDerivative22((a, b) -> a.between(b), T2, T3, 1e-5);
        assertTrue(assert_equal(numericalH2, actualDBetween2, 1e-5));
    }

    // some shared test values - pulled from equivalent test in Pose2
    static Point3 l1;
    static Point3 l2;
    static Point3 l3;
    static Point3 l4;
    static Pose3 x1;
    static Pose3 x2;
    static Pose3 x3;
    static Pose3 xl1;
    static Pose3 xl2;
    static Pose3 xl3;
    static Pose3 xl4;

    static {
        try {
            l1 = new Point3(1, 0, 0);
            l2 = new Point3(1, 1, 0);
            l3 = new Point3(2, 2, 0);
            l4 = new Point3(1, 4, -4);
            x1 = new Pose3();
            x2 = new Pose3(Rot3.Ypr(0.0, 0.0, 0.0), l2);
            x3 = new Pose3(Rot3.Ypr(Math.PI / 4.0, 0.0, 0.0), l2);
            xl1 = new Pose3(Rot3.Ypr(0.0, 0.0, 0.0), new Point3(1, 0, 0));
            xl2 = new Pose3(Rot3.Ypr(0.0, 1.0, 0.0), new Point3(1, 1, 0));
            xl3 = new Pose3(Rot3.Ypr(1.0, 0.0, 0.0), new Point3(2, 2, 0));
            xl4 = new Pose3(Rot3.Ypr(0.0, 0.0, 1.0), new Point3(1, 4, -4));
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }
    }

    Vector1 range_proxy(Pose3 pose, Point3 point) throws Throwable {
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

        // establish range is indeed sqrt2
        assertEquals(Math.sqrt(2.0), x1.range(l2), 1e-9);

        // Another pair
        double actual23 = x2.range(l3, actualH1, actualH2);
        assertEquals(Math.sqrt(2.0), actual23, 1e-9);

        // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose3, Vector6, //
                Point3, Vector3>numericalDerivative21(this::range_proxy, x2, l3, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose3, Vector6, //
                Point3, Vector3>numericalDerivative22(this::range_proxy, x2, l3, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));

        // Another test
        double actual34 = x3.range(l4, actualH1, actualH2);
        assertEquals(5, actual34, 1e-9);

        // // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose3, Vector6, //
                Point3, Vector3>numericalDerivative21(this::range_proxy, x3, l4, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose3, Vector6, //
                Point3, Vector3>numericalDerivative22(this::range_proxy, x3, l4, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));
    }

    Vector1 range_pose_proxy(Pose3 pose, Pose3 point) throws Throwable {
        return new Vector1(pose.range(point));
    }

    @Test
    void testrange_pose() throws Throwable {
        Matrix expectedH1 = new Matrix();
        Matrix actualH1 = new Matrix();
        Matrix expectedH2 = new Matrix();
        Matrix actualH2 = new Matrix();

        // establish range is indeed zero
        assertEquals(1, x1.range(xl1), 1e-9);

        // establish range is indeed sqrt2
        assertEquals(Math.sqrt(2.0), x1.range(xl2), 1e-9);

        // Another pair
        double actual23 = x2.range(xl3, actualH1, actualH2);
        assertEquals(Math.sqrt(2.0), actual23, 1e-9);

        // // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose3, Vector6, //
                Pose3, Vector6>numericalDerivative21(this::range_pose_proxy, x2, xl3, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose3, Vector6, //
                Pose3, Vector6>numericalDerivative22(this::range_pose_proxy, x2, xl3, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));

        // Another test
        double actual34 = x3.range(xl4, actualH1, actualH2);
        assertEquals(5, actual34, 1e-9);

        // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose3, Vector6, //
                Pose3, Vector6>numericalDerivative21(this::range_pose_proxy, x3, xl4, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Vector1, Vector1, //
                Pose3, Vector6, //
                Pose3, Vector6>numericalDerivative22(this::range_pose_proxy, x3, xl4, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));
    }

    Unit3 bearing_proxy(Pose3 pose, Point3 point) throws Throwable {
        return pose.bearing(point);
    }

    @Test
    void testBearing() throws Throwable {
        Matrix expectedH1 = new Matrix();
        Matrix actualH1 = new Matrix();
        Matrix expectedH2 = new Matrix();
        Matrix actualH2 = new Matrix();

        assertTrue(assert_equal(new Unit3(1, 0, 0), x1.bearing(l1, actualH1, actualH2), 1e-9));

        // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Unit3, Vector2, //
                Pose3, Vector6, //
                Point3, Vector3>numericalDerivative21(this::bearing_proxy, x1, l1, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Unit3, Vector2, //
                Pose3, Vector6, //
                Point3, Vector3>numericalDerivative22(this::bearing_proxy, x1, l1, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1, 1e-5));
        assertTrue(assert_equal(expectedH2, actualH2, 1e-5));
    }

    @Test
    void testBearing2() throws Throwable {
        Matrix expectedH1 = new Matrix();
        Matrix actualH1 = new Matrix();
        Matrix expectedH2 = new Matrix();
        Matrix actualH2 = new Matrix();

        assertTrue(assert_equal(new Unit3(0, 0.6, -0.8), x2.bearing(l4, actualH1, actualH2), 1e-9));

        // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Unit3, Vector2, //
                Pose3, Vector6, //
                Point3, Vector3>numericalDerivative21(this::bearing_proxy, x2, l4, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Unit3, Vector2, //
                Pose3, Vector6, //
                Point3, Vector3>numericalDerivative22(this::bearing_proxy, x2, l4, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1, 1e-5));
        assertTrue(assert_equal(expectedH2, actualH2, 1e-5));
    }

    @Test
    void testPoseToPoseBearing() throws Throwable {
        Matrix expectedH1 = new Matrix();
        Matrix actualH1 = new Matrix();
        Matrix expectedH2 = new Matrix();
        Matrix actualH2 = new Matrix();
        Matrix H2block = new Matrix();

        assertTrue(assert_equal(new Unit3(0, 1, 0), xl1.bearing(xl2, actualH1, actualH2), 1e-9));

        // Check numerical derivatives
        ThrowingFunction2<Pose3, Pose3, Unit3> f = (a, b) -> a.bearing(b);
        expectedH1 = NumericalDerivative.<//
                Unit3, Vector2, //
                Pose3, Vector6, //
                Pose3, Vector6>numericalDerivative21(f, xl1, xl2, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Unit3, Vector2, //
                Pose3, Vector6, //
                Pose3, Vector6>numericalDerivative22(f, xl1, xl2, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1, 1e-5));
        assertTrue(assert_equal(expectedH2, actualH2, 1e-5));
    }

    @Test
    void testExpmapDerivative() {
        // // Iserles05an (Lie-group Methods) says:
        // // scalar is easy: d exp(a(t)) / dt = exp(a(t)) a'(t)
        // // matrix is hard: d exp(A(t)) / dt = exp(A(t)) dexp[-A(t)] A'(t)
        // // where A(t): T -> se(3) is a trajectory in the tangent space of SE(3)
        // // and dexp[A] is a linear map from 4*4 to 4*4 derivatives of se(3)
        // // Hence, the above matrix equation is typed: 4*4 = SE(3) * linear_map(4*4)

        // // In GTSAM, we don't work with the Lie-algebra elements A directly, but with
        // 6-vectors.
        // // xi is easy: d Expmap(xi(t)) / dt = ExmapDerivative[xi(t)] * xi'(t)

        // // Let's verify the above formula.

        // auto xi = [](double t) {
        // Vector6 v;
        // v << 2 * t, sin(t), 4 * t * t, 2 * t, sin(t), 4 * t * t;
        // return v;
        // };
        // auto xi_dot = [](double t) {
        // Vector6 v;
        // v << 2, cos(t), 8 * t, 2, cos(t), 8 * t;
        // return v;
        // };

        // // We define a function T
        // auto T = [xi](double t) { return Pose3::Expmap(xi(t)); };

        // for (double t = -2.0; t < 2.0; t += 0.3) {
        // final Matrix expected = numericalDerivative11<Pose3, double>(T, t);
        // final Matrix actual = Pose3::ExpmapDerivative(xi(t)) * xi_dot(t);
        // CHECK(assert_equal(expected, actual, 1e-7));
        // }
    }

    static class pose3_test_cases {
        static List<Vector3> small;
        static List<Vector3> large;
        static Function<Boolean, List<Vector3>> omegas = (nearZero) -> nearZero ? small : large;
        static List<Vector3> vs;

        static {
            try {
                small = List.of(new Vector3(0, 0, 0), //
                        new Vector3(1e-5, 0, 0), new Vector3(0, 1e-5, 0), new Vector3(0, 0, 1e-5), // ,
                        new Vector3(1e-4, 0, 0), new Vector3(0, 1e-4, 0), new Vector3(0, 0, 1e-4));

                large = List.of(new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0),
                        new Vector3(0, 0, 1), new Vector3(.1, .2, .3), new Vector3(1, -2, 3));

                vs = List.of(new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1),
                        new Vector3(.4, .3, .2), new Vector3(4, 5, 6), new Vector3(-10, -20, 30));
            } catch (Throwable t) {
                throw new RuntimeException(t);
            }
        }
    }

    @Test
    void testExpmapDerivatives() throws Throwable {
        for (boolean nearZero : List.of(true, false)) {
            for (final Vector3 w : pose3_test_cases.omegas.apply(nearZero)) {
                for (Vector3 v : pose3_test_cases.vs) {
                    final Vector6 xi = new Vector6(w, v);
                    ThrowingFunction<Vector6, Pose3> f = (x) -> Pose3.statics.Expmap(x);
                    Matrix expectedH = NumericalDerivative.<Pose3, Vector6, //
                            Vector6, Vector6>numericalDerivative11(f, xi, 1e-5);
                    Matrix actualH = new Matrix();
                    Pose3.statics.Expmap(xi, actualH);
                    assertTrue(assert_equal(expectedH, actualH));
                }
            }
        }
    }

    // Check logmap for all small values, as we don't want wrapping.
    @Test
    void testLogmap() throws Throwable {
        boolean nearZero = true;
        for (final Vector3 w : pose3_test_cases.omegas.apply(nearZero)) {
            for (Vector3 v : pose3_test_cases.vs) {
                final Vector6 xi = new Vector6(w, v);
                Pose3 pose = Pose3.statics.Expmap(xi);
                assertTrue(assert_equal(xi, Pose3.statics.Logmap(pose)));
            }
        }
    }

    // Check logmap derivatives for all values
    @Test
    void testLogmapDerivatives() throws Throwable {
        for (boolean nearZero : List.of(true, false)) {
            for (final Vector3 w : pose3_test_cases.omegas.apply(nearZero)) {
                for (Vector3 v : pose3_test_cases.vs) {
                    final Vector6 xi = new Vector6(w, v);
                    Pose3 pose = Pose3.statics.Expmap(xi);
                    // final Matrix6 expectedH =
                    // numericalDerivative21<Vector6, Pose3, OptionalJacobian<6, 6> >(
                    // &Pose3::Logmap, pose, {});
                    // Matrix actualH;
                    // Pose3::Logmap(pose, actualH);
                    // #ifdef GTSAM_USE_QUATERNIONS
                    // // TODO(Frank): Figure out why quaternions are not as accurate.
                    // // Hint: 6 cases fail on Ubuntu 22.04, but none on MacOS.
                    // assertTrue(assert_equal(expectedH, actualH, 1e-7));
                    // #else
                    // assertTrue(assert_equal(expectedH, actualH));
                    // #endif
                }
            }
        }
    }

    @Test
    void testLogmapDerivative() throws Throwable {
        // Copied from testSO3.cpp
        final Rot3 R2 = new Rot3( // Near pi
                -0.750767, -0.0285082, -0.659952, //
                -0.0102558, -0.998445, 0.0547974, //
                -0.660487, 0.0479084, 0.749307);
        final Rot3 R3 = new Rot3(// Near pi
                -0.747473, -0.00190019, -0.664289, //
                -0.0385114, -0.99819, 0.0461892, //
                -0.663175, 0.060108, 0.746047);
        final Rot3 R4 = new Rot3( // Final pose in a drone experiment
                0.324237, 0.902975, 0.281968, //
                -0.674322, 0.429668, -0.600562, //
                -0.663445, 0.00458662, 0.748211);

        // Now creates poses
        final Pose3 T0 = new Pose3(); // Identity
        final Vector6 xi = new Vector6(0.1, -0.1, 0.1, 0.1, -0.1, 0.1);
        final Pose3 T1 = Pose3.statics.Expmap(xi); // Small rotation
        final Pose3 T2 = new Pose3(R2, new Point3(1, 2, 3));
        final Pose3 T3 = new Pose3(R3, new Point3(1, 2, 3));
        final Pose3 T4 = new Pose3(R4, new Point3(1, 2, 3));
        int i = 0;
        for (final Pose3 T : List.of(T0, T1, T2, T3, T4)) {
            // final bool nearPi = (i == 2 || i == 3); // Flag cases near pi

            // Matrix6 actualH; // H computed by Logmap(T, H) using LogmapDerivative(xi)
            // final Vector6 xi = Pose3::Logmap(T, actualH);

            // // 1. Check self-consistency of analytical derivative calculation:
            // // Does the H returned by Logmap match an independent calculation
            // // of J_r^{-1} using ExpmapDerivative with the computed xi?
            // Matrix6 J_r_inv = Pose3::ExpmapDerivative(xi).inverse(); // J_r^{-1}
            // assertTrue(assert_equal(J_r_inv, actualH));
            // // This test is crucial and should pass

            // // 2. Check analytical derivative against numerical derivative:
            // // Only perform this check AWAY from the pi singularity, where
            // // numerical differentiation of Logmap is expected to be reliable
            // // and should match the analytical derivative.
            // if (!nearPi) {
            // final Matrix expectedH = numericalDerivative11<Vector6, Pose3>(
            // std::bind(&Pose3::Logmap, std::placeholders::_1, nullptr), T, 1e-7);
            // assertTrue(assert_equal(expectedH, actualH, 1e-5)); // 1e-5 needed to pass R4
            // } else {
            // // We accept that the numerical derivative of this specific Logmap
            // implementation
            // // near pi will not match the standard analytical derivative J_r^{-1}.
            // }
            i++;
        }
    }

    @Test
    void testInvariants() throws Throwable {
        Pose3 id = new Pose3();

        assertTrue(Pose3.check_group_invariants(id, id));
        assertTrue(Pose3.check_group_invariants(id, T3));
        assertTrue(Pose3.check_group_invariants(T2, id));
        assertTrue(Pose3.check_group_invariants(T2, T3));

        assertTrue(Pose3.check_manifold_invariants(id, id));
        assertTrue(Pose3.check_manifold_invariants(id, T3));
        assertTrue(Pose3.check_manifold_invariants(T2, id));
        assertTrue(Pose3.check_manifold_invariants(T2, T3));
    }

    @Test
    void testinterpolate() throws Throwable {
        assertTrue(assert_equal(T2, Pose3.interpolate(T2, T3, 0.0)));
        assertTrue(assert_equal(T3, Pose3.interpolate(T2, T3, 1.0)));

        // Trivial example: start at origin and move to (1, 0, 0) while rotating pi/2
        // about z-axis.
        Pose3 start = new Pose3();
        Pose3 end = new Pose3(Rot3.Rz(Math.PI / 2), new Point3(1, 0, 0));
        // This interpolation is easy to calculate by hand.
        double t = 0.5;
        Pose3 expected0 = new Pose3(Rot3.Rz(Math.PI / 4), new Point3(0.5, 0, 0));
        assertTrue(assert_equal(expected0, start.interpolateRt(end, t)));

        // Example from Peter Corke
        // https://robotacademy.net.au/lesson/interpolating-pose-in-3d/
        t = 0.0759; // corresponds to the 10th element when calling `ctraj` in
        // the video
        Pose3 O = new Pose3();
        Pose3 F = new Pose3(
                Rot3.Roll(0.6).compose(Rot3.Pitch(0.8)).compose(Rot3.Yaw(1.4)),
                new Point3(1, 2, 3));

        // The expected answer matches the result presented in the video.
        // Pose3 expected1(interpolate(O.rotation(), F.rotation(), t),
        // interpolate(O.translation(), F.translation(), t));
        // assertTrue(assert_equal(expected1, O.interpolateRt(F, t)));

        // // Non-trivial interpolation, translation value taken from output.
        // Pose3 expected2(interpolate(T2.rotation(), T3.rotation(), t),
        // interpolate(T2.translation(), T3.translation(), t));
        // assertTrue(assert_equal(expected2, T2.interpolateRt(T3, t)));
    }

    // Pose3 testing_interpolate(final Pose3& t1, final Pose3& t2, double gamma) {
    // return interpolate(t1,t2,gamma); }

    @Test
    void testinterpolateJacobians() throws Throwable {
        {
            Pose3 X = Pose3.statics.Identity();
            Pose3 Y = new Pose3(Rot3.Rz(Math.PI / 2), new Point3(1, 0, 0));
            double t = 0.5;
            // Pose3 expectedPoseInterp(Rot3::Rz(M_PI_4), Point3(0.5, -0.207107, 0)); //
            // note: different from test above: this is full Pose3 interpolation
            // Matrix actualJacobianX, actualJacobianY, actualJacobianT;
            // assertTrue(assert_equal(expectedPoseInterp, interpolate(X, Y, t,
            // actualJacobianX,
            // actualJacobianY, actualJacobianT), 1e-5));

            // Matrix expectedJacobianX =
            // numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

            // Matrix expectedJacobianY =
            // numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianY,actualJacobianY,1e-6));

            // Matrix expectedJacobianT =
            // numericalDerivative33<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianT,actualJacobianT,1e-6));
        }
        {
            Pose3 X = Pose3.statics.Identity();
            Pose3 Y = new Pose3(Rot3.statics.Identity(), new Point3(1, 0, 0));
            double t = 0.3;
            // Pose3 expectedPoseInterp(Rot3::Identity(), Point3(0.3, 0, 0));
            // Matrix actualJacobianX, actualJacobianY, actualJacobianT;
            // assertTrue(assert_equal(expectedPoseInterp, interpolate(X, Y, t,
            // actualJacobianX,
            // actualJacobianY, actualJacobianT), 1e-5));

            // Matrix expectedJacobianX =
            // numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

            // Matrix expectedJacobianY =
            // numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianY,actualJacobianY,1e-6));

            // Matrix expectedJacobianT =
            // numericalDerivative33<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianT,actualJacobianT,1e-6));
        }
        {
            Pose3 X = Pose3.statics.Identity();
            Pose3 Y = new Pose3(Rot3.Rz(Math.PI / 2), new Point3(0, 0, 0));
            double t = 0.5;
            // Pose3 expectedPoseInterp(Rot3::Rz(M_PI_4), Point3(0, 0, 0));
            // Matrix actualJacobianX, actualJacobianY, actualJacobianT;
            // assertTrue(assert_equal(expectedPoseInterp, interpolate(X, Y, t,
            // actualJacobianX,
            // actualJacobianY, actualJacobianT), 1e-5));

            // Matrix expectedJacobianX =
            // numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

            // Matrix expectedJacobianY =
            // numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianY,actualJacobianY,1e-6));

            // Matrix expectedJacobianT =
            // numericalDerivative33<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianT,actualJacobianT,1e-6));
        }
        {
            Pose3 X = new Pose3(Rot3.Ypr(0.1, 0.2, 0.3), new Point3(10, 5, -2));
            Pose3 Y = new Pose3(Rot3.Ypr(1.1, -2.2, -0.3), new Point3(-5, 1, 1));
            double t = 0.3;
            Pose3 expectedPoseInterp = new Pose3(Rot3.Rz(Math.PI / 4), new Point3(0, 0, 0));
            Matrix actualJacobianX = new Matrix();
            Matrix actualJacobianY = new Matrix();
            Matrix actualJacobianT = new Matrix();
            // interpolate(X, Y, t, actualJacobianX, actualJacobianY, actualJacobianT);

            // Matrix expectedJacobianX =
            // numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

            // Matrix expectedJacobianY =
            // numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianY,actualJacobianY,1e-6));

            // Matrix expectedJacobianT =
            // numericalDerivative33<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianT,actualJacobianT,1e-6));
        }
    }

    // Pose3 testing_interpolate_rt(final Pose3& t1, final Pose3& t2, double gamma)
    // { return t1.interpolateRt(t2, gamma); }

    @Test
    void testinterpolateRtJacobians() throws Throwable {
        {
            Pose3 X = Pose3.statics.Identity();
            Pose3 Y = new Pose3(Rot3.Rz(Math.PI / 2), new Point3(1, 0, 0));
            double t = 0.5;
            Matrix actualJacobianX = new Matrix();
            Matrix actualJacobianY = new Matrix();
            Matrix actualJacobianT = new Matrix();
            // X.interpolateRt(Y, t, actualJacobianX, actualJacobianY, actualJacobianT);

            // Matrix expectedJacobianX =
            // numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate_rt, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

            // Matrix expectedJacobianY =
            // numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate_rt, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianY,actualJacobianY,1e-6));

            // Matrix expectedJacobianT =
            // numericalDerivative33<Pose3,Pose3,Pose3,double>(testing_interpolate_rt, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianT,actualJacobianT,1e-6));
        }
        {
            Pose3 X = Pose3.statics.Identity();
            Pose3 Y = new Pose3(Rot3.statics.Identity(), new Point3(1, 0, 0));
            double t = 0.3;
            Matrix actualJacobianX = new Matrix();
            Matrix actualJacobianY = new Matrix();
            Matrix actualJacobianT = new Matrix();
            // X.interpolateRt(Y, t, actualJacobianX, actualJacobianY, actualJacobianT);

            // Matrix expectedJacobianX =
            // numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate_rt, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

            // Matrix expectedJacobianY =
            // numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate_rt, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianY,actualJacobianY,1e-6));

            // Matrix expectedJacobianT =
            // numericalDerivative33<Pose3,Pose3,Pose3,double>(testing_interpolate_rt, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianT,actualJacobianT,1e-6));
        }
        {
            Pose3 X = Pose3.statics.Identity();
            Pose3 Y = new Pose3(Rot3.Rz(Math.PI / 2), new Point3(0, 0, 0));
            double t = 0.5;
            Matrix actualJacobianX = new Matrix();
            Matrix actualJacobianY = new Matrix();
            Matrix actualJacobianT = new Matrix();
            // X.interpolateRt(Y, t, actualJacobianX, actualJacobianY, actualJacobianT);

            // Matrix expectedJacobianX =
            // numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate_rt, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

            // Matrix expectedJacobianY =
            // numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate_rt, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianY,actualJacobianY,1e-6));

            // Matrix expectedJacobianT =
            // numericalDerivative33<Pose3,Pose3,Pose3,double>(testing_interpolate_rt, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianT,actualJacobianT,1e-6));
        }
        {
            Pose3 X = new Pose3(Rot3.Ypr(0.1, 0.2, 0.3), new Point3(10, 5, -2));
            Pose3 Y = new Pose3(Rot3.Ypr(1.1, -2.2, -0.3), new Point3(-5, 1, 1));
            double t = 0.3;
            Matrix actualJacobianX = new Matrix();
            Matrix actualJacobianY = new Matrix();
            Matrix actualJacobianT = new Matrix();
            // X.interpolateRt(Y, t, actualJacobianX, actualJacobianY, actualJacobianT);

            // Matrix expectedJacobianX =
            // numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate_rt, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

            // Matrix expectedJacobianY =
            // numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate_rt, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianY,actualJacobianY,1e-6));

            // Matrix expectedJacobianT =
            // numericalDerivative33<Pose3,Pose3,Pose3,double>(testing_interpolate_rt, X, Y,
            // t);
            // assertTrue(assert_equal(expectedJacobianT,actualJacobianT,1e-6));
        }
    }

    @Test
    void testexpressionWrappers() throws Throwable {
        Pose3 X = new Pose3(Rot3.Ypr(0.1, 0.2, 0.3), new Point3(10, 5, -2));
        Pose3 Y = new Pose3(Rot3.Ypr(1.1, -2.2, -0.3), new Point3(-5, 1, 1));
        double t = 0.3;
        Values vals = new Values();
        vals.insert(new Key(0), X);
        vals.insert(new Key(1), Y);
        // vals.insert(2,t);

        {
            // interpolate (templated wrapper applies to all classes)
            // Matrix expectedJacobianX, expectedJacobianY, expectedJacobianT;
            // std::vector<Matrix> Hlist = {{},{},{}};
            // Pose3 expected = interpolate(X, Y, t, expectedJacobianX, expectedJacobianY,
            // expectedJacobianT);
            // Pose3 actual = interpolate(Pose3_(Key(0)), Pose3_(Key(1)),
            // Double_(Key(2))).value(vals, Hlist);

            // assertTrue(assert_equal(expected,actual,1e-6));
            // assertTrue(assert_equal(expectedJacobianX,Hlist[0],1e-6));
            // assertTrue(assert_equal(expectedJacobianY,Hlist[1],1e-6));
            // assertTrue(assert_equal(expectedJacobianT,Hlist[2],1e-6));
        }
        {
            // interpolateRt (Pose3 specialisation)
            // Matrix expectedJacobianX, expectedJacobianY, expectedJacobianT;
            // std::vector<Matrix> Hlist = {{},{},{}};
            // Pose3 expected = X.interpolateRt(Y, t, expectedJacobianX, expectedJacobianY,
            // expectedJacobianT);
            // Pose3 actual = interpolateRt(Pose3_(Key(0)), Pose3_(Key(1)),
            // Double_(Key(2))).value(vals, Hlist);

            // assertTrue(assert_equal(expected,actual,1e-6));
            // assertTrue(assert_equal(expectedJacobianX,Hlist[0],1e-6));
            // assertTrue(assert_equal(expectedJacobianY,Hlist[1],1e-6));
            // assertTrue(assert_equal(expectedJacobianT,Hlist[2],1e-6));
        }
    }

    @Test
    void testCreate() throws Throwable {
        Matrix actualH1 = new Matrix();
        Matrix actualH2 = new Matrix();
        // Pose3 actual = Pose3::Create(R, P2, actualH1, actualH2);
        // assertTrue(assert_equal(T, actual));
        // auto create = [](Rot3 R, Point3 t) { return Pose3::Create(R, t); };
        // assertTrue(assert_equal(numericalDerivative21<Pose3,Rot3,Point3>(create, R,
        // P2),
        // actualH1, 1e-9));
        // assertTrue(assert_equal(numericalDerivative22<Pose3,Rot3,Point3>(create, R,
        // P2),
        // actualH2, 1e-9));
    }

    @Test
    void testExpmapChainRule() throws Throwable {
        // Muliply with an arbitrary matrix and exponentiate
        Matrix M = new Matrix(new double[][] {
                { 1, 2, 3, 4, 5, 6 }, //
                { 7, 8, 9, 1, 2, 3 }, //
                { 4, 5, 6, 7, 8, 9 }, //
                { 1, 2, 3, 4, 5, 6 }, //
                { 7, 8, 9, 1, 2, 3 }, //
                { 4, 5, 6, 7, 8, 9 } });

        ThrowingFunction<Vector6, Pose3> g = (omega) -> Pose3.statics.Expmap(
                new Vector6(M.times(new Vector(omega))));

        // Test the derivatives at zero
        Matrix expected = NumericalDerivative.<Pose3, Vector6, //
                Vector6, Vector6>numericalDerivative11(g, new Vector6(), 1e-5);
        assertTrue(assert_equal(expected, M, 1e-5)); //
        // Pose3::ExpmapDerivative(Z_63) is identity

        // // Test the derivatives at another value
        final Vector6 delta = new Vector6(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
        Matrix expected2 = NumericalDerivative.<Pose3, Vector6, //
                Vector6, Vector6>numericalDerivative11(g, delta, 1e-5);
        // final Matrix6 analytic = Pose3::ExpmapDerivative(M*delta) * M;
        // assertTrue(assert_equal<Matrix6>(expected2, analytic, 1e-5)); // note
        // tolerance
    }

}
