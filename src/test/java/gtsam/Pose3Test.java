package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.function.Function;

import org.junit.jupiter.api.Test;

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
    void testconstructors() throws Throwable {
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
    void testexpmap_a_full() {
        // Pose3 id;
        // Vector v = Z_6x1;
        // v(0) = 0.3;
        // assertTrue(assert_equal(expmap_default<Pose3>(id, v), Pose3(R,
        // Point3(0,0,0))));
        // v(3)=0.2;v(4)=0.394742;v(5)=-2.08998;
        // assertTrue(assert_equal(Pose3(R, P),expmap_default<Pose3>(id, v),1e-5));
    }

    @Test
    void testexpmap_a_full2() {
        // Pose3 id;
        // Vector v = Z_6x1;
        // v(0) = 0.3;
        // assertTrue(assert_equal(expmap_default<Pose3>(id, v), Pose3(R,
        // Point3(0,0,0))));
        // v(3)=0.2;v(4)=0.394742;v(5)=-2.08998;
        // assertTrue(assert_equal(Pose3(R, P),expmap_default<Pose3>(id, v),1e-5));
    }

    @Test
    void testexpmap_b() throws Throwable {
        Pose3 p1 = new Pose3(new Rot3(), new Point3(100, 0, 0));
        Pose3 p2 = p1.retract(new Vector6(0.0, 0.0, 0.1, 0.0, 0.0, 0.0));
        Pose3 expected = new Pose3(Rot3.Rodrigues(0.0, 0.0, 0.1), new Point3(100.0, 0.0, 0.0));
        assertTrue(assert_equal(expected, p2, 1e-2));
    }

    // // test case for screw motion in the plane
    // namespace screwPose3 {
    // double a=0.3, c=cos(a), s=sin(a), w=0.3;
    // Vector xi = (Vector(6) << 0.0, 0.0, w, w, 0.0, 1.0).finished();
    // Rot3 expectedR(c, -s, 0, s, c, 0, 0, 0, 1);
    // Point3 expectedT(0.29552, 0.0446635, 1);
    // Pose3 expected(expectedR, expectedT);
    // }

    // // Checks correct exponential map (Expmap) with brute force matrix
    // exponential
    @Test
    void testexpmap_c_full() {
        // assertTrue(assert_equal(screwPose3::expected,
        // expm<Pose3>(screwPose3::xi),1e-6));
        // assertTrue(assert_equal(screwPose3::expected,
        // Pose3::Expmap(screwPose3::xi),1e-6));
    }

    // // assert that T*exp(xi)*T^-1 is equal to exp(Ad_T(xi))
    @Test
    void testAdjoint_full() {
        // Pose3 expected = T * Pose3::Expmap(screwPose3::xi) * T.inverse();
        // Vector xiprime = T.Adjoint(screwPose3::xi);
        // assertTrue(assert_equal(expected, Pose3::Expmap(xiprime), 1e-6));

        // Pose3 expected2 = T2 * Pose3::Expmap(screwPose3::xi) * T2.inverse();
        // Vector xiprime2 = T2.Adjoint(screwPose3::xi);
        // assertTrue(assert_equal(expected2, Pose3::Expmap(xiprime2), 1e-6));

        // Pose3 expected3 = T3 * Pose3::Expmap(screwPose3::xi) * T3.inverse();
        // Vector xiprime3 = T3.Adjoint(screwPose3::xi);
        // assertTrue(assert_equal(expected3, Pose3::Expmap(xiprime3), 1e-6));
    }

    // // Check Adjoint numerical derivatives
    @Test
    void testAdjoint_jacobians() {
        // Vector6 xi = (Vector6() << 0.1, 1.2, 2.3, 3.1, 1.4, 4.5).finished();

        // // Check evaluation sanity check
        // EQUALITY(static_cast<gtsam::Vector>(T.AdjointMap() * xi), T.Adjoint(xi));
        // EQUALITY(static_cast<gtsam::Vector>(T2.AdjointMap() * xi), T2.Adjoint(xi));
        // EQUALITY(static_cast<gtsam::Vector>(T3.AdjointMap() * xi), T3.Adjoint(xi));

        // // Check jacobians
        // Matrix6 actualH1, actualH2, expectedH1, expectedH2;
        // auto Ad = [&](const Pose3& T, const Vector6& xi) { return T.Adjoint(xi); };

        // T.Adjoint(xi, actualH1, actualH2);
        // expectedH1 = numericalDerivative21(Ad, T, xi);
        // expectedH2 = numericalDerivative22(Ad, T, xi);
        // assertTrue(assert_equal(expectedH1, actualH1));
        // assertTrue(assert_equal(expectedH2, actualH2));

        // T2.Adjoint(xi, actualH1, actualH2);
        // expectedH1 = numericalDerivative21(Ad, T2, xi);
        // expectedH2 = numericalDerivative22(Ad, T2, xi);
        // assertTrue(assert_equal(expectedH1, actualH1));
        // assertTrue(assert_equal(expectedH2, actualH2));

        // T3.Adjoint(xi, actualH1, actualH2);
        // expectedH1 = numericalDerivative21(Ad, T3, xi);
        // expectedH2 = numericalDerivative22(Ad, T3, xi);
        // assertTrue(assert_equal(expectedH1, actualH1));
        // assertTrue(assert_equal(expectedH2, actualH2));
    }

    // // Check AdjointTranspose and jacobians
    @Test
    void testAdjointTranspose() {
        // Vector6 xi = (Vector6() << 0.1, 1.2, 2.3, 3.1, 1.4, 4.5).finished();

        // // Check evaluation
        // EQUALITY(static_cast<Vector>(T.AdjointMap().transpose() * xi),
        // T.AdjointTranspose(xi));
        // EQUALITY(static_cast<Vector>(T2.AdjointMap().transpose() * xi),
        // T2.AdjointTranspose(xi));
        // EQUALITY(static_cast<Vector>(T3.AdjointMap().transpose() * xi),
        // T3.AdjointTranspose(xi));

        // // Check jacobians
        // Matrix6 actualH1, actualH2, expectedH1, expectedH2;
        // auto AdT = [&](const Pose3& T, const Vector6& xi) {
        // return T.AdjointTranspose(xi);
        // };

        // T.AdjointTranspose(xi, actualH1, actualH2);
        // expectedH1 = numericalDerivative21(AdT, T, xi);
        // expectedH2 = numericalDerivative22(AdT, T, xi);
        // assertTrue(assert_equal(expectedH1, actualH1, 1e-8));
        // assertTrue(assert_equal(expectedH2, actualH2));

        // T2.AdjointTranspose(xi, actualH1, actualH2);
        // expectedH1 = numericalDerivative21(AdT, T2, xi);
        // expectedH2 = numericalDerivative22(AdT, T2, xi);
        // assertTrue(assert_equal(expectedH1, actualH1, 1e-8));
        // assertTrue(assert_equal(expectedH2, actualH2));

        // T3.AdjointTranspose(xi, actualH1, actualH2);
        // expectedH1 = numericalDerivative21(AdT, T3, xi);
        // expectedH2 = numericalDerivative22(AdT, T3, xi);
        // assertTrue(assert_equal(expectedH1, actualH1, 1e-8));
        // assertTrue(assert_equal(expectedH2, actualH2));
    }

    // // assert that T*Hat(xi)*T^-1 is equal to Hat(Ad_T(xi))
    @Test
    void testAdjoint_hat() {
        // Matrix4 expected = T.matrix() * Pose3::Hat(screwPose3::xi) *
        // T.matrix().inverse();
        // Matrix4 xiprime = Pose3::Hat(T.Adjoint(screwPose3::xi));
        // assertTrue(assert_equal(expected, xiprime, 1e-6));

        // Matrix4 expected2 = T2.matrix() * Pose3::Hat(screwPose3::xi) *
        // T2.matrix().inverse();
        // Matrix4 xiprime2 = Pose3::Hat(T2.Adjoint(screwPose3::xi));
        // assertTrue(assert_equal(expected2, xiprime2, 1e-6));

        // Matrix4 expected3 = T3.matrix() * Pose3::Hat(screwPose3::xi) *
        // T3.matrix().inverse();
        // Matrix4 xiprime3 = Pose3::Hat(T3.Adjoint(screwPose3::xi));
        // assertTrue(assert_equal(expected3, xiprime3, 1e-6));
    }

    // /** Agrawal06iros version of exponential map */
    // Pose3 Agrawal06iros(const Vector& xi) {
    // Vector w = xi.head(3);
    // Vector v = xi.tail(3);
    // double t = w.norm();
    // if (t < 1e-5)
    // return Pose3(Rot3(), Point3(v));
    // else {
    // Matrix W = skewSymmetric(w/t);
    // Matrix A = I_3x3 + ((1 - cos(t)) / t) * W + ((t - sin(t)) / t) * (W * W);
    // return Pose3(Rot3::Expmap (w), Point3(A * v));
    // }
    // }

    @Test
    void testexpmaps_galore_full() {
        // Vector xi; Pose3 actual;
        // xi = (Vector(6) << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6).finished();
        // actual = Pose3::Expmap(xi);
        // assertTrue(assert_equal(expm<Pose3>(xi), actual,1e-6));
        // assertTrue(assert_equal(Agrawal06iros(xi), actual,1e-6));
        // assertTrue(assert_equal(xi, Pose3::Logmap(actual),1e-6));

        // xi = (Vector(6) << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6).finished();
        // for (double theta=1.0;0.3*theta<=M_PI;theta*=2) {
        // Vector txi = xi*theta;
        // actual = Pose3::Expmap(txi);
        // assertTrue(assert_equal(expm<Pose3>(txi,30), actual,1e-6));
        // assertTrue(assert_equal(Agrawal06iros(txi), actual,1e-6));
        // Vector log = Pose3::Logmap(actual);
        // assertTrue(assert_equal(actual, Pose3::Expmap(log),1e-6));
        // assertTrue(assert_equal(txi,log,1e-6)); // not true once wraps
        // }

        // // Works with large v as well, but expm needs 10 iterations!
        // xi = (Vector(6) << 0.2, 0.3, -0.8, 100.0, 120.0, -60.0).finished();
        // actual = Pose3::Expmap(xi);
        // assertTrue(assert_equal(expm<Pose3>(xi,10), actual,1e-5));
        // assertTrue(assert_equal(Agrawal06iros(xi), actual,1e-9));
        // assertTrue(assert_equal(xi, Pose3::Logmap(actual),1e-9));
    }

    // // Check translation and its pushforward
    @Test
    void testtranslation() {
        // Matrix actualH;
        // assertTrue(assert_equal(Point3(3.5, -8.2, 4.2), T.translation(actualH),
        // 1e-8));

        // auto f = [](const Pose3& T) { return T.translation(); };
        // Matrix numericalH = numericalDerivative11<Point3, Pose3>(f, T);
        // assertTrue(assert_equal(numericalH, actualH, 1e-6));
    }

    // // Check rotation and its pushforward
    @Test
    void testrotation() throws Throwable {
        Matrix actualH = new Matrix();
        // assertTrue(assert_equal(R, T.rotation(actualH), 1e-8));

        // auto f = [](const Pose3& T) { return T.rotation(); };
        // Matrix numericalH = numericalDerivative11<Rot3, Pose3>(f, T);
        // assertTrue(assert_equal(numericalH, actualH, 1e-6));
    }

    @Test
    void testAdjoint_compose_full() {
        // // To debug derivatives of compose, assert that
        // // T1*T2*exp(Adjoint(inv(T2),x) = T1*exp(x)*T2
        // const Pose3& T1 = T;
        // Vector x = (Vector(6) << 0.1, 0.1, 0.1, 0.4, 0.2, 0.8).finished();
        // Pose3 expected = T1 * Pose3::Expmap(x) * T2;
        // Vector y = T2.inverse().Adjoint(x);
        // Pose3 actual = T1 * T2 * Pose3::Expmap(y);
        // assertTrue(assert_equal(expected, actual, 1e-6));
    }

    // // Check compose and its pushforward
    // // NOTE: testing::compose<Pose3>(t1,t2) = t1.compose(t2) (see lieProxies.h)
    @Test
    void testcompose() {
        // Matrix actual = (T2*T2).matrix();
        // Matrix expected = T2.matrix()*T2.matrix();
        // assertTrue(assert_equal(actual,expected,1e-8));

        // Matrix actualDcompose1, actualDcompose2;
        // T2.compose(T2, actualDcompose1, actualDcompose2);

        // Matrix numericalH1 = numericalDerivative21(testing::compose<Pose3>, T2, T2);
        // assertTrue(assert_equal(numericalH1,actualDcompose1,5e-3));
        // assertTrue(assert_equal(T2.inverse().AdjointMap(),actualDcompose1,5e-3));

        // Matrix numericalH2 = numericalDerivative22(testing::compose<Pose3>, T2, T2);
        // assertTrue(assert_equal(numericalH2,actualDcompose2,1e-4));
    }

    // // Check compose and its pushforward, another case
    @Test
    void testcompose2() {
        // const Pose3& T1 = T;
        // Matrix actual = (T1*T2).matrix();
        // Matrix expected = T1.matrix()*T2.matrix();
        // assertTrue(assert_equal(actual,expected,1e-8));

        // Matrix actualDcompose1, actualDcompose2;
        // T1.compose(T2, actualDcompose1, actualDcompose2);

        // Matrix numericalH1 = numericalDerivative21(testing::compose<Pose3>, T1, T2);
        // assertTrue(assert_equal(numericalH1,actualDcompose1,5e-3));
        // assertTrue(assert_equal(T2.inverse().AdjointMap(),actualDcompose1,5e-3));

        // Matrix numericalH2 = numericalDerivative22(testing::compose<Pose3>, T1, T2);
        // assertTrue(assert_equal(numericalH2,actualDcompose2,1e-5));
    }

    @Test
    void testinverse() throws Throwable {
        Matrix actualDinverse = new Matrix();
        Matrix actual = T.inverse(actualDinverse).matrix();
        Matrix expected = T.matrix().inverse();
        assertTrue(assert_equal(actual, expected, 1e-8));

        // Matrix numericalH = numericalDerivative11(testing::inverse<Pose3>, T);
        // assertTrue(assert_equal(numericalH,actualDinverse,5e-3));
        // assertTrue(assert_equal(-T.AdjointMap(),actualDinverse,5e-3));
    }

    @Test
    void testinverseDerivatives2() throws Throwable {
        Rot3 R = Rot3.Rodrigues(0.3, 0.4, -0.5);
        Point3 t = new Point3(3.5, -8.2, 4.2);
        Pose3 T = new Pose3(R, t);

        // Matrix numericalH = numericalDerivative11(testing::inverse<Pose3>, T);
        // Matrix actualDinverse;
        // T.inverse(actualDinverse);
        // assertTrue(assert_equal(numericalH,actualDinverse,5e-3));
        // assertTrue(assert_equal(-T.AdjointMap(),actualDinverse,5e-3));
    }

    @Test
    void testcompose_inverse() throws Throwable {
        Matrix actual = (T.compose(T.inverse())).matrix();
        Matrix expected = Matrix.I_4x4();
        assertTrue(assert_equal(actual, expected, 1e-8));
    }

    // Point3 transformFrom_(const Pose3& pose, const Point3& point) {
    // return pose.transformFrom(point);
    // }

    @Test
    void testDtransform_from1_a() throws Throwable {
        Matrix actualDtransform_from1 = new Matrix();
        // T.transformFrom(P, actualDtransform_from1, {});
        // Matrix numerical = numericalDerivative21(transformFrom_, T, P);
        // assertTrue(assert_equal(numerical, actualDtransform_from1, 1e-8));
    }

    @Test
    void testDtransform_from1_b() throws Throwable {
        Pose3 origin = new Pose3();
        // Matrix actualDtransform_from1;
        // origin.transformFrom(P, actualDtransform_from1, {});
        // Matrix numerical = numericalDerivative21(transformFrom_, origin, P);
        // assertTrue(assert_equal(numerical, actualDtransform_from1, 1e-8));
    }

    @Test
    void testDtransform_from1_c() throws Throwable {
        Point3 origin = new Point3(0, 0, 0);
        Pose3 T0 = new Pose3(R, origin);
        Matrix actualDtransform_from1 = new Matrix();
        // T0.transformFrom(P, actualDtransform_from1, {});
        // Matrix numerical = numericalDerivative21(transformFrom_, T0, P);
        // assertTrue(assert_equal(numerical, actualDtransform_from1, 1e-8));
    }

    @Test
    void testDtransform_from1_d() throws Throwable {
        Rot3 I = new Rot3();
        Point3 t0 = new Point3(100, 0, 0);
        Pose3 T0 = new Pose3(I, t0);
        // Matrix actualDtransform_from1;
        // T0.transformFrom(P, actualDtransform_from1, {});
        // // print(computed, "Dtransform_from1_d computed:");
        // Matrix numerical = numericalDerivative21(transformFrom_, T0, P);
        // // print(numerical, "Dtransform_from1_d numerical:");
        // assertTrue(assert_equal(numerical, actualDtransform_from1, 1e-8));
    }

    @Test
    void testDtransform_from2() throws Throwable {
        Matrix actualDtransform_from2 = new Matrix();
        // T.transformFrom(P, {}, actualDtransform_from2);
        // Matrix numerical = numericalDerivative22(transformFrom_, T, P);
        // assertTrue(assert_equal(numerical, actualDtransform_from2, 1e-8));
    }

    // Point3 transform_to_(const Pose3& pose, const Point3& point) {
    // return pose.transformTo(point);
    // }
    @Test
    void testDtransform_to1() throws Throwable {
        Matrix computed = new Matrix();
        // T.transformTo(P, computed, {});
        // Matrix numerical = numericalDerivative21(transform_to_, T, P);
        // assertTrue(assert_equal(numerical, computed, 1e-8));
    }

    @Test
    void testDtransform_to2() throws Throwable {
        Matrix computed = new Matrix();
        // T.transformTo(P, {}, computed);
        // Matrix numerical = numericalDerivative22(transform_to_, T, P);
        // assertTrue(assert_equal(numerical, computed, 1e-8));
    }

    @Test
    void testtransform_to_with_derivatives() throws Throwable {
        Matrix actH1 = new Matrix();
        Matrix actH2 = new Matrix();
        // T.transformTo(P, actH1, actH2);
        // Matrix expH1 = numericalDerivative21(transform_to_, T, P),
        // expH2 = numericalDerivative22(transform_to_, T, P);
        // assertTrue(assert_equal(expH1, actH1, 1e-8));
        // assertTrue(assert_equal(expH2, actH2, 1e-8));
    }

    @Test
    void testtransform_from_with_derivatives() {
        // Matrix actH1, actH2;
        // T.transformFrom(P, actH1, actH2);
        // Matrix expH1 = numericalDerivative21(transformFrom_, T, P),
        // expH2 = numericalDerivative22(transformFrom_, T, P);
        // assertTrue(assert_equal(expH1, actH1, 1e-8));
        // assertTrue(assert_equal(expH2, actH2, 1e-8));
    }

    @Test
    void testtransform_to_translate() {
        // Point3 actual =
        // Pose3(Rot3(), Point3(1, 2, 3)).transformTo(Point3(10., 20., 30.));
        // Point3 expected(9., 18., 27.);
        // assertTrue(assert_equal(expected, actual));
    }

    @Test
    void testtransform_to_rotate() {
        // Pose3 transform(Rot3::Rodrigues(0, 0, -1.570796), Point3(0, 0, 0));
        // Point3 actual = transform.transformTo(Point3(2, 1, 10));
        // Point3 expected(-1, 2, 10);
        // assertTrue(assert_equal(expected, actual, 0.001));
    }

    // // Check transformPoseFrom and its pushforward
    // Pose3 transformPoseFrom_(const Pose3& wTa, const Pose3& aTb) {
    // return wTa.transformPoseFrom(aTb);
    // }

    @Test
    void testtransformPoseFrom() {
        // Matrix actual = (T2*T2).matrix();
        // Matrix expected = T2.matrix()*T2.matrix();
        // assertTrue(assert_equal(actual, expected, 1e-8));

        // Matrix H1, H2;
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

    // Pose3 transformPoseTo_(const Pose3& pose, const Pose3& pose2) {
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
        // Point3 actual = T3.transformFrom(Point3(0, 0, 0));
        Point3 expected = new Point3(1., 2., 3.);
        // assertTrue(assert_equal(expected, actual));
    }

    @Test
    void testtransform_roundtrip() throws Throwable {
        // Point3 actual = T3.transformFrom(T3.transformTo(Point3(12., -0.11, 7.0)));
        Point3 expected = new Point3(12., -0.11, 7.0);
        // assertTrue(assert_equal(expected, actual));
    }

    @Test
    void testRetract_LocalCoordinates() throws Throwable {
        Vector6 d = new Vector6(1, 2, 3, 4, 5, 6);
        // d/=10;
        // const Rot3 R = Rot3::Retract(d.head<3>());
        // Pose3 t = Pose3::Retract(d);
        // assertTrue(assert_equal(d, Pose3::LocalCoordinates(t)));
    }

    @Test
    void testretract_localCoordinates() throws Throwable {
        Vector6 d12 = new Vector6(1, 2, 3, 4, 5, 6);
        // d12/=10;
        // Pose3 t1 = T, t2 = t1.retract(d12);
        // assertTrue(assert_equal(d12, t1.localCoordinates(t2)));
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
    void testsubgroups() {
        // // Frank - Below only works for correct "Agrawal06iros style expmap
        // // lines in canonical coordinates correspond to Abelian subgroups in SE(3)
        // Vector d = (Vector(6) << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6).finished();
        // // exp(-d)=inverse(exp(d))
        // assertTrue(assert_equal(Pose3::Expmap(-d),Pose3::Expmap(d).inverse()));
        // // exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
        // Pose3 T2 = Pose3::Expmap(2*d);
        // Pose3 T3 = Pose3::Expmap(3*d);
        // Pose3 T5 = Pose3::Expmap(5*d);
        // assertTrue(assert_equal(T5,T2*T3));
        // assertTrue(assert_equal(T5,T3*T2));
    }

    @Test
    void testbetween() throws Throwable {
        Pose3 expected = T2.inverse().compose(T3);
        // Matrix actualDBetween1,actualDBetween2;
        // Pose3 actual = T2.between(T3, actualDBetween1,actualDBetween2);
        // assertTrue(assert_equal(expected,actual));

        // Matrix numericalH1 = numericalDerivative21(testing::between<Pose3> , T2, T3);
        // assertTrue(assert_equal(numericalH1,actualDBetween1,5e-3));

        // Matrix numericalH2 = numericalDerivative22(testing::between<Pose3> , T2, T3);
        // assertTrue(assert_equal(numericalH2,actualDBetween2,1e-5));
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

    // double range_proxy(const Pose3& pose, const Point3& point) {
    // return pose.range(point);
    // }
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

        // // Check numerical derivatives
        // expectedH1 = numericalDerivative21(range_proxy, x2, l3);
        // expectedH2 = numericalDerivative22(range_proxy, x2, l3);
        // assertTrue(assert_equal(expectedH1,actualH1));
        // assertTrue(assert_equal(expectedH2,actualH2));

        // Another test
        double actual34 = x3.range(l4, actualH1, actualH2);
        assertEquals(5, actual34, 1e-9);

        // // Check numerical derivatives
        // expectedH1 = numericalDerivative21(range_proxy, x3, l4);
        // expectedH2 = numericalDerivative22(range_proxy, x3, l4);
        // assertTrue(assert_equal(expectedH1,actualH1));
        // assertTrue(assert_equal(expectedH2,actualH2));
    }

    // double range_pose_proxy(const Pose3& pose, const Pose3& point) {
    // return pose.range(point);
    // }
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
        // expectedH1 = numericalDerivative21(range_pose_proxy, x2, xl3);
        // expectedH2 = numericalDerivative22(range_pose_proxy, x2, xl3);
        // assertTrue(assert_equal(expectedH1,actualH1));
        // assertTrue(assert_equal(expectedH2,actualH2));

        // Another test
        double actual34 = x3.range(xl4, actualH1, actualH2);
        assertEquals(5, actual34, 1e-9);

        // // Check numerical derivatives
        // expectedH1 = numericalDerivative21(range_pose_proxy, x3, xl4);
        // expectedH2 = numericalDerivative22(range_pose_proxy, x3, xl4);
        // assertTrue(assert_equal(expectedH1,actualH1));
        // assertTrue(assert_equal(expectedH2,actualH2));
    }

    // Unit3 bearing_proxy(const Pose3& pose, const Point3& point) {
    // return pose.bearing(point);
    // }
    @Test
    void testBearing() throws Throwable {
        Matrix expectedH1 = new Matrix();
        Matrix actualH1 = new Matrix();
        Matrix expectedH2 = new Matrix();
        Matrix actualH2 = new Matrix();

        assertTrue(assert_equal(new Unit3(1, 0, 0), x1.bearing(l1, actualH1, actualH2), 1e-9));

        // Check numerical derivatives
        // expectedH1 = numericalDerivative21(bearing_proxy, x1, l1);
        // expectedH2 = numericalDerivative22(bearing_proxy, x1, l1);
        // assertTrue(assert_equal(expectedH1, actualH1, 1e-5));
        // assertTrue(assert_equal(expectedH2, actualH2, 1e-5));
    }

    @Test
    void testBearing2() throws Throwable {
        Matrix expectedH1 = new Matrix();
        Matrix actualH1 = new Matrix();
        Matrix expectedH2 = new Matrix();
        Matrix actualH2 = new Matrix();

        assertTrue(assert_equal(new Unit3(0, 0.6, -0.8), x2.bearing(l4, actualH1, actualH2), 1e-9));

        // // Check numerical derivatives
        // expectedH1 = numericalDerivative21(bearing_proxy, x2, l4);
        // expectedH2 = numericalDerivative22(bearing_proxy, x2, l4);
        // assertTrue(assert_equal(expectedH1, actualH1, 1e-5));
        // assertTrue(assert_equal(expectedH2, actualH2, 1e-5));
    }

    @Test
    void testPoseToPoseBearing() throws Throwable {
        Matrix expectedH1 = new Matrix();
        Matrix actualH1 = new Matrix();
        Matrix expectedH2 = new Matrix();
        Matrix actualH2 = new Matrix();
        Matrix H2block = new Matrix();

        assertTrue(assert_equal(new Unit3(0, 1, 0), xl1.bearing(xl2, actualH1, actualH2), 1e-9));

        // // Check numerical derivatives
        // auto f = [](const Pose3& a, const Pose3& b) { return a.bearing(b); };
        // expectedH1 = numericalDerivative21(f, xl1, xl2);
        // expectedH2 = numericalDerivative22(f, xl1, xl2);
        // assertTrue(assert_equal(expectedH1, actualH1, 1e-5));
        // assertTrue(assert_equal(expectedH2, actualH2, 1e-5));
    }

    @Test
    void testunicycle() {
        // // velocity in X should be X in inertial frame, rather than global frame
        // Vector x_step = Vector::Unit(6,3)*1.0;
        // assertTrue(assert_equal(Pose3(Rot3::Ypr(0,0,0), l1),
        // expmap_default<Pose3>(x1,
        // x_step), tol));
        // assertTrue(assert_equal(Pose3(Rot3::Ypr(0,0,0), Point3(2,1,0)),
        // expmap_default<Pose3>(x2, x_step), tol));
        // assertTrue(assert_equal(Pose3(Rot3::Ypr(M_PI/4.0,0,0), Point3(2,2,0)),
        // expmap_default<Pose3>(x3, sqrt(2.0) * x_step), tol));
    }

    @Test
    void testadjointMap() {
        // Matrix res = Pose3::adjointMap(screwPose3::xi);
        // Matrix wh = skewSymmetric(screwPose3::xi(0), screwPose3::xi(1),
        // screwPose3::xi(2));
        // Matrix vh = skewSymmetric(screwPose3::xi(3), screwPose3::xi(4),
        // screwPose3::xi(5));
        // Matrix6 expected;
        // expected << wh, Z_3x3, vh, wh;
        // assertTrue(assert_equal(expected,res,1e-5));
    }

    @Test
    void testAlign1() throws Throwable {
        Pose3 expected = new Pose3(new Rot3(), new Point3(10, 10, 0));

        // Point3Pair ab1(Point3(10,10,0), Point3(0,0,0));
        // Point3Pair ab2(Point3(30,20,0), Point3(20,10,0));
        // Point3Pair ab3(Point3(20,30,0), Point3(10,20,0));
        // const vector<Point3Pair> correspondences{ab1, ab2, ab3};

        // std::optional<Pose3> actual = Pose3::Align(correspondences);
        // assertTrue(assert_equal(expected, *actual));
    }

    @Test
    void testAlign2() throws Throwable {
        Point3 t = new Point3(20, 10, 5);
        Rot3 R = Rot3.RzRyRx(0.3, 0.2, 0.1);
        Pose3 expected = new Pose3(R, t);

        Point3 p1 = new Point3(0, 0, 1);
        Point3 p2 = new Point3(10, 0, 2);
        Point3 p3 = new Point3(20, -10, 30);
        // Point3 q1 = expected.transformFrom(p1),
        // q2 = expected.transformFrom(p2),
        // q3 = expected.transformFrom(p3);
        // const Point3Pair ab1{q1, p1}, ab2{q2, p2}, ab3{q3, p3};
        // const vector<Point3Pair> correspondences{ab1, ab2, ab3};

        // std::optional<Pose3> actual = Pose3::Align(correspondences);
        // assertTrue(assert_equal(expected, *actual, 1e-5));
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
        // const Matrix expected = numericalDerivative11<Pose3, double>(T, t);
        // const Matrix actual = Pose3::ExpmapDerivative(xi(t)) * xi_dot(t);
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
                    // const Matrix6 expectedH =
                    // numericalDerivative21<Pose3, Vector6, OptionalJacobian<6, 6> >(
                    // &Pose3::Expmap, xi, {});
                    Matrix actualH = new Matrix();
                    Pose3.statics.Expmap(xi, actualH);
                    // assertTrue(assert_equal(expectedH, actualH));
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
                    // const Matrix6 expectedH =
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
    void testLogmapDerivative() {
        // // Copied from testSO3.cpp
        // const Rot3 R2((Matrix3() << // Near pi
        // -0.750767, -0.0285082, -0.659952,
        // -0.0102558, -0.998445, 0.0547974,
        // -0.660487, 0.0479084, 0.749307).finished());
        // const Rot3 R3((Matrix3() << // Near pi
        // -0.747473, -0.00190019, -0.664289,
        // -0.0385114, -0.99819, 0.0461892,
        // -0.663175, 0.060108, 0.746047).finished());
        // const Rot3 R4((Matrix3() << // Final pose in a drone experiment
        // 0.324237, 0.902975, 0.281968,
        // -0.674322, 0.429668, -0.600562,
        // -0.663445, 0.00458662, 0.748211).finished());

        // // Now creates poses
        // const Pose3 T0; // Identity
        // const Vector6 xi(0.1, -0.1, 0.1, 0.1, -0.1, 0.1);
        // const Pose3 T1 = Pose3::Expmap(xi); // Small rotation
        // const Pose3 T2(R2, Point3(1, 2, 3));
        // const Pose3 T3(R3, Point3(1, 2, 3));
        // const Pose3 T4(R4, Point3(1, 2, 3));
        // size_t i = 0;
        // for (const Pose3& T : { T0, T1, T2, T3, T4 }) {
        // const bool nearPi = (i == 2 || i == 3); // Flag cases near pi

        // Matrix6 actualH; // H computed by Logmap(T, H) using LogmapDerivative(xi)
        // const Vector6 xi = Pose3::Logmap(T, actualH);

        // // 1. Check self-consistency of analytical derivative calculation:
        // // Does the H returned by Logmap match an independent calculation
        // // of J_r^{-1} using ExpmapDerivative with the computed xi?
        // Matrix6 J_r_inv = Pose3::ExpmapDerivative(xi).inverse(); // J_r^{-1}
        // assertTrue(assert_equal(J_r_inv, actualH)); // This test is crucial and
        // should
        // pass

        // // 2. Check analytical derivative against numerical derivative:
        // // Only perform this check AWAY from the pi singularity, where
        // // numerical differentiation of Logmap is expected to be reliable
        // // and should match the analytical derivative.
        // if (!nearPi) {
        // const Matrix expectedH = numericalDerivative11<Vector6, Pose3>(
        // std::bind(&Pose3::Logmap, std::placeholders::_1, nullptr), T, 1e-7);
        // assertTrue(assert_equal(expectedH, actualH, 1e-5)); // 1e-5 needed to pass R4
        // }
        // else {
        // // We accept that the numerical derivative of this specific Logmap
        // implementation
        // // near pi will not match the standard analytical derivative J_r^{-1}.
        // }
        // i++;
        // }
    }

    // Vector6 testDerivAdjoint(const Vector6& xi, const Vector6& v) {
    // return Pose3::adjointMap(xi) * v;
    // }

    @Test
    void testadjoint() throws Throwable {
        // Vector6 v = (Vector6() << 1, 2, 3, 4, 5, 6).finished();
        // Vector expected = testDerivAdjoint(screwPose3::xi, v);

        Matrix actualH1 = new Matrix();
        Matrix actualH2 = new Matrix();
        // Vector actual = Pose3::adjoint(screwPose3::xi, v, actualH1, actualH2);

        // Matrix numericalH1 = numericalDerivative21<Vector6, Vector6, Vector6>(
        // testDerivAdjoint, screwPose3::xi, v, 1e-5);
        // Matrix numericalH2 = numericalDerivative22<Vector6, Vector6, Vector6>(
        // testDerivAdjoint, screwPose3::xi, v, 1e-5);

        // assertTrue(assert_equal(expected,actual,1e-5));
        // assertTrue(assert_equal(numericalH1,actualH1,1e-5));
        // assertTrue(assert_equal(numericalH2,actualH2,1e-5));
    }

    // Vector6 testDerivAdjointTranspose(const Vector6& xi, const Vector6& v) {
    // return Pose3::adjointMap(xi).transpose() * v;
    // }

    @Test
    void testadjointTranspose() throws Throwable {
        Vector6 xi = new Vector6(0.01, 0.02, 0.03, 1.0, 2.0, 3.0);
        // Vector v = (Vector(6) << 0.04, 0.05, 0.06, 4.0, 5.0, 6.0).finished();
        // Vector expected = testDerivAdjointTranspose(xi, v);

        Matrix actualH1 = new Matrix();
        Matrix actualH2 = new Matrix();
        // Vector actual = Pose3::adjointTranspose(xi, v, actualH1, actualH2);

        // Matrix numericalH1 = numericalDerivative21<Vector6, Vector6, Vector6>(
        // testDerivAdjointTranspose, xi, v, 1e-5);
        // Matrix numericalH2 = numericalDerivative22<Vector6, Vector6, Vector6>(
        // testDerivAdjointTranspose, xi, v, 1e-5);

        // assertTrue(assert_equal(expected,actual,1e-15));
        // assertTrue(assert_equal(numericalH1,actualH1,1e-5));
        // assertTrue(assert_equal(numericalH2,actualH2,1e-5));
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

    //
    // #include "testPoseAdjointMap.h"

    @Test
    void testTransformCovariance6MapTo2d() throws Throwable {
        // // Create 3d scenarios that map to 2d configurations and compare with Pose2
        // results.
        // using namespace test_pose_adjoint_map;
        double degree = Math.PI / 180;

        Vector3 s2 = new Vector3(0.1, 0.3, 0.7);
        Pose2 p2 = new Pose2(1.1, 1.5, 31. * degree);
        // auto cov2 = FullCovarianceFromSigmas<Pose2>(s2);
        // auto transformed2 = TransformCovariance<Pose2>{p2}(cov2);

        // auto match_cov3_to_cov2 = [&](int spatial_axis0, int spatial_axis1, int
        // r_axis,
        // const Pose2::Jacobian &cov2, const Pose3::Jacobian &cov3) -> void
        // {
        // assertTrue(assert_equal(
        // Vector3{cov2.diagonal()},
        // Vector3{cov3(spatial_axis0, spatial_axis0), cov3(spatial_axis1,
        // spatial_axis1), cov3(r_axis, r_axis)}));
        // assertTrue(assert_equal(
        // Vector3{cov2(1, 0), cov2(2, 0), cov2(2, 1)},
        // Vector3{cov3(spatial_axis1, spatial_axis0), cov3(r_axis, spatial_axis0),
        // cov3(r_axis, spatial_axis1)}));
        // };

        // rotate around x axis
        {
            // auto cov3 = FullCovarianceFromSigmas<Pose3>((Vector6{} << s2(2), 0., 0., 0.,
            // s2(0), s2(1)).finished());
            // auto transformed3 = TransformCovariance<Pose3>{{Rot3::RzRyRx(p2.theta(), 0.,
            // 0.), {0., p2.x(), p2.y()}}}(cov3);
            // match_cov3_to_cov2(4, 5, 0, transformed2, transformed3);
        }

        // rotate around y axis
        {
            // auto cov3 = FullCovarianceFromSigmas<Pose3>((Vector6{} << 0., s2(2), 0.,
            // s2(1), 0., s2(0)).finished());
            // auto transformed3 = TransformCovariance<Pose3>{{Rot3::RzRyRx(0., p2.theta(),
            // 0.), {p2.y(), 0., p2.x()}}}(cov3);
            // match_cov3_to_cov2(5, 3, 1, transformed2, transformed3);
        }

        // rotate around z axis
        {
            // auto cov3 = FullCovarianceFromSigmas<Pose3>((Vector6{} << 0., 0., s2(2),
            // s2(0), s2(1), 0.).finished());
            // auto transformed3 = TransformCovariance<Pose3>{{Rot3::RzRyRx(0., 0.,
            // p2.theta()), {p2.x(), p2.y(), 0.}}}(cov3);
            // match_cov3_to_cov2(3, 4, 2, transformed2, transformed3);
        }
    }

    @Test
    void testTransformCovariance6() {
        // // Use simple covariance matrices and transforms to create tests that can be
        // // validated with simple computations.
        // using namespace test_pose_adjoint_map;

        // // rotate 90 around z axis and then 90 around y axis
        {
            // auto cov = FullCovarianceFromSigmas<Pose3>((Vector6{} << 0.1, 0.2, 0.3, 0.5,
            // 0.7, 1.1).finished());
            // auto transformed = TransformCovariance<Pose3>{{Rot3::RzRyRx(0., 90 * degree,
            // 90 * degree), {0., 0., 0.}}}(cov);
            // // x from y, y from z, z from x
            // assertTrue(assert_equal(
            // (Vector6{} << cov(1, 1), cov(2, 2), cov(0, 0), cov(4, 4), cov(5, 5), cov(3,
            // 3)).finished(),
            // Vector6{transformed.diagonal()}));
            // // Both the x and z axes are pointing in the negative direction.
            // assertTrue(assert_equal(
            // (Vector5{} << -cov(2, 1), cov(0, 1), cov(4, 1), -cov(5, 1), cov(3,
            // 1)).finished(),
            // (Vector5{} << transformed(1, 0), transformed(2, 0), transformed(3, 0),
            // transformed(4, 0), transformed(5, 0)).finished()));
        }

        // // translate along the x axis with uncertainty in roty and rotz
        {
            // auto cov = TwoVariableCovarianceFromSigmas<Pose3>(1, 2, 0.7, 0.3);
            // auto transformed = TransformCovariance<Pose3>{{Rot3::RzRyRx(0., 0., 0.),
            // {20., 0., 0.}}}(cov);
            // // The uncertainty in roty and rotz causes off-diagonal covariances
            // assertTrue(assert_equal(0.7 * 0.7 * 20., transformed(5, 1)));
            // assertTrue(assert_equal(0.7 * 0.7 * 20. * 20., transformed(5, 5)));
            // assertTrue(assert_equal(-0.3 * 0.3 * 20., transformed(4, 2)));
            // assertTrue(assert_equal(0.3 * 0.3 * 20. * 20., transformed(4, 4)));
            // assertTrue(assert_equal(-0.3 * 0.7 * 20., transformed(4, 1)));
            // assertTrue(assert_equal(0.3 * 0.7 * 20., transformed(5, 2)));
            // assertTrue(assert_equal(-0.3 * 0.7 * 20. * 20., transformed(5, 4)));
        }

        // // rotate around x axis and translate along the x axis with uncertainty in
        // rotx
        {
            // auto cov = SingleVariableCovarianceFromSigma<Pose3>(0, 0.1);
            // auto transformed = TransformCovariance<Pose3>{{Rot3::RzRyRx(90 * degree, 0.,
            // 0.), {20., 0., 0.}}}(cov);
            // // No change
            // assertTrue(assert_equal(cov, transformed));
        }

        // // rotate around x axis and translate along the x axis with uncertainty in
        // roty
        {
            // auto cov = SingleVariableCovarianceFromSigma<Pose3>(1, 0.1);
            // auto transformed = TransformCovariance<Pose3>{{Rot3::RzRyRx(90 * degree, 0.,
            // 0.), {20., 0., 0.}}}(cov);
            // // Uncertainty is spread to other dimensions.
            // assertTrue(assert_equal(
            // (Vector6{} << 0., 0., 0.1 * 0.1, 0., 0.1 * 0.1 * 20. * 20., 0.).finished(),
            // Vector6{transformed.diagonal()}));
        }
    }

    @Test
    void testinterpolate() {
        // assertTrue(assert_equal(T2, interpolate(T2,T3, 0.0)));
        // assertTrue(assert_equal(T3, interpolate(T2,T3, 1.0)));

        // // Trivial example: start at origin and move to (1, 0, 0) while rotating pi/2
        // // about z-axis.
        // Pose3 start;
        // Pose3 end(Rot3::Rz(M_PI_2), Point3(1, 0, 0));
        // // This interpolation is easy to calculate by hand.
        // double t = 0.5;
        // Pose3 expected0(Rot3::Rz(M_PI_4), Point3(0.5, 0, 0));
        // assertTrue(assert_equal(expected0, start.interpolateRt(end, t)));

        // // Example from Peter Corke
        // // https://robotacademy.net.au/lesson/interpolating-pose-in-3d/
        // t = 0.0759; // corresponds to the 10th element when calling `ctraj` in
        // // the video
        // Pose3 O;
        // Pose3 F(Rot3::Roll(0.6).compose(Rot3::Pitch(0.8)).compose(Rot3::Yaw(1.4)),
        // Point3(1, 2, 3));

        // // The expected answer matches the result presented in the video.
        // Pose3 expected1(interpolate(O.rotation(), F.rotation(), t),
        // interpolate(O.translation(), F.translation(), t));
        // assertTrue(assert_equal(expected1, O.interpolateRt(F, t)));

        // // Non-trivial interpolation, translation value taken from output.
        // Pose3 expected2(interpolate(T2.rotation(), T3.rotation(), t),
        // interpolate(T2.translation(), T3.translation(), t));
        // assertTrue(assert_equal(expected2, T2.interpolateRt(T3, t)));
    }

    // Pose3 testing_interpolate(const Pose3& t1, const Pose3& t2, double gamma) {
    // return interpolate(t1,t2,gamma); }

    @Test
    void testinterpolateJacobians() throws Throwable {
        {
            Pose3 X = Pose3.statics.Identity();
            // Pose3 Y(Rot3::Rz(M_PI_2), Point3(1, 0, 0));
            // double t = 0.5;
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
            // Pose3 Y(Rot3::Identity(), Point3(1, 0, 0));
            // double t = 0.3;
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
            // Pose3 Y(Rot3::Rz(M_PI_2), Point3(0, 0, 0));
            // double t = 0.5;
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
            // Pose3 X(Rot3::Ypr(0.1,0.2,0.3), Point3(10, 5, -2));
            // Pose3 Y(Rot3::Ypr(1.1,-2.2,-0.3), Point3(-5, 1, 1));
            // double t = 0.3;
            // Pose3 expectedPoseInterp(Rot3::Rz(M_PI_4), Point3(0, 0, 0));
            // Matrix actualJacobianX, actualJacobianY, actualJacobianT;
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

    // Pose3 testing_interpolate_rt(const Pose3& t1, const Pose3& t2, double gamma)
    // { return t1.interpolateRt(t2, gamma); }

    @Test
    void testinterpolateRtJacobians() throws Throwable {
        {
            Pose3 X = Pose3.statics.Identity();
            // Pose3 Y(Rot3::Rz(M_PI_2), Point3(1, 0, 0));
            // double t = 0.5;
            // Matrix actualJacobianX, actualJacobianY, actualJacobianT;
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
            // Pose3 Y(Rot3::Identity(), Point3(1, 0, 0));
            // double t = 0.3;
            // Matrix actualJacobianX, actualJacobianY, actualJacobianT;
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
            // Pose3 Y(Rot3::Rz(M_PI_2), Point3(0, 0, 0));
            // double t = 0.5;
            // Matrix actualJacobianX, actualJacobianY, actualJacobianT;
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
            // Pose3 X(Rot3::Ypr(0.1,0.2,0.3), Point3(10, 5, -2));
            // Pose3 Y(Rot3::Ypr(1.1,-2.2,-0.3), Point3(-5, 1, 1));
            // double t = 0.3;
            // Matrix actualJacobianX, actualJacobianY, actualJacobianT;
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
        // double t = 0.3;
        // Values vals;
        // vals.insert(0,X);
        // vals.insert(1,Y);
        // vals.insert(2,t);

        { // interpolate (templated wrapper applies to all classes)
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
        { // interpolateRt (Pose3 specialisation)
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
        // // Muliply with an arbitrary matrix and exponentiate
        // Matrix6 M;
        // M << 1, 2, 3, 4, 5, 6, //
        // 7, 8, 9, 1, 2, 3, //
        // 4, 5, 6, 7, 8, 9, //
        // 1, 2, 3, 4, 5, 6, //
        // 7, 8, 9, 1, 2, 3, //
        // 4, 5, 6, 7, 8, 9;
        // auto g = [&](const Vector6& omega) {
        // return Pose3::Expmap(M*omega);
        // };

        // // Test the derivatives at zero
        // const Matrix6 expected = numericalDerivative11<Pose3, Vector6>(g, Z_6x1);
        // assertTrue(assert_equal<Matrix6>(expected, M, 1e-5)); //
        // Pose3::ExpmapDerivative(Z_6x1) is identity

        // // Test the derivatives at another value
        final Vector6 delta = new Vector6(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
        // const Matrix6 expected2 = numericalDerivative11<Pose3, Vector6>(g, delta);
        // const Matrix6 analytic = Pose3::ExpmapDerivative(M*delta) * M;
        // assertTrue(assert_equal<Matrix6>(expected2, analytic, 1e-5)); // note
        // tolerance
    }

}
