package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import gtsam.NumericalDerivative.ThrowingFunction2;

/**
 * See gtsam/geometry/tests/testRot2.cpp
 */
public class Rot2Test {
    static Rot2 R;
    static Point2 P;
    static {
        try {
            R = Rot2.fromAngle(0.1);
            P = new Point2(0.2, 0.7);
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }
    }

    @Test
    void testconstructors_and_angle() throws Throwable {
        double c = Math.cos(0.1);
        double s = Math.sin(0.1);
        assertEquals(0.1, R.theta(), 1e-9);
        assertTrue(assert_equal(R, new Rot2(0.1)));
        assertTrue(assert_equal(R, Rot2.fromAngle(0.1)));
        assertTrue(assert_equal(R, Rot2.fromCosSin(c, s)));
        assertTrue(assert_equal(R, Rot2.atan2(s * 5, c * 5)));
    }

    @Test
    void testunit() throws Throwable {
        assertTrue(assert_equal(new Point2(1.0, 0.0), Rot2.fromAngle(0).unit()));
        assertTrue(assert_equal(new Point2(0.0, 1.0), Rot2.fromAngle(Math.PI / 2.0).unit()));
    }

    @Test
    void testtranspose() throws Throwable {
        Matrix2 expected = R.inverse().matrix();
        Matrix2 actual = R.transpose();
        assertTrue(assert_equal(expected, actual));
    }

    @Test
    void testcompose() throws Throwable {
        assertTrue(assert_equal(Rot2.fromAngle(0.45),
                Rot2.fromAngle(0.2).compose(Rot2.fromAngle(0.25))));
        assertTrue(assert_equal(Rot2.fromAngle(0.45),
                Rot2.fromAngle(0.25).compose(Rot2.fromAngle(0.2))));

        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        Rot2.fromAngle(1.0).compose(Rot2.fromAngle(2.0), H1, H2);
        assertTrue(assert_equal(Matrix.I_1x1(), H1));
        assertTrue(assert_equal(Matrix.I_1x1(), H2));
    }

    @Test
    void testbetween() throws Throwable {
        assertTrue(assert_equal(Rot2.fromAngle(0.05),
                Rot2.fromAngle(0.2).between(Rot2.fromAngle(0.25))));
        assertTrue(assert_equal(Rot2.fromAngle(-0.05),
                Rot2.fromAngle(0.25).between(Rot2.fromAngle(0.2))));

        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        Rot2.fromAngle(1.0).between(Rot2.fromAngle(2.0), H1, H2);
        assertTrue(assert_equal(Matrix.I_1x1().times(-1), H1));
        assertTrue(assert_equal(Matrix.I_1x1(), H2));
    }

    @Test
    void testequals() throws Throwable {
        assertTrue(R.equals(R));
        Rot2 zero = new Rot2();
        assertTrue(!R.equals(zero));
    }

    @Test
    void testexpmap() throws Throwable {
        Vector1 v = new Vector1(0);
        assertTrue(assert_equal(R.retract(v), R));
    }

    @Test
    void testlogmap() throws Throwable {
        Rot2 rot0 = Rot2.fromAngle(Math.PI / 2.0);
        Rot2 rot = Rot2.fromAngle(Math.PI);
        Vector1 expected = new Vector1(Math.PI / 2.0);
        Vector1 actual = rot0.localCoordinates(rot);
        assertTrue(assert_equal(expected, actual));
    }

    // rotate and derivatives
    @Test
    void testrotate() throws Throwable {
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        Point2 actual = R.rotate(P, H1, H2);
        assertTrue(assert_equal(actual, R.rotate(P)));
        ThrowingFunction2<Rot2, Point2, Point2> rotate_ = //
                (Rot2 R, Point2 p) -> R.rotate(p);
        Matrix numerical1 = NumericalDerivative.<//
                Point2, Vector2, //
                Rot2, Vector1, //
                Point2, Vector2//
        >numericalDerivative21(rotate_, R, P, 1e-3);
        assertTrue(assert_equal(numerical1, H1, 1e-6));
        Matrix numerical2 = NumericalDerivative.<//
                Point2, Vector2, //
                Rot2, Vector1, //
                Point2, Vector2//
        >numericalDerivative22(rotate_, R, P, 1e-3);
        assertTrue(assert_equal(numerical2, H2, 1e-6));
    }

    // // unrotate and derivatives
    // inline Point2 unrotate_(const Rot2& R, const Point2& p) {return
    // R.unrotate(p);}
    @Test
    void testunrotate() {
        // Matrix H1, H2;
        // Point2 w = R * P, actual = R.unrotate(w, H1, H2);
        // assertTrue(assert_equal(actual,P));
        // Matrix numerical1 = numericalDerivative21(unrotate_, R, w);
        // assertTrue(assert_equal(numerical1,H1));
        // Matrix numerical2 = numericalDerivative22(unrotate_, R, w);
        // assertTrue(assert_equal(numerical2,H2));
    }

    // inline Rot2 relativeBearing_(const Point2& pt) {return
    // Rot2::relativeBearing(pt); }
    @Test
    void testrelativeBearing() {
        // Point2 l1(1, 0), l2(1, 1);
        // Matrix expectedH, actualH;

        // // establish relativeBearing is indeed zero
        // Rot2 actual1 = Rot2::relativeBearing(l1, actualH);
        // assertTrue(assert_equal(Rot2(),actual1));

        // // Check numerical derivative
        // expectedH = numericalDerivative11(relativeBearing_, l1);
        // assertTrue(assert_equal(expectedH,actualH));

        // // establish relativeBearing is indeed 45 degrees
        // Rot2 actual2 = Rot2::relativeBearing(l2, actualH);
        // assertTrue(assert_equal(Rot2::fromAngle(M_PI/4.0),actual2));

        // // Check numerical derivative
        // expectedH = numericalDerivative11(relativeBearing_, l2);
        // assertTrue(assert_equal(expectedH,actualH));
    }

    @Test
    void testvec() {
        // // Test the 'vec' method
        // Vector4 expected_vec = Eigen::Map<Vector4>(R.matrix().data());
        // Matrix41 actualH;
        // Vector4 actual_vec = R.vec(actualH);
        // assertTrue(assert_equal(expected_vec, actual_vec));

        // // Verify Jacobian with numerical derivatives
        // auto f = [](const Rot2& p) { return p.vec(); };
        // Matrix41 numericalH = numericalDerivative11<Vector4, Rot2>(f, R);
        // assertTrue(assert_equal(numericalH, actualH, 1e-9));
    }

    // namespace {
    // Rot2 id;
    // Rot2 T1(0.1);
    // Rot2 T2(0.2);
    // } // namespace

    @Test
    void testInvariants() {
        // assertTrue(check_group_invariants(id, id));
        // assertTrue(check_group_invariants(id, T1));
        // assertTrue(check_group_invariants(T2, id));
        // assertTrue(check_group_invariants(T2, T1));

        // assertTrue(check_manifold_invariants(id, id));
        // assertTrue(check_manifold_invariants(id, T1));
        // assertTrue(check_manifold_invariants(T2, id));
        // assertTrue(check_manifold_invariants(T2, T1));
    }

    @Test
    void testLieGroupDerivatives() {
        // CHECK_LIE_GROUP_DERIVATIVES(id, id);
        // CHECK_LIE_GROUP_DERIVATIVES(id, T2);
        // CHECK_LIE_GROUP_DERIVATIVES(T2, id);
        // CHECK_LIE_GROUP_DERIVATIVES(T2, T1);
    }

    @Test
    void testChartDerivatives() {
        // CHECK_CHART_DERIVATIVES(id, id);
        // CHECK_CHART_DERIVATIVES(id, T2);
        // CHECK_CHART_DERIVATIVES(T2, id);
        // CHECK_CHART_DERIVATIVES(T2, T1);
    }

}
