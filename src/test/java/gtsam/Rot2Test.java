package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import gtsam.NumericalDerivative.ThrowingFunction;
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
        Vector1 actual = rot0.local(rot);
        assertTrue(assert_equal(expected, actual));
    }

    /**
     * rotate and derivatives
     */
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
                Point2, Vector2>numericalDerivative21(rotate_, R, P, 1e-3);
        assertTrue(assert_equal(numerical1, H1, 1e-6));
        Matrix numerical2 = NumericalDerivative.<//
                Point2, Vector2, //
                Rot2, Vector1, //
                Point2, Vector2>numericalDerivative22(rotate_, R, P, 1e-3);
        assertTrue(assert_equal(numerical2, H2, 1e-6));
    }

    /**
     * unrotate and derivatives
     */
    @Test
    void testunrotate() throws Throwable {
        ThrowingFunction2<Rot2, Point2, Point2> unrotate_ = //
                (Rot2 R, Point2 p) -> R.unrotate(p);
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        Point2 w = R.rotate(P);
        Point2 actual = R.unrotate(w, H1, H2);
        assertTrue(assert_equal(actual, P));
        Matrix numerical1 = NumericalDerivative.<//
                Point2, Vector2, //
                Rot2, Vector1, //
                Point2, Vector2>numericalDerivative21(unrotate_, R, w, 1e-3);
        assertTrue(assert_equal(numerical1, H1, 1e-6));
        Matrix numerical2 = NumericalDerivative.<//
                Point2, Vector2, //
                Rot2, Vector1, //
                Point2, Vector2>numericalDerivative22(unrotate_, R, w, 1e-3);
        assertTrue(assert_equal(numerical2, H2, 1e-6));
    }

    @Test
    void testrelativeBearing() throws Throwable {
        Point2 l1 = new Point2(1, 0);
        Point2 l2 = new Point2(1, 1);
        Matrix expectedH = new Matrix();
        Matrix actualH = new Matrix();

        // establish relativeBearing is indeed zero
        Rot2 actual1 = Rot2.relativeBearing(l1, actualH);
        assertTrue(assert_equal(new Rot2(), actual1));

        ThrowingFunction<Point2, Rot2> relativeBearing_ = (Point2 pt) -> Rot2.relativeBearing(pt);

        // Check numerical derivative
        expectedH = NumericalDerivative.<//
                Rot2, Vector1, Point2, Vector2//
        >numericalDerivative11(relativeBearing_, l1, 1e-3);
        assertTrue(assert_equal(expectedH, actualH, 1e-6));

        // establish relativeBearing is indeed 45 degrees
        Rot2 actual2 = Rot2.relativeBearing(l2, actualH);
        assertTrue(assert_equal(Rot2.fromAngle(Math.PI / 4.0), actual2));

        // Check numerical derivative
        expectedH = NumericalDerivative.<//
                Rot2, Vector1, Point2, Vector2//
        >numericalDerivative11(relativeBearing_, l2,
                1e-3);
        assertTrue(assert_equal(expectedH, actualH, 1e-6));
    }

    static Rot2 id;
    static Rot2 T1;
    static Rot2 T2;

    static {
        try {
            id = new Rot2();
            T1 = new Rot2(0.1);
            T2 = new Rot2(0.2);
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }
    }

    @Test
    void testInvariants() throws Throwable {
        assertTrue(Rot2.check_group_invariants(id, id));
        assertTrue(Rot2.check_group_invariants(id, T1));
        assertTrue(Rot2.check_group_invariants(T2, id));
        assertTrue(Rot2.check_group_invariants(T2, T1));

        assertTrue(Rot2.check_manifold_invariants(id, id));
        assertTrue(Rot2.check_manifold_invariants(id, T1));
        assertTrue(Rot2.check_manifold_invariants(T2, id));
        assertTrue(Rot2.check_manifold_invariants(T2, T1));
    }
}
