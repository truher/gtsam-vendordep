package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import gtsam.NumericalDerivative.ThrowingFunction;
import gtsam.NumericalDerivative.ThrowingFunction2;

/**
 * see gtsam/geometry/tests/testPoint3.cpp
 */
public class Point3Test {

    static Point3 P;
    static {
        try {
            P = new Point3(0.2, 0.7, -2);
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }
    }

    @Test
    void testConstructor() throws Throwable {
        new Point3(0, 0, 0);
    }

    @Test
    void testInvariants() throws Throwable {
        Point3 p1 = new Point3(1, 2, 3);
        Point3 p2 = new Point3(4, 5, 6);
        assertTrue(Point3.check_group_invariants(p1, p2));
        assertTrue(Point3.check_manifold_invariants(p1, p2));
    }

    @Test
    void testLie() throws Throwable {
        Point3 p1 = new Point3(1, 2, 3);
        Point3 p2 = new Point3(4, 5, 6);
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();

        assertTrue(assert_equal(new Point3(5, 7, 9), p1.compose(p2, H1, H2)));
        assertTrue(assert_equal(Matrix.I_3x3(), H1));
        assertTrue(assert_equal(Matrix.I_3x3(), H2));

        assertTrue(assert_equal(new Point3(3, 3, 3), p1.between(p2, H1, H2)));
        assertTrue(assert_equal(Matrix.I_3x3().times(-1), H1));
        assertTrue(assert_equal(Matrix.I_3x3(), H2));

        assertTrue(assert_equal(new Point3(5, 7, 9), p1.retract(new Vector3(4, 5, 6))));
        assertTrue(assert_equal(new Vector3(3, 3, 3), p1.localCoordinates(p2)));
    }

    @Test
    void testarithmetic() throws Throwable {
        assertTrue(assert_equal(new Point3(-1, -5, -6), new Point3(1, 5, 6).times(-1)));
        assertTrue(assert_equal(new Point3(2, 5, 6), new Point3(1, 4, 5).plus(new Point3(1, 1, 1))));
        assertTrue(assert_equal(new Point3(0, 3, 4), new Point3(1, 4, 5).minus(new Point3(1, 1, 1))));
        assertTrue(assert_equal(new Point3(2, 8, 6), new Point3(1, 4, 3).times(2)));
        assertTrue(assert_equal(new Point3(2, 2, 6), new Point3(1, 1, 3).times(2)));
        assertTrue(assert_equal(new Point3(1, 2, 3), new Point3(2, 4, 6).times(1.0 / 2)));
    }

    @Test
    void testdot() throws Throwable {
        Point3 origin = new Point3(0, 0, 0);
        Point3 ones = new Point3(1, 1, 1);
        assertTrue(origin.dot(new Point3(1, 1, 0)) == 0);
        assertTrue(ones.dot(new Point3(1, 1, 0)) == 2);

        Point3 p = new Point3(1, 0.2, 0.3);
        Point3 q = p.plus(new Point3(0.5, 0.2, -3.0));
        Point3 r = p.plus(new Point3(0.8, 0, 0));
        Point3 t = p.plus(new Point3(0, 0.3, -0.4));
        assertTrue(assert_equal(1.130000, p.dot(p), 1e-8));
        assertTrue(assert_equal(0.770000, p.dot(q), 1e-5));
        assertTrue(assert_equal(1.930000, p.dot(r), 1e-5));
        assertTrue(assert_equal(1.070000, p.dot(t), 1e-5));

        // Use numerical derivatives to calculate the expected Jacobians
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        ThrowingFunction2<Point3, Point3, Vector1> f = (pp, qq) -> new Vector1(Point3.dot(pp, qq));

        {
            Point3.dot(p, q, H1, H2);
            assertTrue(assert_equal(NumericalDerivative.<//
                    Vector1, Vector1, //
                    Point3, Vector3, //
                    Point3, Vector3>numericalDerivative21(f, p, q, 1e-5), H1, 1e-9));
            assertTrue(assert_equal(NumericalDerivative.<//
                    Vector1, Vector1, //
                    Point3, Vector3, //
                    Point3, Vector3>numericalDerivative22(f, p, q, 1e-5), H2, 1e-9));
        }
        {
            Point3.dot(p, r, H1, H2);
            assertTrue(assert_equal(NumericalDerivative.<//
                    Vector1, Vector1, //
                    Point3, Vector3, //
                    Point3, Vector3>numericalDerivative21(f, p, r, 1e-5), H1, 1e-9));
            assertTrue(assert_equal(NumericalDerivative.<//
                    Vector1, Vector1, //
                    Point3, Vector3, //
                    Point3, Vector3>numericalDerivative22(f, p, r, 1e-5), H2, 1e-9));
        }
        {
            Point3.dot(p, t, H1, H2);
            assertTrue(assert_equal(NumericalDerivative.<//
                    Vector1, Vector1, //
                    Point3, Vector3, //
                    Point3, Vector3>numericalDerivative21(f, p, t, 1e-5), H1, 1e-9));
            assertTrue(assert_equal(NumericalDerivative.<//
                    Vector1, Vector1, //
                    Point3, Vector3, //
                    Point3, Vector3>numericalDerivative22(f, p, t, 1e-5), H2, 1e-9));
        }
    }

    @Test
    void testcross() throws Throwable {
        Matrix aH1 = new Matrix();
        Matrix aH2 = new Matrix();
        ThrowingFunction2<Point3, Point3, Point3> f = (pp, qq) -> Point3.cross(pp, qq);

        final Point3 omega = new Point3(0, 1, 0);
        final Point3 theta = new Point3(4, 6, 8);
        Point3.cross(omega, theta, aH1, aH2);
        assertTrue(assert_equal(NumericalDerivative.<//
                Point3, Vector3, //
                Point3, Vector3, //
                Point3, Vector3>numericalDerivative21(f, omega, theta, 1e-5), aH1));
        assertTrue(assert_equal(NumericalDerivative.<//
                Point3, Vector3, //
                Point3, Vector3, //
                Point3, Vector3>numericalDerivative22(f, omega, theta, 1e-5), aH2));
    }

    @Test
    void testcross2() throws Throwable {
        Point3 p = new Point3(1, 0.2, 0.3);
        Point3 q = p.plus(new Point3(0.5, 0.2, -3.0));
        Point3 r = p.plus(new Point3(0.8, 0, 0));
        assertTrue(assert_equal(new Point3(0, 0, 0), p.cross(p), 1e-8));
        assertTrue(assert_equal(new Point3(-0.66, 3.15, 0.1), p.cross(q), 1e-5));
        assertTrue(assert_equal(new Point3(0, 0.24, -0.16), p.cross(r), 1e-5));

        // // Use numerical derivatives to calculate the expected Jacobians
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        ThrowingFunction2<Point3, Point3, Point3> f = (pp, qq) -> Point3.cross(pp, qq);

        {
            Point3.cross(p, q, H1, H2);
            assertTrue(assert_equal(NumericalDerivative.<//
                    Point3, Vector3, //
                    Point3, Vector3, //
                    Point3, Vector3>numericalDerivative21(f, p, q, 1e-5), H1, 1e-9));
            assertTrue(assert_equal(NumericalDerivative.<//
                    Point3, Vector3, //
                    Point3, Vector3, //
                    Point3, Vector3>numericalDerivative22(f, p, q, 1e-5), H2, 1e-9));
        }
        {
            Point3.cross(p, r, H1, H2);
            assertTrue(assert_equal(NumericalDerivative.<//
                    Point3, Vector3, //
                    Point3, Vector3, //
                    Point3, Vector3>numericalDerivative21(f, p, r, 1e-5), H1, 1e-9));
            assertTrue(assert_equal(NumericalDerivative.<//
                    Point3, Vector3, //
                    Point3, Vector3, //
                    Point3, Vector3>numericalDerivative22(f, p, r, 1e-5), H2, 1e-9));
        }
    }

    @Test
    void testnormalize() throws Throwable {
        Matrix actualH = new Matrix();
        Point3 point = new Point3(1, -2, 3); // arbitrary point
        Point3 expected = point.times(1.0 / Math.sqrt(14.0));
        assertTrue(assert_equal(expected, Point3.normalize(point, actualH), 1e-8));
        ThrowingFunction<Point3, Point3> fn = (pp) -> Point3.normalize(pp);
        Matrix expectedH = NumericalDerivative.<//
                Point3, Vector3, //
                Point3, Vector3>numericalDerivative11(fn, point, 1e-5);
        assertTrue(assert_equal(expectedH, actualH, 1e-8));
    }

    Vector1 norm_proxy(Point3 point) throws Throwable {
        return new Vector1(point.norm());
    }

    @Test
    void testnorm() throws Throwable {
        Matrix actualH = new Matrix();
        Point3 point = new Point3(3, 4, 5); // arbitrary point
        double expected = Math.sqrt(50);
        assertEquals(expected, Point3.norm3(point, actualH), 1e-8);
        Matrix expectedH = NumericalDerivative.<//
                Vector1, Vector1, //
                Point3, Vector3>numericalDerivative11(this::norm_proxy, point, 1e-5);
        assertTrue(assert_equal(expectedH, actualH, 1e-8));
    }

    Vector1 testFunc(Point3 P, Point3 Q) throws Throwable {
        return new Vector1(Point3.distance3(P, Q));
    }

    @Test
    void testdistance() throws Throwable {
        Point3 P = new Point3(1., 12.8, -32.);
        Point3 Q = new Point3(52.7, 4.9, -13.3);
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        double d = Point3.distance3(P, Q, H1, H2);
        double expectedDistance = 55.542686;
        Matrix numH1 = NumericalDerivative.<//
                Vector1, Vector1, //
                Point3, Vector3, //
                Point3, Vector3>numericalDerivative21(this::testFunc, P, Q, 1e-5);
        Matrix numH2 = NumericalDerivative.<//
                Vector1, Vector1, //
                Point3, Vector3, //
                Point3, Vector3>numericalDerivative22(this::testFunc, P, Q, 1e-5);
        assertEquals(expectedDistance, d, 1e-5);
        assertTrue(assert_equal(numH1, H1, 1e-8));
        assertTrue(assert_equal(numH2, H2, 1e-8));
    }

}
