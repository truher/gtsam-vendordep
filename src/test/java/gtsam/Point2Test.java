package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * See gtsam/geometry/tests/testPoint2.cpp
 */
public class Point2Test {

    @Test
    void testConstructor() throws Throwable {
        new Point2(0, 0);
    }

    //
    @Test
    void testInvariants() throws Throwable {
        Point2 p1 = new Point2(1, 2);
        Point2 p2 = new Point2(4, 5);
        assertTrue(Point2.check_group_invariants(p1, p2));
        assertTrue(Point2.check_manifold_invariants(p1, p2));
    }

    @Test
    void testconstructor() throws Throwable {
        Point2 p1 = new Point2(1, 2);
        Point2 p2 = p1;
        assertTrue(assert_equal(p1, p2));
    }

    @Test
    void testequality() throws Throwable {
        Point2 p1 = new Point2(1, 2);
        Point2 p2 = new Point2(1, 3);
        assertTrue(!(assert_equal(p1, p2)));
    }

    @Test
    void testLie() throws Throwable {
        Point2 p1 = new Point2(1, 2);
        Point2 p2 = new Point2(4, 5);
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();

        assertTrue(assert_equal(new Point2(5, 7), Point2.companion.Compose(p1, p2, H1, H2)));
        assertTrue(assert_equal(Matrix.I_2x2(), H1));
        assertTrue(assert_equal(Matrix.I_2x2(), H2));

        assertTrue(assert_equal(new Point2(3, 3), Point2.companion.Between(p1, p2, H1, H2)));
        assertTrue(assert_equal(Matrix.I_2x2().times(-1), H1));
        assertTrue(assert_equal(Matrix.I_2x2(), H2));

        assertTrue(assert_equal(new Point2(5, 7), Point2.companion.Retract(p1, new Vector2(4., 5.))));
        assertTrue(assert_equal(new Vector2(3., 3.), Point2.companion.Local(p1, p2)));
    }

    @Test
    void testexpmap() throws Throwable {
        Vector2 d = new Vector2(1, -1);
        Point2 a = new Point2(4, 5);
        Point2 b = Point2.companion.Retract(a, d);
        Point2 c = new Point2(5, 4);
        assertTrue(assert_equal(b, c));
    }

    @Test
    void testarithmetic() throws Throwable {
        assertTrue(assert_equal(new Point2(-5, -6), new Point2(5, 6).times(-1)));
        assertTrue(assert_equal(new Point2(5, 6), new Point2(4, 5).plus(new Point2(1, 1))));
        assertTrue(assert_equal(new Point2(3, 4), new Point2(4, 5).minus(new Point2(1, 1))));
        assertTrue(assert_equal(new Point2(8, 6), new Point2(4, 3).times(2)));
        assertTrue(assert_equal(new Point2(4, 6), new Point2(2, 3).times(2.0)));
        assertTrue(assert_equal(new Point2(2, 3), new Point2(4, 6).times(1.0 / 2)));
    }

    @Test
    void testunit() throws Throwable {
        Point2 p0 = new Point2(10, 0);
        Point2 p1 = new Point2(0, -10);
        Point2 p2 = new Point2(10, 10);
        assertTrue(assert_equal(new Point2(1, 0), p0.normalized(), 1e-6));
        assertTrue(assert_equal(new Point2(0, -1), p1.normalized(), 1e-6));
        assertTrue(assert_equal(new Point2(Math.sqrt(2.0) / 2.0, Math.sqrt(2.0) / 2.0), p2.normalized(), 1e-6));
    }

    // some shared test values
    static Point2 x1;
    static Point2 x2;
    static Point2 x3;
    static Point2 l1;
    static Point2 l2;
    static Point2 l3;
    static Point2 l4;

    static {
        try {
            x1 = new Point2(0, 0);
            x2 = new Point2(1, 1);
            x3 = new Point2(1, 1);
            l1 = new Point2(1, 0);
            l2 = new Point2(1, 1);
            l3 = new Point2(2, 2);
            l4 = new Point2(1, 3);
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }
    }

    Vector1 norm_proxy(Point2 point) throws Throwable {
        return new Vector1(point.norm());
    }

    @Test
    void testnorm() throws Throwable {
        Point2 p0 = new Point2(Math.cos(5.0), Math.sin(5.0));
        assertEquals(1, p0.norm(), 1e-6);
        Point2 p1 = new Point2(4, 5);
        Point2 p2 = new Point2(1, 1);
        assertEquals(5, Point2.distance2(p1, p2), 1e-6);
        assertEquals(5, (p2.minus(p1)).norm(), 1e-6);

        Matrix expectedH = new Matrix();
        Matrix actualH = new Matrix();

        // exception, for (0,0) derivative is [Inf,Inf] but we return [1,1]
        double actual = Point2.norm2(x1, actualH);
        assertEquals(0, actual, 1e-9);
        expectedH = new Matrix(new double[][] { { 1.0, 1.0 } });
        assertTrue(assert_equal(expectedH, actualH));

        actual = Point2.norm2(x2, actualH);
        assertEquals(Math.sqrt(2.0), actual, 1e-9);
        expectedH = NumericalDerivative.<//
                Vector1, Vector1, //
                Point2, Vector2>numericalDerivative11(this::norm_proxy, x2, 1e-5);
        assertTrue(assert_equal(expectedH, actualH));

        // analytical
        expectedH = new Matrix(new double[][] { { x2.x() / actual, x2.y() / actual } });
        assertTrue(assert_equal(expectedH, actualH));
    }

    Vector1 distance_proxy(Point2 location, Point2 point) throws Throwable {
        return new Vector1(Point2.distance2(location, point));
    }

    @Test
    void testdistance() throws Throwable {
        Matrix expectedH1 = new Matrix();
        Matrix actualH1 = new Matrix();
        Matrix expectedH2 = new Matrix();
        Matrix actualH2 = new Matrix();

        // establish distance is indeed zero
        assertEquals(1, Point2.distance2(x1, l1), 1e-9);

        // establish distance is indeed 45 degrees
        assertEquals(Math.sqrt(2.0), Point2.distance2(x1, l2), 1e-9);

        // Another pair
        double actual23 = Point2.distance2(x2, l3, actualH1, actualH2);
        assertEquals(Math.sqrt(2.0), actual23, 1e-9);

        // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Vector1, Vector1, //
                Point2, Vector2, //
                Point2, Vector2>numericalDerivative21(this::distance_proxy, x2, l3, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Vector1, Vector1, //
                Point2, Vector2, //
                Point2, Vector2>numericalDerivative22(this::distance_proxy, x2, l3, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));

        // Another test
        double actual34 = Point2.distance2(x3, l4, actualH1, actualH2);
        assertEquals(2, actual34, 1e-9);

        // Check numerical derivatives
        expectedH1 = NumericalDerivative.<//
                Vector1, Vector1, //
                Point2, Vector2, //
                Point2, Vector2>numericalDerivative21(this::distance_proxy, x3, l4, 1e-5);
        expectedH2 = NumericalDerivative.<//
                Vector1, Vector1, //
                Point2, Vector2, //
                Point2, Vector2>numericalDerivative22(this::distance_proxy, x3, l4, 1e-5);
        assertTrue(assert_equal(expectedH1, actualH1));
        assertTrue(assert_equal(expectedH2, actualH2));
    }
}
