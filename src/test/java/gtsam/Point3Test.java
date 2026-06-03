package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

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

        assertTrue(assert_equal(new Point3(5, 7, 9), Point3.traits.Compose(p1, p2, H1, H2)));
        assertTrue(assert_equal(Matrix.I_3x3(), H1));
        assertTrue(assert_equal(Matrix.I_3x3(), H2));

        assertTrue(assert_equal(new Point3(3, 3, 3), Point3.traits.Between(p1, p2, H1, H2)));
        assertTrue(assert_equal(Matrix.I_3x3().times(-1), H1));
        assertTrue(assert_equal(Matrix.I_3x3(), H2));

        assertTrue(assert_equal(new Point3(5, 7, 9), Point3.traits.Retract(p1, new Vector3(4, 5, 6))));
        assertTrue(assert_equal(new Vector3(3, 3, 3), Point3.traits.Local(p1, p2)));
    }

    @Test
    void testarithmetic() throws Throwable {
        // assertTrue(P * 3 == 3 * P);
        assertTrue(assert_equal(new Point3(-1, -5, -6), new Point3(1, 5, 6).times(-1)));
        // assertTrue(assert_equal<Point3>(Point3(2, 5, 6), Point3(1, 4, 5) + Point3(1,
        // 1,
        // 1)));
        // assertTrue(assert_equal<Point3>(Point3(0, 3, 4), Point3(1, 4, 5) - Point3(1,
        // 1,
        // 1)));
        // assertTrue(assert_equal<Point3>(Point3(2, 8, 6), Point3(1, 4, 3) * 2));
        // assertTrue(assert_equal<Point3>(Point3(2, 2, 6), 2 * Point3(1, 1, 3)));
        // assertTrue(assert_equal<Point3>(Point3(1, 2, 3), Point3(2, 4, 6) / 2));
    }

    @Test
    void testequals() throws Throwable {
        // assertTrue(Point3.traits.Equals(P, P));
        Point3 Q = new Point3(0, 0, 0);
        // assertTrue(!Point3.traits.Equals(P, Q));
    }

    @Test
    void testdot() throws Throwable {
        Point3 origin = new Point3(0, 0, 0);
        Point3 ones = new Point3(1, 1, 1);
        // assertTrue(origin.dot(Point3(1, 1, 0)) == 0);
        // assertTrue(ones.dot(Point3(1, 1, 0)) == 2);

        Point3 p = new Point3(1, 0.2, 0.3);
        Point3 q = p.plus(new Point3(0.5, 0.2, -3.0));
        Point3 r = p.plus(new Point3(0.8, 0, 0));
        Point3 t = p.plus(new Point3(0, 0.3, -0.4));
        // assertTrue(assert_equal(1.130000, p.dot(p), 1e-8));
        // assertTrue(assert_equal(0.770000, p.dot(q), 1e-5));
        // assertTrue(assert_equal(1.930000, p.dot(r), 1e-5));
        // assertTrue(assert_equal(1.070000, p.dot(t), 1e-5));

        // // Use numerical derivatives to calculate the expected Jacobians
        // Matrix H1, H2;
        // auto f = [](const Point3& p, const Point3& q) { return gtsam::dot(p, q); };
        // {
        // gtsam::dot(p, q, H1, H2);
        // assertTrue(assert_equal(numericalDerivative21<double,Point3>(f, p, q), H1,
        // 1e-9));
        // assertTrue(assert_equal(numericalDerivative22<double,Point3>(f, p, q), H2,
        // 1e-9));
        // }
        // {
        // gtsam::dot(p, r, H1, H2);
        // assertTrue(assert_equal(numericalDerivative21<double,Point3>(f, p, r), H1,
        // 1e-9));
        // assertTrue(assert_equal(numericalDerivative22<double,Point3>(f, p, r), H2,
        // 1e-9));
        // }
        // {
        // gtsam::dot(p, t, H1, H2);
        // assertTrue(assert_equal(numericalDerivative21<double,Point3>(f, p, t), H1,
        // 1e-9));
        // assertTrue(assert_equal(numericalDerivative22<double,Point3>(f, p, t), H2,
        // 1e-9));
        // }
    }

    @Test
    void testcross() throws Throwable {
        Matrix aH1 = new Matrix();
        Matrix aH2 = new Matrix();
        // auto f = [](const Point3& p, const Point3& q) { return gtsam::cross(p, q); };
        // const Point3 omega(0, 1, 0), theta(4, 6, 8);
        // cross(omega, theta, aH1, aH2);
        // assertTrue(assert_equal(numericalDerivative21(f, omega, theta), aH1));
        // assertTrue(assert_equal(numericalDerivative22(f, omega, theta), aH2));
    }

    @Test
    void testcross2() throws Throwable {
        Point3 p = new Point3(1, 0.2, 0.3);
        // Point3 q = p + Point3(0.5, 0.2, -3.0);
        // Point3 r = p + Point3(0.8, 0, 0);
        // assertTrue(assert_equal(Point3(0, 0, 0), p.cross(p), 1e-8));
        // assertTrue(assert_equal(Point3(-0.66, 3.15, 0.1), p.cross(q), 1e-5));
        // assertTrue(assert_equal(Point3(0, 0.24, -0.16), p.cross(r), 1e-5));

        // // Use numerical derivatives to calculate the expected Jacobians
        // Matrix H1, H2;
        // auto f = [](const Point3& p, const Point3& q) { return gtsam::cross(p, q); };
        // {
        // gtsam::cross(p, q, H1, H2);
        // assertTrue(assert_equal(numericalDerivative21<Point3,Point3>(f, p, q), H1,
        // 1e-9));
        // assertTrue(assert_equal(numericalDerivative22<Point3,Point3>(f, p, q), H2,
        // 1e-9));
        // }
        // {
        // gtsam::cross(p, r, H1, H2);
        // assertTrue(assert_equal(numericalDerivative21<Point3,Point3>(f, p, r), H1,
        // 1e-9));
        // assertTrue(assert_equal(numericalDerivative22<Point3,Point3>(f, p, r), H2,
        // 1e-9));
        // }
    }

    @Test
    void testdoubleCross() throws Throwable {
        Matrix aH1 = new Matrix();
        Matrix aH2 = new Matrix();
        // auto f = [](const Point3& p, const Point3& q) { return doubleCross(p, q); };
        // const Point3 omega(1, 2, 3), theta(4, 5, 6);
        // doubleCross(omega, theta, aH1, aH2);
        // assertTrue(assert_equal(numericalDerivative21(f, omega, theta), aH1));
        // assertTrue(assert_equal(numericalDerivative22(f, omega, theta), aH2));
    }

    @Test
    void testnormalize() throws Throwable {
        Matrix actualH = new Matrix();
        Point3 point = new Point3(1, -2, 3); // arbitrary point
        // Point3 expected(point / sqrt(14.0));
        // assertTrue(assert_equal(expected, normalize(point, actualH), 1e-8));
        // auto fn = [](const Point3& p) { return normalize(p); };
        // Matrix expectedH = numericalDerivative11<Point3, Point3>(fn, point);
        // assertTrue(assert_equal(expectedH, actualH, 1e-8));
    }

    @Test
    void testmean() throws Throwable {
        Point3 expected = new Point3(2, 2, 2);
        Point3 a1 = new Point3(0, 0, 0);
        Point3 a2 = new Point3(1, 2, 3);
        Point3 a3 = new Point3(5, 4, 3);
        // std::vector<Point3> a_points{a1, a2, a3};
        // Point3 actual = mean(a_points);
        // assertTrue(assert_equal(expected, actual));
    }

    @Test
    void testmean_pair() throws Throwable {
        Point3 a_mean = new Point3(2, 2, 2);
        Point3 b_mean = new Point3(-1, 1, 0);
        // Point3Pair expected = {a_mean, b_mean};
        Point3 a1 = new Point3(0, 0, 0);
        Point3 a2 = new Point3(1, 2, 3);
        Point3 a3 = new Point3(5, 4, 3);
        Point3 b1 = new Point3(-1, 0, 0);
        Point3 b2 = new Point3(-2, 4, 0);
        Point3 b3 = new Point3(0, -1, 0);
        // std::vector<Point3Pair> point_pairs{{a1, b1}, {a2, b2}, {a3, b3}};
        // Point3Pair actual = means(point_pairs);
        // assertTrue(assert_equal(expected.first, actual.first));
        // assertTrue(assert_equal(expected.second, actual.second));
    }

    // double norm_proxy(const Point3& point) {
    // return double(point.norm());
    // }

    @Test
    void testnorm() throws Throwable {
        Matrix actualH = new Matrix();
        Point3 point = new Point3(3, 4, 5); // arbitrary point
        double expected = Math.sqrt(50);
        // assertEquals(expected, norm3(point, actualH), 1e-8);
        // Matrix expectedH = numericalDerivative11<double, Point3>(norm_proxy, point);
        // assertTrue(assert_equal(expectedH, actualH, 1e-8));
    }

    // double testFunc(const Point3& P, const Point3& Q) {
    // return distance3(P, Q);
    // }

    @Test
    void testdistance() throws Throwable {
        Point3 P = new Point3(1., 12.8, -32.);
        Point3 Q = new Point3(52.7, 4.9, -13.3);
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        // double d = distance3(P, Q, H1, H2);
        // double expectedDistance = 55.542686;
        // Matrix numH1 = numericalDerivative21(testFunc, P, Q);
        // Matrix numH2 = numericalDerivative22(testFunc, P, Q);
        // assertEquals(expectedDistance, d, 1e-5);
        // assertTrue(assert_equal(numH1, H1, 1e-8));
        // assertTrue(assert_equal(numH2, H2, 1e-8));
    }

}
