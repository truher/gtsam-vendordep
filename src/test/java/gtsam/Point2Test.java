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

        // assertTrue(assert_equal(Point2(5,7), traits<Point2>::Compose(p1, p2, H1,
        // H2)));
        // assertTrue(assert_equal(I_2x2, H1));
        // assertTrue(assert_equal(I_2x2, H2));

        // assertTrue(assert_equal(Point2(3,3), traits<Point2>::Between(p1, p2, H1,
        // H2)));
        // assertTrue(assert_equal(-I_2x2, H1));
        // assertTrue(assert_equal(I_2x2, H2));

        // assertTrue(assert_equal(Point2(5,7), traits<Point2>::Retract(p1, Vector2(4.,
        // 5.))));
        // assertTrue(assert_equal(Vector2(3.,3.), traits<Point2>::Local(p1,p2)));
    }

    @Test
    void testexpmap() {
        // Vector d(2);
        // d(0) = 1;
        // d(1) = -1;
        // Point2 a(4, 5), b = traits<Point2>::Retract(a,d), c(5, 4);
        // assertTrue(assert_equal(b,c));
    }

    @Test
    void testarithmetic() {
        // assertTrue(assert_equal<Point2>(Point2(-5, -6), -Point2(5, 6)));
        // assertTrue(assert_equal<Point2>(Point2(5, 6), Point2(4, 5) + Point2(1, 1)));
        // assertTrue(assert_equal<Point2>(Point2(3, 4), Point2(4, 5) - Point2(1, 1)));
        // assertTrue(assert_equal<Point2>(Point2(8, 6), Point2(4, 3) * 2));
        // assertTrue(assert_equal<Point2>(Point2(4, 6), 2.0 * Point2(2, 3)));
        // assertTrue(assert_equal<Point2>(Point2(2, 3), Point2(4, 6) / 2));
    }

    @Test
    void testunit() throws Throwable {
        Point2 p0 = new Point2(10, 0);
        Point2 p1 = new Point2(0, -10);
        Point2 p2 = new Point2(10, 10);
        // assertTrue(assert_equal(Point2(1, 0), Point2(p0.normalized()), 1e-6));
        // assertTrue(assert_equal(Point2(0,-1), Point2(p1.normalized()), 1e-6));
        // assertTrue(assert_equal(Point2(sqrt(2.0)/2.0, sqrt(2.0)/2.0),
        // Point2(p2.normalized()), 1e-6));
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

    double norm_proxy(Point2 point) throws Throwable {
        return point.norm();
    }

    @Test
    void testnorm() throws Throwable {
        Point2 p0 = new Point2(Math.cos(5.0), Math.sin(5.0));
        assertEquals(1, p0.norm(), 1e-6);
        Point2 p1 = new Point2(4, 5);
        Point2 p2 = new Point2(1, 1);
        assertEquals(5, Point2.distance2(p1, p2), 1e-6);
        // assertEquals( 5, (p2-p1).norm(), 1e-6);

        // Matrix expectedH, actualH;
        // double actual;

        // // exception, for (0,0) derivative is [Inf,Inf] but we return [1,1]
        // actual = norm2(x1, actualH);
        // assertEquals(0, actual, 1e-9);
        // expectedH = (Matrix(1, 2) << 1.0, 1.0).finished();
        // assertTrue(assert_equal(expectedH,actualH));

        // actual = norm2(x2, actualH);
        // assertEquals(sqrt(2.0), actual, 1e-9);
        // expectedH = numericalDerivative11(norm_proxy, x2);
        // assertTrue(assert_equal(expectedH,actualH));

        // // analytical
        // expectedH = (Matrix(1, 2) << x2.x()/actual, x2.y()/actual).finished();
        // assertTrue(assert_equal(expectedH,actualH));
    }

    // namespace {
    // double distance_proxy(const Point2& location, const Point2& point) {
    // return distance2(location, point);
    // }
    // }
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

        // // Check numerical derivatives
        // expectedH1 = numericalDerivative21(distance_proxy, x2, l3);
        // expectedH2 = numericalDerivative22(distance_proxy, x2, l3);
        // assertTrue(assert_equal(expectedH1,actualH1));
        // assertTrue(assert_equal(expectedH2,actualH2));

        // // Another test
        // double actual34 = distance2(x3, l4, actualH1, actualH2);
        // assertEquals(2, actual34, 1e-9);

        // // Check numerical derivatives
        // expectedH1 = numericalDerivative21(distance_proxy, x3, l4);
        // expectedH2 = numericalDerivative22(distance_proxy, x3, l4);
        // assertTrue(assert_equal(expectedH1,actualH1));
        // assertTrue(assert_equal(expectedH2,actualH2));
    }

    @Test
    void testcircleCircleIntersection() {

        double offset = 0.994987;
        // Test intersections of circle moving from inside to outside

        // list<Point2> inside = circleCircleIntersection(Point2(0,0),5,Point2(0,0),1);
        // EXPECT_LONGS_EQUAL(0,inside.size());

        // list<Point2> touching1 =
        // circleCircleIntersection(Point2(0,0),5,Point2(4,0),1);
        // EXPECT_LONGS_EQUAL(1,touching1.size());
        // assertTrue(assert_equal(Point2(5,0), touching1.front()));

        // list<Point2> common = circleCircleIntersection(Point2(0,0),5,Point2(5,0),1);
        // EXPECT_LONGS_EQUAL(2,common.size());
        // assertTrue(assert_equal(Point2(4.9, offset), common.front(), 1e-6));
        // assertTrue(assert_equal(Point2(4.9, -offset), common.back(), 1e-6));

        // list<Point2> touching2 =
        // circleCircleIntersection(Point2(0,0),5,Point2(6,0),1);
        // EXPECT_LONGS_EQUAL(1,touching2.size());
        // assertTrue(assert_equal(Point2(5,0), touching2.front()));

        // // test rotated case
        // list<Point2> rotated = circleCircleIntersection(Point2(0,0),5,Point2(0,5),1);
        // EXPECT_LONGS_EQUAL(2,rotated.size());
        // assertTrue(assert_equal(Point2(-offset, 4.9), rotated.front(), 1e-6));
        // assertTrue(assert_equal(Point2( offset, 4.9), rotated.back(), 1e-6));

        // // test r1<r2
        // list<Point2> smaller = circleCircleIntersection(Point2(0,0),1,Point2(5,0),5);
        // EXPECT_LONGS_EQUAL(2,smaller.size());
        // assertTrue(assert_equal(Point2(0.1, offset), smaller.front(), 1e-6));
        // assertTrue(assert_equal(Point2(0.1, -offset), smaller.back(), 1e-6));

        // // test offset case, r1>r2
        // list<Point2> offset1 = circleCircleIntersection(Point2(1,1),5,Point2(6,1),1);
        // EXPECT_LONGS_EQUAL(2,offset1.size());
        // assertTrue(assert_equal(Point2(5.9, 1+offset), offset1.front(), 1e-6));
        // assertTrue(assert_equal(Point2(5.9, 1-offset), offset1.back(), 1e-6));

        // // test offset case, r1<r2
        // list<Point2> offset2 = circleCircleIntersection(Point2(6,1),1,Point2(1,1),5);
        // EXPECT_LONGS_EQUAL(2,offset2.size());
        // assertTrue(assert_equal(Point2(5.9, 1-offset), offset2.front(), 1e-6));
        // assertTrue(assert_equal(Point2(5.9, 1+offset), offset2.back(), 1e-6));

    }

}
