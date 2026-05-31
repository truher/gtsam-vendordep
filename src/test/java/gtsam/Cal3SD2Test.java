package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import gtsam.NumericalDerivative.ThrowingFunction2;

/**
 * See gtsam/geometry/tests/testCal3DS2.cpp
 */
public class Cal3SD2Test {

    static Cal3DS2 K;
    static Point2 p;
    static {
        try {
            K = new Cal3DS2(500, 100, 0.1, 320, 240, 1e-3, 2.0 * 1e-3, 3.0 * 1e-3, 4.0 * 1e-3);
            p = new Point2(2, 3);
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }
    }

    @Test
    void testUncalibrate() throws Throwable {
        Vector4 k = K.k();
        double r = p.x() * p.x() + p.y() * p.y();
        double g = 1 + k.at(0) * r + k.at(1) * r * r;
        double tx = 2 * k.at(2) * p.x() * p.y() + k.at(3) * (r + 2 * p.x() * p.x());
        double ty = k.at(2) * (r + 2 * p.y() * p.y()) + 2 * k.at(3) * p.x() * p.y();
        Vector3 v_hat = new Vector3(g * p.x() + tx, g * p.y() + ty, 1.0);
        Vector3 v_i = K.K().times(v_hat);
        Point2 p_i = new Point2(v_i.at(0) / v_i.at(2), v_i.at(1) / v_i.at(2));
        Point2 q = K.uncalibrate(p);
        assertTrue(assert_equal(q, p_i));
    }

    @Test
    void testCalibrate() throws Throwable {
        Point2 pn = new Point2(0.5, 0.5);
        Point2 pi = K.uncalibrate(pn);
        Point2 pn_hat = K.calibrate(pi);
        assertTrue(assert_equal(pn, pn_hat, 1e-5));
    }

    @Test
    void testDuncalibrate1() throws Throwable {
        Matrix computed = new Matrix();
        K.uncalibrate(p, computed, new Matrix());
        ThrowingFunction2<Cal3DS2, Point2, Point2> uncal = (k, pt) -> k.uncalibrate(pt);
        Matrix numerical = NumericalDerivative
                .<Point2, Vector2, Cal3DS2, Vector9, Point2, Vector2>numericalDerivative21(uncal, K, p, 1e-7);
        assertTrue(assert_equal(numerical, computed, 1e-5));
        Matrix separate = K.D2d_calibration(p);
        assertTrue(assert_equal(numerical, separate, 1e-5));
    }

    @Test
    void testDuncalibrate2() throws Throwable {
        Matrix computed = new Matrix();
        K.uncalibrate(p, new Matrix(), computed);
        ThrowingFunction2<Cal3DS2, Point2, Point2> uncal = (k, pt) -> k.uncalibrate(pt);
        Matrix numerical = NumericalDerivative
                .<Point2, Vector2, Cal3DS2, Vector9, Point2, Vector2>numericalDerivative22(uncal, K, p, 1e-7);
        assertTrue(assert_equal(numerical, computed, 1e-5));
        Matrix separate = K.D2d_intrinsic(p);
        assertTrue(assert_equal(numerical, separate, 1e-5));
    }

    @Test
    void testDcalibrate() throws Throwable {
        Point2 pn = new Point2(0.5, 0.5);
        Point2 pi = K.uncalibrate(pn);
        Matrix Dcal = new Matrix();
        Matrix Dp = new Matrix();
        K.calibrate(pi, Dcal, Dp);
        ThrowingFunction2<Cal3DS2, Point2, Point2> cal = (k, pt) -> k.calibrate(pt);
        Matrix numerical1 = NumericalDerivative
                .<Point2, Vector2, Cal3DS2, Vector9, Point2, Vector2>numericalDerivative21(cal, K, pi, 1e-7);
        assertTrue(assert_equal(numerical1, Dcal, 1e-5));
        Matrix numerical2 = NumericalDerivative
                .<Point2, Vector2, Cal3DS2, Vector9, Point2, Vector2>numericalDerivative22(cal, K, pi, 1e-7);
        assertTrue(assert_equal(numerical2, Dp, 1e-5));
    }

    @Test
    void testEqual() throws Throwable {
        assertTrue(assert_equal(K, K, 1e-5));
    }

    @Test
    void testRetract() throws Throwable {
        Cal3DS2 expected = new Cal3DS2(500 + 1, 100 + 2, 0.1 + 3, 320 + 4, 240 + 5, 1e-3 + 6, 2.0 * 1e-3 + 7,
                3.0 * 1e-3 + 8, 4.0 * 1e-3 + 9);

        assertEquals(Cal3DS2.Dim(), 9);
        assertEquals(expected.dim(), 9);

        Vector9 d = new Vector9(1, 2, 3, 4, 5, 6, 7, 8, 9);
        Cal3DS2 actual = K.retract(d);
        assertTrue(assert_equal(expected, actual, 1e-7));
        assertTrue(assert_equal(d, K.localCoordinates(actual), 1e-7));
    }

}
