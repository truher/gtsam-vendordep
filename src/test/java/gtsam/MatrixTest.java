package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class MatrixTest {
    /** Identity covariance produces reasonable samples. */
    @Test
    void testDraw() throws Throwable {
        Matrix m = Matrix.I_3x3();
        m.print("");
        Vector v = m.draw();
        v.print("");
        assertEquals(3, v.dimension());
    }

    /** For bad input, e.g. zero covariance, all the samples are zero. */
    @Test
    void testZeroDraw() throws Throwable {
        Matrix m = new Matrix(3, 3);
        m.print("");
        Vector v = m.draw();
        v.print("");
        assertTrue(assert_equal(new Vector(3), v));
    }

    /** For nonsense input, e.g. negative variances, all the samples are zero. */
    @Test
    void testBadDraw() throws Throwable {
        Matrix m = new Matrix(new double[][] {
                { -1, 0, 0 },
                { 0, -1, 0 },
                { 0, 0, -1 }
        });
        m.print("");
        Vector v = m.draw();
        v.print("");
        assertTrue(assert_equal(new Vector(3), v));
    }

    @Test
    void testManyDraws() throws Throwable {
        // draws from this should look like a gaussian: variance = 1
        Matrix m = new Matrix(new double[][] { { 1.0 } });

        int N = 10000;
        double sumSquaredDeviations = 0;
        for (int i = 0; i < N; ++i) {
            double sample = m.draw().at(0);
            sumSquaredDeviations += Math.pow(sample, 2);
        }
        double variance = sumSquaredDeviations / N;
        double stddev = Math.sqrt(variance);
        assertEquals(1.0, stddev, 1e-2);
    }

}
