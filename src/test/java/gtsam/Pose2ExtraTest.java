package gtsam;

import static gtsam.Testable.assert_equal;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import gtsam.NumericalDerivative.ThrowingFunction;

/** These are not part of gtsam */
public class Pose2ExtraTest {
    @Test
    void testExpmapDerivative() throws Throwable {
        Vector3 w0 = new Vector3(0.1, 0.27, 0.0);
        Matrix3 actualH = Pose2.ExpmapDerivative(w0);
        ThrowingFunction<Vector3, Pose2> h = (v) -> Pose2.Expmap(v, new Matrix(3, 3));
        Matrix expectedH = NumericalDerivative.<Pose2, Vector3, Vector3, Vector3>numericalDerivative11(h, w0, 1e-3);
        assertTrue(assert_equal(expectedH, new Matrix(actualH), 1e-6),
                String.format("expected %s actual %s", expectedH, actualH));
    }

}
