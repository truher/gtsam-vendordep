package gtsam;

import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

import gtsam.noiseModel.Diagonal;

public class PriorFactorExtraTest {

    @Test
    void testMismatchedModelDimension() throws Throwable {
        // the noise vector here has dimension 2, but pose2 has
        // dimension 3.
        assertThrows(IllegalArgumentException.class,
                () -> PriorFactor.PriorFactorPose2(
                        Key.X(0l),
                        new Pose2(0, 0, 0),
                        Diagonal.Sigmas(new Vector2(0.1, 0.1))));

    }

}
