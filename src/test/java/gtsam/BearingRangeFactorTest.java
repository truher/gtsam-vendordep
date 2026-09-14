package gtsam;

import org.junit.jupiter.api.Test;

import gtsam.noiseModel.Diagonal;

public class BearingRangeFactorTest {
    @Test
    void testConstruction() throws Throwable {
        Rot2 bearing = Rot2.fromDegrees(10);
        double range = 1;
        shared_ptr<Diagonal> measurementNoise = Diagonal.Sigmas(new Vector2(0.1, 0.2)); // 0.1 rad std on bearing, 20cm
                                                                                        // on range

        shared_ptr<BearingRangeFactorPose2Point2> f = BearingRangeFactorPose2Point2
                .newBearingRangeFactorPose2Point2(
                        Key.X(1), Key.L(1), bearing, range, measurementNoise);
    }

}
