package gtsam;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

import gtsam.FixedLagSmoother.KeyTimestampMap;
import gtsam.FixedLagSmoother.Result;
import gtsam.noiseModel.Diagonal;

public class BatchFixedLagSmootherExtraTest {
    @Test
    void testCatch0() {
        // this does not trigger IndeterminateSystemException in update()
        BatchFixedLagSmoother smoother = new BatchFixedLagSmoother(
                0.2, new LevenbergMarquardtParams());
        NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
        // note moderate noise here
        newFactors.addPrior(
                Key.X(0l),
                new Pose2(0, 0, 0),
                Diagonal.Sigmas(new Vector3(1, 1, 1)));
        newFactors.add(BetweenFactorPose2.newBetweenFactorPose2(
                Key.X(0l),
                Key.X(20000l),
                new Pose2(0.02, 0, 0),
                Diagonal.Sigmas(new Vector3(0.01, 0.01, 0.01))));
        Values newValues = new Values();
        newValues.insert(Key.X(0l), new Pose2(0, 0, 0));
        newValues.insert(Key.X(20000l), new Pose2(1, 0, 0));
        KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();
        newKeyTimestampMap.put(Key.X(0l), 0.0);
        newKeyTimestampMap.put(Key.X(20000l), 0.02);
        Result result = smoother.update(newFactors, newValues, newKeyTimestampMap);
        long iterations = result.getIterations();
        double error = result.getError();
        assertEquals(2, iterations);
        assertEquals(0, error, 0.001);
    }

    @Test
    void testCatch1() {
        // this triggers IndeterminateSystemException in update()
        // due to the low "between" noise and high prior noise.
        BatchFixedLagSmoother smoother = new BatchFixedLagSmoother(
                0.2, new LevenbergMarquardtParams());
        NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
        // note very high noise here
        newFactors.addPrior(
                Key.X(0l),
                new Pose2(0, 0, 0),
                Diagonal.Sigmas(new Vector3(100, 100, 100)));
        newFactors.add(BetweenFactorPose2.newBetweenFactorPose2(
                Key.X(0l),
                Key.X(20000l),
                new Pose2(0.02, 0, 0),
                Diagonal.Sigmas(new Vector3(0.01, 0.01, 0.01))));
        Values newValues = new Values();
        newValues.insert(Key.X(0l), new Pose2(0, 0, 0));
        newValues.insert(Key.X(20000l), new Pose2(1, 0, 0));
        KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();
        newKeyTimestampMap.put(Key.X(0l), 0.0);
        newKeyTimestampMap.put(Key.X(20000l), 0.02);
        assertThrows(IndeterminateSystemException.class,
                () -> smoother.update(newFactors, newValues, newKeyTimestampMap));
    }

    @Test
    void testPrintKeyTimestampMap() {
        KeyTimestampMap m = new KeyTimestampMap();
        m.put(Key.X(0), 1.0);
        m.print();
    }

}
