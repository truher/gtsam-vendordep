package gtsam;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import gtsam.FixedLagSmoother.KeyTimestampMap;
import gtsam.FixedLagSmoother.Result;
import gtsam.noiseModel.Diagonal;

public class BatchFixedLagSmootherExtraTest {
    @Test
    void testCatch0() throws Throwable {
        // this does not trigger IndeterminateSystemException in update()
        BatchFixedLagSmoother smoother = new BatchFixedLagSmoother(
                0.2, new LevenbergMarquardtParams());
        NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
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
    void testCatch1() throws Throwable {
        // this triggers IndeterminateSystemException in update()
        // due to the low "between" noise and high prior noise.
        BatchFixedLagSmoother smoother = new BatchFixedLagSmoother(
                0.2, new LevenbergMarquardtParams());
        NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
        newFactors.addPrior(
                Key.X(0l),
                new Pose2(0, 0, 0),
                Diagonal.Sigmas(new Vector3(100, 100, 100)));
        newFactors.add(BetweenFactorPose2.newBetweenFactorPose2(
                Key.X(0l),
                Key.X(20000l),
                new Pose2(0.02, 0, 0),
                Diagonal.Sigmas(new Vector3(0.0001, 0.0001, 0.0001))));

        Values newValues = new Values();
        newValues.insert(Key.X(0l), new Pose2(0, 0, 0));
        newValues.insert(Key.X(20000l), new Pose2(1, 0, 0));
        KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();
        newKeyTimestampMap.put(Key.X(0l), 0.0);
        newKeyTimestampMap.put(Key.X(20000l), 0.02);
        Result result = smoother.update(newFactors, newValues, newKeyTimestampMap);
        long iterations = result.getIterations();
        double error = result.getError();
        // zero iterations indicates an error condition!
        assertEquals(0, iterations);
        // this is garbage
        assertEquals(0, error, 0.001);
    }

}
