package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import gtsam.noiseModel.Isotropic;

/**
 * This is a tiny bit of tests/testNonlinearOptimizer.cpp
 * NonlinearOptimizerTest
 */
public class NonlinearOptimizerTest {

    @Test
    void testFactorization() {

        Values config = new Values();
        config.insert(Key.X(1), new Pose2(0., 0., 0.));
        config.insert(Key.X(2), new Pose2(1.5, 0., 0.));

        NonlinearFactorGraph graph = new NonlinearFactorGraph();
        graph.addPrior(Key.X(1), new Pose2(0., 0., 0.), Isotropic.Sigma(3, 1e-10));
        graph.add(BetweenFactorPose2.newBetweenFactorPose2(
                Key.X(1), Key.X(2), new Pose2(1., 0., 0.), Isotropic.Sigma(3, 1)));

        // Ordering ordering;
        // ordering.push_back(X(1));
        // ordering.push_back(X(2));

        LevenbergMarquardtParams params = new LevenbergMarquardtParams();
        params.setLegacyDefaults();
        LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer(graph, config, params);
        optimizer.iterate();

        Values expected = new Values();
        expected.insert(Key.X(1), new Pose2(0., 0., 0.));
        expected.insert(Key.X(2), new Pose2(1., 0., 0.));
        assertTrue(assert_equal(expected, optimizer.values(), 1e-5));
    }
}
