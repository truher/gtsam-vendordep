package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import gtsam.FixedLagSmoother.KeyTimestampMap;
import gtsam.noiseModel.Diagonal;

/**
 * see gtsam/nonlinear/tests/testIncrementalFixedLagSmoother.cpp
 * 
 * The "ExampleWithFactorRemoval" test used reference capture in a way I didn't
 * want to rewrite in Java, so I deleted it.
 */
public class IncrementalFixedLagSmootherTest {

    boolean check_smoother(final NonlinearFactorGraph fullgraph,
            final Values fullinit,
            final IncrementalFixedLagSmoother smoother,
            final Key key) throws Throwable {
        shared_ptr<GaussianFactorGraph> linearized = fullgraph.linearize(fullinit);
        VectorValues delta = linearized.get().optimize();
        Values fullfinal = fullinit.retract(delta);

        Point2 expected = fullfinal.atPoint2(key);
        Point2 actual = smoother.calculateEstimatePoint2(key);

        return assert_equal(expected, actual);
    }

    @Test
    void testExample() throws Throwable {
        // Test the IncrementalFixedLagSmoother in a pure linear environment. Thus,
        // full optimization and the IncrementalFixedLagSmoother should be identical
        // (even with the linearized approximations at the end of the smoothing lag

        // Set up parameters
        shared_ptr<Diagonal> odoNoise = Diagonal.Sigmas(new Vector2(0.1, 0.1));
        shared_ptr<Diagonal> loopNoise = Diagonal.Sigmas(new Vector2(0.1, 0.1));

        // Create a Fixed-Lag Smoother
        IncrementalFixedLagSmoother smoother = new IncrementalFixedLagSmoother(
                12.0, new ISAM2Params());

        // Create containers to keep the full graph
        Values fullinit = new Values();
        NonlinearFactorGraph fullgraph = new NonlinearFactorGraph();

        // i keeps track of the time step
        int i = 0;

        // Add a prior at time 0 and update the HMF
        {
            Key key0 = Key.X(0);

            NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
            Values newValues = new Values();
            KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();

            newFactors.addPrior(key0, new Point2(0.0, 0.0), odoNoise);
            newValues.insert(key0, new Point2(0.01, 0.01));
            newKeyTimestampMap.put(key0, 0.0);

            fullgraph.add(newFactors);
            fullinit.insert(newValues);

            // Update the smoother
            smoother.update(newFactors, newValues, newKeyTimestampMap);

            // // Check
            assertTrue(check_smoother(fullgraph, fullinit, smoother, key0));

            ++i;
        }

        // Add odometry from time 0 to time 5
        while (i <= 5) {
            Key key1 = Key.X(i - 1);
            Key key2 = Key.X(i);

            NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
            Values newValues = new Values();
            KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();

            newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                    key1, key2, new Point2(1.0, 0.0), odoNoise));
            newValues.insert(key2, new Point2(i + 0.1, -0.1));
            newKeyTimestampMap.put(key2, i);

            fullgraph.add(newFactors);
            fullinit.insert(newValues);

            // Update the smoother
            smoother.update(newFactors, newValues, newKeyTimestampMap);

            // Check
            assertTrue(check_smoother(fullgraph, fullinit, smoother, key2));

            ++i;
        }

        // Add odometry from time 5 to 6 to the HMF and a loop closure at time 5 to
        // the TSM
        {
            // Add the odometry factor to the HMF
            Key key1 = Key.X(i - 1);
            Key key2 = Key.X(i);

            NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
            Values newValues = new Values();
            KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();

            newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                    key1, key2, new Point2(1.0, 0.0), odoNoise));
            newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                    Key.X(2), Key.X(5), new Point2(3.5, 0.0), loopNoise));
            newValues.insert(key2, new Point2(i + 0.1, -0.1));
            newKeyTimestampMap.put(key2, i);

            fullgraph.add(newFactors);
            fullinit.insert(newValues);

            // Update the smoother
            smoother.update(newFactors, newValues, newKeyTimestampMap);

            // Check
            assertTrue(check_smoother(fullgraph, fullinit, smoother, key2));

            ++i;
        }

        // Add odometry from time 6 to time 15
        while (i <= 15) {
            Key key1 = Key.X(i - 1);
            Key key2 = Key.X(i);

            NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
            Values newValues = new Values();
            KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();

            // Add the odometry factor twice to ensure the removeFactor test below
            // works, where we need to keep the connectivity of the graph.
            newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                    key1, key2, new Point2(1.0, 0.0), odoNoise));
            newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                    key1, key2, new Point2(1.0, 0.0), odoNoise));
            newValues.insert(key2, new Point2(i + 0.1, -0.1));
            newKeyTimestampMap.put(key2, i);

            fullgraph.add(newFactors);
            fullinit.insert(newValues);

            // Update the smoother
            smoother.update(newFactors, newValues, newKeyTimestampMap);

            // Check
            assertTrue(check_smoother(fullgraph, fullinit, smoother, key2));

            ++i;
        }

        // add/remove an extra factor
        {
            Key key1 = Key.X(i - 1);
            Key key2 = Key.X(i);

            NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
            Values newValues = new Values();
            KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();

            // add 2 odometry factors
            newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                    key1, key2, new Point2(1.0, 0.0), odoNoise));
            newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                    key1, key2, new Point2(1.0, 0.0), odoNoise));
            newValues.insert(key2, new Point2(i + 0.1, -0.1));
            newKeyTimestampMap.put(key2, i);
            ++i;

            fullgraph.add(newFactors);
            fullinit.insert(newValues);

            // Update the smoother
            smoother.update(newFactors, newValues, newKeyTimestampMap);

            // Check
            assertTrue(check_smoother(fullgraph, fullinit, smoother, key2));

            // now remove one of the two and try again
            // empty values and new factors for fake update in which we only remove
            // factors
            NonlinearFactorGraph emptyNewFactors = new NonlinearFactorGraph();
            Values emptyNewValues = new Values();
            KeyTimestampMap emptyNewKeyTimestampMap = new KeyTimestampMap();

            // any index that does not break connectivity of the graph
            int factorIndex = 25;
            FactorIndices factorToRemove = new FactorIndices();
            factorToRemove.add(new Key(factorIndex));

            final NonlinearFactorGraph smootherFactorsBeforeRemove = smoother.getFactors();

            System.out.printf("fullgraph.size() = %d\n", fullgraph.size());
            System.out.printf("smootherFactorsBeforeRemove.size() = %d\n", smootherFactorsBeforeRemove.size());

            // remove factor
            smoother.update(emptyNewFactors, emptyNewValues, emptyNewKeyTimestampMap,
                    factorToRemove);

            // check that the factors in the smoother are right
            NonlinearFactorGraph actual = smoother.getFactors();
            for (int ii = 0; ii < smootherFactorsBeforeRemove.size(); ii++) {
                // check that the factors that were not removed are there
                if (smootherFactorsBeforeRemove.at(ii).get() != null && i != factorIndex) {
                    assertTrue(smootherFactorsBeforeRemove.at(ii).get().equals(actual.at(ii).get()));
                } else {
                    // while the factors that were not there or were removed are no longer there
                    assertTrue(actual.at(ii).get() == null);
                }
            }
        }

        {
            // Do pressure test on marginalization. Enlarge max_i to enhance the test.
            final int max_i = 500;
            while (i <= max_i) {
                Key key_0 = Key.X(i);
                Key key_1 = Key.X(i - 1);
                Key key_2 = Key.X(i - 2);
                Key key_3 = Key.X(i - 3);
                Key key_4 = Key.X(i - 4);
                Key key_5 = Key.X(i - 5);
                Key key_6 = Key.X(i - 6);
                Key key_7 = Key.X(i - 7);
                Key key_8 = Key.X(i - 8);
                Key key_9 = Key.X(i - 9);
                Key key_10 = Key.X(i - 10);

                NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
                Values newValues = new Values();
                KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();

                // To make a complex graph
                final Point2 z = new Point2(1.0, 0.0);
                newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                        key_1, key_0, z, odoNoise));
                if (i % 2 == 0)
                    newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                            key_2, key_1, z, odoNoise));
                if (i % 3 == 0)
                    newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                            key_3, key_2, z, odoNoise));
                if (i % 4 == 0)
                    newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                            key_4, key_3, z, odoNoise));
                if (i % 5 == 0)
                    newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                            key_5, key_4, z, odoNoise));
                if (i % 6 == 0)
                    newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                            key_6, key_5, z, odoNoise));
                if (i % 7 == 0)
                    newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                            key_7, key_6, z, odoNoise));
                if (i % 8 == 0)
                    newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                            key_8, key_7, z, odoNoise));
                if (i % 9 == 0)
                    newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                            key_9, key_8, z, odoNoise));
                if (i % 10 == 0)
                    newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                            key_10, key_9, z, odoNoise));

                newValues.insert(key_0, new Point2(i + 0.1, -0.1));
                newKeyTimestampMap.put(key_0, i);

                fullgraph.add(newFactors);
                fullinit.insert(newValues);

                // Update the smoother
                smoother.update(newFactors, newValues, newKeyTimestampMap);

                // Check
                assertTrue(check_smoother(fullgraph, fullinit, smoother, key_0));

                ++i;
            }
        }
    }
}
