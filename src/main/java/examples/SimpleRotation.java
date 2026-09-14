package examples;

import gtsam.Key;
import gtsam.LevenbergMarquardtOptimizer;
import gtsam.NonlinearFactorGraph;
import gtsam.Rot2;
import gtsam.Values;
import gtsam.shared_ptr;
import gtsam.noiseModel.Isotropic;

/**
 * This example will perform a relatively trivial optimization on
 * a single variable with a single factor.
 */
public class SimpleRotation {
    private static final double degree = Math.PI / 180;

    public static void main(String args[]) throws Throwable {

        /**
         * Step 1: Create a factor to express a unary constraint
         * The "prior" in this case is the measurement from a sensor,
         * with a model of the noise on the measurement.
         *
         * The "Key" created here is a label used to associate parts of the
         * state (stored in "RotValues") with particular factors. They require
         * an index to allow for lookup, and should be unique.
         *
         * In general, creating a factor requires:
         * - A key or set of keys labeling the variables that are acted upon
         * - A measurement value
         * - A measurement model with the correct dimensionality for the factor
         */
        Rot2 prior = Rot2.fromAngle(30 * degree);
        prior.print("goal angle");
        shared_ptr<Isotropic> model = Isotropic.Sigma(1, 1 * degree);
        Key key = Key.X(1);

        /**
         * Step 2: Create a graph container and add the factor to it
         * Before optimizing, all factors need to be added to a Graph container,
         * which provides the necessary top-level functionality for defining a
         * system of constraints.
         *
         * In this case, there is only one factor, but in a practical scenario,
         * many more factors would be added.
         */
        NonlinearFactorGraph graph = new NonlinearFactorGraph();
        graph.addPrior(key, prior, model);
        graph.print("full graph");

        /**
         * Step 3: Create an initial estimate
         * An initial estimate of the solution for the system is necessary to
         * start optimization. This system state is the "RotValues" structure,
         * which is similar in structure to a STL map, in that it maps
         * keys (the label created in step 1) to specific values.
         *
         * The initial estimate provided to optimization will be used as
         * a linearization point for optimization, so it is important that
         * all of the variables in the graph have a corresponding value in
         * this structure.
         *
         * The interface to all RotValues types is the same, it only depends
         * on the type of key used to find the appropriate value map if there
         * are multiple types of variables.
         */
        Values initial = new Values();
        initial.insert(key, Rot2.fromAngle(20 * degree));
        initial.print("initial estimate");

        /**
         * Step 4: Optimize
         * After formulating the problem with a graph of constraints
         * and an initial estimate, executing optimization is as simple
         * as calling a general optimization function with the graph and
         * initial estimate. This will yield a new RotValues structure
         * with the final state of the optimization.
         */
        Values result = new LevenbergMarquardtOptimizer(graph, initial).optimize();
        result.print("final result");

    }

}
