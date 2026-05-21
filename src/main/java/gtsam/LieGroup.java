package gtsam;

import org.team100.foreign.ForeignObject;

/**
 * This is to make NumericalDerivative work. I'm not happy with it,
 * but it does work, kinda.
 * TODO: include dimension as a type parameter.
 */
public interface LieGroup<X extends LieGroup<X>> {
    <T extends ForeignObject> X retract(T v) throws Throwable;
}
