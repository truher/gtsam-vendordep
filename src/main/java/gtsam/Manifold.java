package gtsam;

/**
 * See gtsam/base/Manifold.h.
 * 
 * A manifold is locally homeomorphic to Euclidean (vector) space.
 * 
 * The mapping is described by two operators:
 * 
 * * a.local(b): map b from manifold to vector space tangent at a
 * * a.retract(b): map b from vector space to manifold tangent at a
 * 
 * For a vector space, "local" is "inverse().plus()" and "retract" is "plus"
 * 
 * For a Lie group, "local" is "logmap" and "retract" is "expmap"
 * 
 * In C++ code, the word "localCoordinates" is used instead of "local".
 * 
 * https://en.wikipedia.org/wiki/Manifold
 * 
 * @param <T> the manifold type, e.g. Pose2.
 * @param <V> the type of its tangent vector, e.g. Vector3.
 */
public interface Manifold<//
        T extends Manifold<T, V>, //
        V extends VectorType<V>> {

    /** Zero tangent vector, used by numerical differentiation. */
    V dxZero() throws Throwable;

    int dimension() throws Throwable;

    V local(T other) throws Throwable;

    V local(T g, Matrix H1, Matrix H2) throws Throwable;

    T retract(V v) throws Throwable;

    T retract(V v, Matrix H1, Matrix H2) throws Throwable;
}
