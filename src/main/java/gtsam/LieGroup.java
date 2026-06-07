package gtsam;

/**
 * See gtsam/base/Lie.h.
 * 
 * Traits are implemented using the companion object pattern.
 * 
 * We implement most, but not all, of the following
 * 
 * LieGroup is a large "base class" of implementations (e.g. "compose"),
 * some of which are static (e.g. "Retract"), implemented using a "statics"
 * singleton.
 * 
 * There is also a large LieGroupTraits class, implemented as a singleton.
 *
 * @param <T> the Lie group type, e.g. Pose2.
 * @param <V> the type of its tangent vector, e.g. Vector3.
 */
public interface LieGroup<//
        T extends LieGroup<T, V>, //
        V extends VectorType<V>>
        extends Group<T>, Manifold<T, V> {
    /**
     * See LieGroupTraits, which also restates the Group and Manifold traits.
     */
    public interface Companion<//
            T extends LieGroup<T, V>, //
            V extends VectorType<V>>
            extends Group.Companion<T>, Manifold.Companion<T, V> {
        /** Implement as statics.Identity(). */
        T Identity() throws Throwable;
        
        default V Local(T origin, T other) throws Throwable {
            return origin.localCoordinates(other);
        }

        default V Local(T origin, T other, Matrix H1, Matrix H2) throws Throwable {
            return origin.localCoordinates(other, H1, H2);
        }

        default T Retract(T origin, V v) throws Throwable {
            return origin.retract(v);
        }

        default T Retract(T origin, V v, Matrix H, Matrix Hv) throws Throwable {
            return origin.retract(v, H, Hv);
        }

        /** Implement as statics.Logmap() */
        V Logmap(T g) throws Throwable;

        /** Implement as statics.Logmap() */
        V Logmap(T m, Matrix Hm) throws Throwable;

        /** Implement as statics.Expmap() */
        T Expmap(V v) throws Throwable;

        /** Implement as statics.Expmap() */
        T Expmap(V v, Matrix Hv) throws Throwable;

        //
        // TODO: remove all the defaults below.
        //
        default T Compose(T m1, T m2) throws Throwable {
            return m1.compose(m2);
        }

        default T Compose(T m1, T m2, Matrix H1, Matrix H2) throws Throwable {
            return m1.compose(m2, H1, H2);
        }

        default T Between(T m1, T m2) throws Throwable {
            return m1.between(m2);
        }

        default T Between(T m1, T m2, Matrix H1, Matrix H2) throws Throwable {
            return m1.between(m2, H1, H2);
        }

        default T Inverse(T m) throws Throwable {
            return m.inverse();
        }

        default T Inverse(T m, Matrix H) throws Throwable {
            return m.inverse(H);
        }

        default Matrix AdjointMap(T m) throws Throwable {
            return m.AdjointMap();
        }

    }

    /**
     * Static methods defined or implied in the base class.
     */
    public interface Statics<//
            T extends LieGroup<T, V>, //
            V extends VectorType<V>> {


        V Logmap(T g) throws Throwable;

        V Logmap(T g, Matrix H) throws Throwable;

        T Expmap(V v) throws Throwable;

        T Expmap(V v, Matrix H) throws Throwable;


        // TODO: delete
        T Retract(V v) throws Throwable;

        // TODO: delete
        V LocalCoordinates(T g) throws Throwable;

        // TODO: delete
        T Retract(V v, Matrix H) throws Throwable;

        // TODO: delete
        V LocalCoordinates(T g, Matrix H) throws Throwable;
    }

    Companion<T, V> companion();

    Statics<T, V> statics();

    // these are from the CRTP helper, LieGroup
    T compose(T h) throws Throwable;

    T compose(T h, Matrix H1, Matrix H2) throws Throwable;

    T between(T h) throws Throwable;

    T between(T h, Matrix H1, Matrix H2) throws Throwable;

    T inverse(Matrix H) throws Throwable;

    Matrix AdjointMap() throws Throwable;

    // TODO: delete this
    default T expmap(V v) throws Throwable {
        return compose(statics().Expmap(v));
    }

    // TODO: delete this
    default V logmap(T g) throws Throwable {
        return statics().Logmap(between(g));
    }

    // TODO: delete this
    T expmap(V v, Matrix H1, Matrix H2) throws Throwable;

    // TODO: delete this
    V logmap(T g, Matrix H1, Matrix H2) throws Throwable;

    // TODO: delete this
    default T retract(V v) throws Throwable {
        return compose(statics().Retract(v));
    }

    // TODO: delete this
    default V localCoordinates(T g) throws Throwable {
        return statics().LocalCoordinates(between(g));
    }

    // TODO: delete this
    T retract(V v, Matrix H1, Matrix H2) throws Throwable;

    // TODO: delete this
    V localCoordinates(T g, Matrix H1, Matrix H2) throws Throwable;
}
