package gtsam;

/**
 * See gtsam/base/Lie.h.
 * 
 * TODO: add Jacobians.
 */
public interface LieGroup<T extends LieGroup<T>>
        extends Group<T>, Manifold<T> {
    public interface Traits<T extends LieGroup<T>>
            extends Group.Traits<T>, Manifold.Traits<T> {
        T Expmap(Vector v) throws Throwable;

        Vector Logmap(T g) throws Throwable;

        // *Hg = traits<T>::AdjointMap(g);
    }

    Traits<T> traits();
}
