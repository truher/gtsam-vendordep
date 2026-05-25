package gtsam;

/**
 * See gtsam/base/Group.h.
 * 
 * There's no "base class" for Group types, just some
 * required methods, and "traits," implemented as a singleton.
 * 
 * Note that the C++ makes a distinction between "additive" and "multiplicative"
 * groups, but we do not, it's up to the implementation. There is only one
 * additive group, "Cyclic," which we never use.
 * 
 * @param <T> the group type (note: there are no non-Lie groups here.)
 */
public interface Group<T extends Group<T>> {

    /**
     * See MultiplicativeGroupTraits and AdditiveGroupTraits.
     * Group traits are all static.
     */
    public interface Traits<T extends Group<T>> {
        /** Required by IsGroup. */
        T Identity() throws Throwable;

        /** Required by IsGroup. */
        default T Compose(T g, T h) throws Throwable {
            return g.compose(h);
        }

        /** Required by IsGroup. */
        default T Between(T g, T h) throws Throwable {
            return g.inverse().compose(h);
        }

        /** Required by IsGroup. */
        default T Inverse(T g) throws Throwable {
            return g.inverse();
        }
    }

    Traits<T> traits();

    /** Actually operator*, required by MultiplicativeGroupTraits */
    T compose(T h) throws Throwable;

    /** Required by MultiplicativeGroupTraits */
    T inverse() throws Throwable;
}
