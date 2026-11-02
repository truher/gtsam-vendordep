package gtsam;

import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/**
 * in gtsam a key is a typedef for uint64.
 * TODO: remove the duplication here.
 */
public class Key {

    private static final MethodHandle symbol_shorthand_A = symbol("A");
    private static final MethodHandle symbol_shorthand_B = symbol("B");
    private static final MethodHandle symbol_shorthand_C = symbol("C");
    private static final MethodHandle symbol_shorthand_D = symbol("D");
    private static final MethodHandle symbol_shorthand_E = symbol("E");
    private static final MethodHandle symbol_shorthand_F = symbol("F");
    private static final MethodHandle symbol_shorthand_G = symbol("G");
    private static final MethodHandle symbol_shorthand_H = symbol("H");
    private static final MethodHandle symbol_shorthand_I = symbol("I");
    private static final MethodHandle symbol_shorthand_J = symbol("J");
    private static final MethodHandle symbol_shorthand_K = symbol("K");
    private static final MethodHandle symbol_shorthand_L = symbol("L");
    private static final MethodHandle symbol_shorthand_M = symbol("M");
    private static final MethodHandle symbol_shorthand_N = symbol("N");
    private static final MethodHandle symbol_shorthand_O = symbol("O");
    private static final MethodHandle symbol_shorthand_P = symbol("P");
    private static final MethodHandle symbol_shorthand_Q = symbol("Q");
    private static final MethodHandle symbol_shorthand_R = symbol("R");
    private static final MethodHandle symbol_shorthand_S = symbol("S");
    private static final MethodHandle symbol_shorthand_T = symbol("T");
    private static final MethodHandle symbol_shorthand_U = symbol("U");
    private static final MethodHandle symbol_shorthand_V = symbol("V");
    private static final MethodHandle symbol_shorthand_W = symbol("W");
    private static final MethodHandle symbol_shorthand_X = symbol("X");
    private static final MethodHandle symbol_shorthand_Y = symbol("Y");
    private static final MethodHandle symbol_shorthand_Z = symbol("Z");
    private static final MethodHandle symbol_print = Lib.ff("symbol_print", null, JAVA_LONG);

    private static final MethodHandle symbol(String label) {
        return Lib.ff("symbol_shorthand_" + label, JAVA_LONG, JAVA_LONG);
    }

    private static Object invoke(MethodHandle h, Object... args) {
        return Lib.invoke(h, args);
    }

    public final long j;

    public Key(long _j) {
        j = _j;
    }

    public void print() {
        invoke(symbol_print, j);
    }

    public static Key A(long j) {
        return new Key((long) invoke(symbol_shorthand_A, j));
    }

    public static Key B(long j) {
        return new Key((long) invoke(symbol_shorthand_B, j));
    }

    public static Key C(long j) {
        return new Key((long) invoke(symbol_shorthand_C, j));
    }

    public static Key D(long j) {
        return new Key((long) invoke(symbol_shorthand_D, j));
    }

    public static Key E(long j) {
        return new Key((long) invoke(symbol_shorthand_E, j));
    }

    public static Key F(long j) {
        return new Key((long) invoke(symbol_shorthand_F, j));
    }

    public static Key G(long j) {
        return new Key((long) invoke(symbol_shorthand_G, j));
    }

    public static Key H(long j) {
        return new Key((long) invoke(symbol_shorthand_H, j));
    }

    public static Key I(long j) {
        return new Key((long) invoke(symbol_shorthand_I, j));
    }

    public static Key J(long j) {
        return new Key((long) invoke(symbol_shorthand_J, j));
    }

    public static Key K(long j) {
        return new Key((long) invoke(symbol_shorthand_K, j));
    }

    public static Key L(long j) {
        return new Key((long) invoke(symbol_shorthand_L, j));
    }

    public static Key M(long j) {
        return new Key((long) invoke(symbol_shorthand_M, j));
    }

    public static Key N(long j) {
        return new Key((long) invoke(symbol_shorthand_N, j));
    }

    public static Key O(long j) {
        return new Key((long) invoke(symbol_shorthand_O, j));
    }

    public static Key P(long j) {
        return new Key((long) invoke(symbol_shorthand_P, j));
    }

    public static Key Q(long j) {
        return new Key((long) invoke(symbol_shorthand_Q, j));
    }

    public static Key R(long j) {
        return new Key((long) invoke(symbol_shorthand_R, j));
    }

    public static Key S(long j) {
        return new Key((long) invoke(symbol_shorthand_S, j));
    }

    public static Key T(long j) {
        return new Key((long) invoke(symbol_shorthand_T, j));
    }

    public static Key U(long j) {
        return new Key((long) invoke(symbol_shorthand_U, j));
    }

    public static Key V(long j) {
        return new Key((long) invoke(symbol_shorthand_V, j));
    }

    public static Key W(long j) {
        return new Key((long) invoke(symbol_shorthand_W, j));
    }

    public static Key X(long j) {
        return new Key((long) invoke(symbol_shorthand_X, j));
    }

    public static Key Y(long j) {
        return new Key((long) invoke(symbol_shorthand_Y, j));
    }

    public static Key Z(long j) {
        return new Key((long) invoke(symbol_shorthand_Z, j));
    }

}
