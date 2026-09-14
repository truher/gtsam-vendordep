package gtsam;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import gtsam.noiseModel.Isotropic;

public class BetweenFactorExtraTest {
    @Test
    void testKeys() throws Throwable {
        shared_ptr<Isotropic> model = Isotropic.Sigma(1, 1, true);
        double measured = 0.0;
        shared_ptr<BetweenFactorDouble> factor = BetweenFactorDouble.newBetweenFactorDouble(
                new Key(1), new Key(2), measured, model);
        KeyVector kv = factor.get().keys();
        assertEquals(2, kv.size());
        assertEquals(1, kv.at(0).j);
        assertEquals(2, kv.at(1).j);
    }

}
