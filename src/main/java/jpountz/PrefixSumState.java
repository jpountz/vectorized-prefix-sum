package jpountz;

import org.openjdk.jmh.annotations.Level;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.Setup;
import org.openjdk.jmh.annotations.State;

@State(Scope.Benchmark)
public class PrefixSumState {

  public static final int ARRAY_LENGTH = 128;

  final int[] input = new int[ARRAY_LENGTH];
  final int[] output = new int[ARRAY_LENGTH];

  @Setup(Level.Trial)
  public void setup() {
    for (int i = 0; i < input.length; ++i) {
      input[i] = (i+1) & 0x0F;
    }
  }

}
