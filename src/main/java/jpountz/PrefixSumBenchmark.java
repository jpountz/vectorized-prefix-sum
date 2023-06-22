package jpountz;

import java.util.concurrent.TimeUnit;

import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.Fork;
import org.openjdk.jmh.annotations.Measurement;
import org.openjdk.jmh.annotations.OutputTimeUnit;
import org.openjdk.jmh.annotations.Warmup;
import org.openjdk.jmh.infra.Blackhole;

import jdk.incubator.vector.IntVector;

@OutputTimeUnit(TimeUnit.MICROSECONDS)
@Warmup(iterations = 5, time = 1, timeUnit = TimeUnit.SECONDS)
@Measurement(iterations = 5, time = 1, timeUnit = TimeUnit.SECONDS)
@Fork(1)
public class PrefixSumBenchmark {

  // See this good resource on using SIMD for prefix sums: https://en.algorithmica.org/hpc/algorithms/prefix/

  @Benchmark
  public void prefixSumScalar(PrefixSumState state, Blackhole bh) {
    int[] input = state.input;
    int[] output = state.output;

    output[0] = input[0];
    for (int i = 1; i < PrefixSumState.ARRAY_LENGTH; ++i) {
      output[i] = output[i-1] + input[i];
    }

    bh.consume(output);
  }

  @Benchmark
  public void prefixSumScalarInlined(PrefixSumState state, Blackhole bh) {
    int[] input = state.input;
    int[] output = state.output;

    output[0] = input[0];
    output[1] = output[0] + input[1];
    output[2] = output[1] + input[2];
    output[3] = output[2] + input[3];
    output[4] = output[3] + input[4];
    output[5] = output[4] + input[5];
    output[6] = output[5] + input[6];
    output[7] = output[6] + input[7];
    output[8] = output[7] + input[8];
    output[9] = output[8] + input[9];
    output[10] = output[9] + input[10];
    output[11] = output[10] + input[11];
    output[12] = output[11] + input[12];
    output[13] = output[12] + input[13];
    output[14] = output[13] + input[14];
    output[15] = output[14] + input[15];
    output[16] = output[15] + input[16];
    output[17] = output[16] + input[17];
    output[18] = output[17] + input[18];
    output[19] = output[18] + input[19];
    output[20] = output[19] + input[20];
    output[21] = output[20] + input[21];
    output[22] = output[21] + input[22];
    output[23] = output[22] + input[23];
    output[24] = output[23] + input[24];
    output[25] = output[24] + input[25];
    output[26] = output[25] + input[26];
    output[27] = output[26] + input[27];
    output[28] = output[27] + input[28];
    output[29] = output[28] + input[29];
    output[30] = output[29] + input[30];
    output[31] = output[30] + input[31];
    output[32] = output[31] + input[32];
    output[33] = output[32] + input[33];
    output[34] = output[33] + input[34];
    output[35] = output[34] + input[35];
    output[36] = output[35] + input[36];
    output[37] = output[36] + input[37];
    output[38] = output[37] + input[38];
    output[39] = output[38] + input[39];
    output[40] = output[39] + input[40];
    output[41] = output[40] + input[41];
    output[42] = output[41] + input[42];
    output[43] = output[42] + input[43];
    output[44] = output[43] + input[44];
    output[45] = output[44] + input[45];
    output[46] = output[45] + input[46];
    output[47] = output[46] + input[47];
    output[48] = output[47] + input[48];
    output[49] = output[48] + input[49];
    output[50] = output[49] + input[50];
    output[51] = output[50] + input[51];
    output[52] = output[51] + input[52];
    output[53] = output[52] + input[53];
    output[54] = output[53] + input[54];
    output[55] = output[54] + input[55];
    output[56] = output[55] + input[56];
    output[57] = output[56] + input[57];
    output[58] = output[57] + input[58];
    output[59] = output[58] + input[59];
    output[60] = output[59] + input[60];
    output[61] = output[60] + input[61];
    output[62] = output[61] + input[62];
    output[63] = output[62] + input[63];
    output[64] = output[63] + input[64];
    output[65] = output[64] + input[65];
    output[66] = output[65] + input[66];
    output[67] = output[66] + input[67];
    output[68] = output[67] + input[68];
    output[69] = output[68] + input[69];
    output[70] = output[69] + input[70];
    output[71] = output[70] + input[71];
    output[72] = output[71] + input[72];
    output[73] = output[72] + input[73];
    output[74] = output[73] + input[74];
    output[75] = output[74] + input[75];
    output[76] = output[75] + input[76];
    output[77] = output[76] + input[77];
    output[78] = output[77] + input[78];
    output[79] = output[78] + input[79];
    output[80] = output[79] + input[80];
    output[81] = output[80] + input[81];
    output[82] = output[81] + input[82];
    output[83] = output[82] + input[83];
    output[84] = output[83] + input[84];
    output[85] = output[84] + input[85];
    output[86] = output[85] + input[86];
    output[87] = output[86] + input[87];
    output[88] = output[87] + input[88];
    output[89] = output[88] + input[89];
    output[90] = output[89] + input[90];
    output[91] = output[90] + input[91];
    output[92] = output[91] + input[92];
    output[93] = output[92] + input[93];
    output[94] = output[93] + input[94];
    output[95] = output[94] + input[95];
    output[96] = output[95] + input[96];
    output[97] = output[96] + input[97];
    output[98] = output[97] + input[98];
    output[99] = output[98] + input[99];
    output[100] = output[99] + input[100];
    output[101] = output[100] + input[101];
    output[102] = output[101] + input[102];
    output[103] = output[102] + input[103];
    output[104] = output[103] + input[104];
    output[105] = output[104] + input[105];
    output[106] = output[105] + input[106];
    output[107] = output[106] + input[107];
    output[108] = output[107] + input[108];
    output[109] = output[108] + input[109];
    output[110] = output[109] + input[110];
    output[111] = output[110] + input[111];
    output[112] = output[111] + input[112];
    output[113] = output[112] + input[113];
    output[114] = output[113] + input[114];
    output[115] = output[114] + input[115];
    output[116] = output[115] + input[116];
    output[117] = output[116] + input[117];
    output[118] = output[117] + input[118];
    output[119] = output[118] + input[119];
    output[120] = output[119] + input[120];
    output[121] = output[120] + input[121];
    output[122] = output[121] + input[122];
    output[123] = output[122] + input[123];
    output[124] = output[123] + input[124];
    output[125] = output[124] + input[125];
    output[126] = output[125] + input[126];
    output[127] = output[126] + input[127];

    bh.consume(output);
  }
  
  @Benchmark
  public void prefixSumVector128(PrefixSumState state, Blackhole bh) {
    int[] input = state.input;
    int[] output = state.output;

    IntVector vec0 = IntVector.fromArray(IntVector.SPECIES_128, input, 0);
    vec0 = vec0.add(vec0.unslice(1));
    vec0 = vec0.add(vec0.unslice(2));
    vec0.intoArray(output, 0);

    for (int i = IntVector.SPECIES_128.length(); i < PrefixSumState.ARRAY_LENGTH; i += IntVector.SPECIES_128.length()) {
      IntVector vec = IntVector.fromArray(IntVector.SPECIES_128, input, i);
      vec = vec.add(vec.unslice(1));
      vec = vec.add(vec.unslice(2));
      vec = vec.add(IntVector.broadcast(IntVector.SPECIES_128, output[i-1]));
      vec.intoArray(output, i);
    }

    bh.consume(output);
  }

  @Benchmark
  public void prefixSumVector256(PrefixSumState state, Blackhole bh) {
    int[] input = state.input;
    int[] output = state.output;

    IntVector vec0 = IntVector.fromArray(IntVector.SPECIES_256, input, 0);
    vec0 = vec0.add(vec0.unslice(1));
    vec0 = vec0.add(vec0.unslice(2));
    vec0 = vec0.add(vec0.unslice(4));
    vec0.intoArray(output, 0);

    for (int i = IntVector.SPECIES_256.length(); i < PrefixSumState.ARRAY_LENGTH; i += IntVector.SPECIES_256.length()) {
      IntVector vec = IntVector.fromArray(IntVector.SPECIES_256, input, i);
      vec = vec.add(vec.unslice(1));
      vec = vec.add(vec.unslice(2));
      vec = vec.add(vec.unslice(4));
      vec = vec.add(IntVector.broadcast(IntVector.SPECIES_256, output[i-1]));
      vec.intoArray(output, i);
    }

    bh.consume(output);
  }

  @Benchmark
  public void prefixSumVector512(PrefixSumState state, Blackhole bh) {
    int[] input = state.input;
    int[] output = state.output;

    IntVector vec0 = IntVector.fromArray(IntVector.SPECIES_512, input, 0);
    vec0 = vec0.unslice(1);
    vec0 = vec0.unslice(2);
    vec0 = vec0.unslice(4);
    vec0 = vec0.unslice(8);
    vec0.intoArray(output, 0);

    for (int i = IntVector.SPECIES_512.length(); i < PrefixSumState.ARRAY_LENGTH; i += IntVector.SPECIES_512.length()) {
      IntVector vec = IntVector.fromArray(IntVector.SPECIES_512, input, i);
      vec = vec.add(vec.unslice(1));
      vec = vec.add(vec.unslice(2));
      vec = vec.add(vec.unslice(4));
      vec = vec.add(vec.unslice(8));
      vec = vec.add(IntVector.broadcast(IntVector.SPECIES_512, output[i-1]));
      vec.intoArray(output, i);
    }

    bh.consume(output);
  }

}
