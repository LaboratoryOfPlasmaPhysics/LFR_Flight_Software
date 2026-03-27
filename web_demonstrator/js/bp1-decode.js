// Decode BP1 binary output (11 bytes per frequency bin) into displayable values

const BYTES_PER_BIN = 11;

function decodeFloat6_10(hi, lo) {
  const value = (hi << 8) | lo;
  const exponent = (value >> 10) & 0x3F;
  const mantissa = value & 0x3FF;
  if (exponent === 0 && mantissa === 0) return 0;
  return (1 + mantissa / 1024) * Math.pow(2, exponent - 27);
}

function decodeSignedByte(byte) {
  return (byte - 128) / 127.5;
}

function decodePackedByte(byte) {
  const nvecZSign = (byte >> 7) & 1;
  const ellipticity = ((byte >> 3) & 0x0F) / 15;  // 4 bits, normalize to [0,1]
  const dop = (byte & 0x07) / 7;                   // 3 bits, normalize to [0,1]
  return { nvecZSign, ellipticity, dop };
}

export function decodeBP1(buffer, nBins) {
  const psde = new Float32Array(nBins);
  const psdb = new Float32Array(nBins);
  const nvecX = new Float32Array(nBins);
  const nvecY = new Float32Array(nBins);
  const ellipticity = new Float32Array(nBins);
  const dop = new Float32Array(nBins);

  for (let i = 0; i < nBins; i++) {
    const off = i * BYTES_PER_BIN;
    psde[i] = decodeFloat6_10(buffer[off], buffer[off + 1]);
    psdb[i] = decodeFloat6_10(buffer[off + 2], buffer[off + 3]);
    nvecX[i] = decodeSignedByte(buffer[off + 4]);
    nvecY[i] = decodeSignedByte(buffer[off + 5]);
    const packed = decodePackedByte(buffer[off + 6]);
    ellipticity[i] = packed.ellipticity;
    dop[i] = packed.dop;
  }

  return { psde, psdb, nvecX, nvecY, ellipticity, dop };
}
