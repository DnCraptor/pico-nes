package net.jpountz.lz4;

/*
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

import static net.jpountz.util.Utils.checkRange;

/**
 * {@link LZ4Decompressor} implemented with JNI bindings to the original C
 * implementation of LZ4.
 */
final class LZ4JNIUnknownSizeDecompressor extends LZ4UnknownSizeDecompressor {

  public static final LZ4UnknownSizeDecompressor INSTANCE = new LZ4JNIUnknownSizeDecompressor();

  public final int decompress(byte[] src, int srcOff, int srcLen, byte[] dest, int destOff, int maxDestLen) {
    checkRange(src, srcOff, srcLen);
    checkRange(dest, destOff, maxDestLen);
    final int result = LZ4JNI.LZ4_decompress_unknownOutputSize(src, srcOff, srcLen, dest, destOff, maxDestLen);
    if (result < 0) {
      throw new LZ4Exception("Error decoding offset " + (srcOff - result) + " of input buffer");
    }
    return result;
  }
}
