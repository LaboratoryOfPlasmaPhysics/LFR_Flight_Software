def ieee(f, word=32):
    """Generate the binary string representing a floating-point value in IEEE754 code."""
    f = float(f)
    if word not in (16, 32, 64, 128):
        raise ValueError("IEEE754 is defined for 16, 32, 64 and 128 bit words.")
    exp_bin = range({16:5, 32:8, 64:11, 128:15}[word])
    frac_bin = range(word - len(exp_bin) - 1)
 
    # Sign.
    sign_bin = [int(f < 0)]
    f = abs(f)
 
    # Find exponent (adding the bias).
    bias = 2**(len(exp_bin)-1) -1
    exponent = bias
    while f >= 2:
            exponent += 1
            f /= 2
    while f < 1 and exponent > 0:
        exponent -= 1
        f *= 2
    if not 0 <= exponent < 2*(bias+1):
        raise ValueError("Exponent overflow: Absolute exponent must be smaller than %d." % (bias + 1))
 
    # Encode exponent in binary.
    for i in exp_bin[::-1]:
        exp_bin[i] = int(exponent % 2)
        exponent /= 2
 
    # Remove the leading 1 bit.
    f -= 1
    for i in frac_bin:
        f *= 2
        frac_bin[i] = int(f >= 1)
        f -= frac_bin[i]
 
    # Join the binary string components together.
    return "".join(map(str, sign_bin + exp_bin + frac_bin))
    
def ieee_decode(binary):
    """Decode a binary string representing a floating-point value in IEEE754 code."""
 
    # Determine the word size of the binary.
    word = len(binary)
    if word not in (16, 32, 64, 128):
        raise ValueError("IEEE754 is defined for 16, 32, 64 and 128 bit words.")
 
    # Determine the length of the exponent.
    e = {16:5, 32:8, 64:11, 128:15}[word]
 
    # Turn the binary string into a bit-list.
    binary = [int(c) for c in binary]
 
    # Split the components.
    sign = -2 * binary[0] + 1
    exp_bin = binary[1:1+e]
    frac_bin = binary[1+e:]
 
    # Decode the exponent.
    bias = 2**(e-1) - 1
    exponent = -bias
    c = 1
    for i in exp_bin[::-1]:
        exponent += i * c
        c *= 2
 
    # Decode the fraction.
    f = float(2**exponent)
    power = f
    for i in frac_bin:
        power *= 0.5
        f += i * power
 
    return sign * f

if __name__ == "__main__":
    val =  ieee(24.)
    val
    print val[:16]
    print val[0], val[1], val[2]
    val_in_int = int(val[:16], 2)
    print val_in_int
    print hex( val_in_int )