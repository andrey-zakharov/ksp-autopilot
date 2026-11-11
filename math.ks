// --------------------------------------------------------------------------
// Ограничение значения
// --------------------------------------------------------------------------
function clamp { parameter valueNum. parameter loNum. parameter hiNum.
    return MIN(MAX(valueNum, loNum), hiNum).
}
