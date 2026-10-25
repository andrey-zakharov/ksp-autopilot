// ==========================================================================
// utils.ks — общие утилитные функции для ascent, exec_node, land_body
// ==========================================================================
// Подключение: RUN utils. (в начале каждого скрипта после PARAMETER)
// ==========================================================================

// Ограничение значения диапазоном [lo, hi]
FUNCTION clamp {
    PARAMETER x.
    PARAMETER lo.
    PARAMETER hi.
    IF x < lo { RETURN lo. }
    IF x > hi { RETURN hi. }
    RETURN x.
}

// Безопасное деление: при нулевом/отрицательном знаменателе возвращаем fallback
FUNCTION safeDiv {
    PARAMETER a.
    PARAMETER b.
    PARAMETER fallback.
    IF b <= 0.0001 { RETURN fallback. }
    RETURN a / b.
}

// Давление атмосферы на заданной высоте (в атмосферах)
FUNCTION atmPressure {
    PARAMETER altit.
    IF NOT BODY:ATM:EXISTS { RETURN 0. }
    LOCAL p IS BODY:ATM:ALTITUDEPRESSURE(altit).
    RETURN MAX(0, p).
}

// Плотность атмосферы на заданной высоте (кг/м³)
FUNCTION atmDensity {
    PARAMETER altit.
    IF NOT BODY:ATM:EXISTS { RETURN 0. }
    LOCAL pPa IS atmPressure(altit) * 101325.
    IF pPa <= 0 { RETURN 0. }
    LOCAL temp IS 288.15 - 0.0065 * altit.
    IF temp < 216.65 { SET temp TO 216.65. }
    RETURN pPa / (287.05 * temp).
}

// Динамическое давление (Па)
FUNCTION dynPressure {
    PARAMETER altit.
    PARAMETER veloc.
    IF NOT BODY:ATM:EXISTS { RETURN 0. }
    LOCAL rho IS atmDensity(altit).
    RETURN 0.5 * rho * veloc * veloc.
}

// Угол тангажа вектора относительно горизонта
FUNCTION vectorPitch {
    PARAMETER vec.
    IF vec:MAG < 0.1 { RETURN 90. }
    LOCAL sinGamma IS VDOT(vec, SHIP:UP:FOREVECTOR) / vec:MAG.
    SET sinGamma TO clamp(sinGamma, -1, 1).
    RETURN ARCSIN(sinGamma).
}