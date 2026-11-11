function disTime { parameter disEta.
    if disEta <= 60 {
        return ROUND(disEta, 2) + "с".
    }

    local minutes is 60.
    local hours is 60 * minutes.
    local days is KUniverse:HOURSPERDAY * hours.
    local daysStr is "".
    local hoursStr is "".
    local minutesStr is "".

    if ( disEta >= days ) {
        set daysStr to FLOOR(disEta / days) + "д".
        set disEta to MOD(disEta, days).
    }

    if ( disEta >= hours ) {
        set hoursStr to FLOOR(disEta / hours) + "ч".
        set disEta to MOD(disEta, hours).
    }
    if ( disEta >= minutes ) {
        set minutesStr to FLOOR(disEta / minutes) + "мин".
        set disEta to MOD(disEta, minutes).
    }

    return daysStr + hoursStr + minutesStr + FLOOR(disEta) + "с".
}

function disMet { parameter meters. parameter digs is 1.
    if ( meters >= 1000000 ) {
        return round(meters / 1000000, digs) + "Mm".
    }
    if ( meters >= 1000 ) {
        return round(meters / 1000, digs) + "km".
    }
    return round(meters, digs) + "m".
}
