
function showVec { parameter drawVec. parameter color is WHITE. parameter label is "". parameter origin is SHIP:POSITION.
    local v1 is VECDRAW(origin, drawVec, color, label).
    set v1:show to true.
    return v1.
}
