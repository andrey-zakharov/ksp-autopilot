function printTargetOrbit {
    if not HASTARGET {
        print "Нет цели (TARGET).".
        return.
    }
    local inc to TARGET:ORBIT:INCLINATION.
    local lan to TARGET:ORBIT:LAN.
    local ecc to TARGET:ORBIT:ECCENTRICITY.
    local sma to TARGET:ORBIT:SEMIMAJORAXIS.
    local per to TARGET:ORBIT:PERIOD.
    print "Цель: " + TARGET:NAME.
    print "  Тело:   " + TARGET:ORBIT:BODY:NAME.
    print "  inc:    " + ROUND(inc, 4) + " deg".
    print "  LAN:    " + ROUND(lan, 4) + " deg".
    print "  ecc:    " + ROUND(ecc, 6).
    print "  SMA:    " + ROUND(sma) + " м".
    print "  Период: " + ROUND(per, 1) + " с".
}
printTargetOrbit.