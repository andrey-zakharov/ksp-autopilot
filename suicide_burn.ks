// ==========================================================================
// suicide_burn.ks — финальное торможение перед посадкой
// ==========================================================================
PARAMETER clearance IS 30.
PARAMETER exitSpeed IS 10.
PARAMETER maxTiltDeg IS 8.
PARAMETER touchdownSpeed IS 0.
PARAMETER maxIgniteTiltDeg IS 15.
PARAMETER slewDeg IS 10.
// maxTiltDeg - 0 on clearance height, 90 on above safe orbit
RUN utils.
global function calc_burn_dist { 
    parameter sbTiltDeg is 0.
    parameter touchdownSpeed is 0.
    if body <> body {
        return -1.
    }

    LOCAL sbMass IS MAX(0.001, SHIP:MASS).

    // Для управления используем МАКСИМАЛЬНУЮ тягу, а не текущую доступную.
    // AVAILABLETHRUST может зависеть от текущего дросселя и давать заниженную оценку.
    LOCAL sbMaxThrust IS SHIP:MAXTHRUST.
    IF sbMaxThrust <= 0.01 {
        SET sbMaxThrust TO SHIP:AVAILABLETHRUST.
    }

    local sbMaxAcc IS MAX(0.01, sbMaxThrust / sbMass).
    LOCAL sbG IS safeDiv(BODY:MU, (BODY:RADIUS + SHIP:ALTITUDE)^2, 0.01).

    local sbVVertAbs IS ABS(VDOT(SHIP:VELOCITY:SURFACE, SHIP:UP:FOREVECTOR)).

    // Максимальное чистое вертикальное торможение вниз, которое мы можем дать.
    local sbNetMax IS MAX(0.01, sbMaxAcc * cos(sbTiltDeg) - sbG).

    // Остановочный путь при максимальной тяге.
    local sbStopDist is (sbVVertAbs * sbVVertAbs - touchdownSpeed * touchdownSpeed) /
                        (2 * sbNetMax).

    return sbStopDist.
}

//print "Время прожига для полной остановки: " + round(delay_time, 2) + " сек.".


GLOBAL sbThrottle IS 0.
GLOBAL sbSteer IS SHIP:FACING.
GLOBAL sbDiagTimer IS 0.
GLOBAL sbPrevVVert IS 0.
GLOBAL sbAccMeas IS 0.
GLOBAL sbLastSec IS TIME:SECONDS.
GLOBAL sbAlt IS MAX(0, SHIP:ALTITUDE - SHIP:GEOPOSITION:TERRAINHEIGHT).
GLOBAL sbVVert IS VDOT(SHIP:VELOCITY:SURFACE, SHIP:UP:FOREVECTOR).

SET sbPrevVVert TO sbVVert.

LOCK THROTTLE TO sbThrottle.
LOCK STEERING TO sbSteer.

UNTIL SHIP:STATUS = "LANDED" OR
      SHIP:STATUS = "SPLASHED" OR
      sbAlt <= clearance OR sbVVert > 0 OR
      sbAlt <= 1.0 {

    SET sbNow TO TIME:SECONDS.
    SET sbDt TO clamp(sbNow - sbLastSec, 0.01, 0.25).
    SET sbLastSec TO sbNow.

    SET sbAlt TO MAX(0, SHIP:ALTITUDE - SHIP:GEOPOSITION:TERRAINHEIGHT).

    LOCAL sbUp IS SHIP:UP:FOREVECTOR.
    LOCAL sbVel IS SHIP:VELOCITY:SURFACE.

    SET sbVVert TO VDOT(sbVel, sbUp).

    LOCAL sbVHoriz IS sbVel - (sbVVert * sbUp).
    LOCAL sbVHorizMag IS sbVHoriz:MAG.
    LOCAL sbVVertAbs IS ABS(sbVVert).

    LOCAL sbMass IS MAX(0.001, SHIP:MASS).

    // Для управления используем МАКСИМАЛЬНУЮ тягу, а не текущую доступную.
    // AVAILABLETHRUST может зависеть от текущего дросселя и давать заниженную оценку.
    LOCAL sbMaxThrust IS SHIP:MAXTHRUST.
    IF sbMaxThrust <= 0.01 {
        SET sbMaxThrust TO SHIP:AVAILABLETHRUST.
    }

    LOCAL sbMaxAcc IS MAX(0.01, sbMaxThrust / sbMass).
    LOCAL sbG IS safeDiv(BODY:MU, (BODY:RADIUS + SHIP:ALTITUDE)^2, 0.01).

    // Наклон только для гашения остаточной горизонтали.
    // При низком TWR не наклоняемся сильно.
    LOCAL sbTiltDeg IS 0.
    LOCAL sbDesired IS sbUp.
    LOCAL sbAnti IS V(0,0,0).

    IF sbVHorizMag > 0.1 {
        SET sbTiltDeg TO MIN(maxTiltDeg, sbVHorizMag * 0.5).
        SET sbAnti TO (-1) * sbVHoriz:NORMALIZED.
        SET sbDesired TO (COS(sbTiltDeg) * sbUp + SIN(sbTiltDeg) * sbAnti):NORMALIZED.
    }

    LOCAL sbVertFrac IS MAX(0.1, COS(sbTiltDeg)).

    // Максимальное чистое вертикальное торможение вниз, которое мы можем дать.
    LOCAL sbNetMax IS MAX(0.01, sbMaxAcc * sbVertFrac - sbG).

    // Высота, которую реально можно использовать для торможения.
    LOCAL sbEffAlt IS MAX(0.1, sbAlt - clearance).

    // Остановочный путь при максимальной тяге.
    LOCAL sbStopDist IS (sbVVertAbs * sbVVertAbs - touchdownSpeed * touchdownSpeed) /
                        (2 * sbNetMax).
    IF sbStopDist < 0 {
        SET sbStopDist TO 0.
    }

    // Фактическое ускорение по изменению вертикальной скорости.
    // vVert отрицательный при падении. Если он становится менее отрицательным — тормозим вверх.
    SET sbAccMeas TO 0.
    IF sbDt > 0.01 {
        SET sbAccMeas TO (sbVVert - sbPrevVVert) / sbDt.
    }
    SET sbPrevVVert TO sbVVert.

    // По умолчанию — аварийное полное торможение.
    SET sbThrottle TO 1.0.

    // Если запас высоты есть, можно попробовать более плавный режим.
    // Но если фактическое торможение хуже нужного — сразу возвращаем 100%.
    IF sbEffAlt > sbStopDist * 1.15 + sbVVertAbs * 0.35 {

        LOCAL sbReqDecel IS (sbVVertAbs * sbVVertAbs - touchdownSpeed * touchdownSpeed) /
                            (2 * sbEffAlt).
        IF sbReqDecel < 0 {
            SET sbReqDecel TO 0.
        }

        // Небольшой запас, чтобы не прилететь точно в ноль высоты с остаточной скоростью.
        LOCAL sbTargetDecel IS sbReqDecel * 1.10.

        LOCAL sbDesiredAccel IS sbG + sbTargetDecel.

        SET sbThrottle TO clamp(
            sbDesiredAccel / MAX(0.01, sbMaxAcc * sbVertFrac),
            0,
            1
        ).

        // Если реальное ускорение заметно хуже расчетного — форсируем полную тягу.
        IF sbAccMeas < sbTargetDecel * 0.65 {
            SET sbThrottle TO 1.0.
        }
    }

    // У самой земли не даем ракете уйти вверх.
    IF sbVVert > 0 AND sbAlt < MAX(50, clearance * 1.5) {
        LOCAL hoverThr IS safeDiv(sbG, MAX(0.01, sbMaxAcc * sbVertFrac), 0.25).
        SET sbThrottle TO MIN(sbThrottle, hoverThr).
    }

    SET sbThrottle TO thrustGate(sbThrottle, sbDesired, maxIgniteTiltDeg).
    SET sbThrottle TO clamp(sbThrottle, 0, 1).

    SET sbSteer TO sbDesired.

    // Диагностика.
    IF sbNow >= sbDiagTimer {
        PRINT "SB alt=" + ROUND(sbAlt) +
              " eff=" + ROUND(sbEffAlt) +
              " stop=" + ROUND(sbStopDist) + " dist=" + ROUND(calc_burn_dist(sbTiltDeg)) +
              " vV=" + ROUND(sbVVert,1) +
              " aM=" + ROUND(sbAccMeas,2) +
              " netMax=" + ROUND(sbNetMax,2) +
              " vH=" + ROUND(sbVHorizMag,1) +
              " tilt=" + ROUND(VANG(SHIP:FACING:FOREVECTOR, sbUp),1) +
              " thr=" + ROUND(sbThrottle * 100) + "%".
        SET sbDiagTimer TO sbNow + 1.
    }

    WAIT 0.
}

SET sbThrottle TO 0.
WAIT 0.05.

UNLOCK THROTTLE.
UNLOCK STEERING.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.