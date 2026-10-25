// ==========================================================================
// find_landing_site.ks — выбор точки посадки
// ==========================================================================
// Вход:  initLat, initLng (если 0 — вычисляется автоматически)
// Выход: landLat, landLng, landTerrain, landSlope (глобальные переменные)
//
// Логика:
// 1. Если HASTARGET — берём координаты цели
// 2. Если initLat/initLng заданы — используем их
// 3. Иначе — вычисляем точку впереди по орбите с учётом времени торможения
// 4. Ищем ровную площадку рядом с начальной точкой
// ==========================================================================

PARAMETER initLat IS 0.
PARAMETER initLng IS 0.
PARAMETER maxSlope IS 5.
PARAMETER slopeStep IS 50.
PARAMETER searchItersPerFrame IS 8.
PARAMETER maxSearchIters IS 200.

RUN utils.

// ==========================================================================
//  УТИЛИТЫ ПОИСКА ПЛОЩАДКИ
// ==========================================================================

FUNCTION terrainH { PARAMETER lat. PARAMETER lng.
    RETURN BODY:GEOPOSITIONLATLNG(lat, lng):TERRAINHEIGHT. }

FUNCTION slopeAt { PARAMETER lat. PARAMETER lng. PARAMETER step.
    LOCAL dLat TO (step / BODY:RADIUS) * (180 / CONSTANT:PI).
    LOCAL cosLat TO COS(lat * CONSTANT:PI / 180).
    LOCAL dLng TO 0.
    IF cosLat > 0.001 {
        SET dLng TO (step / (BODY:RADIUS * cosLat)) * (180 / CONSTANT:PI).
    } ELSE {
        SET dLng TO 180.
    }
    LOCAL hE TO terrainH(lat, lng + dLng).
    LOCAL hW TO terrainH(lat, lng - dLng).
    LOCAL hN TO terrainH(lat + dLat, lng).
    LOCAL hS TO terrainH(lat - dLat, lng).
    LOCAL gradX TO (hE - hW) / (2 * step).
    LOCAL gradY TO (hN - hS) / (2 * step).
    RETURN ARCTAN(SQRT(gradX*gradX + gradY*gradY)). }

SET searchState TO List().

FUNCTION initSearch { PARAMETER lat. PARAMETER lng.
    SET searchState TO List(FALSE, lat, lng, slopeAt(lat, lng, slopeStep), 0, 500). }

FUNCTION searchStep { PARAMETER maxSlopeDeg.
    LOCAL done     TO searchState[0].
    LOCAL cLat     TO searchState[1].
    LOCAL cLng     TO searchState[2].
    LOCAL cSlope   TO searchState[3].
    LOCAL iter     TO searchState[4].
    LOCAL step     TO searchState[5].
    LOCAL k TO 0.
    UNTIL done OR (k >= searchItersPerFrame) OR (iter >= maxSearchIters) OR (step < 5) {
        LOCAL dLat TO (step / BODY:RADIUS) * (180 / CONSTANT:PI).
        LOCAL cosLat TO COS(cLat * CONSTANT:PI / 180).
        LOCAL dLng TO 0.
        IF cosLat > 0.001 {
            SET dLng TO (step / (BODY:RADIUS * cosLat)) * (180 / CONSTANT:PI).
        } ELSE {
            SET dLng TO 180.
        }
        LOCAL bestLat TO cLat. LOCAL bestLng TO cLng. LOCAL bestSlope TO cSlope.
        LOCAL offsets TO List(V(-1,-1,0),V(-1,0,0),V(-1,1,0),V(0,-1,0),V(0,1,0),V(1,-1,0),V(1,0,0),V(1,1,0)).
        FOR o IN offsets {
            LOCAL tLat TO cLat + o:X*dLat. LOCAL tLng TO cLng + o:Y*dLng.
            LOCAL tSlope TO slopeAt(tLat, tLng, slopeStep).
            IF tSlope < bestSlope {
                SET bestSlope TO tSlope. SET bestLat TO tLat. SET bestLng TO tLng.
            }
        }
        IF bestSlope < cSlope {
            SET cLat TO bestLat. SET cLng TO bestLng. SET cSlope TO bestSlope.
        } ELSE {
            SET step TO step * 0.5.
        }
        SET iter TO iter + 1.
        SET k TO k + 1.
        IF cSlope <= maxSlopeDeg { SET done TO TRUE. }
    }
    IF NOT done {
        IF (iter >= maxSearchIters) OR (step < 5) { SET done TO TRUE. }
    }
    SET searchState TO List(done, cLat, cLng, cSlope, iter, step).
    RETURN done. }

FUNCTION findFlatSpot { PARAMETER lat. PARAMETER lng. PARAMETER maxSlopeDeg.
    initSearch(lat, lng).
    LOCAL s0 TO searchState[3].
    PRINT "Поиск площадки, стартовый уклон " + ROUND(s0,2) + " deg, допуск " + maxSlopeDeg + " deg".
    LOCAL searchStart TO TIME:SECONDS.
    UNTIL searchStep(maxSlopeDeg) {
        IF (TIME:SECONDS - searchStart) > 2 {
            PRINT "  ...итерация " + searchState[4] + ", шаг " + ROUND(searchState[5]) +
                  " м, текущий уклон " + ROUND(searchState[3],2) + " deg".
            SET searchStart TO TIME:SECONDS.
        }
        WAIT 0.
    }
    IF searchState[3] <= maxSlopeDeg {
        PRINT "Площадка найдена за " + searchState[4] + " итераций.".
    } ELSE {
        PRINT "ВНИМАНИЕ: ровная площадка не найдена. Уклон " + ROUND(searchState[3],2) + " deg.".
    }
    RETURN List(searchState[1], searchState[2], searchState[3]). }

// ==========================================================================
//  ВЫБОР НАЧАЛЬНОЙ ТОЧКИ
// ==========================================================================

PRINT "=== Выбор точки посадки ===".

// Удаляем старые ноды: POSITIONAT/VELOCITYAT учитывают их в прогнозе,
// из-за остаточной ноды прогноз может "прыгнуть" на другую сторону орбиты
IF ALLNODES:LENGTH > 0 {
    FOR oldNode IN ALLNODES {
        REMOVE oldNode.
    }
    WAIT 0.
}

SET startLat TO initLat.
SET startLng TO initLng.

IF HASTARGET {
    // Приоритет 1: активная цель
    SET startLat TO TARGET:GEOPOSITION:LAT.
    SET startLng TO TARGET:GEOPOSITION:LNG.
    PRINT "Цель: " + TARGET:NAME + " -> " + ROUND(startLat,4) + ", " + ROUND(startLng,4).

} ELSE IF (ABS(initLat) > 0.0001) OR (ABS(initLng) > 0.0001) {
    // Приоритет 2: явные координаты
    PRINT "Заданные координаты: " + ROUND(startLat,4) + ", " + ROUND(startLng,4).

} ELSE {
    // Приоритет 3: ближайшая точка ВПЕРЕД по орбите
    // vOrb — орбитальная скорость, aBrake — торможение посадочной ступени
    SET vOrb TO SHIP:VELOCITY:ORBIT:MAG.
    SET aBrake TO MAX(0.1, SHIP:MAXTHRUST / SHIP:MASS).

    // Время полной остановки горизонтальной скорости
    SET tBrake TO vOrb / aBrake.

    // За время торможения пролетается половина "безынерционного" пути,
    // поэтому гореть надо, когда до площадки осталось tBrake/2 по орбитальному времени
    SET tToSite TO tBrake / 2.

    // Запас на разворот и задержку управления (несколько секунд)
    SET margin TO 10.

    SET lookahead TO tToSite + margin.

    // Не заглядываем дальше четверти периода — иначе можно оказаться "сзади"
    SET lookahead TO clamp(lookahead, 5, SHIP:ORBIT:PERIOD / 4).

    SET nowUT TO TIME:SECONDS.
    SET futurePos TO POSITIONAT(SHIP, nowUT + lookahead).
    SET futureGeo TO BODY:GEOPOSITIONOF(futurePos).
    SET startLat TO futureGeo:LAT.
    SET startLng TO futureGeo:LNG.

    // Диагностика: сейчас и куда смотрим
    SET nowGeo TO SHIP:GEOPOSITION.
    PRINT "Сейчас:          " + ROUND(nowGeo:LAT,4) + ", " + ROUND(nowGeo:LNG,4).
    PRINT "Точка посадки:   " + ROUND(startLat,4) + ", " + ROUND(startLng,4).
    PRINT "vOrb=" + ROUND(vOrb,1) + " м/с, aBrake=" + ROUND(aBrake,2) +
          " м/с2, tBrake=" + ROUND(tBrake,1) + " с".
    PRINT "Заглядываем на +" + ROUND(lookahead,1) + " с (tToSite=" +
          ROUND(tToSite,1) + " + запас " + margin + " с).".
}

// ==========================================================================
//  ПОИСК РОВНОЙ ПЛОЩАДКИ
// ==========================================================================

SET site TO findFlatSpot(startLat, startLng, maxSlope).

// Возвращаем результат через глобальные переменные
SET landLat TO site[0].
SET landLng TO site[1].
SET landSlope TO site[2].
SET landGeo TO BODY:GEOPOSITIONLATLNG(landLat, landLng).
SET landTerrain TO landGeo:TERRAINHEIGHT.

PRINT ">>> Итоговая площадка: " + ROUND(landLat,5) + ", " + ROUND(landLng,5).
PRINT ">>> Высота рельефа: " + ROUND(landTerrain) + " м, уклон: " + ROUND(landSlope,2) + " deg".