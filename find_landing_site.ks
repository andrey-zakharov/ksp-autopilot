// ==========================================================================
// find_landing_site.ks — выбор точки посадки
// ==========================================================================
// Вход:  initLat, initLng (если 0 — вычисляется автоматически)
// Выход:
//   Основные:  landingLat, landingLng, landingTerrain, landingSlope
//   Для совместимости: landLat, landLng, landTerrain, landSlope
//
// Логика:
// 1. Если HASTARGET — берём координаты цели
// 2. Если initLat/initLng заданы — используем их
// 3. Иначе — вычисляем точку впереди по орбите с учётом времени торможения
// 4. Ищем ровную площадку рядом с начальной точкой (с ограничением радиуса)
// ==========================================================================

PARAMETER initLat IS 0.
PARAMETER initLng IS 0.
PARAMETER maxSlope IS 5.
PARAMETER slopeStep IS 50.
PARAMETER searchItersPerFrame IS 8.
PARAMETER maxSearchIters IS 200.

// Параметры автоматического выбора точки под орбитой
// autoMarginSec   — запас времени в секундах
// autoLeadFactor  — 1.0 = (время торможения + запас), 0.5 = точка касания при немедленном старте
// autoMaxLeadSec  — жесткий лимит времени лидирования (0 = без лимита)
// maxSearchRadius — максимальное удаление площадки от расчетной точки (м), 0 = без лимита
PARAMETER autoMarginSec IS 30.
PARAMETER autoLeadFactor IS 1.0.
PARAMETER autoMaxLeadSec IS 0.
PARAMETER maxSearchRadius IS 10000.

RUN utils.

// ==========================================================================
//  УТИЛИТЫ ПОИСКА ПЛОЩАДКИ И ГЕОМЕТРИИ
// ==========================================================================

FUNCTION terrainH { PARAMETER lat. PARAMETER lng.
    RETURN BODY:GEOPOSITIONLATLNG(lat, lng):TERRAINHEIGHT.
}

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
    RETURN ARCTAN(SQRT(gradX*gradX + gradY*gradY)).
}

FUNCTION distanceLatLng { PARAMETER latA. PARAMETER lngA. PARAMETER latB. PARAMETER lngB.
    LOCAL latARad TO latA * CONSTANT:PI / 180.
    LOCAL latBRad TO latB * CONSTANT:PI / 180.
    LOCAL dLatRad TO (latB - latA) * CONSTANT:PI / 180.
    LOCAL dLngRad TO (lngB - lngA) * CONSTANT:PI / 180.
    LOCAL aVal TO SIN(dLatRad/2)*SIN(dLatRad/2) + COS(latARad)*COS(latBRad)*SIN(dLngRad/2)*SIN(dLngRad/2).
    IF aVal > 1 { SET aVal TO 1. }
    LOCAL cVal TO 2 * ARCSIN(SQRT(aVal)).
    RETURN BODY:RADIUS * cVal * CONSTANT:PI / 180.
}

// ==========================================================================
//  ИНКРЕМЕНТАЛЬНЫЙ ПОИСК (для обхода лимита CPU за один кадр)
// ==========================================================================

SET searchState TO List().

FUNCTION initSearch { PARAMETER lat. PARAMETER lng.
    SET searchState TO List(FALSE, lat, lng, slopeAt(lat, lng, slopeStep), 0, 500).
}

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
            LOCAL tLat TO cLat + o:X*dLat.
            LOCAL tLng TO cLng + o:Y*dLng.
            LOCAL tSlope TO slopeAt(tLat, tLng, slopeStep).
            IF tSlope < bestSlope {
                SET bestSlope TO tSlope.
                SET bestLat TO tLat.
                SET bestLng TO tLng.
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
    RETURN done.
}

FUNCTION findFlatSpot { PARAMETER lat. PARAMETER lng. PARAMETER maxSlopeDeg.
    initSearch(lat, lng).
    LOCAL s0 TO searchState[3].
    PRINT "Поиск площадки, стартовый уклон " + ROUND(s0,2) + " deg, допуск " + maxSlopeDeg + " deg".
    LOCAL searchStart TO TIME:SECONDS.

    UNTIL searchStep(maxSlopeDeg) {
        IF (TIME:SECONDS - searchStart) > 2 {
            PRINT "  ...итерация " + searchState[4] + ", шаг " + ROUND(searchState[5]) + " м, текущий уклон " + ROUND(searchState[3],2) + " deg".
            SET searchStart TO TIME:SECONDS.
        }
        WAIT 0.
    }

    IF searchState[3] <= maxSlopeDeg {
        PRINT "Площадка найдена за " + searchState[4] + " итераций.".
    } ELSE {
        PRINT "ВНИМАНИЕ: ровная площадка не найдена. Уклон " + ROUND(searchState[3],2) + " deg.".
    }
    RETURN List(searchState[1], searchState[2], searchState[3]).
}

// ==========================================================================
//  ВЫБОР НАЧАЛЬНОЙ ТОЧКИ
// ==========================================================================

PRINT "=== Выбор точки посадки ===".

// Удаляем старые ноды, чтобы POSITIONAT не учитывал их в прогнозе
IF ALLNODES:LENGTH > 0 {
    FOR oldNode IN ALLNODES { REMOVE oldNode. }
    WAIT 0.
}

SET startLat TO initLat.
SET startLng TO initLng.

IF HASTARGET {
    // Приоритет 1: активная цель
    SET startLat TO TARGET:GEOPOSITION:LAT.
    SET startLng TO TARGET:GEOPOSITION:LNG.
    PRINT "Цель: " + TARGET:NAME + " -> " + ROUND(startLat,4) + ", " + ROUND(startLng,4).

} ELSE IF (ABS(initLat) > 0.0) OR (ABS(initLng) > 0.0) {
    // Приоритет 2: явные координаты
    PRINT "Заданные координаты: " + ROUND(startLat,4) + ", " + ROUND(startLng,4).

} ELSE {
    // Приоритет 3: автоматический выбор точки под орбитой
    // Берем ближайшую точку впереди, чтобы было время затормозить + запас
    SET vOrb TO SHIP:VELOCITY:ORBIT:MAG.

    // Определяем доступное торможение
    SET brakeAcc TO SHIP:AVAILABLETHRUST / MAX(0.001, SHIP:MASS).
    IF brakeAcc < 0.1 { SET brakeAcc TO SHIP:MAXTHRUST / MAX(0.001, SHIP:MASS). }
    IF brakeAcc < 0.1 { SET brakeAcc TO 0.1. } // Защита от деления на 0

    SET brakeSec TO vOrb / brakeAcc.

    // Если мы уже на суборбитальной траектории (падаем), нет смысла ждать полное время торможения.
    // Берем только запас на реакцию/разворот.
    IF SHIP:ORBIT:PERIAPSIS < 0 {
        SET leadSec TO autoMarginSec.
        PRINT "Траектория суборбитальная: выбор точки рядом с текущей позицией.".
    } ELSE {
        // Для орбиты: время торможения + запас
        SET leadSec TO autoMarginSec + autoLeadFactor * brakeSec.
    }

    // Опциональный жесткий лимит
    IF autoMaxLeadSec > 0 AND leadSec > autoMaxLeadSec {
        SET leadSec TO autoMaxLeadSec.
    }

    // Если время больше периода, сворачиваем в фазу орбиты (чтобы не ждать витков)
    SET periodSec TO MAX(1, SHIP:ORBIT:PERIOD).
    IF leadSec >= periodSec {
        SET leadSec TO leadSec - FLOOR(leadSec / periodSec) * periodSec.
        IF leadSec < autoMarginSec { SET leadSec TO autoMarginSec. }
    }


    SET nowUT TO TIME:SECONDS.
    SET futurePos TO POSITIONAT(SHIP, nowUT + leadSec).
    SET futureGeo TO BODY:GEOPOSITIONOF(futurePos).

    SET startLat TO futureGeo:LAT.
    SET startLng TO futureGeo:LNG.

    // Диагностика
    SET nowGeo TO SHIP:GEOPOSITION.

    PRINT "Сейчас:            " + ROUND(nowGeo:LAT,4) + ", " + ROUND(nowGeo:LNG,4).
    PRINT "Авто-точка:        " + ROUND(startLat,4) + ", " + ROUND(startLng,4).
    PRINT "vOrb=" + ROUND(vOrb,1) + " м/с, aBrake=" + ROUND(brakeAcc,2) + " м/с2, tBrake=" + ROUND(brakeSec,1) + " с".
    PRINT "Лидирование: +" + ROUND(leadSec,1) + " с (фактор " + autoLeadFactor + ")".
}

// ==========================================================================
//  ПОИСК РОВНОЙ ПЛОЩАДКИ
// ==========================================================================

SET site TO findFlatSpot(startLat, startLng, maxSlope).

// Ограничение ухода градиентного поиска
IF maxSearchRadius > 0 {
    SET searchDrift TO distanceLatLng(startLat, startLng, site[0], site[1]).
    IF searchDrift > maxSearchRadius {
        PRINT "Площадка ушла на " + ROUND(searchDrift) + " м (лимит " + ROUND(maxSearchRadius) + " м).".
        PRINT "Возврат к расчетной точке.".
        SET site TO List(startLat, startLng, slopeAt(startLat, startLng, slopeStep)).
    }
}

// ==========================================================================
//  ВОЗВРАТ РЕЗУЛЬТАТА
//  Используем отдельные глобальные имена, чтобы их не перекрыли
//  параметры вызывающего скрипта.
// ==========================================================================

SET landingLat TO site[0].
SET landingLng TO site[1].
SET landingSlope TO site[2].

SET landingGeo TO BODY:GEOPOSITIONLATLNG(landingLat, landingLng).
SET landingTerrain TO landingGeo:TERRAINHEIGHT.

// Старые имена оставляем для совместимости, но основной источник —
// landingLat / landingLng / landingTerrain / landingSlope
SET landLat TO landingLat.
SET landLng TO landingLng.
SET landSlope TO landingSlope.
SET landTerrain TO landingTerrain.

PRINT ">>> Итоговая площадка: " + ROUND(landingLat,5) + ", " + ROUND(landingLng,5).
PRINT ">>> Высота рельефа: " + ROUND(landingTerrain) + " м, уклон: " + ROUND(landingSlope,2) + " deg".