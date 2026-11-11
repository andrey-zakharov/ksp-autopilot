// ==========================================================================
// Расчёт dV для перехода с орбиты корабля на орбиту TARGET.
// Манёвр наклонения выполняется на большей по высоте орбите.
//
// Возвращает LIST(ok, totalDv, dv1, dv2, dvInc, desc):
//   ok      — true если расчёт успешен
//   totalDv — суммарный dV всех импульсов
//   dv1     — первый импульс (подъём/опускание апсиды)
//   dv2     — второй импульс (округление + наклонение)
//   dvInc   — компонента смены наклонения (входит в один из импульсов)
//   desc    — текстовое описание стратегии
// ==========================================================================
function calcTransferToTarget {
    local bad to LIST(false, 0, 0, 0, 0, "ошибка").

    // Проверка: цель существует
    if not HASTARGET { return bad. }

    // Проверка: оба на одном теле
    if SHIP:ORBIT:BODY:NAME <> TARGET:ORBIT:BODY:NAME {
        return LIST(false, 0, 0, 0, 0, "разные тела").
    }

    local muVal to BODY:MU.
    local bodyRad to BODY:RADIUS.

    // Текущая орбита корабля (радиусы от центра тела)
    local curPe to bodyRad + SHIP:ORBIT:PERIAPSIS.
    local curAp to bodyRad + SHIP:ORBIT:APOAPSIS.
    local curSma to (curPe + curAp) / 2.
    local curInc to SHIP:ORBIT:INCLINATION.

    // Целевая орбита (радиусы от центра тела)
    local tgtPe to bodyRad + TARGET:ORBIT:PERIAPSIS.
    local tgtAp to bodyRad + TARGET:ORBIT:APOAPSIS.
    local tgtSma to (tgtPe + tgtAp) / 2.
    local tgtInc to TARGET:ORBIT:INCLINATION.

    // Проверки валидности
    if curPe < bodyRad or curAp < bodyRad { return bad. }
    if tgtPe < bodyRad or tgtAp < bodyRad { return bad. }

    // Разница наклонений
    local dInc to ABS(curInc - tgtInc).

    local dv1 to 0.
    local dv2 to 0.
    local dvInc to 0.
    local desc to "".

    if tgtSma >= curSma {
        // ============================================================
        // Целевая орбита ВЫШЕ или равна текущей.
        // Импульс 1: на перицентре текущей — поднять апоцентр.
        // Импульс 2: на апоцентре переходной (целевая высота) —
        //            смена наклонения + округление.
        // ============================================================
        set desc to "подъём + наклонение на целевой высоте".

        // Скорость на перицентре текущей орбиты
        local vPeriCur to SQRT(muVal * (2 / curPe - 1 / curSma)).

        // Переходная орбита: periцентр = curPe, apoцентр = tgtAp
        local transSma to (curPe + tgtAp) / 2.
        local vPeriTrans to SQRT(muVal * (2 / curPe - 1 / transSma)).

        // Импульс 1: проградный на перицентре
        set dv1 to ABS(vPeriTrans - vPeriCur).

        // Скорость на апоцентре переходной орбиты
        local vApoTrans to SQRT(muVal * (2 / tgtAp - 1 / transSma)).

        // Скорость на апоцентре целевой орбиты
        local vApoTgt to SQRT(muVal * (2 / tgtAp - 1 / tgtSma)).

        // Импульс 2: комбинированный (наклонение + округление)
        if dInc > 0.01 {
            // Векторный манёвр: dV = |v2_vec - v1_vec|
            // где угол между векторами = dInc
            set dv2 to SQRT(vApoTrans * vApoTrans +
                            vApoTgt * vApoTgt -
                            2 * vApoTrans * vApoTgt * COS(dInc)).
            // Компонента наклонения (для информации)
            set dvInc to 2 * vApoTrans * SIN(dInc / 2).
        } else {
            set dv2 to ABS(vApoTgt - vApoTrans).
            set dvInc to 0.
        }

    } else {
        // ============================================================
        // Текущая орбита ВЫШЕ целевой.
        // Импульс 1: на апоцентре текущей — смена наклонения
        //            (скорость минимальна → наклонение дешевле всего).
        // Импульс 2: на перицентре текущей — опустить апоцентр.
        // Импульс 3: на апоцентре переходной — округление.
        // Объединяем 1+2 если возможно, иначе считаем отдельно.
        // ============================================================
        set desc to "наклонение на текущем апоцентре + спуск".

        // Скорость на апоцентре текущей орбиты
        local vApoCur to SQRT(muVal * (2 / curAp - 1 / curSma)).

        // Наклонение на апоцентре (самая дешёвая точка)
        if dInc > 0.01 {
            set dvInc to 2 * vApoCur * SIN(dInc / 2).
        } else {
            set dvInc to 0.
        }

        // Импульс 1: смена наклонения на апоценте
        set dv1 to dvInc.

        // Переходная орбита: periцентр = tgtPe, apoцентр = curAp
        local transSma to (tgtPe + curAp) / 2.

        // Скорость на перицентре текущей орбиты
        local vPeriCur to SQRT(muVal * (2 / curPe - 1 / curSma)).

        // Скорость на перицентре переходной орбиты
        local vPeriTrans to SQRT(muVal * (2 / curPe - 1 / transSma)).

        // Импульс 2: ретроградный на перицентре (опускаем апоцентр)
        local dvLower to ABS(vPeriTrans - vPeriCur).

        // Скорость на апоцентре переходной орбиты
        local vApoTrans to SQRT(muVal * (2 / tgtPe - 1 / transSma)).

        // Скорость на перицентре целевой орбиты
        local vPeriTgt to SQRT(muVal * (2 / tgtPe - 1 / tgtSma)).

        // Импульс 3: округление на перицентре целевой
        local dvRound to ABS(vPeriTgt - vApoTrans).

        set dv2 to dvLower + dvRound.
    }

    local totalDv to dv1 + dv2.

    print "Расчёт перехода: " + desc.
    print "  Текущая: Pe=" + ROUND(curPe - bodyRad) +
          " Ap=" + ROUND(curAp - bodyRad) +
          " inc=" + ROUND(curInc, 2).
    print "  Целевая: Pe=" + ROUND(tgtPe - bodyRad) +
          " Ap=" + ROUND(tgtAp - bodyRad) +
          " inc=" + ROUND(tgtInc, 2).
    print "  dInc=" + ROUND(dInc, 2) + " deg".
    print "  dV1=" + ROUND(dv1, 1) +
          " dV2=" + ROUND(dv2, 1) +
          " dV_накл=" + ROUND(dvInc, 1) +
          " ИТОГО=" + ROUND(totalDv, 1) + " м/с".

    return LIST(true, totalDv, dv1, dv2, dvInc, desc).
}

calcTransferToTarget.