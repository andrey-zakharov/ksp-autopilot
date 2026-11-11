
// --------------------------------------------------------------------------
// Подсчёт ресурсов на конкретной ступени
// stageIdx: индекс ступени (0 = текущая, 1 = следующая, ...)
//           -1 = текущая (STAGE)
// --------------------------------------------------------------------------
function stageResources { parameter stageIdx.
    local res to LEXICON().
    
    // Инициализация всех счётчиков
    set res["LIQUIDFUEL"] to 0.
    set res["OXIDIZER"] to 0.
    set res["SOLIDFUEL"] to 0.
    set res["MONOPROP"] to 0.
    set res["XENON"] to 0.
    set res["ELECTRICCHARGE"] to 0.
    set res["INTAKEAIR"] to 0.
    
    // Флаги наличия типов ресурсов
    set res["HAS_LF"] to false.
    set res["HAS_OX"] to false.
    set res["HAS_SOLID"] to false.
    set res["HAS_AIR"] to false.
    set res["HAS_PROP"] to false.

    // Выбор ступени
    local stg to STAGE.
    if stageIdx >= 0 and stageIdx < SHIP:STAGES:LENGTH {
        set stg to SHIP:STAGES[stageIdx].
    }

    for r in stg:RESOURCES {
        if r:CAPACITY > 0.1 {
            if r:NAME = "LiquidFuel" {
                set res["HAS_LF"] to true.
                set res["HAS_PROP"] to true.
                set res["LIQUIDFUEL"] to res["LIQUIDFUEL"] + r:AMOUNT.
            } else if r:NAME = "Oxidizer" {
                set res["HAS_OX"] to true.
                set res["HAS_PROP"] to true.
                set res["OXIDIZER"] to res["OXIDIZER"] + r:AMOUNT.
            } else if r:NAME = "SolidFuel" {
                set res["HAS_SOLID"] to true.
                set res["HAS_PROP"] to true.
                set res["SOLIDFUEL"] to res["SOLIDFUEL"] + r:AMOUNT.
            } else if r:NAME = "MonoPropellant" {
                set res["HAS_PROP"] to true.
                set res["MONOPROP"] to res["MONOPROP"] + r:AMOUNT.
            } else if r:NAME = "XenonGas" {
                set res["HAS_PROP"] to true.
                set res["XENON"] to res["XENON"] + r:AMOUNT.
            } else if r:NAME = "ElectricCharge" {
                set res["ELECTRICCHARGE"] to res["ELECTRICCHARGE"] + r:AMOUNT.
            } else if r:NAME = "IntakeAir" {
                set res["HAS_AIR"] to true.
                set res["INTAKEAIR"] to res["INTAKEAIR"] + r:AMOUNT.
            }
        }
    }
    return res.
}

// --------------------------------------------------------------------------
// Проверка наличия usable топлива на конкретной ступени
// stageIdx: индекс ступени (-1 = текущая)
// --------------------------------------------------------------------------
function stageHasUsablePropellant { parameter stageIdx.
    local res to stageResources(stageIdx).
    
    if not res["HAS_PROP"] { return true. }
    if res["HAS_SOLID"] and res["SOLIDFUEL"] > 0.1 { return true. }
    if res["HAS_LF"] and res["HAS_OX"] {
        if res["LIQUIDFUEL"] > 0.1 and res["OXIDIZER"] > 0.1 { return true. }
        if res["LIQUIDFUEL"] > 0.1 and res["HAS_AIR"] { return true. }
        return false.
    }
    if res["HAS_LF"] { return res["LIQUIDFUEL"] > 0.1. }
    if res["MONOPROP"] > 0.1 or res["XENON"] > 0.1 { return true. }
    return false.
}


// --------------------------------------------------------------------------
// Проверка: нужно ли отделять ступень (тяга или топливо текущей ступени)
// --------------------------------------------------------------------------
function shouldStage {
    if SHIP:MAXTHRUST <= 0.01 { return true. }
    if not stageHasUsablePropellant(-1) { return true. }
    return false.
}
