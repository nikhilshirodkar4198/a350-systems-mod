--[[
===============================================================================
  A350_BTV.lua  -  Brake To Vacate, OANS airport map and ROW/ROP
                   for the FlightFactor A350 in X-Plane 12 and X-Plane 11
  Version 1.0.0  (FlyWithLua NG+ / NG)

  Copyright (c) 2026 The A350 BTV author
  SPDX-License-Identifier: MIT  (see LICENSE.txt in the download)

  Not affiliated with, endorsed by or supported by Airbus or FlightFactor.
  For flight simulation only - never use for real-world operations.

  What it does
  - OANS airport map on the 3D-cockpit ND: range knob to ZOOM (left of 10 NM, like the A350), in ROSE NAV, ARC or
    PLAN, on the ground and in flight. On ZOOM the knob selects 5 / 2 / 1 / 0.5 / 0.2 NM (one range for all modes);
    past 5 NM it returns to 10.
    Runways, taxiways, exits, stands, ground markings and OpenStreetMap buildings; menus and tabs like the A350
    (MAP DATA, LANDING, ARPT SEL, STATUS).
  - BTV (brake to vacate): select an exit (or a stop distance) on the OANS map, arm BTV, and after touchdown the
    script brakes so that the exit is reached at 10 kt, braking as late as possible like the real system.
    Armed with the A/BRK pushbutton next to the gear lever (in flight; on the ground it stays RTO), which lights.
    "BTV" in the FMA (cyan = armed, green = braking). A go-around below 600 ft disarms BTV (above 600 ft again).
    Brake pedals or X-Plane's brake commands (e.g. key B) disconnect BTV - no rudder pedals needed.
  - ROW / ROP (runway overrun warning / protection) with the runway condition from the OANS LANDING page:
    DRY/WET stop lines on the ND, RWY TOO SHORT / MAX BRAKING / MAX REVERSE on the PFD, aural alerts.
  - AUTO BRK OFF with the MASTER CAUT light and chime of the aircraft when the crew disconnects BTV.
  - RWY AHEAD on the ground, traffic on the OANS map (X-Plane TCAS interface / LiveTraffic).

  Requirements
  - X-Plane 12 (Windows; Linux and macOS as beta) or X-Plane 11, FlightFactor A350, FlyWithLua NG+ 2.8 or newer
    (X-Plane 11: FlyWithLua NG 2.7).
  - The helper plugin Resources/plugins/zz_A350_BTV_bridge (included in the download). It lets the script draw
    the map after the aircraft has drawn its ND - without it the FF ND paints over the map.

  Settings: FlyWithLua menu -> Macros -> "A350 Systems" (or bind the command FlyWithLua/A350_BTV/toggle_window).
  Full manual: README.md in the download.

  Files (in Resources/plugins/FlyWithLua/Scripts/)
    A350_BTV.lua                    this script
    A350_BTV_data/                  font (B612 Mono, SIL OFL 1.1) and sounds
    A350_BTV_data/font_tls_px2.bin  ND font, made automatically from the aircraft's own display font on the first
                                    start (not shipped; B612 is used if the aircraft's font is not found)
  In the aircraft folder: A350_xp11_cockpit.obj gets the ZOOM position of the range knob (setting "cockpit_extras",
  on by default; backup A350_xp11_cockpit.obj.before_A350_BTV, restored when the setting is switched off).
    A350_BTV_config.txt             settings (created automatically)
    A350_BTV_index.txt              airport index (created automatically from the apt.dat files)
    A350_BTV_data/osm2_<ICAO>.json  cached OpenStreetMap buildings (created automatically)
  Own sounds: A350_BTV_data/sounds/custom/<name>.wav (see A350_BTV_data/sounds/README.txt).

  (Code comments are in German.)
===============================================================================
--]]

if PLANE_ICAO ~= "A359" and PLANE_ICAO ~= "A35K" then return end

-- Prueft, ob ein Dataref existiert. Funktioniert XPLMFindDataRef in dieser
-- FlyWithLua-Version nicht (findet nicht einmal Standard-Datarefs), wird "ja" angenommen.
local FIND_OK = nil
local function dr_exists(name)
    if XPLMFindDataRef == nil then return true end
    if FIND_OK == nil then
        local ok, r = pcall(XPLMFindDataRef, "sim/flightmodel/position/latitude")
        FIND_OK = ok and r ~= nil
        if not FIND_OK then logMsg("A350_BTV: XPLMFindDataRef returns nothing - dataref check disabled") end
    end
    if not FIND_OK then return true end
    local ok, ref = pcall(XPLMFindDataRef, name)
    return ok and ref ~= nil
end

pcall(require, "graphics")

-------------------------------------------------------------------------------
-- Einstellungen / Konstanten
-------------------------------------------------------------------------------
local VERSION = "1.0.0"
local KT = 0.514444            -- m/s pro Knoten
local R_EARTH = 6371000.0
local D2R = math.pi / 180

local P = {
    -- Verzoegerungen [m/s^2]: plan = ab hier wird gebremst, max = Obergrenze
    -- hard = Reserve nach spaetem Aufsetzen / Huepfer: erst darueber wird auf den naechsten Exit umgeplant
    dry  = { plan = 1.5, max = 3.0, hard = 3.8 },
    wet  = { plan = 1.3, max = 2.0, hard = 2.4 },
    cont = { plan = 1.0, max = 1.5, hard = 1.7 },
    td_dist      = 450,   -- angenommener Aufsetzpunkt hinter der Schwelle [m]
    flare_loss   = 3,     -- kt Geschwindigkeitsabbau im Flare (VAPP -> Aufsetzen)
    brake_delay  = 2.0,   -- s nach Aufsetzen bevor BTV bremsen darf
    lead_90      = 50,    -- m vor dem Exit bereits auf Exit-Speed (A350: 10 kt etwa 50 m vor dem gewaehlten Exit)
    lead_ret     = 50,
    kp           = 0.10,  -- Pedal pro m/s^2 Regelabweichung
    ki           = 0.20,  -- Pedal pro (m/s^2 * s)
    pedal_per_a  = 0.08,  -- Vorsteuerung: Pedal pro m/s^2 Zusatzverzoegerung
    rate_up      = 0.4,   -- Pedal/s
    rate_down    = 1.5,
    pilot_margin = 0.25,  -- Pedal-Uebersteuerung durch Pilot
    min_rwy_len  = 1800,  -- fuer automatische Flughafenwahl
    auto_radius  = 40000, -- m
}

local CFG = {
    overlay  = false,  -- FMA-artige Anzeige oben am Bildschirm (v2.15: ab Werk aus - Status im FMA/PFD und ND)
    v_ret    = 10,     -- kt am Schnellabrollweg (A350: 10 kt wie am 90-Grad-Exit)
    v_90     = 10,     -- kt am 90-Grad-Exit
    osm      = true,   -- Gebaeude aus OpenStreetMap laden (Internet, einmal je Flughafen)
    guess_terms = false, -- Terminals aus Gate-Positionen schaetzen, wenn keine OSM-Gebaeude da sind
    nd_on    = true,   -- Karte auf dem ND (3D-Cockpit) mit dem Range-Knopf auf ZOOM
    nd_frame = false,  -- Rahmen um die ND-Flaeche zeigen (Kontrolle)
    nd_layers = true,  -- ND-Karte in Schichten zeichnen (wenig Rechenzeit); aus = wie v2.8 jedes Mal neu
    nd_traffic = 1,    -- Verkehr im OANS: 0 aus, 1 wie TCAS (TRAF an, TCAS TA oder TA/RA), 2 immer
    aural    = true,   -- Ansagen RUNWAY TOO SHORT / MAX REVERSE / KEEP MAX REVERSE (Ton ueber FMOD)
    voice    = 1,      -- Stimme der Ansagen (A350_BTV_data/sounds/voiceN)
    mcaut    = true,   -- MASTER CAUT + Single Chime, wenn die Crew BTV abschaltet (AUTO BRK OFF)
    avol     = 1.0,    -- Lautstaerke der Ansagen (FlyWithLua-Kanal im Interior-Bus, 0..2)
    rops     = true,   -- ROW / ROP
    cfg_ver  = 4,
    cockpit_extras = true, -- Range-Knopf ZOOM im Cockpit-Modell des Flugzeugs (ZOOM-Lampe, 10 aus) - siehe PNL.extras
}

-------------------------------------------------------------------------------
-- Pfade
-------------------------------------------------------------------------------
local SCRIPTS = SCRIPT_DIRECTORY
local XP_ROOT = SYSTEM_DIRECTORY
if XP_ROOT == nil or XP_ROOT == "" then
    XP_ROOT = SCRIPTS:gsub("[Rr]esources[/\\][Pp]lugins[/\\][Ff]ly[Ww]ith[Ll]ua[/\\][Ss]cripts[/\\]?$", "")
end
local INDEX_FILE = SCRIPTS .. "A350_BTV_index.txt"
local CONFIG_FILE = SCRIPTS .. "A350_BTV_config.txt"

local function log(s) logMsg("A350_BTV: " .. s) end

local function load_config()
    local f = io.open(CONFIG_FILE, "r")
    if not f then return end
    local seen = {}
    for line in f:lines() do
        local k, v = line:match("^%s*([%w_]+)%s*=%s*(%S+)")
        if k and CFG[k] ~= nil then
            seen[k] = true
            if type(CFG[k]) == "boolean" then CFG[k] = (v == "true")
            else CFG[k] = tonumber(v) or CFG[k] end
        end
    end
    f:close()
    -- Einstellungen aus v1.x: dort war die ND-Karte ein Fenster ueber dem Cockpit (oft abgeschaltet) -
    -- die neue Karte im Display selbst ist wieder eingeschaltet
    if not seen.cfg_ver then CFG.nd_on, CFG.cfg_ver = true, 2 end
    -- v2.11: Exit-Geschwindigkeit wie im A350 (10 kt auch am Schnellabrollweg)
    if CFG.cfg_ver < 3 then CFG.v_ret, CFG.cfg_ver = 10, 3 end
    -- v2.15: Statuszeile oben am Bildschirm ab Werk aus (einmalig; wieder einschaltbar in den Einstellungen)
    if CFG.cfg_ver < 4 then CFG.overlay, CFG.cfg_ver = false, 4 end
    -- (seit 2.19.1 ohne Schalter im Fenster: immer wie im A350)
    CFG.overlay, CFG.nd_frame, CFG.guess_terms = false, false, false
    CFG.v_ret, CFG.v_90, CFG.nd_traffic = 10, 10, 1
    CFG.rops, CFG.aural, CFG.mcaut = true, true, true
end

local function save_config()
    local f = io.open(CONFIG_FILE, "w")
    if not f then return end
    for _, k in ipairs({ "overlay", "v_ret", "v_90", "osm", "guess_terms", "nd_on", "nd_frame", "nd_layers", "nd_traffic", "aural", "voice", "mcaut", "avol", "rops", "cockpit_extras", "cfg_ver" }) do
        f:write(k, " = ", tostring(CFG[k]), "\n")
    end
    f:close()
end

load_config()

-------------------------------------------------------------------------------
-- Datarefs
-------------------------------------------------------------------------------
local DR = {
    lat      = dataref_table("sim/flightmodel/position/latitude"),
    lon      = dataref_table("sim/flightmodel/position/longitude"),
    gs       = dataref_table("sim/flightmodel/position/groundspeed"),
    hpath    = dataref_table("sim/flightmodel/position/hpath"),
    onground = dataref_table("sim/flightmodel2/gear/on_ground"),
    ra       = dataref_table("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot"),
    lbr      = dataref_table("sim/cockpit2/controls/left_brake_ratio"),
    rbr      = dataref_table("sim/cockpit2/controls/right_brake_ratio"),
    thr      = dataref_table("sim/cockpit2/engine/actuators/throttle_ratio_all"),
    rev      = dataref_table("sim/flightmodel2/engines/thrust_reverser_deploy_ratio"),
    time     = dataref_table("sim/time/total_running_time_sec"),
    paused   = dataref_table("sim/time/paused"),
    psi      = dataref_table("sim/flightmodel/position/true_psi"),
    tas      = dataref_table("sim/flightmodel/position/true_airspeed"),
    mag      = dataref_table("sim/flightmodel/position/mag_psi"),
}
if dr_exists("sim/flightmodel/weight/m_total") then DR.mass = dataref_table("sim/flightmodel/weight/m_total") end
-- Ground Spoiler (Ausschlag je Fluegel [Grad]) und laufende Triebwerke (fuer BTV-Aktivierung / Flugende)
if dr_exists("sim/flightmodel2/wing/speedbrake1_deg") then DR.spl = dataref_table("sim/flightmodel2/wing/speedbrake1_deg") end
if dr_exists("sim/flightmodel/engine/ENGN_running") then DR.engrun = dataref_table("sim/flightmodel/engine/ENGN_running") end
if dr_exists("sim/flightmodel/position/vh_ind") then DR.vs = dataref_table("sim/flightmodel/position/vh_ind") end
if dr_exists("sim/flightmodel/position/elevation") then DR.elev = dataref_table("sim/flightmodel/position/elevation") end
if dr_exists("sim/cockpit2/temperature/outside_air_temp_degc") then
    DR.oat = dataref_table("sim/cockpit2/temperature/outside_air_temp_degc")
end
-- Bremsen: zwei Wege
--   "toliss": AirbusFBW/BrakePedalInputOverride + ...Left/Right (wenn vorhanden)
--   "xp":     sim/operation/override/override_toe_brakes + sim/cockpit2/controls/left/right_brake_ratio
--             (simuliert getretene Pedale; das ToLiss-System liest diese als Pedaleingabe)
local AC_READY = false
local BRAKE_MODE = nil
local ac_tries, ac_next_try = 0, 0
local function bind_aircraft_datarefs(now)
    if AC_READY or now < ac_next_try then return AC_READY end
    ac_next_try = now + 1
    ac_tries = ac_tries + 1
    local has_toliss = dr_exists("AirbusFBW/BrakePedalInputOverride") and dr_exists("AirbusFBW/BrakePedalInputLeft")
    if not has_toliss and ac_tries < 5 then return false end -- ein paar Sekunden auf das Flugzeug-Plugin warten
    -- Diagnose ins Log
    local diag = {}
    for _, n in ipairs({ "AirbusFBW/BrakePedalInputOverride", "AirbusFBW/BrakePedalInputLeft", "AirbusFBW/AutoBrkSel",
                         "AirbusFBW/TotLeftBrake", "AirbusFBW/BrakePedalAnim", "AirbusFBW/AbrkLo", "AirbusFBW/ParkBrake",
                         "sim/operation/override/override_toe_brakes" }) do
        diag[#diag + 1] = n .. "=" .. (dr_exists(n) and "yes" or "NO")
    end
    logMsg("A350_BTV: Datarefs: " .. table.concat(diag, ", "))
    DR.xovr = dataref_table("sim/operation/override/override_toe_brakes")
    if has_toliss then
        DR.ovr = dataref_table("AirbusFBW/BrakePedalInputOverride")
        DR.pl  = dataref_table("AirbusFBW/BrakePedalInputLeft")
        DR.pr  = dataref_table("AirbusFBW/BrakePedalInputRight")
        BRAKE_MODE = "toliss"
    else
        BRAKE_MODE = "xp"
    end
    if dr_exists("AirbusFBW/TotLeftBrake") then DR.totl = dataref_table("AirbusFBW/TotLeftBrake") end
    if dr_exists("sim/joystick/joy_mapped_axis_value") then
        DR.joy = dataref_table("sim/joystick/joy_mapped_axis_value")
        DR.joy_avail = dataref_table("sim/joystick/joy_mapped_axis_avail")
    end
    AC_READY = true
    logMsg("A350_BTV: brake system connected, mode " .. BRAKE_MODE)
    return true
end
if dr_exists("sim/weather/region/runway_friction") then
    DR.fric = dataref_table("sim/weather/region/runway_friction")
elseif dr_exists("sim/weather/runway_friction") then
    -- XP11: 0 = good, 1 = fair, 2 = poor (anders als XP12: 0 trocken, 1-3 nass, ab 4 kontaminiert)
    DR.fric11 = dataref_table("sim/weather/runway_friction")
    if dr_exists("sim/weather/rain_percent") then DR.rain11 = dataref_table("sim/weather/rain_percent") end
end
if dr_exists("sim/cockpit2/gauges/indicators/wind_speed_kts") then
    DR.wspd = dataref_table("sim/cockpit2/gauges/indicators/wind_speed_kts")
    DR.wdir = dataref_table("sim/cockpit2/gauges/indicators/wind_heading_deg_mag")
    DR.tpsi = dataref_table("sim/flightmodel/position/true_psi")
    DR.mpsi = dataref_table("sim/flightmodel/position/mag_psi")
end

-------------------------------------------------------------------------------
-- Geometrie
-------------------------------------------------------------------------------
local function to_xy(lat, lon, lat0, lon0)
    local x = (lon - lon0) * D2R * R_EARTH * math.cos(lat0 * D2R)
    local y = (lat - lat0) * D2R * R_EARTH
    return x, y
end

local function dist_m(lat1, lon1, lat2, lon2)
    local x, y = to_xy(lat2, lon2, lat1, lon1)
    return math.sqrt(x * x + y * y)
end

-- Position (s entlang, d quer - links positiv) relativ zu einem Pistenende
local function rwy_sd(E, lat, lon)
    local x, y = to_xy(lat, lon, E.lat, E.lon)
    return x * E.ux + y * E.uy, E.ux * y - E.uy * x
end

local function hdg_diff(a, b)
    local d = (a - b) % 360
    if d > 180 then d = d - 360 end
    return math.abs(d)
end

local function clamp(x, a, b) if x < a then return a elseif x > b then return b end return x end

-------------------------------------------------------------------------------
-- Status
-------------------------------------------------------------------------------
local S = {
    mode = "OFF",          -- OFF, ARMED, ENGAGED, RELEASE, DONE
    arm_request = false,
    apt = nil,             -- geladener Flughafen
    apt_manual = false,
    rwy = nil,             -- gewaehltes Landepistenende
    rwy_manual = false,
    exits = {},
    exit = nil,            -- gewaehlter Exit
    manual_dist = nil,     -- manuelle Distanz ab Schwelle [m]
    target = nil,          -- aktives Ziel waehrend des Ausrollens
    pedal = 0, integ = 0, braking = false, a_cmd = 0, a_meas = 0, a_req = 0,
    v_prev = nil, t_prev = nil, t_td = 0, air_time = 0, gnd_time = 0, was_air = false,
    ovr_written = false,
    msg = nil, msg_t = 0,
    rec_idx = nil, v_est = 72,
    icao_input = "", dist_input = "", vapp_input = "",
    vapp = nil,            -- eingegebene VAPP [kt]
    hw = nil,              -- Gegenwindkomponente [kt] (nur unter 5000 ft AGL)
    ann1 = nil, ann2 = nil, ann_col = { 0, 1, 1 },
    status = "Initializing ...",
}

local function message(s)
    S.msg, S.msg_t = s, DR.time[0]
    log(s)
end

-- FMS der A350 (ToLiss-Systemplugin): Zielflughafen und Landebahn aus dem Flugplan.
-- Wird weiter unten (bei der FFI-Anbindung) jede Sekunde gelesen; fehlen die Datarefs, bleibt alles nil.
local FMS = { dest = nil, dest_e = nil, rwy = nil, from = nil, applied = nil, poll = nil, st = "not available" }

-- Pistenkennung vereinheitlichen: "7L" -> "07L", " 25C" -> "25C"
function FMS.norm(id)
    id = tostring(id or ""):upper():gsub("%s", "")
    local num, side = id:match("^(%d+)([LCR]?)")
    if num then return string.format("%02d%s", tonumber(num), side) end
    return id
end

-------------------------------------------------------------------------------
-- Hintergrund-Jobs (Coroutinen, ueber mehrere Frames verteilt)
-------------------------------------------------------------------------------
local jobs = {}
local function add_job(name, fn)
    jobs[#jobs + 1] = { name = name, co = coroutine.create(fn) }
end

local function run_jobs(budget)
    local t0 = os.clock()
    while jobs[1] and os.clock() - t0 < budget do
        local j = jobs[1]
        local ok, err = coroutine.resume(j.co)
        if not ok then
            log("Error in job " .. j.name .. ": " .. tostring(err))
            table.remove(jobs, 1)
        elseif coroutine.status(j.co) == "dead" then
            table.remove(jobs, 1)
        end
    end
end

-------------------------------------------------------------------------------
-- Flughafen-Index
-------------------------------------------------------------------------------
local IDX = { ready = false, progress = "", list = {}, by_icao = {} }

local function file_size(path)
    local f = io.open(path, "rb")
    if not f then return nil end
    local sz = f:seek("end")
    f:close()
    return sz
end

local function apt_file_list()
    local files = {}
    local global = XP_ROOT .. "Global Scenery/Global Airports/Earth nav data/apt.dat"
    local global_added = false
    local f = io.open(XP_ROOT .. "Custom Scenery/scenery_packs.ini", "r")
    if f then
        for line in f:lines() do
            line = line:gsub("\r", "")
            local path = line:match("^SCENERY_PACK%s+(.-)%s*$")
            if path then
                if path == "*GLOBAL_AIRPORTS*" then
                    files[#files + 1] = global; global_added = true
                else
                    path = path:gsub("\\", "/")
                    if not path:match("/$") then path = path .. "/" end
                    if not (path:match("^%a:/") or path:match("^/")) then path = XP_ROOT .. path end
                    files[#files + 1] = path .. "Earth nav data/apt.dat"
                end
            end
        end
        f:close()
    end
    if not global_added then
        local g = io.open(global, "r")
        if g then g:close(); files[#files + 1] = global end
    end
    -- XP11: Grund-apt.dat als letzte Quelle (XP12 hat sie nicht mehr)
    local def = XP_ROOT .. "Resources/default scenery/default apt dat/Earth nav data/apt.dat"
    local d = io.open(def, "r")
    if d then d:close(); files[#files + 1] = def end
    return files
end

-- Durchsucht eine apt.dat blockweise nach Flughaefen ("1 ...") und deren Pisten ("100 ...")
local BLOCK = 1024 * 1024
local HDR_PAT = "^1[ \t]+%S+[ \t]+%S+[ \t]+%S+[ \t]+(%S+)"

local function scan_apt_file(path, out)
    local f = io.open(path, "rb")
    if not f then return end
    local carry, fp, cur = "\n", 0, nil
    while true do
        local block = f:read(BLOCK)
        if not block then break end
        local buf = carry .. block
        local base = fp - #carry
        fp = fp + #block
        local lastnl
        for k = #buf, 1, -1 do
            if buf:byte(k) == 10 then lastnl = k; break end
        end
        if not lastnl then
            carry = buf
        else
            local hpos, wpos = {}, {}
            local p = 1
            while true do
                local q = buf:find("\n1 ", p, true)
                if not q or q >= lastnl then break end
                hpos[#hpos + 1] = q; p = q + 1
            end
            p = 1
            while true do
                local q = buf:find("\n100 ", p, true)
                if not q or q >= lastnl then break end
                wpos[#wpos + 1] = q; p = q + 1
            end
            local i, j = 1, 1
            while hpos[i] or wpos[j] do
                local hp, wp = hpos[i], wpos[j]
                if hp and (not wp or hp < wp) then
                    local icao = buf:match(HDR_PAT, hp + 1)
                    cur = icao and { icao = icao:upper(), off = base + hp, len = 0 } or nil
                    i = i + 1
                else
                    if cur then
                        local line = buf:match("^[^\r\n]*", wp + 1)
                        local t, n = {}, 0
                        for tok in line:gmatch("%S+") do n = n + 1; t[n] = tok end
                        local la1, lo1, la2, lo2 = tonumber(t[10]), tonumber(t[11]), tonumber(t[19]), tonumber(t[20])
                        if la1 and lo1 and la2 and lo2 then
                            if not cur.lat then
                                cur.lat, cur.lon = la1, lo1
                                out[#out + 1] = cur
                            end
                            local l = dist_m(la1, lo1, la2, lo2)
                            if l > cur.len then cur.len = l end
                        end
                    end
                    j = j + 1
                end
            end
            carry = buf:sub(lastnl)
        end
        coroutine.yield()
    end
    f:close()
end

local function read_index_cache()
    local cache = {}
    local f = io.open(INDEX_FILE, "r")
    if not f then return cache end
    local cur
    for line in f:lines() do
        local tag = line:sub(1, 1)
        if tag == "F" then
            local path, size = line:match("^F\t(.-)\t(%d+)$")
            if path then
                cur = { size = tonumber(size), entries = {} }
                cache[path] = cur
            end
        elseif tag == "A" and cur then
            local icao, off, lat, lon, len = line:match("^A\t(%S+)\t(%d+)\t(%S+)\t(%S+)\t(%S+)")
            if icao then
                cur.entries[#cur.entries + 1] = { icao = icao, off = tonumber(off),
                    lat = tonumber(lat), lon = tonumber(lon), len = tonumber(len) }
            end
        end
    end
    f:close()
    return cache
end

local function write_index_cache(files_data)
    local f = io.open(INDEX_FILE, "w")
    if not f then log("Could not save the airport index"); return end
    f:write("# A350_BTV airport index v1 - generated automatically\n")
    for _, fd in ipairs(files_data) do
        f:write("F\t", fd.path, "\t", fd.size, "\n")
        for _, e in ipairs(fd.entries) do
            f:write(string.format("A\t%s\t%d\t%.6f\t%.6f\t%d\n", e.icao, e.off, e.lat, e.lon, e.len))
        end
    end
    f:close()
end

local function build_index_job()
    local files = apt_file_list()
    local cache = read_index_cache()
    local files_data, changed, scanned = {}, false, 0
    for i, path in ipairs(files) do
        local size = file_size(path)
        if size then
            local c = cache[path]
            local fd = { path = path, size = size, prio = i }
            if c and c.size == size then
                fd.entries = c.entries
            else
                changed = true
                fd.entries = {}
                IDX.progress = "scanning " .. (path:match("([^/\\]+)/Earth nav data") or path)
                scan_apt_file(path, fd.entries)
                scanned = scanned + 1
            end
            files_data[#files_data + 1] = fd
        end
        if i % 20 == 0 then coroutine.yield() end
    end
    -- Index aufbauen: je ICAO alle Quellen in Prioritaetsreihenfolge
    local by_icao, list = {}, {}
    for _, fd in ipairs(files_data) do
        for _, e in ipairs(fd.entries) do
            e.file, e.prio = fd.path, fd.prio
            local l = by_icao[e.icao]
            if not l then
                l = {}; by_icao[e.icao] = l
                list[#list + 1] = e
            end
            l[#l + 1] = e
        end
    end
    IDX.by_icao, IDX.list = by_icao, list
    if changed then write_index_cache(files_data) end
    IDX.ready = true
    if S.status == "Initializing ..." then S.status = "" end
    IDX.progress = ""
    log(string.format("Index ready: %d airports from %d files (%d newly scanned)", #list, #files_data, scanned))
end

local function nearest_airport(lat, lon, min_len, radius)
    local best, bestd
    local coslat = math.cos(lat * D2R)
    for _, e in ipairs(IDX.list) do
        local la, lo = e.lat, e.lon
        if math.abs(la - lat) < 1.0 then
            local dx = (lo - lon) * coslat
            local dy = la - lat
            local d = dx * dx + dy * dy
            if (not bestd or d < bestd) then
                -- Pistenlaenge: laengste aller Quellen
                local maxlen = 0
                for _, s in ipairs(IDX.by_icao[e.icao]) do if s.len > maxlen then maxlen = s.len end end
                if maxlen >= (min_len or 0) then best, bestd = e, d end
            end
        end
    end
    if best then
        local dm = math.sqrt(bestd) * D2R * R_EARTH
        if radius and dm > radius then return nil end
        return best, dm
    end
end

-------------------------------------------------------------------------------
-- Flughafen laden (Pisten + Taxi-Netzwerk)
-------------------------------------------------------------------------------
local function parse_airport_block(entry)
    local f = io.open(entry.file, "rb")
    if not f then return nil end
    f:seek("set", entry.off)
    local apt = { icao = entry.icao, name = "", rwys = {}, nodes = {}, edges = {}, n_nodes = 0, src = entry.file,
                  polys = {}, lines = {}, stands = {}, roads = {} }
    local first, n = true, 0
    local feat, ring, poly, lin = nil, nil, nil, nil -- 110 = Belag (Polygon aus Ringen), 120 = Linie
    for line in f:lines() do
        line = line:gsub("\r$", "")
        local code = line:match("^(%d+)")
        if first then
            apt.name = line:match("^1%s+%S+%s+%S+%s+%S+%s+%S+%s*(.*)$") or ""
            apt.elev_ft = tonumber(line:match("^%d+%s+(%S+)"))
            first = false
        elseif code == "1" or code == "16" or code == "17" or code == "99" then
            break
        elseif code == "111" or code == "112" or code == "113" or code == "114" or code == "115" or code == "116" then
            -- Knoten von Belagsflaechen (110) und Markierungslinien (120)
            if feat == "pave" or feat == "line" then
                local t, k = {}, 0
                for tok in line:gmatch("%S+") do k = k + 1; t[k] = tok end
                local bez = (code == "112" or code == "114" or code == "116")
                local nd = { lat = tonumber(t[2]), lon = tonumber(t[3]) }
                local ai = 4
                if bez then
                    nd.clat, nd.clon = tonumber(t[4]), tonumber(t[5])
                    ai = 6
                end
                if feat == "line" then
                    -- Attribute: Linientyp (< 100) und/oder Befeuerung (>= 100)
                    for q = ai, math.min(k, ai + 1) do
                        local v = tonumber(t[q])
                        if v then
                            if v >= 100 then nd.light = v else nd.lt = v end
                        end
                    end
                end
                if nd.lat and nd.lon then
                    ring = ring or {}
                    ring[#ring + 1] = nd
                end
                local closed = (code == "113" or code == "114")
                if closed or code == "115" or code == "116" then
                    if ring and #ring >= 2 then
                        if feat == "pave" then
                            poly.rings[#poly.rings + 1] = ring
                        else
                            apt.lines[#apt.lines + 1] = { nodes = ring, closed = closed }
                        end
                    end
                    ring = nil
                end
            end
        elseif code == "110" then
            feat, ring = "pave", nil
            poly = { rings = {}, name = line:match("^110%s+%S+%s+%S+%s+%S+%s*(.-)%s*$") or "" }
            apt.polys[#apt.polys + 1] = poly
        elseif code == "120" then
            feat, ring = "line", nil
        elseif code == "1302" then
            local key, val = line:match("^1302%s+(%S+)%s*(.-)%s*$")
            if key == "iata_code" and val ~= "" then apt.iata = val:upper()
            elseif key == "city" and val ~= "" then apt.city = val:upper() end
        elseif code == "1301" then
            -- ICAO-Kategorie der vorigen Position (A-F)
            local cat = line:match("^1301%s+(%a)")
            local st = apt.stands[#apt.stands]
            if st and cat then st.cat = cat:upper() end
        elseif code == "1300" then
            local t, k = {}, 0
            for tok in line:gmatch("%S+") do k = k + 1; t[k] = tok end
            local la, lo, hd = tonumber(t[2]), tonumber(t[3]), tonumber(t[4])
            if la and lo then
                apt.stands[#apt.stands + 1] = { lat = la, lon = lo, hdg = hd or 0, typ = t[5] or "", cls = t[6] or "",
                    name = (k >= 7) and table.concat(t, " ", 7, k) or "" }
            end
            feat = nil
        elseif code == "100" then
            local t, k = {}, 0
            for tok in line:gmatch("%S+") do k = k + 1; t[k] = tok end
            -- je Ende: Kennung, Lage, versetzte Schwelle [m], Blast Pad / Overrun [m]
            local r = { width = tonumber(t[2]) or 45,
                { id = t[9], lat = tonumber(t[10]), lon = tonumber(t[11]), disp = tonumber(t[12]) or 0, blast = tonumber(t[13]) or 0,
                  mark = tonumber(t[14]) or 0 },
                { id = t[18], lat = tonumber(t[19]), lon = tonumber(t[20]), disp = tonumber(t[21]) or 0, blast = tonumber(t[22]) or 0,
                  mark = tonumber(t[23]) or 0 } }
            if r[1].lat and r[2].lat then apt.rwys[#apt.rwys + 1] = r end
        elseif code == "1201" then
            local la, lo, id = line:match("^1201%s+(%S+)%s+(%S+)%s+%S+%s+(%S+)")
            if id then
                apt.nodes[id] = { lat = tonumber(la), lon = tonumber(lo) }
                apt.n_nodes = apt.n_nodes + 1
            end
        elseif code == "1202" then
            local t, k = {}, 0
            for tok in line:gmatch("%S+") do k = k + 1; t[k] = tok end
            if t[3] then
                local typ = t[5] or "taxiway"
                local name = t[6]
                if k > 6 then name = table.concat(t, " ", 6, k) end
                apt.edges[#apt.edges + 1] = { a = t[2], b = t[3], oneway = (t[4] == "oneway"),
                    rwy = (typ:sub(1, 6) == "runway"), name = name or "", wc = typ:match("^taxiway_(%a)") }
            end
        elseif code == "1206" then
            -- Weg fuer Bodenfahrzeuge (Service-Strassen) zwischen zwei 1201-Knoten
            local a, b = line:match("^1206%s+(%S+)%s+(%S+)")
            if b then apt.roads[#apt.roads + 1] = { a, b } end
        end
        if code and code ~= "110" and code ~= "120" and not (code >= "111" and code <= "116" and #code == 3)
            and code ~= "1300" and code ~= "1302" then
            feat = nil
        end
        n = n + 1
        if n % 1500 == 0 then coroutine.yield() end
    end
    f:close()
    return apt
end

-- Pistenenden aufbereiten
local function prepare_runway_ends(apt)
    local ends = {}
    for _, r in ipairs(apt.rwys) do
        for k = 1, 2 do
            local E, O = r[k], r[3 - k]
            local x, y = to_xy(O.lat, O.lon, E.lat, E.lon)
            local L = math.sqrt(x * x + y * y)
            if L > 100 then
                ends[#ends + 1] = {
                    id = E.id, lat = E.lat, lon = E.lon, disp = E.disp, width = r.width, mark = E.mark, icao = apt.icao,
                    len = L, ux = x / L, uy = y / L,
                    hdg = (math.atan2(x, y) / D2R) % 360,
                }
            end
        end
    end
    table.sort(ends, function(a, b) return a.id < b.id end)
    apt.ends = ends
end

-- Passt die Pistengeometrie zweier Quellen zusammen?
-- Gleicher Flughafen, wenn alle Pisten der Quelle mit weniger Pisten auch in der anderen liegen
-- (Szenerien mit neuer Piste, z.B. VHHH 3. Bahn, gegen aeltere Standarddaten)
local function runways_match(a, b)
    if #a.rwys == 0 or #b.rwys == 0 then return false end
    if #a.rwys > #b.rwys then a, b = b, a end
    for _, r in ipairs(a.rwys) do
        local found = false
        for _, q in ipairs(b.rwys) do
            for k = 1, 2 do
                if dist_m(r[1].lat, r[1].lon, q[k].lat, q[k].lon) < 150
                    and dist_m(r[2].lat, r[2].lon, q[3 - k].lat, q[3 - k].lon) < 150 then
                    found = true
                end
            end
        end
        if not found then return false end
    end
    return true
end


-- Flughafengeometrie fuer die Karte: alles in Metern relativ zu einem Bezugspunkt (x Ost, y Nord)
-- Kartengeometrie und OSM-Gebaeude in einem eigenen Block (haelt die Zahl lokaler Variablen klein)
local build_geo, OSM, file_exists, osm_attach, osm_poll
do
local CHUNK = 24
local GEO_OPS = 0
local function geo_tick(n)
    GEO_OPS = GEO_OPS + n
    if GEO_OPS > 15000 then GEO_OPS = 0; coroutine.yield() end
end

-- Polylinie in Stuecke mit Begrenzungsrechteck zerlegen (zum schnellen Aussortieren)
local function add_chunks(list, xs, ys, extra)
    local n = #xs
    local i = 1
    while i < n do
        local j = math.min(i + CHUNK, n)
        local cx, cy = {}, {}
        local minx, miny, maxx, maxy = math.huge, math.huge, -math.huge, -math.huge
        for k = i, j do
            local x, y = xs[k], ys[k]
            cx[#cx + 1], cy[#cy + 1] = x, y
            if x < minx then minx = x end
            if x > maxx then maxx = x end
            if y < miny then miny = y end
            if y > maxy then maxy = y end
        end
        list[#list + 1] = { xs = cx, ys = cy, minx = minx, miny = miny, maxx = maxx, maxy = maxy, k = extra }
        i = j
    end
end

-- Knotenliste (mit Bezier-Kontrollpunkten) als Polylinie; si = Index des Knotens, der das Segment beginnt
local function sample_nodes(P, closed)
    local xs, ys, si = {}, {}, {}
    local n = #P
    local last = closed and n or (n - 1)
    for i = 1, last do
        local A, B = P[i], P[i % n + 1]
        xs[#xs + 1], ys[#ys + 1], si[#si + 1] = A.x, A.y, i
        if A.cx or B.cx then
            local p1x, p1y = A.cx or A.x, A.cy or A.y
            local p2x, p2y = B.x, B.y
            if B.cx then p2x, p2y = 2 * B.x - B.cx, 2 * B.y - B.cy end
            -- Stuetzpunkte nach Kurvenlaenge (Kontrollpolygon): etwa alle 1,5 m, mindestens wie frueher 4, hoechstens 40
            -- (die Detailstufen duennen danach formtreu aus, siehe decimate)
            local lc = math.sqrt((p1x - A.x) ^ 2 + (p1y - A.y) ^ 2) + math.sqrt((p2x - p1x) ^ 2 + (p2y - p1y) ^ 2)
                     + math.sqrt((B.x - p2x) ^ 2 + (B.y - p2y) ^ 2)
            local ns = math.max(5, math.min(41, math.ceil(lc / 1.5)))
            for q = 1, ns - 1 do
                local t = q / ns
                local u = 1 - t
                local a, b, c, d = u * u * u, 3 * u * u * t, 3 * u * t * t, t * t * t
                xs[#xs + 1] = a * A.x + b * p1x + c * p2x + d * B.x
                ys[#ys + 1] = a * A.y + b * p1y + c * p2y + d * B.y
                si[#si + 1] = i
            end
        end
    end
    if closed then
        xs[#xs + 1], ys[#ys + 1], si[#si + 1] = xs[1], ys[1], n
    else
        xs[#xs + 1], ys[#ys + 1], si[#si + 1] = P[n].x, P[n].y, n
    end
    return xs, ys, si
end

-- Detailstufe einer Linie: formtreu vereinfachen (Douglas-Peucker). mind wie frueher der Punktabstand der Stufe
-- (3 / 6-8 / 20-25 m); die erlaubte Abweichung von der echten Linie ist 5 % davon (0,15 / 0,3-0,4 / 1-1,25 m).
-- Gerade Stuecke verlieren alle Zwischenpunkte, Kurven bleiben rund (frueher: fester Abstand -> eckige Kurven).
local function decimate(xs, ys, mind)
    local n = #xs
    if n <= 2 then return { xs[1], xs[n] }, { ys[1], ys[n] } end
    local tol2 = (mind * 0.05) ^ 2
    local keep = { [1] = true, [n] = true }
    local stack = { 1, n }
    while #stack > 0 do
        local b = table.remove(stack)
        local a = table.remove(stack)
        local ax, ay = xs[a], ys[a]
        local dx, dy = xs[b] - ax, ys[b] - ay
        local L2 = dx * dx + dy * dy
        local best, bi = tol2, nil
        for k = a + 1, b - 1 do
            local px, py = xs[k] - ax, ys[k] - ay
            local d2
            if L2 < 1e-9 then
                d2 = px * px + py * py
            else
                local t = (px * dx + py * dy) / L2
                if t < 0 then t = 0 elseif t > 1 then t = 1 end
                d2 = (px - t * dx) ^ 2 + (py - t * dy) ^ 2
            end
            if d2 > best then best, bi = d2, k end
        end
        if bi then
            keep[bi] = true
            stack[#stack + 1], stack[#stack + 2] = a, bi
            stack[#stack + 1], stack[#stack + 2] = bi, b
        end
    end
    local gx, gy = {}, {}
    for k = 1, n do
        if keep[k] then gx[#gx + 1], gy[#gy + 1] = xs[k], ys[k] end
    end
    return gx, gy
end

-- Kleine Schleifen (Selbstueberschneidungen an Kurven, z.B. aus Bezier-Daten) aus einem Ring entfernen:
-- schneiden sich zwei nahe Segmente, wird der Bereich dazwischen durch den Schnittpunkt ersetzt.
local function fix_ring(xs0, ys0)
    local xs, ys = {}, {}
    local n0 = #xs0
    if n0 > 1 and xs0[1] == xs0[n0] and ys0[1] == ys0[n0] then n0 = n0 - 1 end
    for i = 1, n0 do xs[i], ys[i] = xs0[i], ys0[i] end
    local WIN = 10
    local function pass()
        local n = #xs
        if n < 5 then return false end
        local ox, oy = {}, {}
        local k = 1
        local changed = false
        while k <= n do
            ox[#ox + 1], oy[#oy + 1] = xs[k], ys[k]
            local k2 = k % n + 1
            local ax, ay, bx, by = xs[k], ys[k], xs[k2], ys[k2]
            local jump
            for m = k + 2, math.min(k + WIN, n - 1) do
                local m2 = m + 1
                local cx, cy, dx, dy = xs[m], ys[m], xs[m2], ys[m2]
                local d1 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
                local d2 = (bx - ax) * (dy - ay) - (by - ay) * (dx - ax)
                if (d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0) then
                    local d3 = (dx - cx) * (ay - cy) - (dy - cy) * (ax - cx)
                    local d4 = (dx - cx) * (by - cy) - (dy - cy) * (bx - cx)
                    if (d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0) then
                        local t = d3 / (d3 - d4)
                        ox[#ox + 1], oy[#oy + 1] = ax + (bx - ax) * t, ay + (by - ay) * t
                        jump = m2
                        break
                    end
                end
            end
            if jump then
                k = jump
                changed = true
            else
                k = k + 1
            end
        end
        xs, ys = ox, oy
        return changed
    end
    for _ = 1, 3 do
        local c1 = pass()
        -- Ring drehen, damit auch Schleifen ueber den Anfangspunkt gefunden werden
        local n = #xs
        if n >= 5 then
            local h = math.floor(n / 2)
            local rx, ry = {}, {}
            for i = 1, n do
                local j = (i + h - 1) % n + 1
                rx[i], ry[i] = xs[j], ys[j]
            end
            xs, ys = rx, ry
        end
        local c2 = pass()
        if not c1 and not c2 then break end
    end
    geo_tick(#xs * WIN)
    return xs, ys
end

local function sarea(xs, ys)
    local a, n = 0, #xs
    for i = 1, n do
        local j = i % n + 1
        a = a + xs[i] * ys[j] - xs[j] * ys[i]
    end
    return a / 2
end

-- Ring an Stellen teilen, an denen er sich selbst beruehrt (gleicher Punkt mehrfach): Teilringe
local function split_ring(xs, ys)
    local rings = {}
    local px, py, keys, pos = {}, {}, {}, {}
    for k = 1, #xs do
        local key = math.floor(xs[k] * 50 + 0.5) * 1000003 + math.floor(ys[k] * 50 + 0.5)
        local i = pos[key]
        if i then
            local lx, ly = {}, {}
            for m = i, #px do lx[#lx + 1], ly[#ly + 1] = px[m], py[m] end
            for m = #px, i + 1, -1 do pos[keys[m]] = nil; px[m] = nil; py[m] = nil; keys[m] = nil end
            if #lx >= 3 then rings[#rings + 1] = { lx, ly } end
        else
            px[#px + 1], py[#py + 1], keys[#keys + 1] = xs[k], ys[k], key
            pos[key] = #px
        end
    end
    if #px >= 3 then rings[#rings + 1] = { px, py } end
    return rings
end

-- Kanten tauschen (Delaunay-Kriterium), damit keine extrem spitzen Nadel-Dreiecke entstehen.
-- T = Liste von Index-Tripeln (alle gleich orientiert, Vorzeichen sgn)
local function flip_edges(xs, ys, T, sgn)
    local nT = #T
    if nT < 2 then return end
    local N = #xs + 1
    local emap = {}
    local function key(u, v) if u < v then return u * N + v end return v * N + u end
    local function add_e(u, v, t)
        local k = key(u, v)
        local e = emap[k]
        if not e then emap[k] = { t } else e[#e + 1] = t end
    end
    local function rem_e(u, v, t)
        local e = emap[key(u, v)]
        if e then
            for i = #e, 1, -1 do if e[i] == t then table.remove(e, i) end end
        end
    end
    for t = 1, nT do
        local tr = T[t]
        add_e(tr[1], tr[2], t); add_e(tr[2], tr[3], t); add_e(tr[3], tr[1], t)
    end
    local function orient(a, b, c)
        return ((xs[b] - xs[a]) * (ys[c] - ys[a]) - (ys[b] - ys[a]) * (xs[c] - xs[a])) * sgn
    end
    local function opp(t, u, v)
        local tr = T[t]
        for i = 1, 3 do
            local w = tr[i]
            if w ~= u and w ~= v then return w end
        end
    end
    local stack = {}
    for k, e in pairs(emap) do if #e == 2 then stack[#stack + 1] = k end end
    local flips, maxf = 0, 8 * nT + 100
    local ops = 0
    while #stack > 0 and flips < maxf do
        local k = table.remove(stack)
        local e = emap[k]
        if e and #e == 2 then
            local t1, t2 = e[1], e[2]
            local u, v = math.floor(k / N), k % N
            local p, q = opp(t1, u, v), opp(t2, u, v)
            if p and q and p ~= q then
                -- Reihenfolge so, dass t1 = (u, v, p) gleich orientiert ist
                if orient(u, v, p) < 0 then u, v = v, u end
                local ok = orient(u, v, p) > 0 and orient(v, u, q) > 0
                if ok then
                    -- q im Umkreis von (u, v, p)?
                    local ax, ay = xs[u] - xs[q], ys[u] - ys[q]
                    local bx, by = xs[v] - xs[q], ys[v] - ys[q]
                    local cx, cy = xs[p] - xs[q], ys[p] - ys[q]
                    local det = (ax * ax + ay * ay) * (bx * cy - cx * by) - (bx * bx + by * by) * (ax * cy - cx * ay)
                              + (cx * cx + cy * cy) * (ax * by - bx * ay)
                    local sc = (ax * ax + ay * ay + bx * bx + by * by + cx * cx + cy * cy)
                    if det * sgn > 1e-9 * sc * sc and orient(p, u, q) > 0 and orient(q, v, p) > 0 then
                        -- tauschen: (u,v,p)+(v,u,q) -> (p,u,q)+(q,v,p)
                        rem_e(u, v, t1); rem_e(u, v, t2)
                        rem_e(v, p, t1); rem_e(p, u, t1)
                        rem_e(u, q, t2); rem_e(q, v, t2)
                        T[t1] = { p, u, q }
                        T[t2] = { q, v, p }
                        add_e(p, u, t1); add_e(u, q, t1); add_e(q, p, t1)
                        add_e(q, v, t2); add_e(v, p, t2); add_e(p, q, t2)
                        stack[#stack + 1] = key(p, u); stack[#stack + 1] = key(u, q)
                        stack[#stack + 1] = key(q, v); stack[#stack + 1] = key(v, p)
                        flips = flips + 1
                    end
                end
            end
        end
        ops = ops + 1
        if ops > 3000 then geo_tick(ops * 4); ops = 0 end
    end
end

-- Ohrenschneide-Triangulierung eines einfachen Polygons -> flache Liste x1,y1,x2,y2,x3,y3,...
local function triangulate(xs0, ys0)
    local xs, ys = {}, {}
    local n0 = #xs0
    if n0 > 1 and xs0[1] == xs0[n0] and ys0[1] == ys0[n0] then n0 = n0 - 1 end
    for i = 1, n0 do
        local j = #xs
        if j == 0 or (xs0[i] - xs[j]) ^ 2 + (ys0[i] - ys[j]) ^ 2 > 0.01 then xs[j + 1], ys[j + 1] = xs0[i], ys0[i] end
    end
    local n = #xs
    local out = {}
    if n < 3 then return out end
    local area = 0
    for i = 1, n do
        local j = i % n + 1
        area = area + xs[i] * ys[j] - xs[j] * ys[i]
    end
    local sgn = area > 0 and 1 or -1
    local prv, nxt = {}, {}
    for i = 1, n do prv[i] = (i == 1) and n or (i - 1); nxt[i] = (i == n) and 1 or (i + 1) end
    local function cross(a, b, c)
        return ((xs[b] - xs[a]) * (ys[c] - ys[a]) - (ys[b] - ys[a]) * (xs[c] - xs[a])) * sgn
    end
    local T = {}
    local remaining = n
    local i = 1
    local fails = 0
    local ops = 0
    -- relaxed: Punkte genau auf dem Rand des Ohrs (Polygon beruehrt sich selbst) blockieren nicht
    local relaxed = false
    while remaining > 3 do
        local a, c = prv[i], nxt[i]
        local ear = cross(a, i, c) > 1e-9
        if ear then
            local ax, ay, bx, by, cx, cy = xs[a], ys[a], xs[i], ys[i], xs[c], ys[c]
            local minx, maxx = math.min(ax, bx, cx), math.max(ax, bx, cx)
            local miny, maxy = math.min(ay, by, cy), math.max(ay, by, cy)
            local e1, e2, e3 = 0, 0, 0
            if relaxed then
                e1 = 0.05 * math.sqrt((bx - ax) ^ 2 + (by - ay) ^ 2)
                e2 = 0.05 * math.sqrt((cx - bx) ^ 2 + (cy - by) ^ 2)
                e3 = 0.05 * math.sqrt((ax - cx) ^ 2 + (ay - cy) ^ 2)
            end
            local j = nxt[c]
            while j ~= a do
                local px, py = xs[j], ys[j]
                if px >= minx and px <= maxx and py >= miny and py <= maxy then
                    local d1 = ((bx - ax) * (py - ay) - (by - ay) * (px - ax)) * sgn
                    local d2 = ((cx - bx) * (py - by) - (cy - by) * (px - bx)) * sgn
                    local d3 = ((ax - cx) * (py - cy) - (ay - cy) * (px - cx)) * sgn
                    local inside
                    if relaxed then inside = d1 > e1 and d2 > e2 and d3 > e3
                    else inside = d1 >= 0 and d2 >= 0 and d3 >= 0 end
                    if inside and not ((px == ax and py == ay) or (px == bx and py == by) or (px == cx and py == cy)) then
                        ear = false
                        break
                    end
                end
                j = nxt[j]
                ops = ops + 1
            end
        end
        if not ear and fails > remaining and not relaxed then
            relaxed, fails = true, 0
        elseif ear or fails > remaining then
            if ear or cross(a, i, c) > 0 then T[#T + 1] = { a, i, c } end
            nxt[a], prv[c] = c, a
            remaining = remaining - 1
            i = a
            fails = 0
        else
            i = nxt[i]
            fails = fails + 1
        end
        if ops > 8000 then geo_tick(ops); ops = 0 end
    end
    T[#T + 1] = { prv[i], i, nxt[i] }
    geo_tick(ops + n)
    flip_edges(xs, ys, T, sgn)
    for _, tr in ipairs(T) do
        local a, b, c = tr[1], tr[2], tr[3]
        out[#out + 1] = xs[a]; out[#out + 1] = ys[a]
        out[#out + 1] = xs[b]; out[#out + 1] = ys[b]
        out[#out + 1] = xs[c]; out[#out + 1] = ys[c]
    end
    return out
end

-- Dreiecke in Stuecke mit Begrenzungsrechteck.
-- Zu jeder Ecke wird eine Verschieberichtung gespeichert (Gehrung der beiden Kanten nach aussen):
-- beim Zeichnen wird jedes Dreieck um knapp 1 Pixel vergroessert, damit die Kantenglaettung
-- von ImGui keine hellen/dunklen Haarlinien zwischen benachbarten Dreiecken erzeugt.
local function tri_offsets(ax, ay, bx, by, cx, cy, o, k)
    local s = ((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) >= 0 and 1 or -1
    local X, Y = { ax, bx, cx }, { ay, by, cy }
    local nx, ny = {}, {}
    for e = 1, 3 do
        local f = e % 3 + 1
        local dx, dy = X[f] - X[e], Y[f] - Y[e]
        local L = math.sqrt(dx * dx + dy * dy)
        if L > 1e-6 then nx[e], ny[e] = dy / L * s, -dx / L * s else nx[e], ny[e] = 0, 0 end
    end
    for v = 1, 3 do
        local e1, e2 = (v + 1) % 3 + 1, v -- Kante davor (endet in v) und Kante danach (beginnt in v)
        local n1x, n1y, n2x, n2y = nx[e1], ny[e1], nx[e2], ny[e2]
        local d = 1 + n1x * n2x + n1y * n2y
        local vx, vy = 0, 0
        if d > 1e-3 then vx, vy = (n1x + n2x) / d, (n1y + n2y) / d end
        local l2 = vx * vx + vy * vy
        if l2 > 6.25 then local f = 2.5 / math.sqrt(l2); vx, vy = vx * f, vy * f end
        o[k + 2 * v - 1], o[k + 2 * v] = vx, vy
    end
end

local function tri_chunks(t)
    local list = {}
    local per = 6 * 48
    for i = 1, #t, per do
        local c, o = {}, {}
        local minx, miny, maxx, maxy = math.huge, math.huge, -math.huge, -math.huge
        for k = i, math.min(i + per - 1, #t), 2 do
            local x, y = t[k], t[k + 1]
            c[#c + 1], c[#c + 2] = x, y
            if x < minx then minx = x end
            if x > maxx then maxx = x end
            if y < miny then miny = y end
            if y > maxy then maxy = y end
        end
        for k = 1, #c - 5, 6 do tri_offsets(c[k], c[k + 1], c[k + 2], c[k + 3], c[k + 4], c[k + 5], o, k - 1) end
        list[#list + 1] = { t = c, o = o, minx = minx, miny = miny, maxx = maxx, maxy = maxy }
    end
    return list
end

-- Ring triangulieren: Teile mit gleicher Drehrichtung = Flaeche, Gegenrichtung = Loch (Dreiecks-Stuecke)
local function tri_ring(rx, ry)
    local main = sarea(rx, ry)
    local f, h = {}, {}
    for _, part in ipairs(split_ring(rx, ry)) do
        local a = sarea(part[1], part[2])
        if math.abs(a) > 1 then
            local t = triangulate(part[1], part[2])
            local dst = ((a > 0) == (main > 0)) and f or h
            for k = 1, #t do dst[#dst + 1] = t[k] end
        end
    end
    return tri_chunks(f), tri_chunks(h)
end

local function point_in_ring(xs, ys, px, py)
    local inside = false
    local n = #xs
    local j = n
    for i = 1, n do
        local xi, yi, xj, yj = xs[i], ys[i], xs[j], ys[j]
        if ((yi > py) ~= (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi) then inside = not inside end
        j = i
    end
    return inside
end

-- Linienklassen: "cl" = Rollweg-Mittellinie (gelb), "hold" = Haltelinie (rot)
-- "dy" = gestrichelt gelb, "ws" / "wd" = weiss durchgezogen / gestrichelt (Strassen)
local LINE_CLASS = { [1] = "cl", [7] = "cl", [51] = "cl", [57] = "cl", [60] = "cl",
                     [2] = "dy", [8] = "dy", [9] = "dy", [52] = "dy", [58] = "dy", [59] = "dy",
                     [20] = "ws", [21] = "ws", [22] = "wd",
                     [4] = "hold", [5] = "hold", [6] = "hold", [54] = "hold", [55] = "hold", [56] = "hold" }
local LIGHT_CLASS = { [101] = "cl", [105] = "cl", [107] = "cl", [108] = "cl", [103] = "hold", [104] = "hold" }

function build_geo(apt)
    local G = { pave = {}, pave_m = {}, pave_c = {}, polys = {}, tl = {}, tl_m = {}, tl_c = {}, hold = {}, stands = {}, terms = {},
                dash_y = {}, road = {}, wline = {}, n_white = 0,
                rwys = {}, edges = {}, lbl = {} }
    local r1 = apt.rwys[1]
    local rlat, rlon = r1 and r1[1].lat or 0, r1 and r1[1].lon or 0
    G.ref = { lat = rlat, lon = rlon }
    local function W(lat, lon) return to_xy(lat, lon, rlat, rlon) end
    G.W = W
    G.iata = apt.iata
    for _, E in ipairs(apt.ends) do E.ox, E.oy = W(E.lat, E.lon) end

    -- Pisten
    local minx, miny, maxx, maxy = math.huge, math.huge, -math.huge, -math.huge
    for _, r in ipairs(apt.rwys) do
        local x1, y1 = W(r[1].lat, r[1].lon)
        local x2, y2 = W(r[2].lat, r[2].lon)
        G.rwys[#G.rwys + 1] = { x1 = x1, y1 = y1, x2 = x2, y2 = y2, w = r.width, id1 = r[1].id, id2 = r[2].id,
                                disp1 = r[1].disp or 0, disp2 = r[2].disp or 0,
                                blast1 = math.min(r[1].blast or 0, 400), blast2 = math.min(r[2].blast or 0, 400) }
        minx, maxx = math.min(minx, x1, x2), math.max(maxx, x1, x2)
        miny, maxy = math.min(miny, y1, y2), math.max(maxy, y1, y2)
    end
    if minx == math.huge then minx, miny, maxx, maxy = -1000, -1000, 1000, 1000 end
    G.cx, G.cy = (minx + maxx) / 2, (miny + maxy) / 2
    G.minx, G.miny, G.maxx, G.maxy = minx, miny, maxx, maxy
    G.extent = math.max(maxx - minx, maxy - miny, 1500)

    local function to_pts(nodes)
        local P = {}
        for i, nd in ipairs(nodes) do
            local x, y = W(nd.lat, nd.lon)
            local q = { x = x, y = y, lt = nd.lt, light = nd.light }
            if nd.clat then q.cx, q.cy = W(nd.clat, nd.clon) end
            P[i] = q
        end
        return P
    end

    -- Belag: Raender (braun) + Flaechen (Dreiecke), Loecher werden schwarz uebermalt
    S.status = "Map " .. apt.icao .. ": pavement ..."
    for _, pg in ipairs(apt.polys) do
        local gp = { fill = {}, holes = {}, fill_m = {}, holes_m = {}, fill_c = {}, holes_c = {}, rings_m = {} }
        for ri, ring in ipairs(pg.rings) do
            local xs, ys = sample_nodes(to_pts(ring), true)
            -- wie im A350-OANS: Vorfeld dunkelgrau, Rollwege hell (mit braunem Rand). Vorfeld = eine Parkposition
            -- liegt in der Flaeche oder der Name sagt es (Apron, Ramp, Gate, Stand, ...)
            if ri == 1 then
                local nm = (pg.name or ""):lower()
                gp.apron = nm:find("apron") ~= nil or nm:find("ramp") ~= nil or nm:find("gate") ~= nil
                    or nm:find("stand") ~= nil or nm:find("park") ~= nil or nm:find("cargo") ~= nil
                if not gp.apron then
                    local x0, x1, y0, y1 = math.huge, -math.huge, math.huge, -math.huge
                    for k = 1, #xs do
                        x0, x1 = math.min(x0, xs[k]), math.max(x1, xs[k])
                        y0, y1 = math.min(y0, ys[k]), math.max(y1, ys[k])
                    end
                    for _, st in ipairs(apt.stands) do
                        local sx, sy = W(st.lat, st.lon)
                        if sx >= x0 and sx <= x1 and sy >= y0 and sy <= y1 and point_in_ring(xs, ys, sx, sy) then
                            gp.apron = true
                            break
                        end
                    end
                end
            end
            -- braune Raender nur an Rollwegen (das Vorfeld hat keinen)
            local cx, cy = decimate(xs, ys, 25)
            local hx, hy = decimate(xs, ys, 8)
            if not gp.apron then
                add_chunks(G.pave, xs, ys)
                if #cx >= 3 then add_chunks(G.pave_c, cx, cy) end
                if #hx >= 3 then add_chunks(G.pave_m, hx, hy) end
            end
            local fx, fy = fix_ring(decimate(xs, ys, 3))
            gp.rings_m[ri] = { fx, fy }
            local mx2, my2 = fix_ring(hx, hy)
            local kx2, ky2 = fix_ring(cx, cy)
            -- je Detailstufe: Flaeche und Loecher (beruehrt sich ein Ring selbst, wird er geteilt)
            local lv = { { fx, fy, "fill", "holes" }, { mx2, my2, "fill_m", "holes_m" }, { kx2, ky2, "fill_c", "holes_c" } }
            local prev_f, prev_h
            for li, L in ipairs(lv) do
                local f, h
                if #L[1] >= 3 then f, h = tri_ring(L[1], L[2]) elseif prev_f then f, h = prev_f, prev_h else f, h = {}, {} end
                prev_f, prev_h = f, h
                if ri == 1 then
                    for _, c in ipairs(f) do table.insert(gp[L[3]], c) end
                    for _, c in ipairs(h) do table.insert(gp[L[4]], c) end
                else
                    for _, c in ipairs(f) do table.insert(gp[L[4]], c) end
                end
            end
            geo_tick(#xs)
        end
        local bminx, bminy, bmaxx, bmaxy = math.huge, math.huge, -math.huge, -math.huge
        for _, c in ipairs(gp.fill) do
            bminx, bmaxx = math.min(bminx, c.minx), math.max(bmaxx, c.maxx)
            bminy, bmaxy = math.min(bminy, c.miny), math.max(bmaxy, c.maxy)
        end
        gp.minx, gp.miny, gp.maxx, gp.maxy = bminx, bminy, bmaxx, bmaxy
        if #gp.fill > 0 then G.polys[#G.polys + 1] = gp end
    end
    apt.polys = nil

    -- Innere Grenzen zwischen aneinanderstossenden Belagsflaechen nicht braun zeichnen:
    -- Belag zeilenweise als Intervalle erfassen, dann Randstuecke entfernen, die beidseitig auf Belag liegen
    do
        local RH = 2
        local Y0 = G.miny - 3000
        local rows = {}
        for _, gp in ipairs(G.polys) do
            local cross = {}
            for _, rg in ipairs(gp.rings_m) do
                local xs, ys = rg[1], rg[2]
                local n = #xs
                for k = 1, n do
                    local q = (k == n) and 1 or (k + 1)
                    local x1, y1, x2, y2 = xs[k], ys[k], xs[q], ys[q]
                    if y1 ~= y2 then
                        local ya, yb = math.min(y1, y2), math.max(y1, y2)
                        for j = math.ceil((ya - Y0) / RH - 0.5), math.ceil((yb - Y0) / RH - 0.5) - 1 do
                            local yc = Y0 + (j + 0.5) * RH
                            local l = cross[j]
                            if not l then l = {}; cross[j] = l end
                            l[#l + 1] = x1 + (yc - y1) / (y2 - y1) * (x2 - x1)
                        end
                    end
                end
                geo_tick(n * 2)
            end
            for j, l in pairs(cross) do
                table.sort(l)
                local r = rows[j]
                if not r then r = {}; rows[j] = r end
                for k = 1, #l - 1, 2 do r[#r + 1] = l[k]; r[#r + 1] = l[k + 1] end
            end
        end
        local function covered(x, y)
            local r = rows[math.floor((y - Y0) / RH)]
            if not r then return false end
            for k = 1, #r, 2 do
                if x >= r[k] and x <= r[k + 1] then return true end
            end
            return false
        end
        local function outer_only(list)
            local out = {}
            for _, c in ipairs(list) do
                local xs, ys = c.xs, c.ys
                local cx, cy = {}, {}
                local function flush()
                    if #cx >= 2 then add_chunks(out, cx, cy) end
                    cx, cy = {}, {}
                end
                for k = 1, #xs - 1 do
                    local dx, dy = xs[k + 1] - xs[k], ys[k + 1] - ys[k]
                    local L = math.sqrt(dx * dx + dy * dy)
                    local inner = false
                    if L > 0.01 then
                        local mx, my = (xs[k] + xs[k + 1]) / 2, (ys[k] + ys[k + 1]) / 2
                        local nx, ny = -dy / L * 3, dx / L * 3
                        inner = covered(mx + nx, my + ny) and covered(mx - nx, my - ny)
                    end
                    if inner then
                        flush()
                    else
                        if #cx == 0 then cx[1], cy[1] = xs[k], ys[k] end
                        cx[#cx + 1], cy[#cy + 1] = xs[k + 1], ys[k + 1]
                    end
                end
                flush()
                geo_tick(#xs * 4)
            end
            return out
        end
        G.pave, G.pave_m, G.pave_c = outer_only(G.pave), outer_only(G.pave_m), outer_only(G.pave_c)
    end

    local function on_pavement(px, py)
        for _, gp in ipairs(G.polys) do
            if px >= gp.minx and px <= gp.maxx and py >= gp.miny and py <= gp.maxy then
                local r = gp.rings_m
                if r[1] and point_in_ring(r[1][1], r[1][2], px, py) then
                    local hole = false
                    for k = 2, #r do
                        if point_in_ring(r[k][1], r[k][2], px, py) then hole = true; break end
                    end
                    if not hole then return true end
                end
            end
        end
        return false
    end

    -- Markierungslinien: gelbe Mittellinien (inkl. Einfahrten in die Positionen), rote Haltelinien
    S.status = "Map " .. apt.icao .. ": markings ..."
    local n_cl = 0
    local has_paint = false
    for _, ln in ipairs(apt.lines) do
        for _, nd in ipairs(ln.nodes) do
            if nd.lt and LINE_CLASS[nd.lt] == "cl" then has_paint = true; break end
        end
        if has_paint then break end
    end
    for _, ln in ipairs(apt.lines) do
        local P = to_pts(ln.nodes)
        local xs, ys, si = sample_nodes(P, ln.closed)
        local cur, rx, ry = nil, nil, nil
        local function flush()
            if cur and rx and #rx >= 2 then
                if cur == "cl" then
                    add_chunks(G.tl, rx, ry)
                    local mx2, my2 = decimate(rx, ry, 6)
                    if #mx2 >= 2 then add_chunks(G.tl_m, mx2, my2) end
                    local cx2, cy2 = decimate(rx, ry, 20)
                    if #cx2 >= 2 then add_chunks(G.tl_c, cx2, cy2) end
                    n_cl = n_cl + #rx
                elseif cur == "dy" then
                    add_chunks(G.dash_y, rx, ry)
                elseif cur == "wd" or cur == "ws" then
                    add_chunks((cur == "wd") and G.road or G.wline, rx, ry)
                    G.n_white = G.n_white + 1
                else
                    add_chunks(G.hold, rx, ry)
                end
            end
            cur, rx, ry = nil, nil, nil
        end
        for k = 1, #xs - 1 do
            local nd = P[si[k]]
            local cls
            if has_paint then cls = nd.lt and LINE_CLASS[nd.lt]
            else cls = (nd.lt and LINE_CLASS[nd.lt]) or (nd.light and LIGHT_CLASS[nd.light]) end
            if cls ~= cur then
                flush()
                if cls then cur, rx, ry = cls, { xs[k] }, { ys[k] } end
            end
            if cur then rx[#rx + 1], ry[#ry + 1] = xs[k + 1], ys[k + 1] end
        end
        flush()
        geo_tick(#xs)
    end
    apt.lines = nil

    -- Strassen fuer Bodenfahrzeuge: ohne gemalte weisse Linien aus dem Fahrzeug-Wegenetz (1206), zu Zuegen verbunden
    local rn = apt.road_net
    if rn and G.n_white == 0 then
        local E, adj, seen = {}, {}, {}
        for _, r in ipairs(rn.roads) do
            local a, b = r[1], r[2]
            local key = (a < b) and (a .. "/" .. b) or (b .. "/" .. a)
            if a ~= b and rn.nodes[a] and rn.nodes[b] and not seen[key] then
                seen[key] = true
                E[#E + 1] = { a, b }
                adj[a] = adj[a] or {}; table.insert(adj[a], #E)
                adj[b] = adj[b] or {}; table.insert(adj[b], #E)
            end
        end
        local used = {}
        local function walk(start, k)
            local nd = rn.nodes[start]
            local x, y = W(nd.lat, nd.lon)
            local xs, ys = { x }, { y }
            local cur = start
            while k and not used[k] do
                used[k] = true
                local e = E[k]
                cur = (e[1] == cur) and e[2] or e[1]
                nd = rn.nodes[cur]
                x, y = W(nd.lat, nd.lon)
                xs[#xs + 1], ys[#ys + 1] = x, y
                k = nil
                local a = adj[cur]
                if #a == 2 then
                    for _, kk in ipairs(a) do if not used[kk] then k = kk end end
                end
            end
            if #xs >= 2 then add_chunks(G.road, xs, ys) end
        end
        local ids = {}
        for id in pairs(adj) do ids[#ids + 1] = id end
        table.sort(ids)
        for _, id in ipairs(ids) do
            local a = adj[id]
            if #a ~= 2 then for _, k in ipairs(a) do if not used[k] then walk(id, k) end end end
        end
        for k, e in ipairs(E) do if not used[k] then walk(e[1], k) end end
    end
    apt.road_net = nil

    -- Rollwegnetz (Anzeige-Ersatz, Rollweg-Namen)
    local net = apt.nets[1]
    if net then
        local cache = {}
        local function nxy(id)
            local c = cache[id]
            if c == nil then
                local nd = net.nodes[id]
                c = nd and { W(nd.lat, nd.lon) } or false
                cache[id] = c
            end
            return c
        end
        for _, e in ipairs(net.edges) do
            local A, B = nxy(e.a), nxy(e.b)
            if A and B then
                local len = math.sqrt((B[1] - A[1]) ^ 2 + (B[2] - A[2]) ^ 2)
                -- Breite [m] nach ICAO-Klasse der Kante (taxiway_A..F), sonst 18 m
                local tw = ({ A = 7.5, B = 10.5, C = 15, D = 18, E = 23, F = 25 })[e.wc or ""] or 18
                local ed = { x1 = A[1], y1 = A[2], x2 = B[1], y2 = B[2], name = e.name, rwy = e.rwy, len = len, w = tw }
                G.edges[#G.edges + 1] = ed
                if not e.rwy and e.name ~= "" and e.name ~= "--" then G.lbl[#G.lbl + 1] = ed end
            end
        end
        table.sort(G.lbl, function(a, b) return a.len > b.len end)
        -- Abrollpunkte aller Pisten: benannte Rollweg-Kanten mit genau einem Ende auf einem Pisten-Knoten
        -- (Punkt auf der Pistenmitte, Richtung von der Piste weg; gleicher Name am selben Punkt nur einmal)
        G.vac = {}
        local onrwy = {}
        for _, e in ipairs(net.edges) do if e.rwy then onrwy[e.a], onrwy[e.b] = true, true end end
        for _, e in ipairs(net.edges) do
            if not e.rwy and e.name ~= "" and e.name ~= "--" and (onrwy[e.a] or false) ~= (onrwy[e.b] or false) then
                local P, Q = nxy(onrwy[e.a] and e.a or e.b), nxy(onrwy[e.a] and e.b or e.a)
                if P and Q then
                    local dx, dy = Q[1] - P[1], Q[2] - P[2]
                    local l = math.sqrt(dx * dx + dy * dy)
                    local dup = l < 1
                    for _, v in ipairs(G.vac) do
                        if dup then break end
                        dup = v.name == e.name and (v.x - P[1]) ^ 2 + (v.y - P[2]) ^ 2 < 900
                    end
                    if not dup then G.vac[#G.vac + 1] = { x = P[1], y = P[2], ux = dx / l, uy = dy / l, name = e.name } end
                end
            end
        end
    end

    -- keine gemalten Mittellinien: Rollwege aus dem Rollwegnetz
    local no_paint = n_cl < 50
    if no_paint and #G.edges > 0 then
        for _, e in ipairs(G.edges) do
            if not e.rwy then
                add_chunks(G.tl, { e.x1, e.x2 }, { e.y1, e.y2 })
                add_chunks(G.tl_m, { e.x1, e.x2 }, { e.y1, e.y2 })
                add_chunks(G.tl_c, { e.x1, e.x2 }, { e.y1, e.y2 })
            end
        end
    end
    -- Rollwegnetz als Linienstuecke (Uebersicht bei grosser Reichweite)
    G.tw_net = {}
    for _, e in ipairs(G.edges) do
        if not e.rwy then add_chunks(G.tw_net, { e.x1, e.x2 }, { e.y1, e.y2 }) end
    end

    ---------------------------------------------------------------------------
    -- Parkpositionen: Flaechen (grau, ueberlappende zusammengefasst), gerade Einfahrlinien, Namen
    ---------------------------------------------------------------------------
    S.status = "Map " .. apt.icao .. ": stands ..."
    -- Spannweite / Laenge [m] je ICAO-Kategorie
    local CAT = { A = { 15, 15 }, B = { 24, 22 }, C = { 36, 40 }, D = { 52, 56 }, E = { 65, 72 }, F = { 80, 75 } }
    local stands = {}
    for _, st in ipairs(apt.stands) do
        local x, y = W(st.lat, st.lon)
        local h = st.hdg * D2R
        local fx, fy = math.sin(h), math.cos(h)  -- Bugrichtung
        local cat = st.cat
        if not CAT[cat or ""] then
            local c = st.cls or ""
            cat = c:find("heavy") and "E" or (c:find("jets") and "C") or ((c:find("props") or c:find("turbo")) and "B") or
                  (c:find("helo") and "A") or "C"
        end
        local span, len = CAT[cat][1], CAT[cat][2]
        stands[#stands + 1] = { x = x, y = y, fx = fx, fy = fy, rx = fy, ry = -fx, name = st.name, typ = st.typ,
                                span = span, len = len, cat = cat }
        -- optional: Terminal vor der Nase einer Gate-Position schaetzen (nur wenn eingeschaltet)
        if st.typ == "gate" and span >= 36 then
            local d0, d1 = 8, 40
            local tcx, tcy = x + fx * (d0 + d1) / 2, y + fy * (d0 + d1) / 2
            if not on_pavement(tcx, tcy) and not on_pavement(x + fx * (d0 + 4), y + fy * (d0 + 4)) then
                local tw, rx, ry = span * 0.5, fy, -fx
                G.terms[#G.terms + 1] = {
                    x + fx * d0 + rx * tw, y + fy * d0 + ry * tw, x + fx * d1 + rx * tw, y + fy * d1 + ry * tw,
                    x + fx * d1 - rx * tw, y + fy * d1 - ry * tw, x + fx * d0 - rx * tw, y + fy * d0 - ry * tw,
                    cx = tcx, cy = tcy }
            end
            geo_tick(1500)
        end
        geo_tick(40)
    end
    apt.stands = nil
    G.stands = stands
    local ns = #stands

    -- Rollweg-Mittellinien (gemalt + Netz) in ein Suchraster fuer Strahltests
    local GC = 40
    local grid = {}
    local function gkey(i, j) return j * 100003 + i end
    local function add_seg(ax, ay, bx, by)
        local sg = { ax, ay, bx, by }
        for i = math.floor(math.min(ax, bx) / GC), math.floor(math.max(ax, bx) / GC) do
            for j = math.floor(math.min(ay, by) / GC), math.floor(math.max(ay, by) / GC) do
                local k = gkey(i, j)
                local l = grid[k]
                if not l then l = {}; grid[k] = l end
                l[#l + 1] = sg
            end
        end
    end
    if ns > 0 then
        for _, c in ipairs(G.tl) do
            for k = 1, #c.xs - 1 do add_seg(c.xs[k], c.ys[k], c.xs[k + 1], c.ys[k + 1]) end
            geo_tick(#c.xs * 3)
        end
        for _, e in ipairs(G.edges) do
            if not e.rwy then add_seg(e.x1, e.y1, e.x2, e.y2) end
        end
        geo_tick(#G.edges * 3)
    end
    -- erster Treffer eines Strahls (px,py)+t*(dx,dy) mit einer Mittellinie, t in [tmin, tmax]; fast parallele Linien zaehlen nicht
    local stamp = 0
    local function ray_hit(px, py, dx, dy, tmin, tmax)
        stamp = stamp + 1
        local ex, ey = px + dx * tmax, py + dy * tmax
        local best = nil
        for i = math.floor(math.min(px, ex) / GC), math.floor(math.max(px, ex) / GC) do
            for j = math.floor(math.min(py, ey) / GC), math.floor(math.max(py, ey) / GC) do
                local l = grid[gkey(i, j)]
                if l then
                    for _, sg in ipairs(l) do
                        if sg.st ~= stamp then
                            sg.st = stamp
                            local sx, sy = sg[3] - sg[1], sg[4] - sg[2]
                            local den = dx * sy - dy * sx
                            local sl = math.sqrt(sx * sx + sy * sy)
                            if sl > 0.1 and math.abs(den) > 0.35 * sl then
                                local qx, qy = sg[1] - px, sg[2] - py
                                local t = (qx * sy - qy * sx) / den
                                local u = (qx * dy - qy * dx) / den
                                if t >= tmin and t <= tmax and u >= 0 and u <= 1 and (not best or t < best) then best = t end
                            end
                        end
                    end
                end
            end
        end
        return best
    end

    -- MARS / doppelte Positionen: Bugrad-Punkte sehr nah beieinander
    local parent = {}
    for i = 1, ns do parent[i] = i end
    local function find(i) while parent[i] ~= i do parent[i] = parent[parent[i]]; i = parent[i] end return i end
    -- Grundname einer MARS-Familie: "29A" -> "29", "A2B" -> "A2", "12L" -> "12"
    for _, st in ipairs(stands) do
        local nm = (st.name or ""):upper()
        st.base = nm:match("^(.-%d)[A-Z]$") or nm
    end
    for i = 1, ns do
        local a = stands[i]
        for j = i + 1, ns do
            local b = stands[j]
            local lim = 0.45 * math.max(a.span, b.span) + 3
            local d2 = (a.x - b.x) ^ 2 + (a.y - b.y) ^ 2
            if d2 < lim * lim or (a.base ~= "" and a.base == b.base and d2 < 60 * 60) then parent[find(i)] = find(j) end
        end
        if i % 50 == 0 then geo_tick(ns * 50) end
    end
    local mars = {}
    for i = 1, ns do mars[i] = find(i) end

    -- Laenge bis zur Rollgasse hinter der Position
    for _, st in ipairs(stands) do
        st.hit = ray_hit(st.x, st.y, -st.fx, -st.fy, 10, st.len + 45)
        local tail = st.len + 3
        if st.hit then tail = math.max(18, math.min(tail, st.hit - 7)) end
        st.tail = tail
        st.hw = st.span / 2 + 2
        geo_tick(60)
    end
    -- Breite an die Nachbarn anpassen (nebeneinander liegende Positionen ueberlappen nicht)
    for i = 1, ns do
        local a = stands[i]
        for j = 1, ns do
            local b = stands[j]
            if j ~= i and mars[j] ~= mars[i] and (a.x - b.x) ^ 2 + (a.y - b.y) ^ 2 < 200 * 200 then
                for _, f in ipairs({ 0, 0.5, 1 }) do
                    local px, py = b.x - b.fx * b.tail * f, b.y - b.fy * b.tail * f
                    local qx, qy = px - a.x, py - a.y
                    local along = -(qx * a.fx + qy * a.fy)
                    if along > -5 and along < a.tail + 5 then
                        local lat = math.abs(qx * a.rx + qy * a.ry)
                        a.hw = math.min(a.hw, lat / 2 - 0.75)
                    end
                end
            end
        end
        a.hw = math.max(a.hw, 6, 0.45 * (a.span / 2 + 2))
        geo_tick(ns * 4)
    end
    -- Rechtecke
    for _, st in ipairs(stands) do
        local fx, fy, rx, ry, hw = st.fx, st.fy, st.rx, st.ry, st.hw
        local nx, ny = st.x + fx * 3, st.y + fy * 3
        local tx, ty = st.x - fx * st.tail, st.y - fy * st.tail
        st.box = { nx + rx * hw, ny + ry * hw, nx - rx * hw, ny - ry * hw, tx - rx * hw, ty - ry * hw, tx + rx * hw, ty + ry * hw }
    end
    -- ueberlappende Flaechen zusammenfassen (Trennachsen-Test, Rechtecke um 1,5 m verkleinert)
    local function box_axes_overlap(A, Bx)
        local function proj(bx, ux, uy)
            local mn, mx = math.huge, -math.huge
            for k = 1, 7, 2 do
                local v = bx[k] * ux + bx[k + 1] * uy
                if v < mn then mn = v end
                if v > mx then mx = v end
            end
            return mn, mx
        end
        for _, q in ipairs({ A, Bx }) do
            for e = 1, 2 do
                local k = (e - 1) * 2 + 1
                local ux, uy = q[k + 2] - q[k], q[k + 3] - q[k + 1]
                local L = math.sqrt(ux * ux + uy * uy)
                if L > 0 then
                    ux, uy = ux / L, uy / L
                    local a0, a1 = proj(A, ux, uy)
                    local b0, b1 = proj(Bx, ux, uy)
                    if a1 - 1.5 <= b0 + 1.5 or b1 - 1.5 <= a0 + 1.5 then return false end
                end
            end
        end
        return true
    end
    for i = 1, ns do
        local a = stands[i]
        for j = i + 1, ns do
            local b = stands[j]
            if find(i) ~= find(j) and (a.x - b.x) ^ 2 + (a.y - b.y) ^ 2 < 160 * 160 and box_axes_overlap(a.box, b.box) then
                parent[find(i)] = find(j)
            end
        end
        if i % 40 == 0 then geo_tick(ns * 40) end
    end
    -- je Gruppe: konvexe Huelle als Flaeche
    local groups = {}
    for i = 1, ns do
        local r = find(i)
        groups[r] = groups[r] or {}
        table.insert(groups[r], stands[i])
    end
    G.stand_areas = {}
    for _, grp in pairs(groups) do
        local P = {}
        for _, st in ipairs(grp) do
            for k = 1, 7, 2 do P[#P + 1] = { st.box[k], st.box[k + 1] } end
        end
        table.sort(P, function(p, q) if p[1] ~= q[1] then return p[1] < q[1] end return p[2] < q[2] end)
        local function cr(o, a, b) return (a[1] - o[1]) * (b[2] - o[2]) - (a[2] - o[2]) * (b[1] - o[1]) end
        local H = {}
        for _, p in ipairs(P) do
            while #H >= 2 and cr(H[#H - 1], H[#H], p) <= 0 do H[#H] = nil end
            H[#H + 1] = p
        end
        local lo = #H + 1
        for k = #P - 1, 1, -1 do
            local p = P[k]
            while #H >= lo and cr(H[#H - 1], H[#H], p) <= 0 do H[#H] = nil end
            H[#H + 1] = p
        end
        H[#H] = nil
        if #H >= 3 then
            local hx, hy, t = {}, {}, {}
            local minx, miny, maxx, maxy = math.huge, math.huge, -math.huge, -math.huge
            for k, p in ipairs(H) do
                hx[k], hy[k] = p[1], p[2]
                minx, maxx = math.min(minx, p[1]), math.max(maxx, p[1])
                miny, maxy = math.min(miny, p[2]), math.max(maxy, p[2])
            end
            for k = 2, #H - 1 do
                t[#t + 1], t[#t + 2], t[#t + 3], t[#t + 4], t[#t + 5], t[#t + 6] = hx[1], hy[1], hx[k], hy[k], hx[k + 1], hy[k + 1]
            end
            G.stand_areas[#G.stand_areas + 1] = { x = hx, y = hy, tris = tri_chunks(t), minx = minx, miny = miny, maxx = maxx, maxy = maxy }
        end
        geo_tick(#P * 10)
    end

    -- Namen: MARS-Positionen teilen sich eine Beschriftung ("29_29A_29B"); Platzhalter weglassen
    G.stand_lbls = {}
    do
        local byg = {}
        for i = 1, ns do
            local r = mars[i]
            byg[r] = byg[r] or {}
            table.insert(byg[r], stands[i])
        end
        for _, grp in pairs(byg) do
            local names, sx, sy = {}, 0, 0
            for _, st in ipairs(grp) do
                local nm = st.name or ""
                if nm ~= "" and not nm:lower():find("ramp start") then names[#names + 1] = nm end
                local back = math.min(st.tail * 0.3, 18)
                sx, sy = sx + st.x - st.fx * back, sy + st.y - st.fy * back
            end
            if #names > 0 then
                table.sort(names, function(p, q)
                    if #p ~= #q then return #p < #q end
                    return p < q
                end)
                G.stand_lbls[#G.stand_lbls + 1] = { x = sx / #grp, y = sy / #grp, text = table.concat(names, "_"), n = #names }
            end
        end
    end

    -- Gerade Einfahrlinien fuer Positionen ohne eigene gemalte Linie (entlang der Positionsrichtung)
    if ns > 0 then
        local painted = {}
        if not no_paint then for i, c in ipairs(G.tl) do painted[i] = c end end
        local made = {}
        for _, st in ipairs(stands) do
            local has = false
            for _, c in ipairs(painted) do
                if st.x > c.minx - 15 and st.x < c.maxx + 15 and st.y > c.miny - 15 and st.y < c.maxy + 15 then
                    for k = 1, #c.xs do
                        if (c.xs[k] - st.x) ^ 2 + (c.ys[k] - st.y) ^ 2 < 225 then has = true; break end
                    end
                end
                if has then break end
            end
            -- gleiche Linie schon vorhanden (MARS auf einer Achse)?
            if not has then
                for _, m in ipairs(made) do
                    local qx, qy = st.x - m.x, st.y - m.y
                    local lat = math.abs(qx * m.rx + qy * m.ry)
                    if lat < 4 and (st.fx * m.fx + st.fy * m.fy) > 0.99 then has = true; break end
                end
            end
            if not has then
                local L = (st.hit and st.hit <= st.len + 45) and st.hit or st.tail
                local xs, ys = { st.x, st.x - st.fx * L }, { st.y, st.y - st.fy * L }
                add_chunks(G.tl, xs, ys)
                add_chunks(G.tl_m, xs, ys)
                made[#made + 1] = st
            end
            geo_tick(#painted + 20)
        end
    end

    -- Rollgassen auf dem Vorfeld: Kanten des Rollwegnetzes, die (mit Anfang, Mitte oder Ende) auf einer
    -- Vorfeld-Flaeche liegen - werden im hellen Rollweg-Grau ueber das dunkle Vorfeld gezeichnet
    G.apron_tw = {}
    local aprons = {}
    for _, gp in ipairs(G.polys) do if gp.apron and gp.rings_m[1] then aprons[#aprons + 1] = gp end end
    if #aprons > 0 then
        local function on_apron(px, py)
            for _, gp in ipairs(aprons) do
                if px >= gp.minx and px <= gp.maxx and py >= gp.miny and py <= gp.maxy
                    and point_in_ring(gp.rings_m[1][1], gp.rings_m[1][2], px, py) then return true end
            end
            return false
        end
        for _, e in ipairs(G.edges) do
            if not e.rwy and (on_apron((e.x1 + e.x2) / 2, (e.y1 + e.y2) / 2) or on_apron(e.x1, e.y1) or on_apron(e.x2, e.y2)) then
                G.apron_tw[#G.apron_tw + 1] = e
            end
            geo_tick(#aprons)
        end
    end

    G.has_pave = #G.polys > 0
    apt.geo = G
end

-------------------------------------------------------------------------------
-- Gebaeude aus OpenStreetMap (Terminals cyan, andere Gebaeude blau)
-- Laden einmal je Flughafen im Hintergrund (curl, versteckt gestartet), danach aus dem Cache.
-- Daten (c) OpenStreetMap-Mitwirkende, ODbL.
-------------------------------------------------------------------------------
local IS_WIN = package.config:sub(1, 1) == "\\"
local DATA_DIR = SCRIPTS .. "A350_BTV_data/"
OSM = { busy = nil, t0 = 0, tries = {}, tiles_tried = {}, retries = {}, retry_at = {} }

function file_exists(p)
    local f = io.open(p, "rb")
    if f then f:close(); return true end
    return false
end

local function winpath(p) return (p:gsub("/", "\\")) end

-- kleiner JSON-Parser (gibt zwischendurch die Kontrolle ab)
local function json_decode(str)
    local pos = 1
    local byte, sub, find = string.byte, string.sub, string.find
    local ticks = 0
    local function ws()
        local _, e = find(str, "^[ \t\r\n]*", pos)
        pos = e + 1
    end
    local value
    local ESC = { n = "\n", t = "\t", r = "\r", b = "\b", f = "\f", ["/"] = "/", ["\\"] = "\\", ['"'] = '"' }
    local function str_val()
        local s = pos + 1
        local e = find(str, '"', s, true)
        if not e then error("JSON: unterminated string") end
        local raw = sub(str, s, e - 1)
        if not find(raw, "\\", 1, true) then
            pos = e + 1
            return raw
        end
        local buf, i = {}, s
        while true do
            local c = byte(str, i)
            if not c then error("JSON: end inside string") end
            if c == 34 then break end
            if c == 92 then
                local nx = sub(str, i + 1, i + 1)
                if nx == "u" then buf[#buf + 1] = "?"; i = i + 6
                else buf[#buf + 1] = ESC[nx] or nx; i = i + 2 end
            else
                buf[#buf + 1] = string.char(c); i = i + 1
            end
        end
        pos = i + 1
        return table.concat(buf)
    end
    function value()
        ticks = ticks + 1
        if ticks > 4000 then ticks = 0; coroutine.yield() end
        ws()
        local c = byte(str, pos)
        if c == 123 then -- {
            pos = pos + 1
            local obj = {}
            ws()
            if byte(str, pos) == 125 then pos = pos + 1; return obj end
            while true do
                ws()
                local k = str_val()
                ws()
                pos = pos + 1 -- :
                obj[k] = value()
                ws()
                local d = byte(str, pos)
                pos = pos + 1
                if d == 125 then return obj end
            end
        elseif c == 91 then -- [
            pos = pos + 1
            local arr = {}
            ws()
            if byte(str, pos) == 93 then pos = pos + 1; return arr end
            while true do
                arr[#arr + 1] = value()
                ws()
                local d = byte(str, pos)
                pos = pos + 1
                if d == 93 then return arr end
            end
        elseif c == 34 then
            return str_val()
        elseif c == 116 then pos = pos + 4; return true
        elseif c == 102 then pos = pos + 5; return false
        elseif c == 110 then pos = pos + 4; return nil
        else
            local s, e = find(str, "^-?[%d%.eE%+%-]+", pos)
            if not s then error("JSON: unexpected character at " .. pos) end
            pos = e + 1
            return tonumber(sub(str, s, e))
        end
    end
    return value()
end

local TERM_TAG = { terminal = true, airport_terminal = true }

-- Teilstuecke (offene Linien) zu geschlossenen Ringen verbinden
local function join_rings(parts)
    local rings, open = {}, {}
    local function closed(p) local n = #p.x; return n >= 4 and math.abs(p.x[1] - p.x[n]) < 0.05 and math.abs(p.y[1] - p.y[n]) < 0.05 end
    local function same(ax, ay, bx, by) return math.abs(ax - bx) < 0.05 and math.abs(ay - by) < 0.05 end
    for _, p in ipairs(parts) do
        if closed(p) then rings[#rings + 1] = p elseif #p.x >= 2 then open[#open + 1] = p end
    end
    while #open > 0 do
        local cur = table.remove(open, 1)
        local changed = true
        while changed and not closed(cur) do
            changed = false
            for i, q in ipairs(open) do
                local n, m = #cur.x, #q.x
                local nx, ny = {}, {}
                if same(cur.x[n], cur.y[n], q.x[1], q.y[1]) then
                    for k = 1, n do nx[#nx + 1], ny[#ny + 1] = cur.x[k], cur.y[k] end
                    for k = 2, m do nx[#nx + 1], ny[#ny + 1] = q.x[k], q.y[k] end
                elseif same(cur.x[n], cur.y[n], q.x[m], q.y[m]) then
                    for k = 1, n do nx[#nx + 1], ny[#ny + 1] = cur.x[k], cur.y[k] end
                    for k = m - 1, 1, -1 do nx[#nx + 1], ny[#ny + 1] = q.x[k], q.y[k] end
                elseif same(cur.x[1], cur.y[1], q.x[m], q.y[m]) then
                    for k = 1, m do nx[#nx + 1], ny[#ny + 1] = q.x[k], q.y[k] end
                    for k = 2, n do nx[#nx + 1], ny[#ny + 1] = cur.x[k], cur.y[k] end
                elseif same(cur.x[1], cur.y[1], q.x[1], q.y[1]) then
                    for k = m, 1, -1 do nx[#nx + 1], ny[#ny + 1] = q.x[k], q.y[k] end
                    for k = 2, n do nx[#nx + 1], ny[#ny + 1] = cur.x[k], cur.y[k] end
                end
                if #nx > 0 then
                    cur = { x = nx, y = ny }
                    table.remove(open, i)
                    changed = true
                    break
                end
            end
        end
        if closed(cur) then rings[#rings + 1] = cur end
    end
    return rings
end

local function ring_area(xs, ys)
    local a, n = 0, #xs
    for i = 1, n do
        local j = i % n + 1
        a = a + xs[i] * ys[j] - xs[j] * ys[i]
    end
    return math.abs(a) / 2
end

-- Douglas-Peucker fuer geschlossene Ringe (ohne doppelten Endpunkt)
local function simplify_ring(xs, ys, eps)
    local n = #xs
    if n < 4 then return xs, ys end
    -- Anker: Punkt 1 und der am weitesten entfernte Punkt
    local far, fd = 1, -1
    for i = 2, n do
        local d = (xs[i] - xs[1]) ^ 2 + (ys[i] - ys[1]) ^ 2
        if d > fd then far, fd = i, d end
    end
    local keep = { [1] = true, [far] = true }
    local e2 = eps * eps
    local stack = { { 1, far }, { far, n + 1 } }
    local function P(i) if i > n then i = i - n end return xs[i], ys[i] end
    while #stack > 0 do
        local seg = table.remove(stack)
        local a, b = seg[1], seg[2]
        if b - a > 1 then
            local ax, ay = P(a)
            local bx, by = P(b)
            local vx, vy = bx - ax, by - ay
            local L2 = vx * vx + vy * vy
            local bi, bd = nil, e2
            for i = a + 1, b - 1 do
                local px, py = P(i)
                local d
                if L2 > 0 then
                    local cr = (px - ax) * vy - (py - ay) * vx
                    d = cr * cr / L2
                else
                    d = (px - ax) ^ 2 + (py - ay) ^ 2
                end
                if d > bd then bi, bd = i, d end
            end
            if bi then
                keep[(bi > n) and (bi - n) or bi] = true
                stack[#stack + 1] = { a, bi }
                stack[#stack + 1] = { bi, b }
            end
        end
    end
    local ox, oy = {}, {}
    for i = 1, n do
        if keep[i] then ox[#ox + 1], oy[#oy + 1] = xs[i], ys[i] end
    end
    return ox, oy
end

-- Abstandstransformation (Chamfer 3-4) auf einem Raster: Abstand jeder Zelle zur naechsten Zelle mit m == want
local function chamfer(m, nx, ny, want, d)
    local INF = 1e9
    for i = 1, nx * ny do d[i] = (m[i] == want) and 0 or INF end
    for j = 0, ny - 1 do
        local row = j * nx
        for i = 0, nx - 1 do
            local k = row + i + 1
            local v = d[k]
            if v > 0 then
                if i > 0 and d[k - 1] + 3 < v then v = d[k - 1] + 3 end
                if j > 0 then
                    local u = k - nx
                    if d[u] + 3 < v then v = d[u] + 3 end
                    if i > 0 and d[u - 1] + 4 < v then v = d[u - 1] + 4 end
                    if i < nx - 1 and d[u + 1] + 4 < v then v = d[u + 1] + 4 end
                end
                d[k] = v
            end
        end
        geo_tick(nx)
    end
    for j = ny - 1, 0, -1 do
        local row = j * nx
        for i = nx - 1, 0, -1 do
            local k = row + i + 1
            local v = d[k]
            if v > 0 then
                if i < nx - 1 and d[k + 1] + 3 < v then v = d[k + 1] + 3 end
                if j < ny - 1 then
                    local u = k + nx
                    if d[u] + 3 < v then v = d[u] + 3 end
                    if i < nx - 1 and d[u + 1] + 4 < v then v = d[u + 1] + 4 end
                    if i > 0 and d[u - 1] + 4 < v then v = d[u - 1] + 4 end
                end
                d[k] = v
            end
        end
        geo_tick(nx)
    end
    return d
end

-- Wachsen (grow=true) oder Schrumpfen einer Maske um R Zellen; d und out sind wiederverwendete Puffer
local function morph(m, nx, ny, R, grow, d, out)
    local lim = R * 3 + 0.5
    if grow then
        chamfer(m, nx, ny, 1, d)
        for i = 1, nx * ny do out[i] = (d[i] <= lim) and 1 or 0 end
    else
        chamfer(m, nx, ny, 0, d)
        for i = 1, nx * ny do out[i] = (d[i] > lim) and 1 or 0 end
    end
    geo_tick(nx * ny / 2)
    return out
end

-- Umrisse einer Rastermaske verfolgen -> Ringe in Weltkoordinaten (aussen gegen den Uhrzeigersinn)
local function trace_mask(m, nx, ny, x0, y0, r)
    local function at(i, j)
        if i < 0 or j < 0 or i >= nx or j >= ny then return 0 end
        return m[j * nx + i + 1]
    end
    local W1 = nx + 2
    local outs = {}                        -- Startpunkt -> Liste von Kanten
    local edges = {}
    local function add(vx, vy, dir)
        local e = { vx = vx, vy = vy, dir = dir }
        edges[#edges + 1] = e
        local key = vy * W1 + vx
        local l = outs[key]
        if not l then l = {}; outs[key] = l end
        l[#l + 1] = e
    end
    for j = 0, ny - 1 do
        for i = 0, nx - 1 do
            if m[j * nx + i + 1] == 1 then
                if at(i, j - 1) == 0 then add(i, j, 0) end         -- unten: nach +x
                if at(i + 1, j) == 0 then add(i + 1, j, 1) end     -- rechts: nach +y
                if at(i, j + 1) == 0 then add(i + 1, j + 1, 2) end -- oben: nach -x
                if at(i - 1, j) == 0 then add(i, j + 1, 3) end     -- links: nach -y
            end
        end
        geo_tick(nx)
    end
    local DX, DY = { [0] = 1, 0, -1, 0 }, { [0] = 0, 1, 0, -1 }
    local rings = {}
    for _, e0 in ipairs(edges) do
        if not e0.used then
            local xs, ys = {}, {}
            local e = e0
            local guard = 0
            while e do
                e.used = true
                local ex, ey = e.vx + DX[e.dir], e.vy + DY[e.dir]
                if ex == e0.vx and ey == e0.vy then
                    if e0.dir ~= e.dir then xs[#xs + 1], ys[#ys + 1] = x0 + ex * r, y0 + ey * r end
                    break
                end
                -- naechste Kante: bevorzugt links abbiegen (diagonal beruehrende Zellen getrennt halten)
                local l = outs[ey * W1 + ex]
                local nxt
                if l then
                    for _, pref in ipairs({ (e.dir + 1) % 4, e.dir, (e.dir + 3) % 4 }) do
                        for _, c in ipairs(l) do
                            if not c.used and c.dir == pref then nxt = c; break end
                        end
                        if nxt then break end
                    end
                end
                if not nxt or nxt.dir ~= e.dir then
                    xs[#xs + 1], ys[#ys + 1] = x0 + ex * r, y0 + ey * r -- nur Ecken
                end
                e = nxt
                guard = guard + 1
                if guard > 400000 then break end
            end
            if #xs >= 3 then rings[#rings + 1] = { x = xs, y = ys } end
        end
    end
    return rings
end

local function signed_area(xs, ys)
    local a, n = 0, #xs
    for i = 1, n do
        local j = i % n + 1
        a = a + xs[i] * ys[j] - xs[j] * ys[i]
    end
    return a / 2
end

local function seg_cross(ax, ay, bx, by, cx, cy, dx, dy)
    local function o(px, py, qx, qy, rx, ry) return (qx - px) * (ry - py) - (qy - py) * (rx - px) end
    local d1, d2 = o(ax, ay, bx, by, cx, cy), o(ax, ay, bx, by, dx, dy)
    local d3, d4 = o(cx, cy, dx, dy, ax, ay), o(cx, cy, dx, dy, bx, by)
    return ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0))
end

-- Loecher ueber eine Bruecke in den Aussenring einbinden (fuer die Ohren-Triangulierung)
local function merge_holes(ox, oy, holes)
    table.sort(holes, function(a, b) return a.maxx > b.maxx end)
    for _, h in ipairs(holes) do
        local hx, hy = h.x, h.y
        local hi = 1
        for k = 2, #hx do if hx[k] > hx[hi] then hi = k end end
        local px, py = hx[hi], hy[hi]
        -- naechsten sichtbaren Punkt des Aussenrings suchen
        local cand = {}
        for k = 1, #ox do cand[#cand + 1] = { k, (ox[k] - px) ^ 2 + (oy[k] - py) ^ 2 } end
        table.sort(cand, function(a, b) return a[2] < b[2] end)
        local best
        for c = 1, math.min(#cand, 40) do
            local k = cand[c][1]
            local qx, qy = ox[k], oy[k]
            local ok = true
            local n = #ox
            for e = 1, n do
                local f = e % n + 1
                if e ~= k and f ~= k and seg_cross(px, py, qx, qy, ox[e], oy[e], ox[f], oy[f]) then ok = false; break end
            end
            if ok then
                local m = #hx
                for e = 1, m do
                    local f = e % m + 1
                    if e ~= hi and f ~= hi and seg_cross(px, py, qx, qy, hx[e], hy[e], hx[f], hy[f]) then ok = false; break end
                end
            end
            if ok then best = k; break end
        end
        if best then
            local nx2, ny2 = {}, {}
            for k = 1, best do nx2[#nx2 + 1], ny2[#ny2 + 1] = ox[k], oy[k] end
            local m = #hx
            for s = 0, m do
                local k = (hi - 1 + s) % m + 1
                nx2[#nx2 + 1], ny2[#ny2 + 1] = hx[k], hy[k]
            end
            nx2[#nx2 + 1], ny2[#ny2 + 1] = ox[best], oy[best]
            for k = best + 1, #ox do nx2[#nx2 + 1], ny2[#ny2 + 1] = ox[k], oy[k] end
            ox, oy = nx2, ny2
        end
    end
    return ox, oy
end

-- Nicht als Gebaeude zeigen: Tanks, Daecher/Ueberdachungen, Ruinen, Baustellen
local OSM_SKIP = { storage_tank = true, roof = true, ruins = true, construction = true, carport = true, shed = true,
                   container = true, tank = true, silo = true }

-- Overpass-Antwort in Kartenflaechen umwandeln.
-- Die Gebaeude werden gerastert (Zellen ca. 2 m), kleine Luecken geschlossen, duenne Teile (Fluggastbruecken,
-- Spitzen) entfernt und die Umrisse vereinfacht: grobe, ruhige Flaechen wie im OANS.
-- Antwort(en) lesen: mehrere JSON-Antworten (Kacheln) mit Trenner "@@BTVPART@@", doppelte Elemente nur einmal
local function osm_parse(txt)
    local elements, seen, any = {}, {}, false
    for part in (txt .. "@@BTVPART@@"):gmatch("(.-)@@BTVPART@@") do
        if part:find("%S") then
            local ok, data = pcall(json_decode, part)
            if ok and type(data) == "table" and type(data.elements) == "table" then
                any = true
                for _, el in ipairs(data.elements) do
                    local k = tostring(el.type) .. tostring(el.id)
                    if el.id == nil or not seen[k] then
                        seen[k] = true
                        elements[#elements + 1] = el
                    end
                end
            end
        end
    end
    if not any then return nil end
    return { elements = elements }
end

local function osm_build(apt, txt)
    local G = apt.geo
    local data = osm_parse(txt)
    if not data then
        return nil, "response not readable"
    end
    local W = G.W
    local function geom_xy(geom)
        local xs, ys = {}, {}
        for _, p in ipairs(geom or {}) do
            if p.lat and p.lon then
                local x, y = W(p.lat, p.lon)
                xs[#xs + 1], ys[#ys + 1] = x, y
            end
        end
        return { x = xs, y = ys }
    end
    -- 1) Rohumrisse sammeln
    local polys = {}
    local cnt = 0
    for _, el in ipairs(data.elements) do
        local tags = el.tags or {}
        local bt = tags.building
        local is_term = (tags.aeroway == "terminal") or TERM_TAG[bt or ""]
        local skip = OSM_SKIP[bt or ""] or (tags.man_made ~= nil and not is_term and bt ~= "hangar" and tags.aeroway ~= "hangar")
        local is_bldg = is_term or (bt ~= nil and bt ~= "no") or tags["building:part"] ~= nil
        if is_bldg and (is_term or not skip) then
            local rings
            if el.type == "way" then
                rings = join_rings({ geom_xy(el.geometry) })
            elseif el.type == "relation" then
                local parts = {}
                for _, m in ipairs(el.members or {}) do
                    if m.type == "way" and (m.role == "outer" or m.role == "") then parts[#parts + 1] = geom_xy(m.geometry) end
                end
                rings = join_rings(parts)
            end
            for _, r in ipairs(rings or {}) do
                if #r.x >= 4 and ring_area(r.x, r.y) >= 150 then
                    local minx, miny, maxx, maxy = math.huge, math.huge, -math.huge, -math.huge
                    for k = 1, #r.x do
                        minx, maxx = math.min(minx, r.x[k]), math.max(maxx, r.x[k])
                        miny, maxy = math.min(miny, r.y[k]), math.max(maxy, r.y[k])
                    end
                    polys[#polys + 1] = { x = r.x, y = r.y, term = is_term and true or false, minx = minx, miny = miny, maxx = maxx, maxy = maxy }
                end
            end
        end
        cnt = cnt + 1
        if cnt % 50 == 0 then coroutine.yield() end
    end

    -- 2) benachbarte Gebaeude zu Gruppen zusammenfassen (je Gruppe ein Raster)
    local n = #polys
    local parent = {}
    for i = 1, n do parent[i] = i end
    local function find(i) while parent[i] ~= i do parent[i] = parent[parent[i]]; i = parent[i] end return i end
    local M = 8
    for i = 1, n do
        local a = polys[i]
        for j = i + 1, n do
            local b = polys[j]
            if a.minx - M < b.maxx and b.minx - M < a.maxx and a.miny - M < b.maxy and b.miny - M < a.maxy then
                parent[find(i)] = find(j)
            end
        end
        if i % 40 == 0 then geo_tick(n * 40) end
    end
    local groups = {}
    for i = 1, n do
        local r = find(i)
        groups[r] = groups[r] or {}
        table.insert(groups[r], polys[i])
    end

    -- 3) je Gruppe: rastern, schliessen, oeffnen, Umriss verfolgen, vereinfachen, triangulieren
    local B = { terms = {}, bldg = {}, raw = n }
    local CLOSE_M, OPEN_M = 4.0, 4.5
    -- Rasterpuffer (wiederverwendet, damit nicht staendig grosse Tabellen entstehen)
    local DB = {}
    local BUF = { [true] = { {}, {} }, [false] = { {}, {} } }
    for _, grp in pairs(groups) do
        local minx, miny, maxx, maxy = math.huge, math.huge, -math.huge, -math.huge
        for _, p in ipairs(grp) do
            minx, maxx = math.min(minx, p.minx), math.max(maxx, p.maxx)
            miny, maxy = math.min(miny, p.miny), math.max(maxy, p.maxy)
        end
        local pad = 10
        minx, miny, maxx, maxy = minx - pad, miny - pad, maxx + pad, maxy + pad
        local r = 2.0
        local area = (maxx - minx) * (maxy - miny)
        if area / (r * r) > 350000 then r = math.sqrt(area / 350000) end
        local nx, ny = math.ceil((maxx - minx) / r), math.ceil((maxy - miny) / r)
        local masks = {}
        for _, want_term in ipairs({ true, false }) do
            local b1, b2 = BUF[want_term][1], BUF[want_term][2]
            local m = b1
            for i = 1, nx * ny do m[i] = 0 end
            local any = false
            for _, p in ipairs(grp) do
                if p.term == want_term then
                    any = true
                    -- Scanline-Fuellung (Zellmittelpunkte)
                    local xs, ys = p.x, p.y
                    local pn = #xs
                    local j0 = math.max(0, math.floor((p.miny - miny) / r - 0.5))
                    local j1 = math.min(ny - 1, math.ceil((p.maxy - miny) / r - 0.5))
                    for j = j0, j1 do
                        local yc = miny + (j + 0.5) * r
                        local hits = {}
                        local q = pn
                        for k = 1, pn do
                            local y1, y2 = ys[q], ys[k]
                            if (y1 > yc) ~= (y2 > yc) then
                                hits[#hits + 1] = xs[q] + (yc - y1) / (y2 - y1) * (xs[k] - xs[q])
                            end
                            q = k
                        end
                        table.sort(hits)
                        for h = 1, #hits - 1, 2 do
                            local i0 = math.max(0, math.ceil((hits[h] - minx) / r - 0.5))
                            local i1 = math.min(nx - 1, math.floor((hits[h + 1] - minx) / r - 0.5))
                            local row = j * nx + 1
                            for i = i0, i1 do m[row + i] = 1 end
                        end
                    end
                    geo_tick(pn * (j1 - j0 + 1) / 4 + 50)
                end
            end
            if any then
                local rc, ro = CLOSE_M / r, OPEN_M / r
                morph(b1, nx, ny, rc, true, DB, b2)
                morph(b2, nx, ny, rc, false, DB, b1)
                morph(b1, nx, ny, ro, false, DB, b2)
                morph(b2, nx, ny, ro, true, DB, b1)
                m = b1
            end
            masks[want_term] = any and m or nil
        end
        -- Gebaeude nicht unter Terminals legen
        if masks[true] and masks[false] then
            local mt, mb = masks[true], masks[false]
            for i = 1, nx * ny do if mt[i] == 1 then mb[i] = 0 end end
        end
        for _, want_term in ipairs({ true, false }) do
            local m = masks[want_term]
            if m then
                local rings = trace_mask(m, nx, ny, minx, miny, r)
                local outers, holes = {}, {}
                for _, rg in ipairs(rings) do
                    local sx, sy = simplify_ring(rg.x, rg.y, r * 1.25)
                    if #sx >= 3 then
                        local a = signed_area(sx, sy)
                        local hmax = -math.huge
                        for k = 1, #sx do if sx[k] > hmax then hmax = sx[k] end end
                        if a >= 250 then outers[#outers + 1] = { x = sx, y = sy, a = a }
                        elseif a <= -500 then holes[#holes + 1] = { x = sx, y = sy, maxx = hmax } end
                    end
                end
                for _, o in ipairs(outers) do
                    local mine = {}
                    for _, h in ipairs(holes) do
                        if not h.used and point_in_ring(o.x, o.y, h.x[1], h.y[1]) then mine[#mine + 1] = h; h.used = true end
                    end
                    local fx, fy = o.x, o.y
                    if #mine > 0 then fx, fy = merge_holes(fx, fy, mine) end
                    local t = triangulate(fx, fy)
                    if #t > 0 then
                        local bx0, by0, bx1, by1 = math.huge, math.huge, -math.huge, -math.huge
                        for k = 1, #o.x do
                            bx0, bx1 = math.min(bx0, o.x[k]), math.max(bx1, o.x[k])
                            by0, by1 = math.min(by0, o.y[k]), math.max(by1, o.y[k])
                        end
                        local item = { tris = tri_chunks(t), minx = bx0, miny = by0, maxx = bx1, maxy = by1, ntri = #t / 6 }
                        if want_term then B.terms[#B.terms + 1] = item else B.bldg[#B.bldg + 1] = item end
                    end
                end
            end
        end
        coroutine.yield()
    end
    return B
end

local function osm_file(icao) return DATA_DIR .. "osm2_" .. icao .. ".json" end
local function osm_file_old(icao) return DATA_DIR .. "osm_" .. icao .. ".json" end

-- Hintergrundprogramm ohne Konsolenfenster starten (Windows: WinExec mit SW_HIDE, sonst wie bisher os.execute)
local function run_hidden(cmd)
    local ok = pcall(function()
        local f = require("ffi")
        pcall(f.cdef, [[ unsigned int btv_winexec(const char *, unsigned int) __asm__("WinExec"); ]])
        local r = f.load("kernel32").btv_winexec(cmd, 0)
        assert(r > 31)
    end)
    if not ok then os.execute(cmd) end
end
OSM.run_hidden = run_hidden    -- (auch fuer das Leuchtbild der A/BRK-Taste, PNL.abrk_lit)

-- Download im Hintergrund starten (mode "area" = nur Flughafengelaende, "bbox" = Rechteck)
local OSM_UA = "A350-BTV/" .. VERSION .. " X-Plane-FlyWithLua-airport-map"
local function osm_start(apt, mode)
    local G = apt.geo
    local icao = apt.icao
    local R = 6371000
    local function ll(x, y)
        return G.ref.lat + y / (R * D2R), G.ref.lon + x / (R * D2R * math.cos(G.ref.lat * D2R))
    end
    -- Abfragen: "area" = Flughafengelaende (eine Anfrage), "tiles" = Rechteck um den Flughafen in Kacheln
    -- von hoechstens ~2,5 km (kleine Anfragen kommen auch bei ueberlasteten Servern durch)
    local queries = {}
    if mode == "area" then
        queries[1] = string.format('[out:json][timeout:120];area["aeroway"="aerodrome"]["icao"="%s"]->.a;(way["building"](area.a);' ..
            'way["aeroway"="terminal"](area.a);)->.w;(rel["building"](area.a);rel["aeroway"="terminal"](area.a);)->.r;' ..
            '.w out tags geom;.r out body geom;', icao)
    else
        local x0, y0, x1, y1 = G.minx - 500, G.miny - 500, G.maxx + 500, G.maxy + 500
        local nx, ny = math.max(1, math.ceil((x1 - x0) / 2500)), math.max(1, math.ceil((y1 - y0) / 2500))
        for i = 0, nx - 1 do
            for j = 0, ny - 1 do
                local s1, w1 = ll(x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny)
                local n1, e1 = ll(x0 + (x1 - x0) * (i + 1) / nx, y0 + (y1 - y0) * (j + 1) / ny)
                local bb = string.format("%.5f,%.5f,%.5f,%.5f", s1, w1, n1, e1)
                queries[#queries + 1] = string.format('[out:json][timeout:90];(way["aeroway"="terminal"](%s);way["building"](%s);)->.w;' ..
                    '(rel["building"](%s);rel["aeroway"="terminal"](%s);)->.r;.w out tags geom;.r out body geom;', bb, bb, bb, bb)
            end
        end
    end
    local out = osm_file(icao)
    local tmp = out .. ".tmp"
    local errf = out .. ".err"
    local newf = out .. ".new" -- fertige Antwort; ersetzt den Cache erst nach erfolgreichem Download
    local logf = out .. ".log"   -- Antworten/Fehler von curl (Diagnose)
    os.remove(errf)
    os.remove(newf)
    os.remove(logf)
    local qfiles, parts = {}, {}
    for i, q in ipairs(queries) do
        qfiles[i] = SCRIPTS .. "A350_BTV_osm_query" .. ((i > 1) and tostring(i) or "") .. ".txt"
        parts[i] = out .. ".part" .. i
        local f = io.open(qfiles[i], "w")
        if not f then return false end
        f:write(q)
        f:close()
        os.remove(parts[i])
    end
    -- vier oeffentliche Overpass-Server, kurze Wiederholung bei "ueberlastet"; Teil-Downloads werden verworfen
    -- (maps.mail.ru antwortet oft, wenn overpass-api.de mit 502 ablehnt)
    local URLS = { "https://overpass-api.de/api/interpreter", "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
                   "https://overpass.private.coffee/api/interpreter", "https://overpass.kumi.systems/api/interpreter" }
    local CURL_OPT = '-sS -f -m 150 --retry 2 --retry-delay 10 --retry-max-time 240 -A "' .. OSM_UA .. '"'
    if IS_WIN then
        local bat = SCRIPTS .. "A350_BTV_osm.bat"
        local vbs = SCRIPTS .. "A350_BTV_osm.vbs"
        local bf = io.open(bat, "w")
        if not bf then return false end
        local T, D, L = winpath(tmp), winpath(DATA_DIR):gsub("\\$", ""), winpath(logf)
        bf:write("@echo off\r\n")
        bf:write(string.format('if not exist "%s" mkdir "%s"\r\n', D, D))
        bf:write(string.format('del /q "%s" 2>nul\r\n', T))
        -- alle drei Server gleichzeitig fragen (je eine eigene kleine Batch-Datei im Hintergrund), die erste
        -- vollstaendige Antwort gilt - deutlich schneller als nacheinander, wenn ein Server langsam ist
        for i = 1, #queries do
            local P, Q = winpath(parts[i]), winpath(qfiles[i])
            for k, u in ipairs(URLS) do
                local Pk = P .. "." .. k
                local job = SCRIPTS .. "A350_BTV_osm_job" .. i .. "_" .. k .. ".bat"
                local jf = io.open(job, "w")
                if not jf then return false end
                jf:write("@echo off\r\n")
                jf:write(string.format('curl.exe %s -w " [%%%%{url_effective} HTTP %%%%{http_code} %%%%{time_total}s]" ' ..
                    '--data-urlencode "data@%s" -o "%s" "%s" >>"%s" 2>&1\r\n', CURL_OPT, Q, Pk, u, L))
                -- (ohne Klammerbloecke: Pfade mit Klammern, z.B. "Program Files (x86)", wuerden sie zerbrechen)
                jf:write("if errorlevel 1 goto bad\r\n")
                jf:write(string.format('ren "%s" "%s"\r\nexit /b\r\n:bad\r\n', Pk, (Pk .. ".ok"):match("[^\\]+$")))
                jf:write(string.format('del /q "%s" 2>nul\r\necho x> "%s.bad"\r\n', Pk, Pk))
                jf:close()
                bf:write(string.format('del /q "%s" "%s.ok" "%s.bad" 2>nul\r\n', Pk, Pk, Pk))
                -- (mit cmd /c: "start" allein startet die Batch-Datei mit cmd /K - das Fenster bliebe unsichtbar offen,
                -- drei je Flughafen, und Steam haelt X-Plane danach fuer "laeuft noch")
                bf:write(string.format('start "" /b cmd /c "%s"\r\n', winpath(job)))
            end
            bf:write("set /a n=0\r\n")
            bf:write(":warten" .. i .. "\r\n")
            for k = 1, #URLS do bf:write(string.format('if exist "%s.%d.ok" goto nimm%d_%d\r\n', P, k, i, k)) end
            local allbad = {}
            for k = 1, #URLS do allbad[k] = string.format('if exist "%s.%d.bad" ', P, k) end
            bf:write(table.concat(allbad) .. "goto fehler\r\n")
            bf:write("set /a n+=1\r\n")
            bf:write("if %n% geq 600 goto fehler\r\n")
            bf:write("ping -n 2 127.0.0.1 >nul\r\n")
            bf:write("goto warten" .. i .. "\r\n")
            for k = 1, #URLS do
                bf:write(string.format(':nimm%d_%d\r\nmove /y "%s.%d.ok" "%s" >nul\r\ngoto da%d\r\n', i, k, P, k, P, i))
            end
            bf:write(":da" .. i .. "\r\n")
        end
        -- alle Teile da: zusammenfuegen (Trenner zwischen den JSON-Antworten)
        bf:write(string.format('type nul > "%s"\r\n', T))
        for i = 1, #queries do
            bf:write(string.format('type "%s" >> "%s"\r\n', winpath(parts[i]), T))
            bf:write(string.format('echo @@BTVPART@@>> "%s"\r\n', T))
        end
        bf:write(string.format('move /y "%s" "%s" >nul\r\n', T, winpath(newf)))
        bf:write('goto ende\r\n:fehler\r\n')
        bf:write(string.format('echo fehler > "%s"\r\n', winpath(errf)))
        bf:write(':ende\r\n')
        for i = 1, #queries do
            bf:write(string.format('del /q "%s" "%s.*.bad" "%s.*.ok" 2>nul\r\n', winpath(parts[i]), winpath(parts[i]), winpath(parts[i])))
        end
        bf:close()
        local vf = io.open(vbs, "w")
        if not vf then return false end
        vf:write(string.format('CreateObject("WScript.Shell").Run """%s""", 0, False\r\n', winpath(bat)))
        vf:close()
        run_hidden(string.format('wscript.exe "%s"', winpath(vbs)))
    else
        local steps = { string.format('mkdir -p "%s"; rm -f "%s"', DATA_DIR, tmp) }
        local okc = {}
        for i = 1, #queries do
            local tries = {}
            for _, u in ipairs(URLS) do
                tries[#tries + 1] = string.format('curl %s -w " [%%{url_effective} HTTP %%{http_code} %%{time_total}s]" ' ..
                    '--data-urlencode "data@%s" -o "%s" "%s" >>"%s" 2>&1', CURL_OPT, qfiles[i], parts[i], u, logf)
            end
            okc[#okc + 1] = "(" .. table.concat(tries, " || ") .. ")"
        end
        local join = {}
        for i = 1, #queries do join[#join + 1] = string.format('cat "%s" >> "%s"; echo "@@BTVPART@@" >> "%s"', parts[i], tmp, tmp) end
        local rm = {}
        for i = 1, #queries do rm[#rm + 1] = string.format('"%s"', parts[i]) end
        local sh = steps[1] .. "; if " .. table.concat(okc, " && ") .. "; then " .. table.concat(join, "; ") ..
            string.format('; mv "%s" "%s"; else echo fehler > "%s"; fi; rm -f %s', tmp, newf, errf, table.concat(rm, " "))
        os.execute("(" .. sh .. ") >/dev/null 2>&1 &")
    end
    log(string.format("OSM download: %d request(s)", #queries))
    OSM.busy = { icao = icao, mode = mode, t0 = os.time() }
    S.osm_status = "Loading buildings from OpenStreetMap ..."
    log("OSM download started: " .. icao .. " (" .. mode .. ")")
    return true
end

local function osm_load_file(apt, path, mode)
    apt.osm_pending = true
    add_job("osm " .. apt.icao, function()
        local okj, ej = pcall(function()
        local f = io.open(path, "rb")
        if not f then return end
        local txt = f:read("*a")
        f:close()
        local B, err = osm_build(apt, txt)
        if not B then
            S.osm_status = "OSM data invalid (" .. tostring(err) .. ")"
            os.remove(path)
            return
        end
        if #B.terms + #B.bldg == 0 and mode ~= "tiles" and not OSM.tiles_tried[apt.icao] then
            -- Flughafengelaende in OSM nicht gefunden (auch aus Cache/Vorab-Download) -> Rechteck um die Pisten
            -- (in Kacheln), einmal je Sitzung
            OSM.tiles_tried[apt.icao] = true
            os.remove(path)
            if not OSM.busy then osm_start(apt, "tiles") end
            return
        end
        apt.geo.osm = B
        S.osm_status = string.format("%d terminal(s), %d buildings from OpenStreetMap", #B.terms, #B.bldg)
        log("OSM " .. apt.icao .. ": " .. S.osm_status)
        end)
        apt.osm_pending = false
        if not okj then error(ej, 0) end
    end)
end

-- nach dem Laden eines Flughafens aufrufen
function osm_attach(apt)
    if not CFG.osm or not apt.geo then return end
    local path = osm_file(apt.icao)
    if file_exists(path) then
        osm_load_file(apt, path, "cache")
        return
    end
    -- Cache einer aelteren Version (ohne Multipolygone): zeigen, bis der neue Download da ist
    local old = osm_file_old(apt.icao)
    if file_exists(old) then osm_load_file(apt, old, "cache") end
    if not OSM.busy and not OSM.tries[apt.icao] then
        OSM.tries[apt.icao] = true
        osm_start(apt, "area")
    end
end

-- Wartet die Karte noch auf die Gebaeude? Wie im A350 kurz "ARPT MAP LOADING" (hoechstens 5 s nach dem Laden des
-- Flughafens); laeuft der erste Download laenger, kommt die Karte ohne Gebaeude und die Gebaeude erscheinen danach.
function btv_osm_waiting(apt)
    if not CFG.osm or not apt or not apt.geo or apt.geo.osm then return false end
    if os.time() - (apt.t_loaded or 0) > 5 then return false end
    if apt.osm_pending then return true end
    return OSM.busy ~= nil and OSM.busy.icao == apt.icao
end

-- Gebaeude der Flughaefen im Flugplan (TO, ALTN, FROM) vorab im Hintergrund laden, einer nach dem anderen -
-- bei der Ankunft liegen sie dann schon im Cache. Der gerade geladene Flughafen geht immer vor.
function OSM.prefetch()
    if not CFG.osm or OSM.busy then return end
    for ic, t in pairs(OSM.retry_at) do
        if os.time() >= t then OSM.retry_at[ic], OSM.tries[ic] = nil, nil end
    end
    local a = S.apt
    if a and a.geo and not a.geo.osm and not a.osm_pending and not OSM.tries[a.icao] then osm_attach(a); return end
    for _, ic in ipairs({ FMS.dest, FMS.altn, FMS.from }) do
        if type(ic) == "string" and ic:match("^%u%u%u%u$") and not OSM.tries[ic]
            and not file_exists(osm_file(ic)) then
            OSM.tries[ic] = true
            log("OSM prefetch for the flight plan: " .. ic)
            osm_start({ icao = ic }, "area")
            return
        end
    end
end

-- jeden Frame: fertigen Download erkennen
local osm_poll_n = 0
function osm_poll()
    osm_poll_n = osm_poll_n + 1
    if osm_poll_n < 40 then return end
    osm_poll_n = 0
    if not OSM.busy then
        local okp, e = pcall(OSM.prefetch)
        if not okp and not OSM.pf_err then OSM.pf_err = true; log("OSM prefetch: " .. tostring(e)) end
        return
    end
    local b = OSM.busy
    local path = osm_file(b.icao)
    if file_exists(path .. ".new") then
        OSM.busy = nil
        os.remove(path)
        os.rename(path .. ".new", path)
        local old = osm_file_old(b.icao)
        for _, x in ipairs({ "", ".log", ".err" }) do os.remove(old .. x) end
        if S.apt and S.apt.icao == b.icao then osm_load_file(S.apt, path, b.mode) end
    elseif file_exists(path .. ".err") or os.time() - b.t0 > ((b.mode == "tiles") and 2400 or 900) then
        OSM.busy = nil
        -- Antworten der Server ins Log (z.B. HTTP 429 = zu viele Anfragen, 504 = Server ueberlastet)
        local lf = io.open(path .. ".log", "r")
        if lf then
            local t = (lf:read("*a") or ""):gsub("[\r\n]+", " | ")
            lf:close()
            if #t > 400 then t = t:sub(-400) end
            log("OSM server: " .. t)
        end
        -- grosse Flughaefen: die Abfrage "Flughafengebiet" ist dem Server manchmal zu schwer - dann Rechteck um die Bahnen
        if b.mode == "area" and S.apt and S.apt.icao == b.icao and S.apt.geo then
            log("OSM download (airport area) failed: " .. b.icao .. " - second attempt in tiles")
            if osm_start(S.apt, "tiles") then return end
        end
        S.osm_status = "Building download failed (no internet / server busy)"
        log("OSM download failed: " .. b.icao)
        -- oeffentliche Server oft kurz ueberlastet: von selbst neu versuchen (nach 1, 2 und 4 min), damit die
        -- Gebaeude bis zum Rollen da sind
        local n = (OSM.retries[b.icao] or 0) + 1
        OSM.retries[b.icao] = n
        if n <= 3 then
            OSM.retry_at[b.icao] = os.time() + 60 * 2 ^ (n - 1)
            log(string.format("OSM %s: new attempt in %d min", b.icao, 2 ^ (n - 1)))
        end
    end
end



end

local function load_airport(entry, manual)
    -- Belagsflaeche [m^2] (Aussenringe, Bezier-Kontrollpunkte ignoriert) - zum Vergleich der Quellen
    local function pave_area(polys)
        if polys.area then return polys.area end
        local sum = 0
        for _, p in ipairs(polys) do
            local r = p.rings[1]
            if r and #r >= 3 then
                local kx = math.cos(r[1].lat * D2R) * D2R * R_EARTH
                local ky = D2R * R_EARTH
                local a, n = 0, #r
                for i = 1, n do
                    local q, w = r[i], r[(i % n) + 1]
                    a = a + (q.lon * kx) * (w.lat * ky) - (w.lon * kx) * (q.lat * ky)
                end
                sum = sum + math.abs(a) / 2
            end
        end
        polys.area = sum
        return sum
    end
    if S.locked and S.locked() and S.apt then message("NOT AVAIL DURING BTV BRAKING"); return end
    add_job("load " .. entry.icao, function()
        S.status = "Loading " .. entry.icao .. " ..."
        local apt = parse_airport_block(entry)
        if not apt then S.status = "Error loading " .. entry.icao; return end
        -- Taxi-Netzwerke: eigenes zuerst, dann andere Quellen mit gleicher Pistengeometrie
        -- (manche Szenerien haben kein oder nur ein unvollstaendiges Netzwerk)
        apt.nets = {}
        if apt.n_nodes > 0 then
            apt.nets[1] = { nodes = apt.nodes, edges = apt.edges, n_nodes = apt.n_nodes, src = entry.file }
        end
        if #apt.roads > 0 then apt.road_net = { nodes = apt.nodes, roads = apt.roads } end
        local alt_pave, alt_lines
        for _, alt in ipairs(IDX.by_icao[entry.icao] or {}) do
            if alt ~= entry then
                local a2 = parse_airport_block(alt)
                if a2 and runways_match(apt, a2) then
                    if a2.n_nodes > 0 then
                        apt.nets[#apt.nets + 1] = { nodes = a2.nodes, edges = a2.edges, n_nodes = a2.n_nodes, src = alt.file }
                    end
                    if not alt_pave or pave_area(a2.polys) > pave_area(alt_pave) then alt_pave = a2.polys end
                    if not alt_lines or #a2.lines > #alt_lines then alt_lines = a2.lines end
                    if not apt.road_net and #a2.roads > 0 then apt.road_net = { nodes = a2.nodes, roads = a2.roads } end
                end
            end
        end
        -- Szenerie ohne (ausreichende) Flaechen-/Liniendaten: aus der Standardquelle ergaenzen.
        -- Massstab ist die Belagsflaeche, nicht die Anzahl: manche Szenerien (z.B. VHHH Taimodels) haben
        -- Dutzende Mini-Flicken und zeichnen den Rest als Objekte/Orthofoto -> ohne grauen Untergrund.
        if alt_pave and (#apt.polys < 10 and #alt_pave > #apt.polys
                         or pave_area(alt_pave) > 2 * pave_area(apt.polys)) then
            log(string.format("%s: pavement from another source (%.2f vs %.2f km2)", entry.icao,
                pave_area(alt_pave) / 1e6, pave_area(apt.polys) / 1e6))
            apt.polys = alt_pave
        end
        if #apt.lines < 20 and alt_lines and #alt_lines > #apt.lines then apt.lines = alt_lines end
        apt.nodes, apt.edges = nil, nil
        apt.n_nodes = 0
        for _, n in ipairs(apt.nets) do apt.n_nodes = math.max(apt.n_nodes, n.n_nodes) end
        prepare_runway_ends(apt)
        build_geo(apt)
        apt.t_loaded = os.time()
        S.apt, S.apt_manual, S.apt_fresh = apt, manual, true
        S.icao_input = apt.icao
        S.osm_status = nil
        osm_attach(apt)
        S.rwy, S.rwy_manual, S.exits, S.exit, S.manual_dist, S.map = nil, false, {}, nil, nil, nil
        S.status = ""
        log(string.format("%s loaded: %d runway ends, %d taxi network(s), %d pavement sections, %d stands",
            apt.icao, #apt.ends, #apt.nets, #apt.geo.pave, #apt.geo.stands))
    end)
end

-------------------------------------------------------------------------------
-- Exit-Erkennung
-------------------------------------------------------------------------------
local function compute_exits_net(net, E)
    local exits = {}
    local inner = math.max(E.width * 0.5, 18)
    local apt = net
    local cache = {}
    local function sd(id)
        local c = cache[id]
        if c then return c end
        local nd = apt.nodes[id]
        if not nd then return nil end
        local s, d = rwy_sd(E, nd.lat, nd.lon)
        c = { s = s, d = d, inside = (math.abs(d) < inner and s > -30 and s < E.len + 30) }
        cache[id] = c
        return c
    end
    -- Nachbarschaftsliste (einmal je Netzwerk)
    local adj = net.adj
    if not adj then
        adj = {}
        for k, e in ipairs(net.edges) do
            if not e.rwy then
                adj[e.a] = adj[e.a] or {}; table.insert(adj[e.a], k)
                adj[e.b] = adj[e.b] or {}; table.insert(adj[e.b], k)
            end
        end
        net.adj = adj
    end
    -- Mittellinie eines Exits verfolgen: vom Pistenknoten den Rollweg entlang (max. ~450 m)
    local function trace(inn_id, out_id, name)
        local A, B = sd(inn_id), sd(out_id)
        local pts = { { A.s, A.d }, { B.s, B.d } }
        local prev, cur = inn_id, out_id
        local ds, dd = B.s - A.s, B.d - A.d
        local L = math.sqrt(ds * ds + dd * dd)
        local total = L
        if L > 0 then ds, dd = ds / L, dd / L end
        for _ = 1, 40 do
            if total > 600 then break end
            local C0 = sd(cur)
            local best, bscore, bnext
            for _, k in ipairs(adj[cur] or {}) do
                local e = net.edges[k]
                local nxt = (e.a == cur) and e.b or e.a
                if nxt ~= prev then
                    local C = sd(nxt)
                    if C and not C.inside then
                        local cs, cd = C.s - C0.s, C.d - C0.d
                        local cl = math.sqrt(cs * cs + cd * cd)
                        if cl > 0.5 then
                            local cosang = (cs * ds + cd * dd) / cl
                            -- nur auf dem gleichen Rollweg weiter (Ende des Exits = Namenswechsel);
                            -- ohne Namen nur geradeaus und hoechstens ~200 m
                            local ok
                            if name ~= "--" then ok = (e.name == name) and cosang > 0.2
                            else ok = cosang > 0.7 and total < 200 end
                            if ok and (not bscore or cosang > bscore) then
                                best, bscore, bnext = C, cosang, nxt
                            end
                        end
                    end
                end
            end
            if not best then break end
            local cs, cd = best.s - C0.s, best.d - C0.d
            local cl = math.sqrt(cs * cs + cd * cd)
            pts[#pts + 1] = { best.s, best.d }
            total = total + cl
            ds, dd = cs / cl, cd / cl
            prev, cur = cur, bnext
        end
        return pts
    end
    -- Namen am aeusseren Knoten fuer unbenannte Kanten
    local names_at = {}
    for _, e in ipairs(apt.edges) do
        if e.name ~= "" and not e.rwy then
            names_at[e.a] = names_at[e.a] or e.name
            names_at[e.b] = names_at[e.b] or e.name
        end
    end
    for _, e in ipairs(apt.edges) do
        if not e.rwy then
            local A, B = sd(e.a), sd(e.b)
            if A and B and A.inside ~= B.inside then
                local inn, out, out_id, inn_id = A, B, e.b, e.a
                if B.inside then inn, out, out_id, inn_id = B, A, e.a, e.b end
                -- Einbahn-Kanten nur in Richtung von der Piste weg
                if not (e.oneway and B.inside) then
                    local ds, dd = out.s - inn.s, out.d - inn.d
                    local ang = math.atan2(math.abs(dd), ds) / D2R
                    if ang <= 115 then
                        local name = e.name ~= "" and e.name or names_at[out_id] or "--"
                        local L = math.sqrt(ds * ds + dd * dd)
                        exits[#exits + 1] = { name = name, s = inn.s, ang = ang,
                            ret = ang < 60, side = dd > 0 and "L" or "R",
                            od_s = L > 0 and ds / L or 0, od_d = L > 0 and dd / L or 0,
                            path = trace(inn_id, out_id, name) }
                    end
                end
            end
        end
    end
    table.sort(exits, function(a, b) return a.s < b.s end)
    -- Duplikate zusammenfassen
    local res = {}
    for _, x in ipairs(exits) do
        local last = res[#res]
        if last and last.name == x.name and math.abs(x.s - last.s) < 60 then
            -- derselbe Rollweg auf der anderen Seite: zweiter Weg (Abrollen nach links und rechts)
            if x.side ~= last.side0 and not last.alt then last.alt = { x } end
            if x.side ~= last.side then last.side = "LR" end
            if x.ang < last.ang then last.ang, last.ret = x.ang, x.ret end
        elseif x.s - E.disp >= 400 and x.s <= E.len + 10 then
            x.side0 = x.side
            res[#res + 1] = x
        end
    end
    local hw = E.width / 2
    local function prep(x)
        local p = x.path
        local cum = { 0 }
        x.t0 = nil
        for i = 2, #p do
            local a, b = p[i - 1], p[i]
            local l = math.sqrt((b[1] - a[1]) ^ 2 + (b[2] - a[2]) ^ 2)
            cum[i] = cum[i - 1] + l
            if not x.t0 and math.abs(b[2]) >= hw then
                local da, db = math.abs(a[2]), math.abs(b[2])
                local f = (db > da) and clamp((hw - da) / (db - da), 0, 1) or 0
                x.t0 = cum[i - 1] + f * l
            end
        end
        x.cum, x.plen, x.t0 = cum, cum[#cum], x.t0 or 0
    end
    for _, x in ipairs(res) do
        x.dist = x.s - E.disp
        prep(x)
        for _, a in ipairs(x.alt or {}) do prep(a) end
    end
    return res
end

-- Erstes Netzwerk, das fuer diese Piste brauchbare Exits liefert
local function compute_exits(apt, E)
    local best, best_net = {}, nil
    if not apt then return best end
    for _, net in ipairs(apt.nets) do
        local ex = compute_exits_net(net, E)
        if #ex >= 2 then return ex, net.src, net end
        if #ex > #best then best, best_net = ex, net end
    end
    if not best_net then best_net = apt.nets[1] end
    return best, best_net and best_net.src, best_net
end


-------------------------------------------------------------------------------
-- Pistenzustand
-------------------------------------------------------------------------------
-- Aufsetzpunkt hinter der Landeschwelle [m] = Ende der Aiming-Point-Markierung (die dicken "1000-ft-Marker"):
--   USA/Kanada (FAA): beginnt 1020 ft (311 m) hinter der Schwelle, 150 ft (46 m) lang -> 357 m
--   sonst (ICAO/EASA) nach verfuegbarer Landestrecke: < 800 m: 150 m, < 1200: 250, < 2400: 300, sonst 400 m; 45-60 m lang
function P.tdd(E)
    if not E then return P.td_dist end
    if E.tdd then return E.tdd end
    local lda = (E.len or 3000) - (E.disp or 0)
    local m = E.mark or 3
    local faa = (E.icao or ""):match("^[KPC]") ~= nil and m ~= 4 and m ~= 5
    local a0, al
    if faa and lda >= 1300 then a0, al = 311, 46
    elseif lda < 800 then a0, al = 150, 30
    elseif lda < 1200 then a0, al = 250, 45
    elseif lda < 2400 then a0, al = 300, 45
    else a0, al = 400, 60 end
    E.tdd = math.min(a0 + al, lda * 0.4)
    return E.tdd
end

-- wahre / angezeigte Geschwindigkeit am Zielplatz: Luftdichte aus Platzhoehe und der Temperatur dort
-- (aus der Aussentemperatur am Flugzeug mit dem Standard-Temperaturgradienten auf Platzhoehe gerechnet)
function P.tas_ratio()
    local h = ((S.apt and S.apt.elev_ft) or 0) * 0.3048
    local oat = DR.oat and DR.oat[0] or 15
    local alt = DR.elev and DR.elev[0] or h
    local t = clamp(oat + 0.0065 * (alt - h), -50, 55)
    local p = 101325 * (1 - 2.25577e-5 * clamp(h, -400, 4500)) ^ 5.25588
    local rho = p / (287.05 * (t + 273.15))
    return clamp(math.sqrt(1.225 / rho), 0.9, 1.35)
end

local function rwy_cond_auto()
    if DR.fric11 then
        -- XP11: poor = kontaminiert, fair oder Regen = nass
        local fr, rain = DR.fric11[0] or 0, DR.rain11 and DR.rain11[0] or 0
        if fr >= 1.5 then return P.cont, "CONTAMINATED"
        elseif fr >= 0.5 or rain >= 0.1 then return P.wet, "WET" end
        return P.dry, "DRY"
    end
    local fr = DR.fric and DR.fric[0] or 0
    if fr >= 4 then return P.cont, "CONTAMINATED"
    elseif fr >= 1 then return P.wet, "WET" end
    return P.dry, "DRY"
end
-- Pistenzustand wie im A350 (Reiter LANDING, Code 6..1 wie RWYCC / LDG PERF CODE) oder AUTO aus dem Wetter
P.RWYC = { [6] = { "DRY", P.dry }, [5] = { "GOOD", P.wet }, [4] = { "GOOD/MED", { plan = 1.15, max = 1.75, hard = 2.05 } },
           [3] = { "MEDIUM", P.cont }, [2] = { "MED/POOR", { plan = 0.85, max = 1.25, hard = 1.45 } },
           [1] = { "POOR", { plan = 0.7, max = 1.0, hard = 1.2 } } }
local function rwy_cond()
    local c = S.cond_sel and P.RWYC[S.cond_sel]
    if c then return c[2], c[1] end
    return rwy_cond_auto()
end

-- ROW: Haltepunkte trocken / nass (Pistenkoordinaten). Ueber 500 ft vorausberechnet (VAPP, Aufsetzpunkt), darunter
-- in Echtzeit: Groundspeed minus Flare-Abbau, Aufsetzpunkt aus Hoehe (3-Grad-Pfad) und Position; am Boden ab der
-- aktuellen Position mit der aktuellen Groundspeed. DRY: Autobrake-HI-Verzoegerung, WET: nass mit voller Umkehr.
function S.row_stops(E)
    local v, td = S.v_est or 70, E.disp + P.tdd(E)
    local mg = DR.onground[1] == 1 or DR.onground[2] == 1
    S.row_rt = false
    if mg and S.air_time == 0 and DR.gs[0] > 20 * KT then
        local s = rwy_sd(E, DR.lat[0], DR.lon[0])
        if s > -100 and s < E.len then v, td, S.row_rt = DR.gs[0], s, true end
    elseif S.air_time > 2 and DR.ra[0] < 500 then
        local s = rwy_sd(E, DR.lat[0], DR.lon[0])
        local h = DR.ra[0] * 0.3048
        v = math.max(DR.gs[0] - P.flare_loss * KT, 30 * KT)
        td = math.max(td, s + math.max(h - 15, 0) / math.tan(3 * D2R) + 300)
        S.row_rt = true
    end
    S.row_td, S.row_v = td, v
    return td + v * v / (2 * P.dry.max), td + v * v / (2 * P.wet.max)
end

-------------------------------------------------------------------------------
-- Toene (v2.12)
-- AUD: Ansagen RUNWAY TOO SHORT / MAX REVERSE / KEEP MAX REVERSE als WAV-Dateien (A350_BTV_data/sounds/voiceN),
--      abgespielt ueber X-Planes FMOD (FlyWithLua NG+ 2.8.9+: Interior-Bus = Cockpit, X-Plane-Regler "Interior").
--      Eigene Dateien gleichen Namens in sounds/custom/ (.wav/.ogg/.mp3) haben Vorrang.
-- MCW: MASTER CAUT, wenn die Crew BTV abschaltet (A350: AUTO BRK OFF mit Single Chime und MASTER CAUT).
--      Licht = FF-Lampe 1-sim/lamp/224 (MASTR CAUT beider Seiten); auf genau diese Lampe startet die FF-.snd die
--      Single Chime (FMOD-Event Warnings/Mastercaution der FF-Master-Bank). Laeuft die Chime danach nicht
--      (Lampe vom FF ueberschrieben oder schon an), startet das Skript das FF-Event selbst ueber FMOD Studio,
--      ohne FMOD Studio seine single_chime.wav. Aus: MASTER-CAUT-Taste (1-sim/130/button) oder nach 10 min.
-- XFM: Zugriff auf X-Plane (Datarefs schreiben) und FMOD Studio ueber LuaJIT-FFI.
-------------------------------------------------------------------------------
local AUD, XFM, MCW = { cache = {}, q = {}, dir = SCRIPTS .. "A350_BTV_data/sounds/" }, { st = "not started" }, { on = false }
AUD.VOICES = {
    { dir = "rops", name = "REAL" },
    { dir = "voice1", name = "Synthetic 1 (US)" },
    { dir = "voice2", name = "Synthetic 2 (UK)" },
    { dir = "voice3", name = "Synthetic 3 (US, deep)" },
}
AUD.FILE = { rts = "runway_too_short", smr = "set_max_reverse", kmr = "keep_max_reverse", brk = "brake_max_braking",
             abo = "auto_brake_off", chime = "single_chime" }
AUD.KEY = { ["RUNWAY TOO SHORT"] = "rts", ["SET MAX REVERSE"] = "smr", ["KEEP MAX REVERSE"] = "kmr",
            ["BRAKE MAX BRAKING"] = "brk", ["AUTO BRAKE OFF"] = "abo" }

function AUD.path(key)
    -- Tastenklick der A/BRK-Taste im Flug (dort nimmt das Skript den Klick, das FF spielt dann keinen Ton): der
    -- Klick des Flugzeugs aus dessen eigenem Ordner
    if key == "click" then
        local a = AIRCRAFT_PATH or ""
        if a ~= "" and not a:find("[/\\]$") then a = a .. "/" end
        return a .. "sounds/cockpit/button.wav"
    end
    local base = AUD.FILE[key]
    if not base then return nil end
    for _, ext in ipairs({ ".wav", ".ogg", ".mp3" }) do
        local p = AUD.dir .. "custom/" .. base .. ext
        if file_exists(p) then return p end
    end
    if key == "chime" then return AUD.dir .. base .. ".wav" end
    local v = AUD.VOICES[CFG.voice] or AUD.VOICES[1]
    local p = AUD.dir .. v.dir .. "/" .. base .. ".wav"
    if file_exists(p) then return p end
    -- in der gewaehlten Stimme nicht vorhanden: aus einer anderen
    for _, w in ipairs(AUD.VOICES) do
        local q = AUD.dir .. w.dir .. "/" .. base .. ".wav"
        if file_exists(q) then return q end
    end
    return p
end

-- Dauer einer Ansage [s] aus dem WAV-Kopf (fuer fortlaufende Wiederholung ohne Ueberlappen); sonst 1,7 s
local function aud_le(str, i, n)
    local v = 0
    for k = n - 1, 0, -1 do v = v * 256 + str:byte(i + k) end
    return v
end
function AUD.dur(key)
    local p = AUD.path(key)
    if not p then return 1.7 end
    AUD.durs = AUD.durs or {}
    local d = AUD.durs[p]
    if d then return d end
    d = 1.7
    local f = io.open(p, "rb")
    if f then
        local str = f:read(8192) or ""
        f:close()
        if str:sub(1, 4) == "RIFF" and str:sub(9, 12) == "WAVE" then
            local i, br = 13, nil
            while i + 8 <= #str do
                local id, sz = str:sub(i, i + 3), aud_le(str, i + 4, 4)
                if id == "fmt " and i + 20 <= #str then br = aud_le(str, i + 16, 4)
                elseif id == "data" then
                    if br and br > 0 then d = sz / br end
                    break
                end
                i = i + 8 + sz + (sz % 2)
            end
        end
    end
    if d < 0.3 or d > 8 then d = 1.7 end
    AUD.durs[p] = d
    return d
end

local function aud_fmod() return type(load_fmod_sound) == "function" and type(play_sound_on_interior_bus) == "function" end

-- einmal je Datei laden: FMOD (FlyWithLua NG+), sonst der alte OpenAL-Weg von FlyWithLua
function AUD.load(path)
    local c = AUD.cache[path]
    if c ~= nil then return c end
    c = false
    if not file_exists(path) then
        log("SOUND: file missing: " .. path)
    else
        if aud_fmod() then
            local ok, id = pcall(load_fmod_sound, path)
            if ok and type(id) == "number" and id >= 0 then c = { id = id, fmod = true } end
        end
        if not c and type(load_WAV_file) == "function" and type(play_sound) == "function" then
            local ok, id = pcall(load_WAV_file, path)
            if ok and type(id) == "number" and id >= 0 then c = { id = id, fmod = false } end
        end
        if not c then log("SOUND: cannot load " .. path) end
    end
    AUD.cache[path] = c
    return c
end

-- FlyWithLua-Kanal hoerbar halten (Lautstaerke aus den Einstellungen, nicht stumm)
function AUD.bus()
    if AUD.vdr == nil then
        AUD.vdr = false
        if dr_exists("FlyWithLua_InteriorChannelGroup/Volume") then
            AUD.vdr = dataref_table("FlyWithLua_InteriorChannelGroup/Volume")
        end
        if dr_exists("FlyWithLua_InteriorChannelGroup/Mute") then
            AUD.mdr = dataref_table("FlyWithLua_InteriorChannelGroup/Mute")
        end
    end
    local want = clamp(tonumber(CFG.avol) or 1, 0, 2)
    if AUD.vdr then
        local v = tonumber(AUD.vdr[0]) or 0
        if math.abs(v - want) > 0.001 then
            if not AUD.vlog then AUD.vlog = true; log(string.format("SOUND: FlyWithLua interior volume %.2f -> %.2f", v, want)) end
            AUD.vdr[0] = want
        end
    end
    if AUD.mdr and (tonumber(AUD.mdr[0]) or 0) ~= 0 then AUD.mdr[0] = 0 end
end

function AUD.play(key)
    local p = AUD.path(key)
    local c = p and AUD.load(p)
    if not c then return false end
    pcall(AUD.bus)
    local ok
    if c.fmod then ok = pcall(play_sound_on_interior_bus, c.id)
    else
        -- OpenAL (XP11 / FlyWithLua NG 2.7): Lautstaerke aus den Einstellungen direkt am Ton
        if type(set_sound_gain) == "function" then pcall(set_sound_gain, c.id, clamp(tonumber(CFG.avol) or 1, 0, 2)) end
        ok = pcall(play_sound, c.id)
    end
    return ok
end

function AUD.status()
    if aud_fmod() then return "FMOD, interior bus" end
    if type(load_WAV_file) == "function" then return "OpenAL (FlyWithLua without FMOD)" end
    return "no sound functions in this FlyWithLua"
end

-- Test im Einstellungsfenster: alle drei Ansagen nacheinander
function AUD.test()
    local t = DR.time[0]
    AUD.q = {}
    for _, k in ipairs({ "rts", "smr", "kmr", "brk", "abo" }) do
        AUD.q[#AUD.q + 1] = { t = t, k = k }
        t = t + AUD.dur(k) + 0.5
    end
end

-- Ansage (auch bei abgeschalteten Ansagen gilt die Dauer, damit Wiederholungen im gleichen Takt bleiben)
local function say(t)
    log("AURAL: " .. t)
    local k = AUD.KEY[t]
    if not k then return end
    AUD.busy = DR.time[0] + AUD.dur(k)
    if CFG.aural then AUD.play(k) end
end
-- frei fuer die naechste Ansage (gap s nach dem Ende der letzten)?
function AUD.free(gap) return DR.time[0] >= (AUD.busy or 0) + (gap or 0) end
function AUD.later(t, delay)
    AUD.q[#AUD.q + 1] = { t = DR.time[0] + delay, say = t }
    table.sort(AUD.q, function(a, b) return a.t < b.t end)
end

XFM.CDEF = [[
typedef struct { float x, y, z; } btv_fvec_t;
typedef struct { btv_fvec_t position, velocity, forward, up; } btv_f3d_t;
void *btv_s_XPLMFindDataRef(const char *) __asm__("XPLMFindDataRef");
int btv_s_XPLMGetDataRefTypes(void *) __asm__("XPLMGetDataRefTypes");
int btv_s_XPLMCanWriteDataRef(void *) __asm__("XPLMCanWriteDataRef");
int btv_s_XPLMGetDatai(void *) __asm__("XPLMGetDatai");
float btv_s_XPLMGetDataf(void *) __asm__("XPLMGetDataf");
double btv_s_XPLMGetDatad(void *) __asm__("XPLMGetDatad");
void btv_s_XPLMSetDatai(void *, int) __asm__("XPLMSetDatai");
void btv_s_XPLMSetDataf(void *, float) __asm__("XPLMSetDataf");
void btv_s_XPLMSetDatad(void *, double) __asm__("XPLMSetDatad");
void *btv_s_XPLMGetFMODStudio(void) __asm__("XPLMGetFMODStudio");
int btv_s_GetEvent(void *, const char *, void **) __asm__("FMOD_Studio_System_GetEvent");
int btv_s_CreateInstance(void *, void **) __asm__("FMOD_Studio_EventDescription_CreateInstance");
int btv_s_GetInstanceList(void *, void **, int, int *) __asm__("FMOD_Studio_EventDescription_GetInstanceList");
int btv_s_GetPlaybackState(void *, int *) __asm__("FMOD_Studio_EventInstance_GetPlaybackState");
int btv_s_Set3DAttributes(void *, const btv_f3d_t *) __asm__("FMOD_Studio_EventInstance_Set3DAttributes");
int btv_s_Start(void *) __asm__("FMOD_Studio_EventInstance_Start");
int btv_s_Release(void *) __asm__("FMOD_Studio_EventInstance_Release");
int btv_s_GetListenerAttributes(void *, int, btv_f3d_t *, btv_fvec_t *) __asm__("FMOD_Studio_System_GetListenerAttributes");
typedef int (*btv_cmd_f)(void *, int, void *);
void *btv_s_XPLMFindCommand(const char *) __asm__("XPLMFindCommand");
void btv_s_XPLMRegisterCommandHandler(void *, btv_cmd_f, int, void *) __asm__("XPLMRegisterCommandHandler");
void btv_s_XPLMUnregisterCommandHandler(void *, btv_cmd_f, int, void *) __asm__("XPLMUnregisterCommandHandler");
]]

-- XPLM fuer LuaJIT-FFI: Windows XPLM_64.dll; Linux/macOS ueber den X-Plane-Ordner (absoluter Pfad - der
-- Arbeitsordner ist auf dem Mac nicht sicher der X-Plane-Ordner), dann relativ, sonst die im Prozess schon
-- geladenen Symbole (ffi.C). probe = ein per cdef deklariertes Symbol, das es geben muss.
function XFM.xplm_lib(f, probe)
    if f.os == "Windows" then return f.load("XPLM_64") end
    local rel = (f.os == "OSX") and "Resources/plugins/XPLM.framework/XPLM" or "Resources/plugins/XPLM_64.so"
    local cands = {}
    if type(XP_ROOT) == "string" and XP_ROOT ~= "" then
        cands[#cands + 1] = XP_ROOT .. (XP_ROOT:match("[/\\]$") and "" or "/") .. rel
    end
    cands[#cands + 1] = rel
    local err
    for _, p in ipairs(cands) do
        local ok, l = pcall(f.load, p)
        if ok and l then
            if not probe or pcall(function() return l[probe] end) then return l end
        else
            err = l
        end
    end
    if probe and pcall(function() return f.C[probe] end) then return f.C end
    error(tostring(err or "XPLM not found"))
end

function XFM.init()
    if XFM.tried then return XFM.xp ~= nil end
    XFM.tried = true
    local okf, f = pcall(require, "ffi")
    if not okf or type(f) ~= "table" then XFM.st = "no LuaJIT FFI"; return false end
    XFM.ffi = f
    local okc, e = pcall(f.cdef, XFM.CDEF)
    if not okc and not tostring(e):find("redefine") then XFM.st = "FFI: " .. tostring(e); log("SOUND: " .. XFM.st); return false end
    local T = A350_BTV_TEST and A350_BTV_TEST.fms
    if T then
        XFM.xp, XFM.fs, XFM.sys = T.xp, T.fs, T.sys
    else
        local okx, lx = pcall(XFM.xplm_lib, f, "btv_s_XPLMFindDataRef")
        if not okx then XFM.st = "XPLM not loadable"; log("SOUND: " .. XFM.st); return false end
        XFM.xp = lx
    end
    XFM.st = "FFI ok"
    return true
end

function XFM.dref(name)
    if not XFM.xp then return nil end
    local ok, d = pcall(function()
        local r = XFM.xp.btv_s_XPLMFindDataRef(name)
        if r == nil then return nil end
        return { r = r, ty = tonumber(XFM.xp.btv_s_XPLMGetDataRefTypes(r)) or 0,
                 can = XFM.xp.btv_s_XPLMCanWriteDataRef(r) ~= 0 }
    end)
    if ok then return d end
end

function XFM.get(d)
    if bit.band(d.ty, 2) ~= 0 then return tonumber(XFM.xp.btv_s_XPLMGetDataf(d.r)) end
    if bit.band(d.ty, 1) ~= 0 then return tonumber(XFM.xp.btv_s_XPLMGetDatai(d.r)) end
    if bit.band(d.ty, 4) ~= 0 then return tonumber(XFM.xp.btv_s_XPLMGetDatad(d.r)) end
end

-- XP11 / FlyWithLua NG 2.7: peek() (Lesen einer dataref_table) aus einem FFI-Callback (Zeichnen, Maus,
-- Kommandos) laesst seinen Wert auf dem Lua-Stack von FlyWithLua liegen -> "Stack overflow, more than 100
-- elements on stack" und FlyWithLua stoppt alle Skripte. Waehrend eines Callbacks lesen die Tabellen deshalb
-- ueber FFI direkt aus X-Plane (XFM.cb_enter / XFM.cb_leave; XFM.pk_lib setzt PNL beim Start).
function XFM.ffi_peek(t, key)
    local L, r, ty = XFM.pk_lib, rawget(t, "reference"), rawget(t, "reftype") or 0
    if r == nil then return XFM.pk_old(t, key) end
    if bit.band(ty, 4) ~= 0 then return tonumber(L.btv_XPLMGetDatad(r)) end
    if bit.band(ty, 2) ~= 0 then return tonumber(L.btv_XPLMGetDataf(r)) end
    if bit.band(ty, 1) ~= 0 then return tonumber(L.btv_XPLMGetDatai(r)) end
    local k = tonumber(key) or 0
    if bit.band(ty, 8) ~= 0 then XFM.pk_f[0] = 0; L.btv_XPLMGetDatavf(r, XFM.pk_f, k, 1); return tonumber(XFM.pk_f[0]) end
    if bit.band(ty, 16) ~= 0 then XFM.pk_i[0] = 0; L.btv_XPLMGetDatavi(r, XFM.pk_i, k, 1); return tonumber(XFM.pk_i[0]) end
    return XFM.pk_old(t, key)
end

function XFM.cb_enter()
    local M = DATAREF_META_TABLE
    if not XFM.pk_lib or type(M) ~= "table" then return end
    XFM.pk_depth = (XFM.pk_depth or 0) + 1
    if XFM.pk_depth == 1 then XFM.pk_old = M.__index; M.__index = XFM.ffi_peek end
end

function XFM.cb_leave()
    if (XFM.pk_depth or 0) <= 0 then return end
    XFM.pk_depth = XFM.pk_depth - 1
    if XFM.pk_depth == 0 then DATAREF_META_TABLE.__index = XFM.pk_old end
end

function XFM.set(d, v)
    if bit.band(d.ty, 2) ~= 0 then XFM.xp.btv_s_XPLMSetDataf(d.r, v)
    elseif bit.band(d.ty, 1) ~= 0 then XFM.xp.btv_s_XPLMSetDatai(d.r, v)
    elseif bit.band(d.ty, 4) ~= 0 then XFM.xp.btv_s_XPLMSetDatad(d.r, v) end
end

-- GUID eines FMOD-Events aus der GUIDs.txt des Flugzeugs (die Master Bank des FF hat keine Strings-Bank)
function XFM.guid(evname)
    local acf = (type(AIRCRAFT_PATH) == "string" and AIRCRAFT_PATH ~= "") and AIRCRAFT_PATH or (XP_ROOT .. "Aircraft/addon aircraft/AirbusA350/")
    if not acf:match("[/\\]$") then acf = acf .. "/" end
    local f = io.open(acf .. "fmod/GUIDs.txt", "r")
    if not f then return nil end
    local g
    for line in f:lines() do
        local id, n = line:match("^%s*(%b{})%s+(%S+)")
        if n == evname then g = id; break end
    end
    f:close()
    return g
end

function XFM.event(evname)
    XFM.ev = XFM.ev or {}
    local d = XFM.ev[evname]
    if d ~= nil then return d end
    d = false
    if XFM.fs and XFM.sys then
        local g = XFM.guid(evname)
        if g then
            local pp = XFM.ffi.new("void *[1]")
            local ok, r = pcall(function() return XFM.fs.btv_s_GetEvent(XFM.sys, g, pp) end)
            if ok and r == 0 and pp[0] ~= nil then d = pp[0]
            else log("SOUND: FMOD event " .. evname .. " not found (" .. tostring(ok and r or "error") .. ")") end
        else
            log("SOUND: " .. evname .. " not in GUIDs.txt")
        end
    end
    XFM.ev[evname] = d
    return d
end


-- eigene Instanz des Events starten (am Kopf des Hoerers) und gleich freigeben (FMOD loescht sie nach dem Ende)
function XFM.start(evname)
    local d = XFM.event(evname)
    if not d then return false end
    local ok, res = pcall(function()
        local pp = XFM.ffi.new("void *[1]")
        if XFM.fs.btv_s_CreateInstance(d, pp) ~= 0 or pp[0] == nil then return false end
        local a = XFM.ffi.new("btv_f3d_t")
        if XFM.fs.btv_s_GetListenerAttributes(XFM.sys, 0, a, nil) == 0 then XFM.fs.btv_s_Set3DAttributes(pp[0], a) end
        local r = XFM.fs.btv_s_Start(pp[0])
        XFM.fs.btv_s_Release(pp[0])
        return r == 0
    end)
    return ok and res
end

MCW.LAMP = "1-sim/lamp/224"
function MCW.init()
    if MCW.tried then return end
    MCW.tried = true
    XFM.init()
    MCW.d = XFM.dref(MCW.LAMP)
    if not MCW.d and dr_exists(MCW.LAMP) then MCW.t = dataref_table(MCW.LAMP) end
    -- MASTER-CAUT-Taste: 1-sim/130/button (beide Seiten, wechselt bei jedem Druck), dazu die Tastenanimation des FF
    MCW.btn = {}
    for _, b in ipairs({ { "1-sim/130/button", "chg" }, { "AirbusFBW/LMasterCautAnim", "chg" }, { "AirbusFBW/RMasterCautAnim", "chg" } }) do
        if dr_exists(b[1]) then MCW.btn[#MCW.btn + 1] = { t = dataref_table(b[1]), m = b[2], n = b[1] } end
    end
    local ln = MCW.d and ("FFI" .. (MCW.d.can and "" or ", read-only")) or (MCW.t and "FlyWithLua" or "missing")
    MCW.cmds()
    log("MASTER CAUT: lamp " .. MCW.LAMP .. " " .. ln .. ", " .. XFM.st .. ", " .. #MCW.btn .. " button datarefs, "
        .. #(MCW.cmd or {}) .. " commands")
end

-- Befehle der MASTER-CAUT-Tasten (Tastatur, Hardware): vor dem FF mithoeren, nichts abfangen.
-- Wird beim Beenden des Skripts wieder abgemeldet (btv_on_exit).
function MCW.cmds()
    if MCW.cmd or not XFM.xp or not XFM.ffi then return end
    MCW.cmd = {}
    local ok, e = pcall(function()
        MCW.cb = XFM.ffi.cast("btv_cmd_f", function(c, phase, r)
            XFM.cb_enter()
            if phase == 0 and MCW.on then pcall(MCW.clear, "MASTER CAUT command") end
            XFM.cb_leave()
            return 1
        end)
        for _, n in ipairs({ "AirbusFBW/CaptCautPush", "AirbusFBW/CopilotCautPush", "sim/annunciator/clear_master_caution" }) do
            local c = XFM.xp.btv_s_XPLMFindCommand(n)
            if c ~= nil then
                XFM.xp.btv_s_XPLMRegisterCommandHandler(c, MCW.cb, 1, nil)
                MCW.cmd[#MCW.cmd + 1] = c
            end
        end
    end)
    if not ok then log("MASTER CAUT: commands: " .. tostring(e)) end
end

-- Brems-Befehle (Taste B, volle Bremse, linke/rechte Bremse, Joystick-Knoepfe): wie Pedale treten.
-- Der Rueckruf merkt sich nur den Druck (S.brk_cmd), ausgewertet wird im naechsten Bild; der Befehl laeuft normal weiter.
MCW.BRK_CMDS = { "sim/flight_controls/brakes_regular", "sim/flight_controls/brakes_max",
    "sim/flight_controls/brakes_toggle_regular", "sim/flight_controls/brakes_toggle_max",
    "sim/flight_controls/left_brake", "sim/flight_controls/right_brake" }
function MCW.brk_cmds()
    if MCW.bcmd then return end
    MCW.bcmd = {}
    if not XFM.init() then log("brake commands: " .. tostring(XFM.st)); return end
    local ok, e = pcall(function()
        MCW.bcb = XFM.ffi.cast("btv_cmd_f", function(c, phase, r)
            -- (Beginn und Halten; nur waehrend BTV bremst)
            if phase ~= 2 and (S.mode == "ENGAGED" or S.mode == "RELEASE") then S.brk_cmd = true end
            return 1
        end)
        for _, n in ipairs(MCW.BRK_CMDS) do
            local c = XFM.xp.btv_s_XPLMFindCommand(n)
            if c ~= nil then
                XFM.xp.btv_s_XPLMRegisterCommandHandler(c, MCW.bcb, 1, nil)
                MCW.bcmd[#MCW.bcmd + 1] = c
            end
        end
    end)
    log("brake commands: " .. (ok and (#MCW.bcmd .. " watched") or tostring(e)))
end
function MCW.brk_shutdown()
    if MCW.bcmd and MCW.bcb and XFM.xp then
        for _, c in ipairs(MCW.bcmd) do
            pcall(function() XFM.xp.btv_s_XPLMUnregisterCommandHandler(c, MCW.bcb, 1, nil) end)
        end
        MCW.bcmd = {}
    end
end

function MCW.shutdown()
    if MCW.cmd and MCW.cb and XFM.xp then
        for _, c in ipairs(MCW.cmd) do
            pcall(function() XFM.xp.btv_s_XPLMUnregisterCommandHandler(c, MCW.cb, 1, nil) end)
        end
        MCW.cmd = {}
    end
end

function MCW.get()
    if MCW.d then local ok, v = pcall(XFM.get, MCW.d); if ok then return v end end
    if MCW.t then return tonumber(MCW.t[0]) end
end

function MCW.set(v)
    if MCW.d then pcall(XFM.set, MCW.d, v)
    elseif MCW.t then pcall(function() MCW.t[0] = v end) end
end

local function mcw_btn_snap()
    local r = {}
    for i, b in ipairs(MCW.btn or {}) do r[i] = tonumber(b.t[0]) or 0 end
    return r
end

function MCW.trigger(reason)
    if reason ~= "TEST" and not CFG.mcaut then return end
    MCW.init()
    MCW.on, MCW.t0, MCW.fr, MCW.chk, MCW.ovr = true, DR.time[0], 0, true, false
    MCW.b = mcw_btn_snap()
    MCW.set(1)
    log("MASTER CAUT ON (" .. reason .. ")")
end

function MCW.clear(why)
    if not MCW.on then return end
    MCW.on = false
    MCW.set(0)
    log("MASTER CAUT OFF (" .. why .. ")")
end

-- je Bild (FlyWithLua): Chime pruefen, Taste abfragen, Lampe halten
function MCW.update(now)
    if not MCW.on then return end
    MCW.fr = MCW.fr + 1
    local v = MCW.get()
    if MCW.fr >= 2 and v and v < 0.5 then MCW.ovr = true end
    -- (die Single Chime spielt die FF-Soundbank selbst, sobald die Lampe angeht - keine eigene)
    if MCW.chk and MCW.fr >= 3 then
        MCW.chk = false
        MCW.res = (MCW.d or MCW.t) and ("FF light + FF chime" .. (MCW.ovr and " (lamp overwritten by the aircraft)" or "")) or "lamp not found"
        log("MASTER CAUT: " .. MCW.res)
    end
    local snap = mcw_btn_snap()
    for i, b in ipairs(MCW.btn) do
        local old, new = MCW.b[i] or 0, snap[i]
        if (b.m == "chg" and math.abs(new - old) > 0.3) or (b.m == "up" and new > 0.5 and old <= 0.5) then
            MCW.clear("pushed: " .. b.n); return
        end
    end
    MCW.b = snap
    if now - MCW.t0 > 600 then MCW.clear("timeout"); return end
    MCW.set(1)
end

-- aus dem Zeichen-Callback (nach den Flugzeug-Plugins): Lampe noch einmal setzen; steht sie dort nicht mehr
-- auf 1, hat das FF sie seit dem letzten Bild ueberschrieben
function MCW.late()
    if not (MCW.on and MCW.d) then return end
    local ok, v = pcall(XFM.get, MCW.d)
    if ok and v and v < 0.5 and (MCW.fr or 0) >= 1 then MCW.ovr = true end
    pcall(XFM.set, MCW.d, 1)
end

function AUD.frame()
    local now = DR.time[0]
    pcall(AUD.bus)
    if #AUD.q > 0 and now >= AUD.q[1].t then
        local e = table.remove(AUD.q, 1)
        if e.say then say(e.say)
        else AUD.play(e.k); AUD.busy = now + AUD.dur(e.k) end
    end
    MCW.update(now)
end

-- ROW-Linien { Name, Haltepunkt (Pistenkoordinate), Art }: Pistenzustand aus dem Reiter LANDING (RWY COND; AUTO = aus
-- dem Wetter). DRY: wie im A350 DRY- und WET-Linie ("sec" = nur Hinweis IF WET); sonst eine Linie mit dem Zustand.
P.ROW_SHORT = { DRY = "DRY", WET = "WET", CONTAMINATED = "CONT", GOOD = "GOOD", ["GOOD/MED"] = "G/M", MEDIUM = "MED",
                ["MED/POOR"] = "M/P", POOR = "POOR" }
function S.row_lines(E)
    local sdry, swet = S.row_stops(E)
    local c, name = rwy_cond()
    if name == "DRY" then return { { "DRY", sdry, "pri" }, { "WET", swet, "sec" } } end
    local td, v = S.row_td or 0, S.row_v or 0
    return { { P.ROW_SHORT[name] or name, td + v * v / (2 * c.max), "pri" } }
end

-- Ground Spoiler: groesster Ausschlag [Grad] (nil ohne Daten)
local function gnd_spoilers()
    local d = DR.spl
    if not d then return nil end
    local m = 0
    for i = 0, 3 do
        local v = tonumber(d[i])
        if v and math.abs(v) > m then m = math.abs(v) end
    end
    return m
end

local function exit_speed(x)
    if not x then return CFG.v_90 * KT end
    return (x.ret and CFG.v_ret or CFG.v_90) * KT
end

-- Bewertung der Exits: zu kurz / empfohlen
local function rate_exits()
    local c = rwy_cond()
    local v = S.v_est
    S.rec_idx = nil
    local td = P.tdd(S.rwy)
    for i, x in ipairs(S.exits) do
        local ve = exit_speed(x)
        local need_max = td + (v * v - ve * ve) / (2 * c.max)
        local need_nom = td + (v * v - ve * ve) / (2 * c.plan)
        x.too_short = x.dist < need_max
        if not S.rec_idx and x.dist >= need_nom then S.rec_idx = i end
    end
end

local function select_runway(E, manual, src)
    if S.locked and S.locked() then message("NOT AVAIL DURING BTV BRAKING"); return end
    S.rwy, S.rwy_manual = E, manual
    local net
    S.exits, S.exit_src, net = compute_exits(S.apt, E)
    S.map = true
    if S.view and S.view.pan then S.view.pan.btv = { 0, 0 }; S.view.z.btv = 1 end
    for _, v in pairs(S.nd_views or {}) do if v.pan then v.pan.btv = { 0, 0 }; v.z.btv = 1 end end
    S.exit, S.manual_dist = nil, nil
    rate_exits()
end

-------------------------------------------------------------------------------
-- Bremsen schreiben
-------------------------------------------------------------------------------
local function write_pedal(p)
    if BRAKE_MODE == "toliss" then
        DR.ovr[0] = 1
        DR.pl[0] = p
        DR.pr[0] = p
    else
        DR.xovr[0] = 1
        DR.lbr[0] = p
        DR.rbr[0] = p
    end
    S.ovr_written = true
end

-- Pedal-Eingabe des Piloten (Hardware)
local function pilot_pedal()
    if BRAKE_MODE == "toliss" then
        return math.max(DR.lbr[0], DR.rbr[0])
    end
    -- im X-Plane-Modus sind left/right_brake_ratio von uns belegt -> Joystick-Achsen lesen
    -- (Achsfunktion 6 = linke, 7 = rechte Fussbremse)
    if DR.joy and DR.joy_avail and (DR.joy_avail[6] == 1 or DR.joy_avail[7] == 1) then
        return math.max(DR.joy[6] or 0, DR.joy[7] or 0)
    end
    return 0
end

local function release_override()
    if S.ovr_written then
        if DR.ovr then
            DR.pl[0] = 0
            DR.pr[0] = 0
            DR.ovr[0] = 0
        end
        if DR.xovr then
            DR.lbr[0] = 0
            DR.rbr[0] = 0
            DR.xovr[0] = 0
        end
        S.ovr_written = false
    end
    S.pedal, S.integ, S.braking = 0, 0, false
end

local function disconnect(reason)
    -- wie im A350: Abschalten durch die Crew (Pedale, DISARM, A/THR-Taste, Schub, Spoiler) -> "AUTO BRK OFF" im PFD
    if not reason:find("LIFT OFF") then
        S.abo_t = DR.time[0]
        local okm, em = pcall(MCW.trigger, reason)
        if not okm then log("MASTER CAUT: " .. tostring(em)) end
        AUD.later("AUTO BRAKE OFF", 1.0) -- (nach der Single Chime)
    end
    S.rop = false
    release_override()
    S.mode = "OFF"
    S.arm_request = false
    S.target = nil
    message(reason)
end

-------------------------------------------------------------------------------
-- Ziel waehrend des Ausrollens
-------------------------------------------------------------------------------
local function make_target_from_exit(x)
    local lead = x.ret and P.lead_ret or P.lead_90
    return { s = x.s - lead, ve = exit_speed(x), name = x.name, exit = x }
end

local function make_target()
    if S.manual_dist then
        return { s = S.rwy.disp + S.manual_dist, ve = CFG.v_90 * KT, name = string.format("%d M", S.manual_dist) }
    elseif S.exit then
        return make_target_from_exit(S.exit)
    end
end

local function a_required(v, ve, D)
    if D <= 1 then return 99 end
    return math.max(0, (v * v - ve * ve) / (2 * D))
end

-- Falls das Ziel nicht mehr erreichbar ist: naechsten passenden Exit nehmen
local function retarget(v, s, amax)
    local cur = S.target
    for _, x in ipairs(S.exits) do
        if x.s > (cur and cur.s or s) + 1 then
            local t = make_target_from_exit(x)
            if a_required(v, t.ve, t.s - s) <= amax then return t end
        end
    end
    -- kein Exit erreichbar: Pistenende
    return { s = S.rwy.len - 150, ve = 1.5, name = "RWY END" }
end

-------------------------------------------------------------------------------
-- Hauptschleife
-------------------------------------------------------------------------------
local last_auto, last_rate = 1000, -100


local function main_gear_on_ground()
    return DR.onground[1] == 1 or DR.onground[2] == 1
end

-- Landebahn aus dem FMS am geladenen Flughafen (nil, wenn das FMS keine kennt)
function FMS.rwy_end(apt)
    if not (apt and FMS.rwy and FMS.dest == apt.icao) then return nil end
    for _, E in ipairs(apt.ends) do
        if FMS.norm(E.id) == FMS.rwy then return E end
    end
end

-------------------------------------------------------------------------------
-- Runway Proximity Advisory (A350): am Boden unter 40 kt wird der Bereich vor dem Flugzeug (so breit wie die
-- Spannweite, 7 Sekunden voraus) gegen den Pistenbereich geprueft (Piste + 60 m rundherum) -> "RWY AHEAD" im PFD,
-- im ND blinkt die Piste und ihr Name pulsiert; kein Ton. Verschwindet, wenn die Bedingungen nicht mehr gelten
-- oder das Flugzeug auf der Piste ist.
-------------------------------------------------------------------------------
local RWYA = { text = nil, n = 0, r = nil, AREA = 60 }
-- ueberlappen sich zwei konvexe Vierecke? (Trennachsen)
function RWYA.overlap(A, B)
    for _, Q in ipairs({ A, B }) do
        for i = 1, 4 do
            local p, q = Q[i], Q[i % 4 + 1]
            local nx, ny = q[2] - p[2], p[1] - q[1]
            local amin, amax, bmin, bmax = math.huge, -math.huge, math.huge, -math.huge
            for _, v in ipairs(A) do
                local d = v[1] * nx + v[2] * ny
                amin, amax = math.min(amin, d), math.max(amax, d)
            end
            for _, v in ipairs(B) do
                local d = v[1] * nx + v[2] * ny
                bmin, bmax = math.min(bmin, d), math.max(bmax, d)
            end
            if amax < bmin or bmax < amin then return false end
        end
    end
    return true
end
function RWYA.update()
    RWYA.n = RWYA.n + 1
    if RWYA.n % 5 ~= 0 then return end
    RWYA.text, RWYA.r = nil, nil
    local G = S.apt and S.apt.geo
    if not G or not main_gear_on_ground() then return end
    local v = DR.gs[0]
    if v >= 40 * KT or v < 0.5 then return end
    local ax, ay = G.W(DR.lat[0], DR.lon[0])
    local h = DR.psi[0] * D2R
    local fx, fy = math.sin(h), math.cos(h)
    local rx, ry = fy, -fx
    local NOSE, HSPAN = 30, 32.4 -- Bezugspunkt bis Bug [m], halbe Spannweite A350 [m]
    local a0, a1 = NOSE, NOSE + v * 7
    local A = { { ax + fx * a0 + rx * HSPAN, ay + fy * a0 + ry * HSPAN }, { ax + fx * a1 + rx * HSPAN, ay + fy * a1 + ry * HSPAN },
                { ax + fx * a1 - rx * HSPAN, ay + fy * a1 - ry * HSPAN }, { ax + fx * a0 - rx * HSPAN, ay + fy * a0 - ry * HSPAN } }
    for _, r in ipairs(G.rwys) do
        local dx, dy = r.x2 - r.x1, r.y2 - r.y1
        local L = math.sqrt(dx * dx + dy * dy)
        if L > 1 then
            local ux, uy = dx / L, dy / L
            local hw = r.w / 2
            local qx, qy = ax - r.x1, ay - r.y1
            local al, cr = qx * ux + qy * uy, -qx * uy + qy * ux
            -- schon im Pistenbereich (darauf, beim Abrollen oder Kreuzen): keine Meldung - nur beim Hineinrollen
            local on = al > -RWYA.AREA and al < L + RWYA.AREA and math.abs(cr) < hw + RWYA.AREA
            if not on then
                local nx, ny = -uy, ux
                local m, bw = RWYA.AREA, hw + RWYA.AREA
                local xa, ya, xb, yb = r.x1 - ux * m, r.y1 - uy * m, r.x2 + ux * m, r.y2 + uy * m
                local B = { { xa + nx * bw, ya + ny * bw }, { xb + nx * bw, yb + ny * bw },
                            { xb - nx * bw, yb - ny * bw }, { xa - nx * bw, ya - ny * bw } }
                if RWYA.overlap(A, B) then
                    RWYA.text, RWYA.r = r.id1 .. " - " .. r.id2, r
                    return
                end
            end
        end
    end
end

local function auto_airport_and_runway(now, airborne)
    if not IDX.ready or #jobs > 0 then return end
    -- alle ~150 Frames pruefen (unabhaengig von der Sim-Zeit, funktioniert auch in der Pause)
    last_auto = last_auto + 1
    if last_auto < 150 and S.apt and not S.apt_fresh then return end
    last_auto, S.apt_fresh = 0, nil
    local lat, lon = DR.lat[0], DR.lon[0]
    -- Flughafen von Hand (DISPLAY AIRPORT): gilt bis zum naechsten Start bzw. Aufsetzen
    if S.apt_air ~= nil and S.apt_air ~= airborne then S.apt_manual = false end
    S.apt_air = airborne
    -- Flughafen (nicht wechseln, wenn von Hand gewaehlt, Exit gewaehlt oder BTV scharf)
    local locked = S.apt and (S.apt_manual or S.exit or S.manual_dist or S.mode ~= "OFF" or S.arm_request)
    if not locked then
        local e, de = nearest_airport(lat, lon, P.min_rwy_len, P.auto_radius)
        -- im Flug: Zielflughafen aus dem FMS (ausser kurz nach dem Start, solange der Startflughafen nah ist)
        local dest = FMS.dest_e
        if dest and airborne and (not S.apt or S.apt.icao ~= dest.icao) then
            local dd = dist_m(lat, lon, dest.lat, dest.lon)
            local near_dep = e and de < 15000 and e.icao ~= dest.icao and (FMS.from == e.icao or (S.apt and S.apt.icao == e.icao))
            if not near_dep or dd < 40000 then
                log("FMS destination " .. dest.icao .. " - loading airport")
                load_airport(dest, false)
                return
            end
        end
        local keep_dest = airborne and dest and S.apt and S.apt.icao == dest.icao
        if e and not keep_dest and (not S.apt or S.apt.icao ~= e.icao) then
            local cur_d = math.huge
            if S.apt then
                for _, E in ipairs(S.apt.ends) do
                    cur_d = math.min(cur_d, dist_m(lat, lon, E.lat, E.lon))
                    local s, d = rwy_sd(E, lat, lon)
                    if airborne and s > -30000 and s < E.len and hdg_diff(DR.hpath[0], E.hdg) < 20
                        and math.abs(d) < 600 + math.abs(s) * 0.06 then
                        cur_d = 0 -- im Anflug auf eine Piste des aktuellen Flughafens
                    end
                end
            end
            if not S.apt or (cur_d > 1.5 * de and cur_d > 8000) then
                load_airport(e, false)
                return
            end
        end
    end
    if not S.apt or S.rwy_manual or S.locked() then return end
    -- Landebahn aus dem FMS: uebernehmen, sobald sie neu ist (oder noch keine Bahn gewaehlt ist)
    local fe = FMS.rwy_end(S.apt)
    if fe then
        local key = FMS.rwy .. "@" .. S.apt.icao
        if (FMS.applied ~= key or not S.rwy) and S.rwy ~= fe then
            select_runway(fe, false, "FMS")
            message("BTV RWY " .. fe.id .. " (FMS)")
        end
        FMS.applied = key
    end
    -- Piste im Anflug (nicht, solange BTV scharf ist - eine andere Bahn wuerde BTV abschalten)
    if airborne and DR.ra[0] > 300 and not S.arm_request then
        local trk = DR.hpath[0]
        local function on_final(E, before_thr)
            local s, d = rwy_sd(E, lat, lon)
            if s > -30000 and s < (before_thr and E.disp or E.len) and hdg_diff(trk, E.hdg) < 20
                and math.abs(d) < 600 + math.abs(s) * 0.06 then
                return math.abs(d)
            end
        end
        -- aktuelle Piste beibehalten, solange wir noch auf ihr anfliegen
        if S.rwy and on_final(S.rwy, false) then return end
        local best, bestd
        for _, E in ipairs(S.apt.ends) do
            local dd = on_final(E, true)
            if dd and (not bestd or dd < bestd) then best, bestd = E, dd end
        end
        if best and best ~= S.rwy then
            select_runway(best, false, "ALIGN")
            message("BTV RWY " .. best.id .. " (APPROACH)")
        end
    end
end

local function update_annunciator(now)
    local c = rwy_cond()
    S.ann1, S.ann2 = nil, nil
    if S.mode == "ARMED" then
        local t = make_target()
        S.ann1, S.ann_col = "BTV", { 0.2, 0.9, 1.0 }
        if t then S.ann2 = string.format("EXIT %s  %d M", t.name, math.floor(t.s - S.rwy.disp + 0.5)) end
    elseif S.mode == "ENGAGED" or S.mode == "RELEASE" then
        S.ann1, S.ann_col = "BTV", { 0.2, 1.0, 0.3 }
        if S.target then
            local s = rwy_sd(S.rwy, DR.lat[0], DR.lon[0])
            S.ann2 = string.format("%s  %d M  %.1f M/S2", S.target.name, math.max(0, math.floor(S.target.s - s)), S.a_cmd)
        end
    elseif S.arm_request then
        S.ann1, S.ann_col = "BTV", { 1.0, 0.65, 0.0 }
        S.ann2 = "NO EXIT SELECTED"
    end
    if S.msg and now - S.msg_t > 6 then S.msg = nil end
    S.cond_txt = select(2, rwy_cond())
    S.cond_auto = select(2, rwy_cond_auto())
end

local function btv_step()
    -- Laden (Index, Flughafen, Karte, Gebaeude) laeuft immer - auch in der Pause
    run_jobs(0.003)
    osm_poll()
    if FMS.poll then
        local ok, e = pcall(FMS.poll)
        if not ok and not FMS.err then FMS.err = true; log("FMS: " .. tostring(e)) end
    end
    local now = DR.time[0]
    if DR.paused[0] == 1 then
        S.t_prev = nil
        auto_airport_and_runway(now, false)
        update_annunciator(now)
        return
    end
    local dt = S.t_prev and (now - S.t_prev) or 0
    S.t_prev = now
    if dt <= 0 or dt > 0.5 then S.v_prev = DR.gs[0]; return end

    local v = DR.gs[0]
    -- gemessene Verzoegerung (gefiltert)
    if S.v_prev then
        local raw = (S.v_prev - v) / dt
        S.a_meas = S.a_meas + (raw - S.a_meas) * clamp(dt / (0.3 + dt), 0, 1)
    end
    S.v_prev = v

    local mg = main_gear_on_ground()
    local air0 = S.air_time
    if mg then S.gnd_time = S.gnd_time + dt; S.air_time = 0
    else S.air_time = S.air_time + dt; S.gnd_time = 0 end
    local airborne = S.air_time > 2
    -- Landung: Aufsetzen nach mindestens 10 s in der Luft; vorbei bei Rollgeschwindigkeit, Abheben oder Startschub
    if mg and air0 > 10 then S.ldg, S.td_t = true, now end
    if S.ldg and (v < 20 * KT or S.air_time > 5 or (DR.thr[0] > 0.7 and (DR.rev[0] or 0) < 0.1)) then S.ldg = false end

    auto_airport_and_runway(now, airborne)
    RWYA.update()

    if now - last_rate > 1 then
        last_rate = now
        S.hw = nil
        local vapp = S.vapp or FMS.vapp -- Eingabe im Fenster hat Vorrang, sonst VAPP aus dem FMS (PERF APPR)
        S.vapp_src = vapp and "FMS" or nil
        -- ohne VAPP: aus dem Gewicht (VLS ~ Wurzel der Masse; A350-900 bei 207 t Landegewicht ~ VAPP 140 kt)
        if not vapp and DR.mass and DR.mass[0] > 60000 and airborne then
            vapp = clamp(140 * math.sqrt(DR.mass[0] / 207000), 115, 155)
            S.vapp_src = "WEIGHT"
        end
        S.vapp_used = vapp
        if vapp then
            -- Aufsetz-Groundspeed = VAPP - Abbau im Flare - Gegenwind
            local hw = 0
            if S.rwy and DR.wspd and airborne and DR.ra[0] < 5000 then
                local var = DR.tpsi[0] - DR.mpsi[0]
                local wind_from_true = DR.wdir[0] + var
                hw = DR.wspd[0] * math.cos((wind_from_true - S.rwy.hdg) * D2R)
                S.hw = hw
            end
            -- Aufsetz-Groundspeed = (VAPP - Abbau im Flare) als wahre Geschwindigkeit am Platz - Gegenwind
            S.tas_k = P.tas_ratio()
            S.v_est = clamp(((vapp - P.flare_loss) * S.tas_k - hw) * KT, 35, 100)
        elseif airborne and DR.ra[0] < 3000 then
            S.v_est = clamp(v, 55, 85)
        else
            S.v_est = 72
        end
        if S.rwy then rate_exits() end
    end

    local c = rwy_cond()
    local armable = bind_aircraft_datarefs(now) and S.rwy ~= nil and (S.exit ~= nil or S.manual_dist ~= nil)
    if AC_READY and not MCW.bcmd then pcall(MCW.brk_cmds) end
    local arm_cond = armable and S.arm_request

    if S.mode == "OFF" or S.mode == "DONE" then
        if S.mode == "DONE" and now - S.msg_t > 5 then S.mode = "OFF" end
        if S.mode == "OFF" and arm_cond then
            S.mode = "ARMED"
            S.was_air = airborne
            S.arm_snap = { S.rwy, S.exit, S.manual_dist, S.apt }
            S.t_gs, S.gs_seen = nil, false
            S.ga_low, S.ga_t = nil, nil
        end
    elseif S.mode == "ARMED" then
        -- wie im A350: andere Bahn (ANF oder FMS), anderer / kein Exit, andere Stoppstrecke, anderer Flughafen -> DISARM
        local sn = S.arm_snap
        if sn and (sn[1] ~= S.rwy or sn[2] ~= S.exit or sn[3] ~= S.manual_dist or sn[4] ~= S.apt) then
            S.arm_request, arm_cond = false, false
            message("BTV DISARMED")
        end
        -- wie im A350: Durchstarten unter 600 ft -> DISARM, sobald das Flugzeug wieder ueber 600 ft steigt. Hoehe =
        -- kleinerer Wert aus Radiohoehe und Hoehe ueber der Schwelle (hier: ueber dem Flugplatz)
        if arm_cond and airborne then
            local h = DR.ra[0]
            local fe = S.apt and S.apt.elev_ft
            if fe and DR.elev then h = math.min(h, DR.elev[0] / 0.3048 - fe) end
            if h < 600 then
                S.ga_low = true
            elseif S.ga_low then
                S.arm_request, arm_cond = false, false
                S.ga_low, S.ga_t = nil, nil
                message("BTV DISARMED - GO AROUND")
            end
        end
        if not arm_cond then
            S.mode = "OFF"
        else
            if airborne then S.was_air, S.t_gs, S.gs_seen = true, nil, false end
            -- Ground Spoiler ausgefahren (> 20 Grad); ohne Spoiler-Daten 1,5 s (bzw. 3 s) nach dem Hauptfahrwerk
            if S.was_air and mg and not S.t_gs then
                local sp = gnd_spoilers()
                if sp and sp > 20 then S.t_gs, S.gs_seen = now, true
                elseif S.gnd_time > ((sp and sp > 2) and 3 or 1.5) then S.t_gs = now end
            end
            -- A350: BTV aktiv mit dem Ausfahren der Ground Spoiler, wenn das Bugrad am Boden ist, sonst 5 s danach
            local nose = DR.onground[0] == 1
            if S.was_air and mg and S.t_gs and (nose or now - S.t_gs >= 5) and v > 40 * KT then
                S.row = nil
                local s, d = rwy_sd(S.rwy, DR.lat[0], DR.lon[0])
                if math.abs(d) < S.rwy.width * 0.5 + 25 and s > -50 and s < S.rwy.len then
                    S.mode, S.t_td, S.was_air = "ENGAGED", now, false
                    S.brk_cmd = false
                    S.target = make_target()
                    S.braking, S.integ, S.pedal, S.a_cmd = false, 0, 0, 0
                    local a = a_required(v, S.target.ve, S.target.s - s)
                    if a > c.hard + 0.1 then
                        S.target = retarget(v, s, c.hard)
                        message("BTV: EXIT NOT REACHABLE -> " .. S.target.name)
                    else
                        message("BTV ACTIVE -> " .. S.target.name)
                    end
                else
                    S.was_air = false
                    message("BTV: NOT LANDED ON RWY " .. S.rwy.id)
                end
            end
        end
    elseif S.mode == "ENGAGED" then
        -- Abschaltbedingungen
        if S.gs_seen and mg and v > 15 * KT then
            local sp = gnd_spoilers()
            if sp and sp < 5 then disconnect("BTV DISCONNECT - SPOILERS"); return end
        end
        local pp = pilot_pedal()
        if pp > S.pedal + P.pilot_margin and pp > 0.3 then
            disconnect("BTV DISCONNECT - PEDALS"); return
        end
        if S.brk_cmd then
            S.brk_cmd = false
            disconnect("BTV DISCONNECT - BRAKES"); return
        end
        if not S.arm_request then
            disconnect("BTV DISCONNECT"); return
        end
        local rev = DR.rev[0] or 0
        if DR.thr[0] > 0.7 and rev < 0.1 then
            disconnect("BTV DISCONNECT - THRUST"); return
        end
        -- nur bei echtem Abheben (Touch-and-go / Durchstart), nicht bei kurzem Huepfer beim Aufsetzen
        if S.air_time > 4 and DR.ra[0] > 50 then
            disconnect("BTV DISCONNECT - LIFT OFF"); return
        end

        local s = rwy_sd(S.rwy, DR.lat[0], DR.lon[0])
        local T = S.target
        local D = T.s - s
        local a_req = a_required(v, T.ve, D)
        S.a_req = a_req
        -- ROP: haelt das Flugzeug auch mit Maximalbremsung nicht vor dem Pistenende -> Maximalbremsung,
        -- "MAX REVERSE" (PFD + Ansage, bis beide Umkehrer voll draussen), bei 80 kt "KEEP MAX REVERSE"
        local rop = CFG.rops and mg and v > 5 * KT and s + v * v / (2 * c.hard) > S.rwy.len
        if rop and not S.rop then
            S.rop_80 = v > 80 * KT
            message("ROP: MAX BRAKING")
        elseif not rop and S.rop then
            S.integ = S.pedal
            message("ROP: BTV BRAKING")
        end
        S.rop = rop
        if rop then
            local rv = (DR.rev[0] or 0) >= 0.9 and (DR.rev[1] or 0) >= 0.9
            if S.rop_80 and v <= 80 * KT then S.rop_80, S.kmr_pending = false, true end
            if not rv and not S.kmr_pending and AUD.free(0.3) then say("SET MAX REVERSE") end
        end
        S.rop_hold = rop -- (ROP bremst weiter, auch wenn der Exit schon passiert ist)

        -- Ziel erreicht (nicht, solange ROP bremst)
        if (v <= T.ve + 0.3 or D < -40) and not S.rop_hold then
            S.mode = "RELEASE"
            message(v <= T.ve + 0.5 and "BTV: EXIT SPEED REACHED" or "BTV: EXIT PASSED")
        else
            -- Ziel unerreichbar (auch mit Reserve) -> naechster Exit (nicht mehr kurz davor, nicht waehrend eines Huepfers)
            if mg and a_req > c.hard + 0.1 and not S.manual_dist and D > 150 and v > T.ve + 10 * KT then
                local nt = retarget(v, s, c.hard)
                if nt.name ~= T.name then
                    S.target = nt
                    message("BTV: NEW EXIT -> " .. nt.name)
                    T, D = nt, nt.s - s
                    a_req = a_required(v, T.ve, D)
                end
            end
            local t_since = now - S.t_td
            if mg and not S.braking and ((t_since >= P.brake_delay and a_req >= c.plan) or (t_since >= 0.8 and a_req >= c.max * 0.85)) then
                S.braking = true
                S.integ = clamp((a_req - S.a_meas) * P.pedal_per_a, 0, 1)
            end
            if S.braking and not mg then
                -- Huepfer: Bremse lösen, bis die Raeder wieder am Boden sind
                S.pedal = math.max(0, S.pedal - P.rate_down * dt)
            elseif S.braking then
                if D > 40 or S.a_cmd == 0 then S.a_cmd = clamp(a_req, 0, c.hard) end
                local err = S.a_cmd - S.a_meas
                -- harte Grenze: deutlich mehr als die Reserve -> Integrator schnell abbauen
                if S.a_meas > c.hard + 0.3 then err = math.min(err, -2) end
                S.integ = clamp(S.integ + P.ki * err * dt, 0, 1)
                local want = clamp(S.integ + P.kp * err, 0, 1)
                if want > S.pedal then S.pedal = math.min(want, S.pedal + P.rate_up * dt)
                else S.pedal = math.max(want, S.pedal - P.rate_down * dt) end
            end
            if S.rop and mg then
                S.braking = true
                S.pedal = math.min(1, S.pedal + 2.0 * dt)
            end
            write_pedal(S.pedal)
            -- Selbstpruefung: reagiert das ToLiss-System nicht auf die Pedal-Datarefs -> X-Plane-Weg
            if BRAKE_MODE == "toliss" and DR.totl and S.pedal > 0.3 then
                if DR.totl[0] < 0.02 and S.a_meas < S.a_cmd - 0.3 then
                    S.noresp_t = (S.noresp_t or 0) + dt
                else
                    S.noresp_t = 0
                end
                if S.noresp_t > 1.5 then
                    DR.pl[0], DR.pr[0], DR.ovr[0] = 0, 0, 0
                    BRAKE_MODE = "xp"
                    message("BTV: USING X-PLANE PEDALS")
                end
            end
        end
    end

    -- ROP ohne BTV (Handbremsung oder FF-Autobrake, die keine ROP-Maximalbremsung kann): nach dem Aufsetzen auf der
    -- gewaehlten Bahn bis Rollgeschwindigkeit. "BRAKE, MAX BRAKING, MAX BRAKING" bis die Pedale voll getreten sind,
    -- dann "SET MAX REVERSE" bis beide Umkehrer voll draussen sind, bei 80 kt "KEEP MAX REVERSE";
    -- im PFD "MAX BRAKING" bzw. "MAX REVERSE" (rot)
    local mrop = nil
    -- ROW (mit und ohne BTV): im Endanflug unter 500 ft auf die gewaehlte Bahn (sinkend, Kurs passt) und nach dem
    -- Aufsetzen bis Rollgeschwindigkeit; waehrend BTV bremst uebernimmt ROP
    S.row, S.row_on = nil, false
    if CFG.rops and S.rwy and not (S.mode == "ENGAGED" or S.mode == "RELEASE") then
        local E = S.rwy
        local on = false
        if airborne and DR.ra[0] <= 500 and S.air_time > 5 then
            local s, d = rwy_sd(E, DR.lat[0], DR.lon[0])
            local dh = math.abs(((DR.hpath[0] - E.hdg + 540) % 360) - 180)
            on = s < E.len * 0.5 and math.abs(d) < 600 and dh < 30 and not (DR.vs and DR.vs[0] > 1.5)
        elseif mg and S.ldg then
            on = true
        end
        if on then
            S.row_on = true
            for _, l in ipairs(S.row_lines(E)) do
                if l[2] > E.len then
                    if l[3] == "pri" then S.row = "dry" elseif S.row == nil then S.row = "wet" end
                end
            end
            if S.row == "dry" and airborne and DR.ra[0] < 200 and AUD.free(0.15) then say("RUNWAY TOO SHORT") end
        end
    end

    if CFG.rops and S.ldg and S.mode ~= "ENGAGED" and S.rwy and mg and v > 30 * KT then
        local s, d = rwy_sd(S.rwy, DR.lat[0], DR.lon[0])
        if math.abs(d) < S.rwy.width * 0.5 + 25 and s > -50 and s < S.rwy.len and s + v * v / (2 * c.hard) > S.rwy.len then
            local ped = math.max(DR.lbr[0] or 0, DR.rbr[0] or 0, pilot_pedal()) >= 0.8
            local rv = (DR.rev[0] or 0) >= 0.9 and (DR.rev[1] or 0) >= 0.9
            mrop = (not ped) and "brk" or ((not rv) and "rev" or "ok")
            if not S.mrop then S.mrop_80 = v > 80 * KT; message("ROP: MAX BRAKING") end
            if S.mrop_80 and v <= 80 * KT then S.mrop_80, S.kmr_pending = false, true end
        end
    end
    S.mrop = mrop
    if S.kmr_pending then
        if not (S.rop or S.mrop) then S.kmr_pending = false
        elseif AUD.free(0.1) then S.kmr_pending = false; say("KEEP MAX REVERSE") end
    elseif S.mrop == "brk" and AUD.free(0.25) then say("BRAKE MAX BRAKING")
    elseif S.mrop == "rev" and AUD.free(0.3) then say("SET MAX REVERSE") end

    if S.mode ~= "ENGAGED" then S.rop = false end
    if S.mode ~= "ENGAGED" and S.mode ~= "RELEASE" then S.brk_cmd = false end
    if S.mode == "RELEASE" then
        S.pedal = math.max(0, S.pedal - P.rate_down * dt)
        if S.brk_cmd then S.pedal = 0 end
        if S.pedal <= 0 then
            release_override()
            S.mode = "DONE"
            S.arm_request = false
            S.target = nil
            S.msg_t = now
        else
            write_pedal(S.pedal)
        end
    end

    update_annunciator(now)
end

local nd_err_logged = false
function btv_frame()
    local oks, errs = pcall(AUD.frame)
    if not oks and not AUD.err then AUD.err = true; log("SOUND: " .. tostring(errs)) end
    local ok, err = pcall(btv_step)
    if not ok then
        pcall(release_override)
        S.mode = "OFF"
        log("Runtime error: " .. tostring(err))
        S.msg, S.msg_t = "BTV FAULT - SEE LOG.TXT", DR.time[0]
    end
    if btv_nd_frame then
        local ok2, err2 = pcall(btv_nd_frame)
        if not ok2 and not nd_err_logged then
            nd_err_logged = true
            log("ND map: " .. tostring(err2))
        end
    end
end

-------------------------------------------------------------------------------
-- Anzeige (FMA-artig, oben in der Bildschirmmitte) - seit v2.15 abschaltbar und ab Werk aus: der BTV-Status
-- steht wie im A350 im FMA/PFD und auf dem ND
-------------------------------------------------------------------------------
function btv_draw()
    if btv_nd_cam then pcall(btv_nd_cam) end
    if not CFG.overlay then return end
    local l1, l2, msg = S.ann1, S.ann2, S.msg
    if not l1 and not msg then return end
    local cx = SCREEN_WIDTH / 2
    local y = SCREEN_HIGHT - 70
    if graphics then
        XPLMSetGraphicsState(0, 0, 0, 1, 1, 0, 0)
        graphics.set_color(0, 0, 0, 0.55)
        graphics.draw_rectangle(cx - 140, y - 46, cx + 140, y + 24)
    end
    if l1 then
        local c = S.ann_col
        if glColor4f and draw_string_Helvetica_18 then
            glColor4f(c[1], c[2], c[3], 1)
            draw_string_Helvetica_18(cx - 18, y, l1)
        else
            draw_string(cx - 10, y, l1, c[1], c[2], c[3])
        end
        if l2 then draw_string(cx - 130, y - 18, l2, c[1], c[2], c[3]) end
    end
    if msg then draw_string(cx - 130, y - 36, msg, 1.0, 0.75, 0.2) end
end

-------------------------------------------------------------------------------
-- Fenster (ImGui): Rollwegkarte im OANS-Stil mit allen Bedienelementen
-- Hinweis: Jede PushStyleColor/BeginDisabled wird ueber eine lokale Variable
-- wieder geschlossen - nie ueber einen Zustand, den ein Klick aendern kann.
-------------------------------------------------------------------------------
local wnd = nil
local COL = { cyan = 0xFFFFE633, green = 0xFF4DFF4D, amber = 0xFF00A6FF, red = 0xFF4040FF,
              grey = 0xFF999999, white = 0xFFFFFFFF }

local function opt(fname, ...)
    local f = imgui[fname]
    if f then return f(...) end
end

local function text(s, col)
    if col then imgui.PushStyleColor(imgui.constant.Col.Text, col) end
    imgui.TextUnformatted(s)
    if col then imgui.PopStyleColor() end
end

local function label(s, col)
    opt("AlignTextToFramePadding")
    text(s, col)
end

local FONT_H = 13

-- Zeichenbefehle der Karte (siehe unten). DL.H gesetzt = Zeichnen in die Cockpit-Textur (ND im 3D-Cockpit)
local DL = { n = 0, cap = 61000, H = nil, W = 0, Hh = 0 }

-------------------------------------------------------------------------------
-- Strichschrift fuer die Karte im Cockpit-Display (Hershey "Simplex", gemeinfrei):
-- je Zeichen links/rechts-Grenze und Linienzuege, Koordinaten = Zeichen - 'R', y nach unten,
-- Grossbuchstaben von -12 bis 9 (21 Einheiten). " R" = Stift heben.
-------------------------------------------------------------------------------
local HERSHEY = [==[
JZ
MWRFRT RRYQZR[SZRY
JZNFNM RVFVM
H]SBLb RYBRb RLOZO RKUYU
H\PBP_ RTBT_ RYIWGTFPFMGKIKKLMMNOOUQWRXSYUYXWZT[P[MZKX
F^[FI[ RNFPHPJOLMMKMIKIIJGLFNFPGSHVHYG[F RWTUUTWTYV[X[ZZ[X[VYTWT
E_\O\N[MZMYNXPVUTXRZP[L[JZIYHWHUISJRQNRMSKSIRGPFNGMIMKNNPQUXWZY[[[\Z\Y
MWRHQGRFSGSIRKQL
KYVBTDRGPKOPOTPYR]T`Vb
KYNBPDRGTKUPUTTYR]P`Nb
JZRLRX RMOWU RWOMU
E_RIR[ RIR[R
NVSWRXQWRVSWSYQ[
E_IR[R
NVRVQWRXSWRV
G][BIb
H\QFNGLJKOKRLWNZQ[S[VZXWYRYOXJVGSFQF
H\NJPISFS[
H\LKLJMHNGPFTFVGWHXJXLWNUQK[Y[
H\MFXFRNUNWOXPYSYUXXVZS[P[MZLYKW
H\UFKTZT RUFU[
H\WFMFLOMNPMSMVNXPYSYUXXVZS[P[MZLYKW
H\XIWGTFRFOGMJLOLTMXOZR[S[VZXXYUYTXQVOSNRNOOMQLT
H\YFO[ RKFYF
H\PFMGLILKMMONSOVPXRYTYWXYWZT[P[MZLYKWKTLRNPQOUNWMXKXIWGTFPF
H\XMWPURRSQSNRLPKMKLLINGQFRFUGWIXMXRWWUZR[P[MZLX
NVROQPRQSPRO RRVQWRXSWRV
NVROQPRQSPRO RSWRXQWRVSWSYQ[
F^ZIJRZ[
E_IO[O RIU[U
F^JIZRJ[
I[LKLJMHNGPFTFVGWHXJXLWNVORQRT RRYQZR[SZRY
E`WNVLTKQKOLNMMPMSNUPVSVUUVS RQKOMNPNSOUPV RWKVSVUXVZV\T]Q]O\L[JYHWGTFQFNGLHJJILHOHRIUJWLYNZQ[T[WZYYZX RXKWSWUXV
I[RFJ[ RRFZ[ RMTWT
G\KFK[ RKFTFWGXHYJYLXNWOTP RKPTPWQXRYTYWXYWZT[K[
H]ZKYIWGUFQFOGMILKKNKSLVMXOZQ[U[WZYXZV
G\KFK[ RKFRFUGWIXKYNYSXVWXUZR[K[
H[LFL[ RLFYF RLPTP RL[Y[
HZLFL[ RLFYF RLPTP
H]ZKYIWGUFQFOGMILKKNKSLVMXOZQ[U[WZYXZVZS RUSZS
G]KFK[ RYFY[ RKPYP
NVRFR[
JZVFVVUYTZR[P[NZMYLVLT
G\KFK[ RYFKT RPOY[
HYLFL[ RL[X[
F^JFJ[ RJFR[ RZFR[ RZFZ[
G]KFK[ RKFY[ RYFY[
G]PFNGLIKKJNJSKVLXNZP[T[VZXXYVZSZNYKXIVGTFPF
G\KFK[ RKFTFWGXHYJYMXOWPTQKQ
G]PFNGLIKKJNJSKVLXNZP[T[VZXXYVZSZNYKXIVGTFPF RSWY]
G\KFK[ RKFTFWGXHYJYLXNWOTPKP RRPY[
H\YIWGTFPFMGKIKKLMMNOOUQWRXSYUYXWZT[P[MZKX
JZRFR[ RKFYF
G]KFKULXNZQ[S[VZXXYUYF
I[JFR[ RZFR[
F^HFM[ RRFM[ RRFW[ R\FW[
H\KFY[ RYFK[
I[JFRPR[ RZFRP
H\YFK[ RKFYF RK[Y[
KYOBOb RPBPb ROBVB RObVb
KYKFY^
KYTBTb RUBUb RNBUB RNbUb
JZRDJR RRDZR
I[Ib[b
NVSKQMQORPSORNQO
I\XMX[ RXPVNTMQMONMPLSLUMXOZQ[T[VZXX
H[LFL[ RLPNNPMSMUNWPXSXUWXUZS[P[NZLX
I[XPVNTMQMONMPLSLUMXOZQ[T[VZXX
I\XFX[ RXPVNTMQMONMPLSLUMXOZQ[T[VZXX
I[LSXSXQWOVNTMQMONMPLSLUMXOZQ[T[VZXX
MYWFUFSGRJR[ ROMVM
I\XMX]W`VaTbQbOa RXPVNTMQMONMPLSLUMXOZQ[T[VZXX
I\MFM[ RMQPNRMUMWNXQX[
NVQFRGSFREQF RRMR[
MWRFSGTFSERF RSMS^RaPbNb
IZMFM[ RWMMW RQSX[
NVRFR[
CaGMG[ RGQJNLMOMQNRQR[ RRQUNWMZM\N]Q][
I\MMM[ RMQPNRMUMWNXQX[
I\QMONMPLSLUMXOZQ[T[VZXXYUYSXPVNTMQM
H[LMLb RLPNNPMSMUNWPXSXUWXUZS[P[NZLX
I\XMXb RXPVNTMQMONMPLSLUMXOZQ[T[VZXX
KXOMO[ ROSPPRNTMWM
J[XPWNTMQMNNMPNRPSUTWUXWXXWZT[Q[NZMX
MYRFRWSZU[W[ ROMVM
I\MMMWNZP[S[UZXW RXMX[
JZLMR[ RXMR[
G]JMN[ RRMN[ RRMV[ RZMV[
J[MMX[ RXMM[
JZLMR[ RXMR[P_NaLbKb
J[XMM[ RMMXM RM[X[
KYTBRCQDPFPHQJRKSMSOQQ RRCQEQGRISJTLTNSPORSTTVTXSZR[Q]Q_Ra RQSSUSWRYQZP\P^Q`RaTb
NVRBRb
KYPBRCSDTFTHSJRKQMQOSQ RRCSESGRIQJPLPNQPURQTPVPXQZR[S]S_Ra RSSQUQWRYSZT\T^S`RaPb
F^IUISJPLONOPPTSVTXTZS[Q RISJQLPNPPQTTVUXUZT[Q[O
]==]
local SF = { g = {}, cap = 9, xs = 0.84, th = 1.15 }
do
    local code = 32
    for line in HERSHEY:gmatch("[^\n]+") do
        local g = { l = line:byte(1) - 82, r = line:byte(2) - 82, s = {} }
        local cur
        for i = 3, #line - 1, 2 do
            local a, b = line:byte(i), line:byte(i + 1)
            if a == 32 and b == 82 then
                cur = nil
            else
                if not cur then cur = {}; g.s[#g.s + 1] = cur end
                cur[#cur + 1], cur[#cur + 2] = a - 82, b - 82
            end
        end
        SF.g[code] = g
        code = code + 1
    end
    -- Gradzeichen (Code 1): kleiner Kreis oben
    local c = {}
    for k = 0, 12 do
        local a = k / 12 * 2 * math.pi
        c[#c + 1], c[#c + 2] = 3 * math.cos(a), -9 + 3 * math.sin(a)
    end
    SF.g[1] = { l = -5, r = 5, s = { c } }
end
-- UTF-8 auf den Zeichenvorrat abbilden
local function sf_prep(s)
    if not s:find("[\128-\255]") then return s end
    s = s:gsub("\194\176", "\1"):gsub("\194\169", "(C)"):gsub("\195[\132\164]", "AE"):gsub("\195[\150\182]", "OE")
    s = s:gsub("\195[\156\188]", "UE"):gsub("\195\159", "SS")
    return (s:gsub("[\192-\255][\128-\191]*", "?"))
end
local function sf_width(str)
    local s = sf_prep(str)
    local k = SF.cap / 21 * SF.xs
    local w = 0
    for i = 1, #s do
        local g = SF.g[s:byte(i)] or SF.g[63]
        w = w + (g.r - g.l)
    end
    return w * k
end

-- Schrift fuer das Cockpit-Display: B612 Mono Bold (die Airbus-Cockpitschrift, SIL Open Font License).
-- Scripts/A350_BTV_data/font_b612_px.bin: je Groesse (Versalhoehe 8..22 Panel-Pixel) eigene, gehintete Zeichen;
-- gezeichnet wird 1:1 auf ganze Panel-Pixel - so scharf wie die Schrift des FF-ND. Gedrehter Text linear gefiltert.
-- Aeltere Datei font_b612.bin (ein grosser Satz mit Mip-Stufen) geht weiterhin; ohne beide: Strichschrift.
local TF = { ok = false, xs = 0.92, v = 1, ppu = 1.675, ppx = 1.68 }
-- Schriftgroessen auf dem ND (Versalhoehe in ND-Einheiten, 1 Einheit = 1,675 Panel-Pixel): 11, 10 und 13 Pixel
-- wie im FF-ND; h = Hoehe des Zeilenkastens
local TXT = { cap = 6.57, small = 5.97, big = 7.76, h = 10.2 }

-- Atlasgroesse (v2) fuer eine Versalhoehe: Eintrag und Faktor (exact: 1 = pixelgenau, solange es die Groesse gibt)
local function tf_pick(cap, exact)
    local px = cap * TF.ppu
    local k = math.floor(px + 0.5)
    local tab, lo, hi = TF.by_cap, TF.cmin, TF.cmax
    -- TF.reg: normale (nicht fette) Schrift, z.B. "BTV" im FMA wie die FF-Schrift daneben
    if TF.reg and TF.by_cap_r and TF.rmin then tab, lo, hi = TF.by_cap_r, TF.rmin, TF.rmax end
    if k < lo then k = lo elseif k > hi then k = hi end
    local sz = tab[k]
    if exact and math.abs(px - k) <= 0.75 then return sz, 1 end
    return sz, px / k
end

local function text_w(str, cap)
    if DL.H then
        cap = cap or TXT.cap
        if TF.ok then
            local n = #sf_prep(str)
            if TF.v == 2 then
                local sz, f = tf_pick(cap, true)
                return n * sz.adv * f / TF.ppx
            end
            return n * TF.adv * TF.xs * cap / TF.cap_px
        end
        local c = SF.cap
        SF.cap = cap
        local w = sf_width(str)
        SF.cap = c
        return w
    end
    -- (nur im ImGui-Fenster; im Cockpit-Zeichnen waere das ein Absturz)
    if DL.panel then return #str * (cap or TXT.cap) * 0.8 end
    if imgui.CalcTextSize then
        local w = imgui.CalcTextSize(str)
        if type(w) == "number" then return w end
    end
    return #str * 7
end

local MODE_TXT = { OFF = "BTV OFF", ARMED = "BTV ARMED", ENGAGED = "BTV ACTIVE", RELEASE = "BTV ACTIVE", DONE = "BTV COMPLETE" }
local MODE_COL = { OFF = COL.grey, ARMED = COL.cyan, ENGAGED = COL.green, RELEASE = COL.green, DONE = COL.white }

-- aktuelles Ziel (Exit oder Stoppunkt) in Pistenkoordinaten
local function current_target()
    if (S.mode == "ENGAGED" or S.mode == "RELEASE") and S.target then
        return S.target.s, S.target.name, S.target.exit
    elseif S.exit then
        return S.exit.s, S.exit.name, S.exit
    elseif S.manual_dist and S.rwy then
        return S.rwy.disp + S.manual_dist, "STOP", nil
    end
end

-- geschaetzte Pistenbelegungszeit Schwelle -> Exit
local function rot_estimate(ts, x)
    local E = S.rwy
    local v = S.v_est
    local ve = exit_speed(x)
    local td = P.tdd(E)
    local D = ts - (E.disp + td)
    if D <= 0 or v <= ve then return nil end
    return td / v + 2 * D / (v + ve) + ((x and x.ret) and 5 or 10)
end

-------------------------------------------------------------------------------
-- Karte im OANS-Stil (Rollkarte + BTV)
-------------------------------------------------------------------------------
local MC = {
    -- OANS-Farben (ABGR); Flaechen, Rand, Linien und Gebaeude aus Bildern des echten A350-OANS:
    -- Vorfeld (57,57,67), Rollweg und Piste nah (156,158,181), Rand (131,59,33), Parkposition (100,102,123),
    -- Mittellinie (223,210,108), Gebaeude hellblau (0,190,206) und blau (0,142,255); Piste weit weg weiss
    bg = 0xFF000000, apron = 0xFF433939, pave = 0xFFB59E9C, shoulder = 0xFF213B83, runway = 0xFFB59E9C,
    rwy_far = 0xFFEBEBEB,
    stand_area = 0xFF7B6664, stand_line = 0xFF433939, terminal = 0xFFCEBE00,
    taxiline = 0xFF6CD2DF, hold = 0xFF2828E6,
    label = 0xFF00D4F2, white = 0xFFFFFFFF, green = 0xFF00FF00, sel = 0xFFFFE633,
    ffw = 0xFFDBF0FF, -- warmes Weiss der FF-Meldungskaesten (TA ONLY), aus einem Bild des FF-ND
    mag = 0xFFFF40FF, acft = 0xFFE475FA, ring = 0xFFE6E6E6, amber = 0xFF00A6FF,
    short = 0xFF5050FF, rec = 0xFF4DFF4D, grey = 0xFF9A9A9A, red = 0xFF2828FF,
    stand_txt = 0xFFFFE6A0, stand_bg = 0xFF3A2410, stand_frame = 0xFF8A6A40, building = 0xFFFF8E00,
    twy = 0xFF5E5E5E, blast = 0xFF4A4A4A, chevron = 0xFF00D4F2,
    key_bg = 0xFF1A1A1A, key_frame = 0xFFFFFFFF, -- Rahmen der Tasten, Felder, Reiter und Listen: weiss wie im A350
    dis = 0xFF6E6E6E, menu_bg = 0xFF333333, menu_hi = 0xFF575757, tab_bg = 0xFF3C3C3C,
    cross = 0xFF00E6FF, cursor = 0xFFE475FA, lubber = 0xFF00E6FF, road = 0xFFBEBEBE,
}
local MV = { press = false, drag = 0, sy_set = nil, h_last = 0 }

-- Zeichenbefehle der Karte mit Eckpunkt-Zaehler: ImGui adressiert je Fenster nur 65535 Eckpunkte
-- (16-Bit-Indizes); darueber entstehen grosse Dreiecks-Artefakte. Kosten mit Kantenglaettung, eher hoch geschaetzt.
-- Mit DL.H (Karte im Cockpit-Display) werden lokale Kartenkoordinaten [0,W]x[0,Hh] zugeschnitten, auf die
-- Panel-Textur abgebildet und in einen Eckpunkt-Puffer geschrieben, der in einem Rutsch mit OpenGL gezeichnet wird.
function DL.map(x, y)
    local H = DL.H
    local w = H[7] * x + H[8] * y + 1
    return (H[1] * x + H[2] * y + H[3]) / w, (H[4] * x + H[5] * y + H[6]) / w
end

-- Eckpunkt-Puffer fuer OpenGL (Panel-Koordinaten); wird beim ersten Gebrauch angelegt (LuaJIT-FFI)
-- t: Texturkoordinaten (Schriftatlas); Flaechen und Linien nehmen einen weissen Fleck im Atlas (wu, wv)
local GLB = { n = 0, cap = 0, v = nil, c = nil, t = nil, wu = 0, wv = 0, px = 0.125, ts = 0.3 } -- px: Panel-Einheiten je Geraetepixel, ts: je Kartenpixel
-- Ein-/Ausblenden: Deckkraft beim Eintragen verringern (GLB.fa = 0..1; nil = voll)
function GLB.fade(col)
    local a = math.floor(bit.rshift(col, 24) * GLB.fa + 0.5)
    return bit.bor(bit.band(col, 0x00FFFFFF), bit.lshift(a, 24))
end
local function glb_tri(ax, ay, bx, by, cx, cy, col)
    local n = GLB.n
    if n + 3 > GLB.cap then return end
    if GLB.fa then col = GLB.fade(col) end
    local v, c, t, i = GLB.v, GLB.c, GLB.t, 2 * n
    v[i], v[i + 1], v[i + 2], v[i + 3], v[i + 4], v[i + 5] = ax, ay, bx, by, cx, cy
    local wu, wv = GLB.wu, GLB.wv
    t[i], t[i + 1], t[i + 2], t[i + 3], t[i + 4], t[i + 5] = wu, wv, wu, wv, wu, wv
    c[n], c[n + 1], c[n + 2] = col, col, col
    GLB.n = n + 3
end
-- achsenparalleles Rechteck mit Texturausschnitt (Schriftzeichen): a = Ecke zu (u0,v0), b = Ecke zu (u1,v1)
local function glb_quad_uv(ax, ay, bx, by, u0, v0, u1, v1, col)
    local n = GLB.n
    if n + 6 > GLB.cap then return end
    if GLB.fa then col = GLB.fade(col) end
    local v, c, t, i = GLB.v, GLB.c, GLB.t, 2 * n
    v[i], v[i + 1], v[i + 2], v[i + 3], v[i + 4], v[i + 5] = ax, ay, bx, ay, bx, by
    v[i + 6], v[i + 7], v[i + 8], v[i + 9], v[i + 10], v[i + 11] = ax, ay, bx, by, ax, by
    t[i], t[i + 1], t[i + 2], t[i + 3], t[i + 4], t[i + 5] = u0, v0, u1, v0, u1, v1
    t[i + 6], t[i + 7], t[i + 8], t[i + 9], t[i + 10], t[i + 11] = u0, v0, u1, v1, u0, v1
    for k = 0, 5 do c[n + k] = col end
    GLB.n = n + 6
end
-- beliebiges Viereck mit Texturausschnitt (gedrehte Schrift): Ecken oben links, oben rechts, unten rechts, unten links
local function glb_quad4(x1, y1, x2, y2, x3, y3, x4, y4, u0, v0, u1, v1, col)
    local n = GLB.n
    if n + 6 > GLB.cap then return end
    if GLB.fa then col = GLB.fade(col) end
    local v, c, t, i = GLB.v, GLB.c, GLB.t, 2 * n
    v[i], v[i + 1], v[i + 2], v[i + 3], v[i + 4], v[i + 5] = x1, y1, x2, y2, x3, y3
    v[i + 6], v[i + 7], v[i + 8], v[i + 9], v[i + 10], v[i + 11] = x1, y1, x3, y3, x4, y4
    t[i], t[i + 1], t[i + 2], t[i + 3], t[i + 4], t[i + 5] = u0, v0, u1, v0, u1, v1
    t[i + 6], t[i + 7], t[i + 8], t[i + 9], t[i + 10], t[i + 11] = u0, v0, u1, v1, u0, v1
    for k = 0, 5 do c[n + k] = col end
    GLB.n = n + 6
end
-- Zeichenflaeche setzen: H = Abbildung lokal -> Panel (achsenparallel), W x Hh lokale Groesse
function DL.begin(H, W, Hh)
    DL.H, DL.W, DL.Hh = H, W, Hh
    TF.ppx, TF.ppu = H[1], -H[5]
    DL.clip = { H[3], H[1] * W + H[3], H[5] * Hh + H[6], H[6] }
end
-- Linie mit halber Breite hw (Panel-Einheiten), eckigen Enden und einem Geraetepixel weichem Rand
local function glb_line(ax, ay, bx, by, col, hw)
    local dx, dy = bx - ax, by - ay
    local L = math.sqrt(dx * dx + dy * dy)
    if L < 1e-9 then return end
    local n = GLB.n
    if n + 18 > GLB.cap then return end
    if GLB.fa then col = GLB.fade(col) end
    local f = GLB.px
    if hw < f * 0.5 then hw = f * 0.5 end
    local ux, uy = dx / L, dy / L
    ax, ay, bx, by = ax - ux * hw, ay - uy * hw, bx + ux * hw, by + uy * hw
    local nx, ny = -uy, ux
    local ix, iy, ox, oy = nx * hw, ny * hw, nx * (hw + f), ny * (hw + f)
    local c0 = bit.band(col, 0x00FFFFFF)
    local v, c, i = GLB.v, GLB.c, 2 * n
    local t, wu, wv = GLB.t, GLB.wu, GLB.wv
    for k = i, i + 35, 2 do t[k], t[k + 1] = wu, wv end
    -- Kern
    local p1x, p1y, p2x, p2y = ax + ix, ay + iy, bx + ix, by + iy
    local p3x, p3y, p4x, p4y = bx - ix, by - iy, ax - ix, ay - iy
    v[i], v[i + 1], v[i + 2], v[i + 3], v[i + 4], v[i + 5] = p1x, p1y, p2x, p2y, p3x, p3y
    v[i + 6], v[i + 7], v[i + 8], v[i + 9], v[i + 10], v[i + 11] = p1x, p1y, p3x, p3y, p4x, p4y
    for k = 0, 5 do c[n + k] = col end
    -- Rand links (p1-p2 innen, aussen transparent)
    local q1x, q1y, q2x, q2y = ax + ox, ay + oy, bx + ox, by + oy
    v[i + 12], v[i + 13], v[i + 14], v[i + 15], v[i + 16], v[i + 17] = p1x, p1y, p2x, p2y, q2x, q2y
    v[i + 18], v[i + 19], v[i + 20], v[i + 21], v[i + 22], v[i + 23] = p1x, p1y, q2x, q2y, q1x, q1y
    c[n + 6], c[n + 7], c[n + 8], c[n + 9], c[n + 10], c[n + 11] = col, col, c0, col, c0, c0
    -- Rand rechts
    local r1x, r1y, r2x, r2y = ax - ox, ay - oy, bx - ox, by - oy
    v[i + 24], v[i + 25], v[i + 26], v[i + 27], v[i + 28], v[i + 29] = p4x, p4y, p3x, p3y, r2x, r2y
    v[i + 30], v[i + 31], v[i + 32], v[i + 33], v[i + 34], v[i + 35] = p4x, p4y, r2x, r2y, r1x, r1y
    c[n + 12], c[n + 13], c[n + 14], c[n + 15], c[n + 16], c[n + 17] = col, col, c0, col, c0, c0
    GLB.n = n + 18
end

-- ein Durchgang des Polygon-Zuschnitts (Sutherland-Hodgman) an einer Rechteckkante
local function dl_clip_pass(ix, iy, axis, lim, keep_ge)
    local ox, oy = {}, {}
    local m = #ix
    if m == 0 then return ox, oy end
    local px, py = ix[m], iy[m]
    local pv = (axis == 1) and px or py
    local pin
    if keep_ge then pin = pv >= lim else pin = pv <= lim end
    for k = 1, m do
        local cx, cy = ix[k], iy[k]
        local cv = (axis == 1) and cx or cy
        local cin
        if keep_ge then cin = cv >= lim else cin = cv <= lim end
        if cin ~= pin then
            local t = (lim - pv) / (cv - pv)
            ox[#ox + 1], oy[#oy + 1] = px + (cx - px) * t, py + (cy - py) * t
        end
        if cin then ox[#ox + 1], oy[#oy + 1] = cx, cy end
        px, py, pv, pin = cx, cy, cv, cin
    end
    return ox, oy
end
-- konvexes Polygon (lokal) zuschneiden, abbilden und als Faecher zeichnen
local function dl_poly(xs, ys, col)
    local W, Hh = DL.W, DL.Hh
    xs, ys = dl_clip_pass(xs, ys, 1, 0, true)
    xs, ys = dl_clip_pass(xs, ys, 1, W, false)
    xs, ys = dl_clip_pass(xs, ys, 2, 0, true)
    xs, ys = dl_clip_pass(xs, ys, 2, Hh, false)
    local n = #xs
    if n < 3 then return end
    local ax, ay = DL.map(xs[1], ys[1])
    local bx, by = DL.map(xs[2], ys[2])
    for k = 3, n do
        local cx, cy = DL.map(xs[k], ys[k])
        DL.n = DL.n + 6
        glb_tri(ax, ay, bx, by, cx, cy, col)
        bx, by = cx, cy
    end
end
function DL.tri(x1, y1, x2, y2, x3, y3, col)
    if not DL.H then
        DL.n = DL.n + 6; imgui.DrawList_AddTriangleFilled(x1, y1, x2, y2, x3, y3, col)
        return
    end
    local W, Hh = DL.W, DL.Hh
    local minx, maxx = math.min(x1, x2, x3), math.max(x1, x2, x3)
    local miny, maxy = math.min(y1, y2, y3), math.max(y1, y2, y3)
    if maxx < 0 or minx > W or maxy < 0 or miny > Hh then return end
    if minx >= 0 and maxx <= W and miny >= 0 and maxy <= Hh then
        local a, b = DL.map(x1, y1)
        local c, d = DL.map(x2, y2)
        local e, f = DL.map(x3, y3)
        DL.n = DL.n + 6; glb_tri(a, b, c, d, e, f, col)
        return
    end
    dl_poly({ x1, x2, x3 }, { y1, y2, y3 }, col)
end
local function dl_lb(pp, qq, t0, t1)
    if pp == 0 then
        if qq < 0 then return nil end
        return t0, t1
    end
    local r = qq / pp
    if pp < 0 then
        if r > t1 then return nil end
        if r > t0 then t0 = r end
    else
        if r < t0 then return nil end
        if r < t1 then t1 = r end
    end
    return t0, t1
end
function DL.line(x1, y1, x2, y2, col, th)
    if not DL.H then
        DL.n = DL.n + 8; imgui.DrawList_AddLine(x1, y1, x2, y2, col, th)
        return
    end
    -- Zuschnitt (Liang-Barsky) auf [0,W]x[0,Hh]
    local W, Hh = DL.W, DL.Hh
    local dx, dy = x2 - x1, y2 - y1
    local t0, t1 = 0, 1
    if not (x1 >= 0 and x1 <= W and y1 >= 0 and y1 <= Hh and x2 >= 0 and x2 <= W and y2 >= 0 and y2 <= Hh) then
        t0, t1 = dl_lb(-dx, x1, t0, t1)
        if t0 then t0, t1 = dl_lb(dx, W - x1, t0, t1) end
        if t0 then t0, t1 = dl_lb(-dy, y1, t0, t1) end
        if t0 then t0, t1 = dl_lb(dy, Hh - y1, t0, t1) end
        if not t0 then return end
    end
    local ax, ay = DL.map(x1 + t0 * dx, y1 + t0 * dy)
    local bx, by = DL.map(x1 + t1 * dx, y1 + t1 * dy)
    DL.n = DL.n + 8; glb_line(ax, ay, bx, by, col, (th or 1) * GLB.ts * 0.5)
end
function DL.quadf(x1, y1, x2, y2, x3, y3, x4, y4, col)
    if not DL.H then
        DL.n = DL.n + 8; imgui.DrawList_AddQuadFilled(x1, y1, x2, y2, x3, y3, x4, y4, col)
        return
    end
    DL.tri(x1, y1, x2, y2, x3, y3, col)
    DL.tri(x1, y1, x3, y3, x4, y4, col)
end
function DL.quad(x1, y1, x2, y2, x3, y3, x4, y4, col, th)
    if not DL.H then
        DL.n = DL.n + 16; imgui.DrawList_AddQuad(x1, y1, x2, y2, x3, y3, x4, y4, col, th)
        return
    end
    DL.line(x1, y1, x2, y2, col, th); DL.line(x2, y2, x3, y3, col, th)
    DL.line(x3, y3, x4, y4, col, th); DL.line(x4, y4, x1, y1, col, th)
end
function DL.rectf(x1, y1, x2, y2, col)
    if not DL.H then
        DL.n = DL.n + 4; imgui.DrawList_AddRectFilled(x1, y1, x2, y2, col)
        return
    end
    DL.quadf(x1, y1, x2, y1, x2, y2, x1, y2, col)
end
function DL.rect(x1, y1, x2, y2, col, rnd, fl, th)
    if not DL.H then
        DL.n = DL.n + 16; imgui.DrawList_AddRect(x1, y1, x2, y2, col, rnd, fl, th)
        return
    end
    DL.quad(x1, y1, x2, y1, x2, y2, x1, y2, col, th)
end
-- Text in der Strichschrift: x, y = linke obere Ecke des Textkastens (FONT_H hoch)
local function sf_draw(x, y, col, str, cap)
    local s = sf_prep(str)
    local k = (cap or SF.cap) / 21
    local kx = k * SF.xs
    local by = y + (FONT_H - (cap or SF.cap)) / 2 + 12 * k
    if x > DL.W or y > DL.Hh or y + FONT_H < 0 then return end
    local th = SF.th
    for i = 1, #s do
        local g = SF.g[s:byte(i)] or SF.g[63]
        local ox = x - g.l * kx
        for _, st in ipairs(g.s) do
            local px, py = ox + st[1] * kx, by + st[2] * k
            for j = 3, #st - 1, 2 do
                local qx, qy = ox + st[j] * kx, by + st[j + 1] * k
                DL.line(px, py, qx, qy, col, th)
                px, py = qx, qy
            end
        end
        x = x + (g.r - g.l) * kx
        if x > DL.W then return end
    end
end
do
-- Text aus dem Schriftatlas: x, y = linke obere Ecke des Textkastens (FONT_H hoch), cap = Versalhoehe
local function tf_index(ch)
    if ch == 1 then return TF.count - 1 elseif ch > 32 and ch < 127 then return ch - 32 end
end
local function tf_draw(x, y, col, str, cap)
    local s = sf_prep(str)
    cap = cap or TXT.cap
    local W, Hh = DL.W, DL.Hh
    if TF.v == 2 then
        -- pixelgenau: Grundlinie und Zeichen auf ganze Panel-Pixel
        local H, cl = DL.H, DL.clip
        local sz, f = tf_pick(cap, true)
        local yb = y + (FONT_H + cap) / 2
        local bx, by = H[1] * x + H[3], H[5] * yb + H[6]
        if f == 1 then bx, by = math.floor(bx + 0.5), math.floor(by + 0.5) end
        local adv, cw, chh = sz.adv * f, sz.cw * f, sz.ch * f
        local top = by + sz.base * f
        local bot = top - chh
        if top < cl[3] or bot > cl[4] then return end
        local du, dv = sz.cw / TF.W, sz.ch / TF.H
        for i = 1, #s do
            local idx = tf_index(s:byte(i))
            if idx then
                local x0p = bx + (i - 1) * adv - sz.left * f
                local x1p = x0p + cw
                if x1p > cl[1] and x0p < cl[2] then
                    local u0 = (sz.x0 + (idx % sz.cols) * sz.cw) / TF.W
                    local v0 = (sz.y0 + math.floor(idx / sz.cols) * sz.ch) / TF.H
                    local u1, v1 = u0 + du, v0 + dv
                    local ax, ay, qx, qy = x0p, top, x1p, bot
                    if ax < cl[1] then u0 = u0 + du * (cl[1] - ax) / cw; ax = cl[1] end
                    if qx > cl[2] then u1 = u1 - du * (qx - cl[2]) / cw; qx = cl[2] end
                    if ay > cl[4] then v0 = v0 + dv * (ay - cl[4]) / chh; ay = cl[4] end
                    if qy < cl[3] then v1 = v1 - dv * (cl[3] - qy) / chh; qy = cl[3] end
                    DL.n = DL.n + 6
                    glb_quad_uv(ax, ay, qx, qy, u0, v0, u1, v1, col)
                end
            end
        end
        return
    end
    local k = cap / TF.cap_px
    local kx = k * TF.xs
    local cy0 = y + (FONT_H - cap) / 2 - TF.cap_top * k -- Oberkante der Zeichenzelle
    local cy1 = cy0 + TF.chh * k
    if cy1 < 0 or cy0 > Hh then return end
    local gw = TF.cw * kx
    local xx = x - TF.pad * kx
    local du, dv = TF.cw / TF.W, TF.chh / TF.H
    for i = 1, #s do
        local idx = tf_index(s:byte(i))
        if idx then
            local x0, x1, y0, y1 = xx, xx + gw, cy0, cy1
            if x1 > 0 and x0 < W then
                local u0 = (idx % TF.cols) * du
                local v0 = math.floor(idx / TF.cols) * dv
                local u1, v1 = u0 + du, v0 + dv
                -- am Rand der Zeichenflaeche abschneiden (Texturkoordinaten mitfuehren)
                if x0 < 0 then u0 = u0 + du * (0 - x0) / gw; x0 = 0 end
                if x1 > W then u1 = u1 - du * (x1 - W) / gw; x1 = W end
                if y0 < 0 then v0 = v0 + dv * (0 - y0) / (cy1 - cy0); y0 = 0 end
                if y1 > Hh then v1 = v1 - dv * (y1 - Hh) / (cy1 - cy0); y1 = Hh end
                local ax, ay = DL.map(x0, y0)
                local bx, by = DL.map(x1, y1)
                DL.n = DL.n + 6
                glb_quad_uv(ax, ay, bx, by, u0, v0, u1, v1, col)
            end
        end
        xx = xx + TF.adv * kx
        if xx > W then return end
    end
end

function DL.text(x, y, col, s, cap)
    if DL.H then
        if TF.ok then tf_draw(x, y, col, s, cap) else sf_draw(x, y, col, s, cap or TXT.cap) end
        return
    end
    DL.n = DL.n + 4 * #s; imgui.DrawList_AddText(x, y, col, s)
end

-- Breite von gedrehtem Text (lokale Einheiten)
function DL.text_rot_w(s, cap)
    cap = cap or TXT.cap
    if DL.H and TF.ok and TF.v == 2 then
        local sz, f = tf_pick(cap, false)
        return #sf_prep(s) * sz.adv * f / TF.ppu
    end
    return text_w(s, cap)
end

-- gedrehter Text: Mitte (cx, cy), ang = Richtung der Grundlinie (Bogenmass, Bildschirm: y nach unten)
function DL.text_rot(cx, cy, ang, col, s, cap)
    cap = cap or TXT.cap
    if not (DL.H and TF.ok and TF.v == 2) then
        local w = text_w(s, cap)
        DL.text(cx - w / 2, cy - FONT_H / 2, col, s, cap)
        return
    end
    s = sf_prep(s)
    local sz, f = tf_pick(cap, false)
    local k = f / TF.ppu -- lokale Einheiten je Atlaspixel
    local ux, uy = math.cos(ang), math.sin(ang)
    local vx, vy = -uy, ux
    local n = #s
    local w = n * sz.adv * k
    local ox = cx - ux * w / 2 + vx * sz.cap * k / 2
    local oy = cy - uy * w / 2 + vy * sz.cap * k / 2
    local W, Hh = DL.W, DL.Hh
    local du, dv = sz.cw / TF.W, sz.ch / TF.H
    local gy0, gy1 = -sz.base * k, (sz.ch - sz.base) * k
    for i = 1, n do
        local idx = tf_index(s:byte(i))
        if idx then
            local gx0 = ((i - 1) * sz.adv - sz.left) * k
            local gx1 = gx0 + sz.cw * k
            local p1x, p1y = ox + ux * gx0 + vx * gy0, oy + uy * gx0 + vy * gy0
            local p2x, p2y = ox + ux * gx1 + vx * gy0, oy + uy * gx1 + vy * gy0
            local p3x, p3y = ox + ux * gx1 + vx * gy1, oy + uy * gx1 + vy * gy1
            local p4x, p4y = ox + ux * gx0 + vx * gy1, oy + uy * gx0 + vy * gy1
            local mnx, mxx = math.min(p1x, p2x, p3x, p4x), math.max(p1x, p2x, p3x, p4x)
            local mny, mxy = math.min(p1y, p2y, p3y, p4y), math.max(p1y, p2y, p3y, p4y)
            if mnx >= 0 and mxx <= W and mny >= 0 and mxy <= Hh then
                local u0 = (sz.x0 + (idx % sz.cols) * sz.cw) / TF.W
                local v0 = (sz.y0 + math.floor(idx / sz.cols) * sz.ch) / TF.H
                local a1, b1 = DL.map(p1x, p1y)
                local a2, b2 = DL.map(p2x, p2y)
                local a3, b3 = DL.map(p3x, p3y)
                local a4, b4 = DL.map(p4x, p4y)
                DL.n = DL.n + 6
                glb_quad4(a1, b1, a2, b2, a3, b3, a4, b4, u0, v0, u0 + du, v0 + dv, col)
            end
        end
    end
end
end

function DL.circle(x, y, r, col, seg, th)
    if not DL.H then
        DL.n = DL.n + 4 * (seg or 24); imgui.DrawList_AddCircle(x, y, r, col, seg, th)
        return
    end
    local n = seg or 24
    local px, py = x + r, y
    for k = 1, n do
        local a = k / n * 2 * math.pi
        local qx, qy = x + r * math.cos(a), y + r * math.sin(a)
        DL.line(px, py, qx, qy, col, th)
        px, py = qx, qy
    end
end
function DL.circlef(x, y, r, col, seg)
    if not DL.H then
        DL.n = DL.n + 2 * (seg or 24); imgui.DrawList_AddCircleFilled(x, y, r, col, seg)
        return
    end
    local n = seg or 24
    local px, py = x + r, y
    for k = 1, n do
        local a = k / n * 2 * math.pi
        local qx, qy = x + r * math.cos(a), y + r * math.sin(a)
        DL.tri(x, y, px, py, qx, qy, col)
        px, py = qx, qy
    end
end

local function overlap(r, list)
    for _, q in ipairs(list) do
        if r[1] < q[3] and q[1] < r[3] and r[2] < q[4] and q[2] < r[4] then return true end
    end
    return false
end

local function box_text(x, y, str, col, frame_col, bg_col)
    local w = text_w(str)
    DL.rectf(x - 2, y - 1, x + w + 2, y + FONT_H + 1, bg_col or MC.bg)
    if frame_col then DL.rect(x - 3, y - 2, x + w + 3, y + FONT_H + 2, frame_col, 0, 0, 1.5) end
    DL.text(x, y, col, str)
end

local function segs_text(x, y, segs)
    for _, sg in ipairs(segs) do
        DL.text(x, y, sg[2], sg[1])
        x = x + text_w(sg[1])
    end
    return x
end

-- eigenes Flugzeug wie im A350-OANS: duenne magentafarbene Linien ohne Rand (Rumpf, gerade Flaechen, Leitwerk),
-- Verhaeltnisse aus einem Bild des echten Symbols: Laenge L, Spannweite 0,90 L bei 0,42 L hinter der Nase,
-- Hoehenleitwerk 0,32 L bei 0,83 L; Rumpf ~0,073 L, Flaechen ~0,05 L dick. Bezugspunkt = Mitte der Flaechen.
-- ff = ND (alle Modi, ein Symbol wie im A350): Form, Groesse und Lage wie das gelbe Flugzeugsymbol des FF-ND, aus
-- Bildschirmfotos vermessen (ND-Einheiten): Nase 9,3 vor, Rumpfende 20,1 hinter den Flaechen, Spannweite 30,
-- Hoehenleitwerk 12 breit bei 16,8; Linien 1,6 dick; Flaechenmitte 0,66 rechts und 0,66 unter dem Bezugspunkt
local function draw_plane(px, py, th, col, k, ff)
    local L = ff and 1 or 41 * (k or 1)
    local fx, fy = math.sin(th), -math.cos(th)
    local rx, ry = math.cos(th), math.sin(th)
    if ff then px, py = px + 0.66, py + 0.66 end
    local function seg(a1, b1, a2, b2, w) -- a = vorwaerts, b = rechts (Vielfache von L)
        DL.line(px + (fx * a1 + rx * b1) * L, py + (fy * a1 + ry * b1) * L,
                px + (fx * a2 + rx * b2) * L, py + (fy * a2 + ry * b2) * L, col, w * L)
    end
    if ff then
        seg(9.3, 0, -20.1, 0, 1.6)         -- Rumpf
        seg(0, -15, 0, 15, 1.6)            -- Flaechen
        seg(-16.8, -6, -16.8, 6, 1.6)      -- Hoehenleitwerk
        return
    end
    seg(0.42, 0, -0.58, 0, 0.073)          -- Rumpf
    seg(0, -0.45, 0, 0.45, 0.05)           -- Flaechen
    seg(-0.41, -0.16, -0.41, 0.16, 0.07)   -- Hoehenleitwerk
end

-- Punkt auf der Exit-Mittellinie bei Bogenlaenge t [m] (ueber das Ende hinaus verlaengert)
local function path_point(x, t)
    local p, cum = x.path, x.cum
    local n = #p
    for i = 1, n - 1 do
        if t <= cum[i + 1] or i == n - 1 then
            local L = cum[i + 1] - cum[i]
            local f = L > 0 and (t - cum[i]) / L or 0
            local a, b = p[i], p[i + 1]
            local us, ud = 0, 0
            if L > 0 then us, ud = (b[1] - a[1]) / L, (b[2] - a[2]) / L end
            return a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f, us, ud
        end
    end
    return p[1][1], p[1][2], 0, 0
end

-- Mittellinie zwischen ta und tb [m] als Bildschirm-Polylinie
local function path_screen(x, ta, tb, XY)
    local pts = {}
    local s, d = path_point(x, ta)
    pts[1] = { XY(s, d) }
    for i = 2, #x.path - 1 do
        if x.cum[i] > ta and x.cum[i] < tb then pts[#pts + 1] = { XY(x.path[i][1], x.path[i][2]) } end
    end
    s, d = path_point(x, tb)
    pts[#pts + 1] = { XY(s, d) }
    return pts
end

local function draw_poly(pts, col, th)
    for i = 1, #pts - 1 do
        DL.line(pts[i][1], pts[i][2], pts[i + 1][1], pts[i + 1][2], col, th)
    end
end

local function dist_to_poly(pts, mx, my)
    local best = math.huge
    for i = 1, #pts - 1 do
        local ax, ay, bx, by = pts[i][1], pts[i][2], pts[i + 1][1], pts[i + 1][2]
        local vx, vy = bx - ax, by - ay
        local L2 = vx * vx + vy * vy
        local f = L2 > 0 and clamp(((mx - ax) * vx + (my - ay) * vy) / L2, 0, 1) or 0
        local dx, dy = ax + vx * f - mx, ay + vy * f - my
        best = math.min(best, dx * dx + dy * dy)
    end
    return math.sqrt(best)
end

-------------------------------------------------------------------------------
-- Ansichten
-- Fenster (Makro): "auto" | "btv" (Landebahn oben) | "taxi" (Kurs oben, folgt dem Flugzeug) | "north" (Norden oben).
-- ND im Cockpit nach dem EFIS-Modus des FCU:
--   ARC  = Taxi-Ansicht: Kurs oben, Bogen wie im FF-ND, folgt immer dem Flugzeug (nicht verschiebbar)
--   ROSE = Kurs oben, Rose, folgt immer dem Flugzeug; PLAN = Norden oben, verschiebbar (Doppelklick -> CENTER ON ACFT)
--   im Flug nur PLAN: BTV-Ansicht (Landebahn gerade nach oben), ohne Landebahn Norden oben
-- Reichweite 0,1 .. 5 NM mit dem Mausrad.
-------------------------------------------------------------------------------
local function new_view()
    -- Reichweite: ein gemeinsamer Wert fuer ARC / ROSE / PLAN / NAV - wie im A350 stellt der Range-Knopf auf ZOOM die
    -- Reichweite fuer alle Modi (rng.arc, rng.plan ... lesen und schreiben alle denselben Wert)
    local rng = setmetatable({}, { __index = function(t) return rawget(t, "_v") end,
                                   __newindex = function(t, k, v) rawset(t, "_v", v) end })
    return { mode = "auto", z = { btv = 1, taxi = 1, north = 1 },
             pan = { btv = { 0, 0 }, north = { 0, 0 }, arc = { 0, 0 }, rose = { 0, 0 } },
             taxi_free = nil, rng = rng, free = {}, plan_acft = false,
             ui = { tab = "ARPT", amode = "ICAO", kind = "RWY", dd = nil } }
end

local function acft_world(G)
    return G.W(DR.lat[0], DR.lon[0])
end

-- In der Luft gibt es im Fenster nur BTV (Landebahn oben) oder NORTH; V.amode = Wahl im Flug (nil = BTV, sobald eine Bahn gewaehlt ist).
-- Am Boden: AUTO / TAXI / NORTH - die BTV-Ansicht nur waehrend der BTV-Bremsung nach der Landung.
local function in_air() return S.air_time > 2 end
-- BTV bedienen (Bahn, Exit, ARM) nur in der Luft - oder solange BTV aktiv/scharf ist (Ausrollen)
local function btv_ui() return in_air() or S.mode ~= "OFF" or S.arm_request end
-- waehrend BTV bremst: Bahn, Exit, Stoppstrecke und Flughafen bleiben unveraendert (scharf: Aendern schaltet
-- BTV ab, wie im A350)
function S.locked() return S.mode == "ENGAGED" or S.mode == "RELEASE" end
local LOCK_MSG = "NOT AVAIL DURING BTV BRAKING"
local function effective_mode(V, G)
    if in_air() then
        if S.rwy and V.amode ~= "north" then return "btv" end
        return "north"
    end
    if S.mode == "ENGAGED" or S.mode == "RELEASE" then return "btv" end
    if V.mode == "taxi" or V.mode == "north" then return V.mode end
    local ax, ay = acft_world(G)
    local near = ((ax - G.cx) ^ 2 + (ay - G.cy) ^ 2) < (G.extent * 0.8 + 1500) ^ 2
    if near then return "taxi" end
    return "north"
end

local MODE_NAME = { btv = "BTV", taxi = "TAXI", north = "NORTH" }

-- ND-Zeichenflaeche: lokale Koordinaten 300 x 380 (1 Einheit = 1,68 Panel-Pixel, 16,8 x 21,2 cm)
local ND_VW, ND_VH = 300, 380
-- EFIS-Modus je Seite (pnl_frame liest ihn aus dem FCU): "arc", "rose" (LS/VOR/NAV) oder "plan"
-- force[s] = Taxi-Ansicht nach einer BTV-Landung erzwungen (wie ARC, auch ohne Range 10), bis Range- oder
-- EFIS-Knopf bewegt werden
local EFIS = { mode = { [0] = "arc", [1] = "arc" }, raw = {}, force = {} }
-- Kompass je Modus: Mitte x, y und Radius - ARC-Radius wie der Bogen des FF-ND (aus Bildschirmfotos vermessen).
-- ARC auf ZOOM: ohne Vertikalprofil steht das Flugzeug wie im A350 unten im ND (Bild des echten OANS: Flaechen
-- bei 310 von 380 Einheiten; im FF-ND mit Vertikalprofil bei 239), gleicher Radius, waagrecht mittig.
-- Jetzt fest NDUI.NOTCH.acft_gap (21,6) ueber der oberen Meldungszeile: zu 303,55, Bedienfeld offen 237,4;
-- ROSE/PLAN offen 34 darueber (Mitte 127)
-- ROSE/PLAN: Kompassrose so gross wie im FF-ND ab 10 NM (Radius 98 Einheiten, aus einem FF-Bild NAV 10 NM
-- vermessen), mittig im ND
EFIS.G = { arc = { 150, 303.55, 193 }, rose = { 150, 190, 98 }, plan = { 150, 190, 98 } }
-- BTV-Bedienfeld offen (Doppelpfeil unten rechts): wie im A350 nach oben geschoben (Abstaende zu TA ONLY wie oben)
EFIS.GP = { arc = { 150, 237.4, 193 }, rose = { 150, 127, 98 }, plan = { 150, 127, 98 } }
-- Kompass des FF-ND selbst (fuer die Pruefung, ob das FF-ND gezeichnet ist)
EFIS.GFF = { arc = { 150, 239, 193 }, rose = { 150, 190, 126 }, plan = { 150, 190, 126 } }
-- Modus der Karte einer Seite: ARC = Taxi-Ansicht (Kurs oben, folgt immer dem Flugzeug), ROSE, PLAN
function EFIS.vmode(s)
    if EFIS.force[s] then return "arc" end
    return EFIS.mode[s] or "arc"
end
-- Reichweiten wie im A350 (Airport Navigation): NM vom Flugzeugsymbol bis zum Kompassbogen (ARC) bzw. Radius der
-- Kompassrose (ROSE/PLAN); der gestrichelte Innenring liegt bei der halben Reichweite
local RANGES = { 0.2, 0.5, 1, 2, 5 }

-- Text mit Grundlinie: x links (oder rechts mit right = true), base = Grundlinie; Rueckgabe = rechtes Ende
local function txt_b(x, base, col, str, cap, right)
    cap = cap or TXT.cap
    local w = text_w(str, cap)
    if right then x = x - w end
    DL.text(x, base - cap - (FONT_H - cap) / 2, col, str, cap)
    return x + w
end

-- Text mittig in einer Taste
local function nd_key_text(r, col, str, cap)
    cap = cap or TXT.cap
    DL.text((r[1] + r[3]) / 2 - text_w(str, cap) / 2, (r[2] + r[4]) / 2 - FONT_H / 2, col, str, cap)
end


-- Zusatzdaten eines Flughafens aus dem Kopf seines apt.dat-Blocks (einmal gelesen): Name, Stadt, IATA
local META = {}
function META.get(e)
    if e.meta then return e.meta end
    local m = { name = "" }
    e.meta = m
    local f = io.open(e.file, "rb")
    if f then
        f:seek("set", e.off)
        for k = 1, 60 do
            local line = f:read("*l")
            if not line then break end
            line = line:gsub("\r$", "")
            if k == 1 then
                m.name = (line:match("^1%s+%S+%s+%S+%s+%S+%s+%S+%s*(.-)%s*$") or ""):upper()
            else
                local key, val = line:match("^1302%s+(%S+)%s*(.-)%s*$")
                if key then
                    if key == "city" and val ~= "" then m.city = val:upper()
                    elseif key == "iata_code" and val ~= "" then m.iata = val:upper() end
                elseif line:match("^%d") then
                    break -- die 1302-Zeilen stehen direkt nach dem Kopf
                end
            end
        end
        f:close()
    end
    if e.name == nil then e.name = m.name end
    return m
end

-- Breite/Laenge wie im Airbus: 48°24.0N/011°48.0E
function META.latlon(lat, lon)
    local function f(v, w, p, n)
        local a = math.abs(v)
        local d = math.floor(a)
        local mnt = (a - d) * 60
        if mnt >= 59.95 then d, mnt = d + 1, 0 end
        return string.format("%0" .. w .. "d\194\176%04.1f%s", d, mnt, v >= 0 and p or n)
    end
    return f(lat, 2, "N", "S") .. "/" .. f(lon, 3, "E", "W")
end

-------------------------------------------------------------------------------
-- Auswahllisten (Dropdown / Kontextmenue) auf der Karte: Flughafen, Landebahn, "SEL RWY" beim Antippen
-- einer Piste. Eintrag: { t = Text } oder { cols = { { Text, x, Farbe }, ... } }, fn = Aktion.
-------------------------------------------------------------------------------
local MENU = { lh = FONT_H + 6 }
-- (MENU.lh wird mit FONT_H zusammen gesetzt: Fenster bzw. Cockpit-Display)

function MENU.open(MV, items, ax, ay, B, o)
    o = o or {}
    if #items == 0 then return end
    local w = o.title and text_w(o.title) or 0
    for _, it in ipairs(items) do
        local iw = 0
        if it.cols then
            local c = it.cols[#it.cols]
            iw = c[2] + text_w(c[1])
        else
            iw = text_w(it.t)
        end
        w = math.max(w, iw)
    end
    w = w + 16
    local h = (#items + (o.title and 1 or 0)) * MENU.lh + 6
    local x = o.right and (ax - w) or ax
    local y = o.up and (ay - h) or ay
    x = clamp(x, B[1] + 2, B[1] + B[3] - w - 2)
    y = clamp(y, B[2] + 2, B[2] + B[4] - h - 2)
    MV.menu = { items = items, x = x, y = y, w = w, h = h, title = o.title }
end

function MENU.at(m, px, py)
    if not px or px < m.x or px > m.x + m.w or py < m.y or py > m.y + m.h then return nil end
    local k = math.floor((py - m.y - 3) / MENU.lh) + 1 - (m.title and 1 or 0)
    if k >= 1 and k <= #m.items then return k end
    return 0 -- im Menue, aber kein Eintrag (Titel/Rand)
end

function MENU.draw(m, hx, hy)
    DL.rectf(m.x, m.y, m.x + m.w, m.y + m.h, MC.bg)
    DL.rect(m.x, m.y, m.x + m.w, m.y + m.h, MC.white, 0, 0, 1.2)
    local y = m.y + 3
    if m.title then
        DL.text(m.x + 8, y + 3, MC.white, m.title)
        y = y + MENU.lh
        DL.line(m.x + 4, y - 1, m.x + m.w - 4, y - 1, MC.key_frame, 1)
    end
    local hk = MENU.at(m, hx, hy)
    for k, it in ipairs(m.items) do
        if k == hk and it.fn then DL.rect(m.x + 3, y, m.x + m.w - 3, y + MENU.lh, MC.sel, 0, 0, 1.2) end
        if it.cols then
            for _, c in ipairs(it.cols) do DL.text(m.x + 8 + c[2], y + 3, c[3] or MC.sel, c[1]) end
        else
            DL.text(m.x + 8, y + 3, it.c or MC.sel, it.t)
        end
        y = y + MENU.lh
    end
end

-------------------------------------------------------------------------------
-- Karte: Ansicht, Flaechen, Pisten, Beschriftungen (Fenster und ND gemeinsam).
-- C = Zeichen-Zusammenhang: Bereich (x0, y0, cw, ch), Ansicht (mode, hv, wcx, wcy, sc, ax, ay), Abbildungen
-- XY (Welt -> Bildschirm), INV, XYr (Landebahn-Koordinaten), nd (ND-Seite oder nil = Fenster).
-------------------------------------------------------------------------------
local MAPX = {}

-- verteilter Aufbau der ND-Karte: Zeitbudget dieses Bildes aufgebraucht? (MAPX.bud; ohne Budget nie)
-- (keine Coroutinen: das LuaJIT von FlyWithLua stuerzt mit FFI in Coroutinen und FFI-Callbacks ab)
function MAPX.late()
    local b = MAPX.bud
    return b ~= nil and b.clk() >= b.deadline
end

-- kleinste Reichweite (Index), bei der ein Radius r [m] in den Kompassring passt
function MAPX.fit_rng(r_m)
    for i, v in ipairs(RANGES) do if v * 1852 >= r_m then return i end end
    return #RANGES
end

-- Ansicht im ND nach EFIS-Modus
function MAPX.view_nd(C, apt, V, MV, inp, side)
    local G = apt.geo
    local mode = EFIS.vmode(side)
    local GT = (V.ui and V.ui.panel) and EFIS.GP or EFIS.G
    local eg = GT[mode]
    local ax, ay, Rr = eg[1], eg[2], eg[3]
    local acx, acy = acft_world(G)
    local air = in_air()
    local E = S.rwy
    if E and not E.ox then E = nil end
    -- im Flug: PLAN = BTV-Ansicht mit gewaehlter Landebahn - die Bahn steht gerade nach oben (Anflugrichtung);
    -- NAV (ROSE) bleibt am Boden und im Flug wie ARC: Kurs oben, Flugzeug in der Mitte der Rose
    local vk = mode
    local btv = air and mode == "plan" and E ~= nil
    -- Kartenmitte:
    --   ARC  = immer das Flugzeug (Taxi-Ansicht, nicht verschiebbar)
    --   ROSE = immer das Flugzeug (wie ARC, nicht verschiebbar)
    --   PLAN = im Flug die Landebahn bzw. der Flughafen, am Boden das Flugzeug (nah) oder der Flughafen;
    --          nach Verschieben usw. fest (V.free.plan), CENTER ON ACFT -> Flugzeug
    V.free = V.free or {}
    local key = apt.icao .. "/" .. (E and E.id or "-") .. (air and "/A" or "/G")
    if V.plan_key ~= key then
        -- PLAN: Ausgangslage bei neuem Flughafen, neuer Landebahn, nach Start bzw. Landung
        V.plan_key = key
        V.free.plan, V.plan_acft = nil, false
        -- (ROSE/PLAN: Reichweite = Durchmesser, ein Radius r braucht die Reichweite 2 r)
        if E and air then
            V.rng.plan = MAPX.fit_rng(2 * (E.len / 2 + 200) / 1.15) -- ganze Bahn im Bild
        elseif air then
            V.rng.plan = MAPX.fit_rng(G.extent / 1.15)
        else
            V.rng.plan = V.rng.plan or MAPX.fit_rng(2 * 1852)
        end
    end
    if V.show_apt == apt.icao then
        -- DISPLAY AIRPORT: wie im A350 nur in PLAN (und der Flughafenkarte im Flug); ROSE/ARC zeigen das Flugzeug
        V.show_apt = nil
        V.free.plan = { G.cx, G.cy }
        V.rng.plan = MAPX.fit_rng(G.extent / 1.15)
    end
    if V.td_rng then V.rng[vk], V.td_rng = V.td_rng, nil end -- Taxi-Ansicht nach dem Aufsetzen
    V.rng[vk] = V.rng[vk] or MAPX.fit_rng((mode == "plan") and 2 * 1852 or ((mode == "arc") and 926 or 1852))
    -- Mausrad = Reichweite (nicht ueber Listen und Tasten)
    local wh = inp.wheel or 0
    if wh ~= 0 and not MV.wheel_ui then
        if vk == "arc" and V.rng_cont then V.rng[vk] = MAPX.fit_rng(V.rng_cont * 1852); V.rng_cont = nil end
        local n = math.max(1, math.floor(math.abs(wh) + 0.5))
        V.rng[vk] = clamp(V.rng[vk] + ((wh > 0) and -n or n), 1, #RANGES)
        V.rng_user = true
    end
    local rng = (vk == "arc" and V.rng_cont) or RANGES[V.rng[vk]]
    -- Reichweitenwechsel als echter Zoom: die angezeigte Reichweite folgt der gewaehlten in ~0,3 s (gleichmaessig im
    -- Logarithmus, jede Stufe gleich schnell); Ringbeschriftung (C.rng) sofort die neue. Die Annaeherungs-
    -- Reichweite (rng_cont) ist schon stetig und wird nicht zusaetzlich verzoegert.
    local now = DR.time[0]
    local cur = V.rdisp
    if not cur or (vk == "arc" and V.rng_cont) then
        cur = rng
    else
        local dt = clamp(now - (V.rd_t or now), 0, 0.1)
        cur = math.exp(math.log(cur) + (math.log(rng) - math.log(cur)) * (1 - math.exp(-dt / 0.08)))
        if math.abs(math.log(cur / rng)) < 0.003 then cur = rng end
    end
    V.rdisp, V.rd_t, V.zanim = cur, now, cur ~= rng
    -- wie im A350/FF-ND: in ARC ist die Reichweite der Radius des Bogens, in ROSE/PLAN der Durchmesser der Rose
    -- (10 NM -> Kreis "5"; ZOOM 0,2 NM -> Kreis "0.1")
    local sc = Rr / (((mode == "arc") and cur or cur / 2) * 1852)
    local hv = DR.psi[0]
    if mode == "plan" then hv = btv and E.hdg or 0 end
    local hc, hs = math.cos(hv * D2R), math.sin(hv * D2R)
    local follow = (mode ~= "plan")
    local function center()
        if follow then return acx, acy end
        local f = V.free[vk]
        if f then return f[1], f[2] end
        if mode == "rose" or V.plan_acft then return acx, acy end
        if air then
            if E then return E.ox + E.ux * E.len / 2, E.oy + E.uy * E.len / 2 end
            return G.cx, G.cy
        end
        local near = ((acx - G.cx) ^ 2 + (acy - G.cy) ^ 2) < (G.extent * 0.6 + 1000) ^ 2
        if near then return acx, acy end
        return G.cx, G.cy
    end
    -- Ziehen = verschieben (nur PLAN; nicht, wenn auf einer Taste oder Liste angefangen)
    if inp.active and not MV.press_ui and ((inp.dx or 0) ~= 0 or (inp.dy or 0) ~= 0) then
        if not follow then
            local X, Y = inp.dx / sc, -inp.dy / sc
            local ox, oy = X * hc + Y * hs, -X * hs + Y * hc
            local cx, cy = center()
            V.free[vk] = { cx - ox, cy - oy } -- ab jetzt fest (folgt nicht mehr dem Flugzeug)
        end
        MV.drag = MV.drag + math.abs(inp.dx) + math.abs(inp.dy)
    end
    local wcx, wcy = center()
    C.mode, C.hv, C.wcx, C.wcy, C.sc, C.ax, C.ay, C.Rr, C.rng, C.hc, C.hs = mode, hv, wcx, wcy, sc, ax, ay, Rr, rng, hc, hs
    C.acx, C.acy, C.btv, C.taxi, C.vk = acx, acy, btv, EFIS.force[side] and true or false, vk
    C.appr = (vk == "arc") and V.appr or nil
end

-- Ansicht im Fenster (AUTO / BTV / TAXI / NORTH)
function MAPX.view_win(C, apt, V, MV, inp)
    local G = apt.geo
    local E = S.rwy
    local x0, y0, cw, ch = C.x0, C.y0, C.cw, C.ch
    local mx, my, wheel = inp.mx, inp.my, inp.wheel or 0
    local mode = effective_mode(V, G)
    if mode == "btv" and not E then mode = "north" end
    V.eff = mode
    local hv, wcx, wcy, sc0
    local acx, acy = acft_world(G)
    local ax, ay = x0 + cw / 2, y0 + ch / 2
    local function base_center()
        if mode == "btv" then
            return E.ox + E.ux * E.len / 2 + V.pan.btv[1], E.oy + E.uy * E.len / 2 + V.pan.btv[2]
        elseif mode == "taxi" then
            if V.taxi_free then return V.taxi_free[1], V.taxi_free[2] end
            return acx, acy
        end
        return G.cx + V.pan.north[1], G.cy + V.pan.north[2]
    end
    if mode == "btv" then hv, sc0 = E.hdg, ch / (E.len * 1.25)
    elseif mode == "taxi" then hv, sc0 = DR.psi[0], ch / 800
    else hv, sc0 = 0, ch / (G.extent * 1.1) end
    wcx, wcy = base_center()
    local sc = sc0 * V.z[mode]
    local hc, hs = math.cos(hv * D2R), math.sin(hv * D2R)
    local function rot_inv(X, Y) return X * hc + Y * hs, -X * hs + Y * hc end
    local function set_center(nx, ny)
        if mode == "btv" then
            V.pan.btv[1] = nx - (E.ox + E.ux * E.len / 2)
            V.pan.btv[2] = ny - (E.oy + E.uy * E.len / 2)
        elseif mode == "taxi" then
            V.taxi_free = { nx, ny }
        else
            V.pan.north[1], V.pan.north[2] = nx - G.cx, ny - G.cy
        end
    end
    -- Zoom mit dem Mausrad (um den Mauszeiger; beim Folgen um das Flugzeug)
    if wheel ~= 0 then
        local nz = clamp(V.z[mode] * 1.25 ^ wheel, 0.3, 60)
        if mode == "taxi" and not V.taxi_free then
            V.z[mode] = nz
        else
            local wx, wy = rot_inv((mx - ax) / sc, (ay - my) / sc)
            wx, wy = wcx + wx, wcy + wy
            V.z[mode] = nz
            local sc2 = sc0 * nz
            local ox, oy = rot_inv((mx - ax) / sc2, (ay - my) / sc2)
            set_center(wx - ox, wy - oy)
        end
    end
    -- Verschieben (Ziehen)
    if inp.active and ((inp.dx or 0) ~= 0 or (inp.dy or 0) ~= 0) then
        local ox, oy = rot_inv(inp.dx / sc, -inp.dy / sc)
        set_center(wcx - ox, wcy - oy)
        MV.drag = MV.drag + math.abs(inp.dx) + math.abs(inp.dy)
    end
    wcx, wcy = base_center()
    sc = sc0 * V.z[mode]
    C.mode, C.hv, C.wcx, C.wcy, C.sc, C.ax, C.ay, C.hc, C.hs = mode, hv, wcx, wcy, sc, ax, ay, hc, hs
    C.acx, C.acy = acx, acy
end

-- Abbildungen und sichtbarer Bereich
function MAPX.setup(C)
    local ax, ay, sc, hc, hs, wcx, wcy = C.ax, C.ay, C.sc, C.hc, C.hs, C.wcx, C.wcy
    local function XY(x, y)
        local dx, dy = x - wcx, y - wcy
        return ax + (dx * hc - dy * hs) * sc, ay - (dx * hs + dy * hc) * sc
    end
    C.XY = XY
    C.INV = function(px, py)
        local X, Y = (px - ax) / sc, (ay - py) / sc
        return wcx + X * hc + Y * hs, wcy - X * hs + Y * hc
    end
    local E = S.rwy
    C.E = E
    if E then
        C.XYr = function(s, d) return XY(E.ox + s * E.ux - d * E.uy, E.oy + s * E.uy + d * E.ux) end
    end
    local vcx, vcy = C.INV(C.x0 + C.cw / 2, C.y0 + C.ch / 2)
    local vr = math.sqrt(C.cw * C.cw + C.ch * C.ch) / 2 / sc + 30
    C.vminx, C.vmaxx, C.vminy, C.vmaxy = vcx - vr, vcx + vr, vcy - vr, vcy + vr
end

-------------------------------------------------------------------------------
-- Flaechen, Gebaeude, Pisten, Markierungen
-------------------------------------------------------------------------------
-- Schritte des Kartenaufbaus: jeder gibt true zurueck, wenn er fertig ist; false = Zeitbudget dieses Bildes
-- aufgebraucht, beim naechsten Aufruf macht er dort weiter. (Fenster: MAPX.surfaces, alles auf einmal.)
function MAPX.surface_steps(C)
    local G, XY, sc = C.G, C.XY, C.sc
    local x0, y0, cw, ch = C.x0, C.y0, C.cw, C.ch
    local vminx, vmaxx, vminy, vmaxy = C.vminx, C.vmaxx, C.vminy, C.vmaxy
    local E = C.E
    local function in_view(minx, miny, maxx, maxy)
        return maxx >= vminx and minx <= vmaxx and maxy >= vminy and miny <= vmaxy
    end
    C.in_view = in_view
    local late = MAPX.late

    -- Detailstufe: 1 = grob (25 m), 2 = mittel (8 m), 3 = fein (3 m)
    local lod = (sc < 0.35) and 1 or ((sc < 0.9) and 2 or 3)
    -- Mindestabstand der Punkte in Pixel^2 (klein: die Stufen sind schon formtreu vereinfacht - groessere Werte
    -- schneiden Kurven wieder eckig ab; reicht das Budget nicht, vergroessert stroke ihn selbst)
    local md2 = (lod == 1) and 4 or ((lod == 2) and 2.25 or 1)
    C.lod, C.md2 = lod, md2
    -- Eckpunkt-Budget fuer Flaechen und Linien; der Rest bleibt fuer Beschriftungen, Kompass, Kopfzeilen
    -- (Cockpit-ND: Puffer fassen 140000 - mehr Budget, damit auch bei grosser Reichweite alles gezeichnet wird)
    local BULK = C.nd and 90000 or 54000
    C.BULK = BULK
    -- jedes Dreieck um 0,6 Pixel vergroessern: keine Haarlinien zwischen Dreiecken (nicht beim Ein-/Ausblenden:
    -- durchscheinend wuerden die Ueberlappungen als Naehte sichtbar)
    local ex = (C.fading and 0 or 0.6) / sc
    local function draw_tris(list, col, lim)
        for _, c in ipairs(list) do
            if in_view(c.minx, c.miny, c.maxx, c.maxy) then
                local t, o = c.t, c.o
                if DL.n + #t > lim then return false end -- 6 Werte = 6 Eckpunkte je Dreieck
                for k = 1, #t, 6 do
                    local ax1, ay1 = XY(t[k] + o[k] * ex, t[k + 1] + o[k + 1] * ex)
                    local bx1, by1 = XY(t[k + 2] + o[k + 2] * ex, t[k + 3] + o[k + 3] * ex)
                    local cx1, cy1 = XY(t[k + 4] + o[k + 4] * ex, t[k + 5] + o[k + 5] * ex)
                    DL.tri(ax1, ay1, bx1, by1, cx1, cy1, col)
                end
            end
        end
        return true
    end
    local function count_tris(list)
        local nv = 0
        for _, c in ipairs(list) do
            if in_view(c.minx, c.miny, c.maxx, c.maxy) then nv = nv + #c.t end
        end
        return nv
    end
    -- Linienzuege: sichtbare Stuecke in Bildschirmpunkte (Mindestabstand), bei Platzmangel staerker ausduennen
    local function stroke(lists, col, th, mind2, room)
        local pl, nseg = {}, 0
        for _, list in ipairs(lists) do
            for _, c in ipairs(list) do
                if in_view(c.minx, c.miny, c.maxx, c.maxy) then
                    local xs, ys = c.xs, c.ys
                    local n = #xs
                    local px, py = XY(xs[1], ys[1])
                    local pts = { px, py }
                    for k = 2, n do
                        local qx, qy = XY(xs[k], ys[k])
                        local dx, dy = qx - px, qy - py
                        if dx * dx + dy * dy >= mind2 or k == n then
                            pts[#pts + 1], pts[#pts + 2] = qx, qy
                            px, py = qx, qy
                        end
                    end
                    pl[#pl + 1] = pts
                    nseg = nseg + #pts / 2 - 1
                end
            end
        end
        local m2, it = math.max(mind2, 1), 0
        while nseg * 8 > room and it < 5 do
            m2, it, nseg = m2 * 4, it + 1, 0
            for i, pts in ipairs(pl) do
                local np = #pts
                if np > 4 then
                    local out = { pts[1], pts[2] }
                    local px, py = pts[1], pts[2]
                    for k = 3, np - 1, 2 do
                        local qx, qy = pts[k], pts[k + 1]
                        local dx, dy = qx - px, qy - py
                        if dx * dx + dy * dy >= m2 or k == np - 1 then
                            out[#out + 1], out[#out + 2] = qx, qy
                            px, py = qx, qy
                        end
                    end
                    pl[i] = out
                end
                nseg = nseg + #pl[i] / 2 - 1
            end
        end
        local lim = DL.n + room
        for _, pts in ipairs(pl) do
            for k = 1, #pts - 3, 2 do
                if DL.n + 8 > lim then return end
                DL.line(pts[k], pts[k + 1], pts[k + 2], pts[k + 3], col, th)
            end
        end
    end
    -- gestrichelte Linienzuege: Strich "on", Luecke "off" (ND-Einheiten); das Muster laeuft ueber Stueckgrenzen weiter
    local function dashed(list, col, th, on, off, room)
        local lim, per = DL.n + room, on + off
        local ph, lx, ly = 0, nil, nil
        for _, c in ipairs(list or {}) do
            if in_view(c.minx, c.miny, c.maxx, c.maxy) then
                local xs, ys = c.xs, c.ys
                local px, py = XY(xs[1], ys[1])
                if not (lx and math.abs(lx - px) < 0.01 and math.abs(ly - py) < 0.01) then ph = 0 end
                for k = 2, #xs do
                    local qx, qy = XY(xs[k], ys[k])
                    local dx, dy = qx - px, qy - py
                    local L = math.sqrt(dx * dx + dy * dy)
                    local t = 0
                    while t < L do
                        local inside = ph < on
                        local t2 = math.min(L, t + (inside and (on - ph) or (per - ph)))
                        if t2 <= t then t2 = math.min(L, t + 1e-3) end
                        if inside then
                            if DL.n + 8 > lim then return end
                            DL.line(px + dx * t / L, py + dy * t / L, px + dx * t2 / L, py + dy * t2 / L, col, th)
                        end
                        ph = (ph + t2 - t) % per
                        t = t2
                    end
                    px, py = qx, qy
                end
                lx, ly = px, py
            else
                lx = nil
            end
        end
    end
    local function quad(q, col)
        local p1x, p1y = XY(q[1], q[2])
        local p2x, p2y = XY(q[3], q[4])
        local p3x, p3y = XY(q[5], q[6])
        local p4x, p4y = XY(q[7], q[8])
        DL.quadf(p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, col)
    end

    local steps = {}
    local pave_L, pave_lim, pave_i = lod, 0, 1
    local FILL = { "fill_c", "fill_m", "fill" }
    local HOLE = { "holes_c", "holes_m", "holes" }
    -- 1) Hintergrund; feinste Detailstufe der Rollflaechen, die ins Budget passt
    steps[#steps + 1] = function()
        DL.n = 0
        DL.rectf(x0, y0, x0 + cw, y0 + ch, MC.bg)
        if G.has_pave then
            local room = (BULK - DL.n) * 0.45
            local L = lod
            while L > 1 do
                local nv = 0
                for _, gp in ipairs(G.polys) do
                    if in_view(gp.minx, gp.miny, gp.maxx, gp.maxy) then
                        nv = nv + count_tris(gp[FILL[L]]) + count_tris(gp[HOLE[L]])
                    end
                end
                if nv <= room then break end
                L = L - 1
            end
            pave_L = L
            pave_lim = math.max(DL.n + room, BULK * 0.5)
        end
        return true
    end
    -- 2) Rollflaechen (lange Liste: unterbrechbar)
    steps[#steps + 1] = function()
        if G.has_pave then
            local polys, L = G.polys, pave_L
            while pave_i <= #polys do
                local gp = polys[pave_i]
                pave_i = pave_i + 1
                if in_view(gp.minx, gp.miny, gp.maxx, gp.maxy) then
                    if draw_tris(gp[FILL[L]], gp.apron and MC.apron or MC.pave, pave_lim) then
                        draw_tris(gp[HOLE[L]], MC.bg, pave_lim + 4000)
                    end
                end
                if pave_i % 8 == 0 and late() then return false end
            end
            -- Rollgassen auf dem Vorfeld im Rollweg-Grau (Breite nach ICAO-Klasse). Jedes Stueck um die halbe Breite
            -- verlaengert (schliesst die Fugen an Knicken und Kreuzungen) - runde Enden je Kante waren bei grossen
            -- Flughaefen zu teuer: das Budget reichte nicht, Rollwege fehlten (KATL bei 0,5 NM)
            local lim = DL.n + BULK * 0.35
            for _, e in ipairs(G.apron_tw or {}) do
                if in_view(math.min(e.x1, e.x2) - 30, math.min(e.y1, e.y2) - 30, math.max(e.x1, e.x2) + 30, math.max(e.y1, e.y2) + 30) then
                    if DL.n + 8 > lim then break end
                    local L = e.len or 0
                    local ext = (L > 0.1) and math.min(e.w / 2, L) / L or 0
                    local dx, dy = (e.x2 - e.x1) * ext, (e.y2 - e.y1) * ext
                    local px, py = XY(e.x1 - dx, e.y1 - dy)
                    local qx, qy = XY(e.x2 + dx, e.y2 + dy)
                    DL.line(px, py, qx, qy, MC.pave, math.max(e.w * sc, 1))
                end
            end
            return true
        end
        -- keine Flaechen in den Daten: Rollwege als breite Baender aus dem Rollwegnetz (brauner Rand nur bis 0,5 NM)
        local wpx = math.max(23 * sc, 2)
        local lim = BULK * 0.5
        for pass = (sc >= 0.12) and 1 or 2, 2 do
            local col, th = (pass == 1) and MC.shoulder or MC.pave, (pass == 1) and (wpx + math.max(4, 7 * sc)) or wpx
            for _, e in ipairs(G.edges) do
                if not e.rwy and in_view(math.min(e.x1, e.x2) - 30, math.min(e.y1, e.y2) - 30, math.max(e.x1, e.x2) + 30, math.max(e.y1, e.y2) + 30) then
                    if DL.n + 8 > lim * pass / 2 then break end
                    -- (um die halbe Breite verlaengert statt runder Enden, wie die Rollgassen oben)
                    local L = e.len or 0
                    local ext = (L > 0.1) and math.min(th / sc / 2, L) / L or 0
                    local dx, dy = (e.x2 - e.x1) * ext, (e.y2 - e.y1) * ext
                    local px, py = XY(e.x1 - dx, e.y1 - dy)
                    local qx, qy = XY(e.x2 + dx, e.y2 + dy)
                    DL.line(px, py, qx, qy, col, th)
                end
            end
        end
        return true
    end
    -- 3) Parkpositionen
    steps[#steps + 1] = function()
        -- Parkpositionen: hellere Flaechen mit dunklem Rand (ueberlappende sind zusammengefasst)
        if sc >= (C.nd and 0.09 or 0.6) and G.stand_areas then
            local lim = DL.n + (BULK - DL.n) * 0.2
            for _, a in ipairs(G.stand_areas) do
                if in_view(a.minx, a.miny, a.maxx, a.maxy) then
                    if not draw_tris(a.tris, MC.stand_area, lim) then break end
                end
            end
            local lim2 = lim + 3000
            for _, a in ipairs(G.stand_areas) do
                if in_view(a.minx, a.miny, a.maxx, a.maxy) then
                    local n = #a.x
                    if DL.n + 8 * n > lim2 then break end
                    local px, py = XY(a.x[n], a.y[n])
                    for k = 1, n do
                        local qx, qy = XY(a.x[k], a.y[k])
                        DL.line(px, py, qx, qy, MC.stand_line, 1)
                        px, py = qx, qy
                    end
                end
            end
        end
        return true
    end
    -- 4) Raender der Rollflaechen
    steps[#steps + 1] = function()
        -- braune Raender der Rollwege - wie im A350 nur bis 0,5 NM Reichweite (darueber nur die graue Karte);
        -- Breite wie im OANS-Bild: etwa 3 Einheiten bei 0,2 NM, 2 bei 0,5 NM
        if G.has_pave and sc >= 0.12 then
            stroke({ (pave_L == 1) and G.pave_c or ((pave_L == 2) and G.pave_m or G.pave) }, MC.shoulder, (sc >= 0.4) and 3 or 2,
                md2, (BULK - DL.n) * 0.2)
        end
        return true
    end
    -- 5) Gebaeude
    steps[#steps + 1] = function()
        -- Gebaeude: aus OpenStreetMap (Terminals cyan, andere blau) - sonst optional geschaetzte Terminals
        if G.osm then
            local lim = DL.n + (BULK - DL.n) * 0.35
            for _, b in ipairs(G.osm.terms) do
                if in_view(b.minx, b.miny, b.maxx, b.maxy) then draw_tris(b.tris, MC.terminal, lim) end
            end
            -- (andere Gebaeude erst ab etwa 2 NM Reichweite - weiter draussen nur noch Flecken)
            for _, b in ipairs((sc >= 0.05) and G.osm.bldg or {}) do
                if in_view(b.minx, b.miny, b.maxx, b.maxy) then
                    if not draw_tris(b.tris, MC.building, lim) then break end
                end
            end
        elseif CFG.guess_terms and sc >= 0.08 then
            for _, tq in ipairs(G.terms) do
                if tq.cx > vminx - 100 and tq.cx < vmaxx + 100 and tq.cy > vminy - 100 and tq.cy < vmaxy + 100 then
                    if DL.n + 8 > BULK then break end
                    quad(tq, MC.terminal)
                end
            end
        end
        return true
    end
    -- 6) Pisten
    steps[#steps + 1] = function() MAPX.runways(C); return true end
    -- 7) Markierungen
    steps[#steps + 1] = function()
        -- Markierungen wie im A350-OANS nur bis 0,5 NM Reichweite (ab 1 NM nur die graue Karte und Gebaeude):
        -- gelbe Rollweg-Mittellinien, rote Haltelinien, gestrichelte Linien (Strassen ab 0,5 NM)
        if sc >= 0.12 then
            local th = (sc >= 0.9) and 2 or ((sc >= 0.35) and 1.5 or 1)
            if sc >= 0.18 then
                -- Strassen fuer Bodenfahrzeuge weiss gestrichelt, gestrichelte gelbe Markierungen
                local r2 = math.max(0, BULK - DL.n) * 0.08
                dashed(G.road, MC.road, (sc >= 0.9) and 1.2 or 1, 4, 3.5, r2)
                stroke({ G.wline or {} }, MC.road, (sc >= 0.9) and 1.2 or 1, md2, r2)
                dashed(G.dash_y, MC.taxiline, (sc >= 0.9) and 1.5 or 1, 4.5, 3.5, r2)
            end
            -- feste Reserve fuer die gelben Mittellinien: frueher bekamen sie nur den Rest nach Flaechen und Gebaeuden
            -- und fielen bei grosser Reichweite (ganzer Flughafen im Bild) ganz oder teilweise weg
            local room = math.max(math.max(0, BULK - DL.n) * 0.85, BULK * 0.25)
            -- die echten (formtreu vereinfachten) Mittellinien in jeder Stufe - nicht mehr die geraden Kanten des
            -- Rollwegnetzes in der Uebersicht (eckig, ohne Kurven); das Netz nur, wenn es keine Linien gibt
            local tl = (lod == 1) and G.tl_c or ((lod == 2) and G.tl_m or G.tl)
            if #tl == 0 and #G.tw_net > 0 then tl = G.tw_net end
            stroke({ tl }, MC.taxiline, th, md2, room)
            stroke({ G.hold }, MC.hold, (sc >= 0.9) and 3 or ((sc >= 0.25) and 2 or 1.5), 1, math.max(0, BULK - DL.n))
        end
        return true
    end
    return steps
end

-- alle Schritte auf einmal (Fenster; ND sofort)
function MAPX.surfaces(C)
    local bud = MAPX.bud
    MAPX.bud = nil
    for _, f in ipairs(MAPX.surface_steps(C)) do f() end
    MAPX.bud = bud
end

-------------------------------------------------------------------------------
-- Pisten: braune Schulter, graue Flaeche, weisse Markierungen
-------------------------------------------------------------------------------
function MAPX.runways(C)
    local G, XY, sc, E = C.G, C.XY, C.sc, C.E
    local in_view = C.in_view
    local function rq(r, a0, a1, c0, c1, col)
        -- Rechteck entlang der Piste: a = Abstand ab Ende 1, c = seitlich (links +)
        local ux, uy, nx, ny = r.ux, r.uy, -r.uy, r.ux
        -- schmaler als ~1,5 Pixel: als geglaettete Linie (sonst fallen einzelne Pixel aus, die Linie wirkt gepunktet)
        local wa, wc = math.abs(a1 - a0) * sc, math.abs(c1 - c0) * sc
        if wc < 1.6 and wa >= wc then
            local cm = (c0 + c1) / 2
            local p1x, p1y = XY(r.x1 + ux * a0 + nx * cm, r.y1 + uy * a0 + ny * cm)
            local p2x, p2y = XY(r.x1 + ux * a1 + nx * cm, r.y1 + uy * a1 + ny * cm)
            DL.line(p1x, p1y, p2x, p2y, col, wc)
            return
        elseif wa < 1.6 and wc > wa then
            local am = (a0 + a1) / 2
            local p1x, p1y = XY(r.x1 + ux * am + nx * c0, r.y1 + uy * am + ny * c0)
            local p2x, p2y = XY(r.x1 + ux * am + nx * c1, r.y1 + uy * am + ny * c1)
            DL.line(p1x, p1y, p2x, p2y, col, wa)
            return
        end
        local p1x, p1y = XY(r.x1 + ux * a0 + nx * c0, r.y1 + uy * a0 + ny * c0)
        local p2x, p2y = XY(r.x1 + ux * a1 + nx * c0, r.y1 + uy * a1 + ny * c0)
        local p3x, p3y = XY(r.x1 + ux * a1 + nx * c1, r.y1 + uy * a1 + ny * c1)
        local p4x, p4y = XY(r.x1 + ux * a0 + nx * c1, r.y1 + uy * a0 + ny * c1)
        DL.quadf(p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, col)
    end
    -- Punkt in Pistenkoordinaten (a ab Ende 1, c seitlich, links +) -> Bildschirm
    local function rp(r, a, c)
        return XY(r.x1 + r.ux * a - r.uy * c, r.y1 + r.uy * a + r.ux * c)
    end
    -- gefuellter Winkel "Λ" wie auf der Piste: Spitze bei a (Richtung d = +1/-1 entlang der Piste), Laenge hl,
    -- halbe Breite hw2, Dicke t (entlang der Piste); die Kerbe innen ist offen
    local function chevron(r, a, d, c0, hl, hw2, t, col)
        local k = hw2 * (1 - t / hl)
        local Ax, Ay = rp(r, a, c0)
        local Nx, Ny = rp(r, a - d * t, c0)
        local Lx, Ly = rp(r, a - d * hl, c0 + hw2)
        local Lix, Liy = rp(r, a - d * hl, c0 + k)
        local Rx, Ry = rp(r, a - d * hl, c0 - hw2)
        local Rix, Riy = rp(r, a - d * hl, c0 - k)
        DL.quadf(Ax, Ay, Lx, Ly, Lix, Liy, Nx, Ny, col)
        DL.quadf(Ax, Ay, Nx, Ny, Rix, Riy, Rx, Ry, col)
    end
    -- sichtbare Pisten; Reihenfolge: kurze zuerst, die Landebahn zuletzt - an Kreuzungen laufen so die
    -- Markierungen der wichtigeren Bahn durch. Erst alle braunen Schultern, dann Flaechen und Markierungen
    -- (sonst schneidet die Schulter der einen Bahn die andere ab).
    local vis = {}
    C.mid = {}
    for _, r in ipairs(G.rwys) do
        local dx, dy = r.x2 - r.x1, r.y2 - r.y1
        local L = math.sqrt(dx * dx + dy * dy)
        if L > 1 then
            r.ux, r.uy, r.L = dx / L, dy / L, L
            local bm = math.max(r.blast1 or 0, r.blast2 or 0)
            if in_view(math.min(r.x1, r.x2) - 200 - bm, math.min(r.y1, r.y2) - 200 - bm,
                       math.max(r.x1, r.x2) + 200 + bm, math.max(r.y1, r.y2) + 200 + bm) then
                r.rank = ((E and (r.id1 == E.id or r.id2 == E.id)) and 1e6 or 0) + L
                vis[#vis + 1] = r
            end
        end
    end
    table.sort(vis, function(p, q) return p.rank < q.rank end)
    C.vis = vis
    -- wie im A350-OANS: nah grau wie die Rollwege mit braunem Rand und weissen Markierungen, weiter weg (ohne
    -- Markierungen) als weisser Balken ohne Rand
    for _, r in ipairs(vis) do
        local hw = r.w / 2
        if sc >= 0.12 then rq(r, -25 - (r.blast1 or 0), r.L + 25 + (r.blast2 or 0), hw + 9, -hw - 9, MC.shoulder) end
    end
    for _, r in ipairs(vis) do
        local L = r.L
        local hw = r.w / 2
        local b1, b2 = r.blast1 or 0, r.blast2 or 0
        rq(r, 0, L, hw, -hw, (r.w * sc >= 8) and MC.runway or MC.rwy_far)
        -- Blast Pad / Overrun vor dem Pistenende: dunkler, gelbe Winkel (Spitze zur Piste)
        for endi = 1, 2 do
            local bl = (endi == 1) and b1 or b2
            if bl > 5 then
                local a0, a1 = (endi == 1) and -bl or L, (endi == 1) and 0 or L + bl
                rq(r, a0, a1, hw, -hw, MC.blast)
                if r.w * sc >= 10 and bl >= 25 then
                    local th = math.max(1.2, 1.5 * sc)
                    local sp = math.max(30, bl / 6)
                    local d = (endi == 1) and 1 or -1  -- Richtung zur Piste
                    local e0 = (endi == 1) and 0 or L
                    local t = sp * 0.6
                    while t < bl - 8 do
                        local apex = e0 - d * t
                        local back = apex - d * hw * 0.55
                        local ax1, ay1 = rp(r, apex, 0)
                        local lx1, ly1 = rp(r, back, hw - 3)
                        local rx1, ry1 = rp(r, back, -hw + 3)
                        DL.line(lx1, ly1, ax1, ay1, MC.chevron, th)
                        DL.line(rx1, ry1, ax1, ay1, MC.chevron, th)
                        t = t + sp
                    end
                end
            end
        end
        if r.w * sc >= 8 then
            -- Randlinien
            rq(r, 0, L, hw - 0.5, hw - 1.4, MC.white)
            rq(r, 0, L, -hw + 1.4, -hw + 0.5, MC.white)
            -- Mittellinie (30 m Strich, 20 m Luecke)
            local t = r.disp1 + 60
            while t < L - r.disp2 - 60 do
                rq(r, t, math.min(t + 30, L - r.disp2 - 60), 0.45, -0.45, MC.white)
                t = t + 50
            end
            -- je Ende: Schwellenbalken, Aufsetzzone, Zielpunkt (erst nah sichtbar)
            for endi = 1, (r.w * sc >= 18) and 2 or 0 do
                local disp = (endi == 1) and r.disp1 or r.disp2
                local function A(a) if endi == 1 then return a else return L - a end end
                local function bar(a0, a1, c0, c1)
                    local x0a, x1a = A(a0), A(a1)
                    rq(r, math.min(x0a, x1a), math.max(x0a, x1a), c0, c1, MC.white)
                end
                -- Schwellenbalken (Klaviertasten), mittig eine Luecke
                local nb = (r.w >= 55) and 8 or ((r.w >= 40) and 6 or 4)
                local span = hw - 3
                local bw = 1.8
                local gap = (span - 3) / nb
                for k = 0, nb - 1 do
                    local c = 3 + k * gap
                    bar(disp + 6, disp + 36, c + bw, c)
                    bar(disp + 6, disp + 36, -c, -c - bw)
                end
                -- Zielpunkt (2 breite Balken bei 400 m)
                bar(disp + 400, disp + 460, hw - 6, hw - 15)
                bar(disp + 400, disp + 460, -hw + 15, -hw + 6)
                -- Aufsetzzone
                for _, z in ipairs({ { 150, 3 }, { 300, 3 }, { 550, 2 }, { 700, 2 }, { 850, 1 } }) do
                    for k = 0, z[2] - 1 do
                        local c = 7 + k * 3.6
                        bar(disp + z[1], disp + z[1] + 22, c + 1.8, c)
                        bar(disp + z[1], disp + z[1] + 22, -c, -c - 1.8)
                    end
                end
                if disp > 0 and disp <= 20 then bar(disp - 1, disp + 1, hw, -hw) end
            end
            -- versetzte Schwelle (schon bei kleinerem Massstab): weisser Querbalken, davor eine Reihe gefuellter
            -- Winkel, im versetzten Bereich Pfeile auf der Mittellinie (zur Schwelle) - wie in der X-Plane-Szenerie:
            -- duenner langer Schaft, getrennt davon ein gefuellter Winkel "Λ", der Schaft endet in dessen Kerbe
            for endi = 1, 2 do
                local disp = (endi == 1) and r.disp1 or r.disp2
                if disp > 20 then
                    local d = (endi == 1) and 1 or -1
                    local function A(a) if endi == 1 then return a else return L - a end end
                    rq(r, math.min(A(disp - 3), A(disp)), math.max(A(disp - 3), A(disp)), hw, -hw, MC.white)
                    local hl = math.min(hw * 0.4, math.max(4.5, 4 / sc)) -- Laenge = halbe Breite des Winkels
                    local tk = hl * 0.34                                  -- Dicke des Winkels
                    local ws = math.max(tk * 0.707, 0.9 / sc)             -- Schaftbreite (mindestens ~1 Pixel)
                    local ls = hl * 2.4                                   -- Schaftlaenge
                    if r.w * sc >= 14 then
                        local nh = math.max(2, math.floor((r.w - 6) / 9))
                        local pitch = (r.w - 6) / nh
                        local hh = math.min(math.max(pitch * 0.55, 2.2 / sc), disp * 0.3)
                        for k = 0, nh - 1 do
                            local c = -hw + 3 + pitch * (k + 0.5)
                            chevron(r, A(disp - 5), d, c, hh, pitch * 0.4, hh * 0.32, MC.white)
                        end
                    end
                    local total = ls + tk + hl * 0.2 + hl
                    local step = math.max(110, 2.2 * total)
                    local t = 25
                    while t + total < disp - 30 do
                        local apex = t + ls + tk + hl * 0.2
                        rq(r, math.min(A(t), A(t + ls)), math.max(A(t), A(t + ls)), ws / 2, -ws / 2, MC.white)
                        chevron(r, A(apex), d, 0, hl, hl, tk, MC.white)
                        t = t + step
                    end
                end
            end
        end
        local mx2, my2 = XY((r.x1 + r.x2) / 2, (r.y1 + r.y2) / 2)
        C.mid[r] = { mx2, my2 }
    end
end

-------------------------------------------------------------------------------
-- Beschriftungen mit Kollisionspruefung, Exits, DRY/WET, Pistenkennungen, Markierungen (Kreuze, Fahnen)
-------------------------------------------------------------------------------
-- Punkt in konvexem Vieleck (Bildschirm)
function MAPX.in_poly(pts, x, y)
    local n, sgn = #pts, 0
    for i = 1, n do
        local a, b = pts[i], pts[i % n + 1]
        local c = (b[1] - a[1]) * (y - a[2]) - (b[2] - a[2]) * (x - a[1])
        if c ~= 0 then
            local s = c > 0 and 1 or -1
            if sgn == 0 then sgn = s elseif s ~= sgn then return false end
        end
    end
    return true
end

-- gedrehtes Rechteck/Vieleck im Bildschirm: Rahmen und Fuellung
function MAPX.poly_fill(pts, col)
    for i = 2, #pts - 1 do DL.tri(pts[1][1], pts[1][2], pts[i][1], pts[i][2], pts[i + 1][1], pts[i + 1][2], col) end
end
function MAPX.poly_line(pts, col, th)
    local n = #pts
    for i = 1, n do
        local a, b = pts[i], pts[i % n + 1]
        DL.line(a[1], a[2], b[1], b[2], col, th)
    end
end

-- Pistenkennung (QFU) an jedem Pistenende wie im OANS: "Haus" (Spitze zur Piste), Schrift so gedreht, wie sie der
-- Pilot im Anflug liest; FMS-Landebahn mit gruenem Dreieck davor, BTV-Bahn cyan mit Rahmen. Antippen = BTV-Bahn.
function MAPX.qfu_labels(C)
    local XY, apt = C.XY, S.apt
    local poly_fill, poly_line = MAPX.poly_fill, MAPX.poly_line
    local fe = FMS.rwy_end(apt)
    local cap = TXT.cap
    local placed, hits = C.placed, C.hit
    local landE = S.rwy
    for _, r in ipairs(C.vis) do
        local skip = C.appr and not (landE and (r.id1 == landE.id or r.id2 == landE.id))
        for endi = 1, skip and 0 or 2 do
            local id = (endi == 1) and r.id1 or r.id2
            local ex, ey = (endi == 1) and r.x1 or r.x2, (endi == 1) and r.y1 or r.y2
            local d = (endi == 1) and 1 or -1
            local px, py = XY(ex, ey)
            local qx, qy = XY(ex + d * r.ux * 100, ey + d * r.uy * 100)
            local sx, sy = qx - px, qy - py
            local sl = math.sqrt(sx * sx + sy * sy)
            if sl > 1e-6 then
                sx, sy = sx / sl, sy / sl -- zur Piste
                local tw = DL.text_rot_w(id, cap)
                local W2, H2 = tw / 2 + 2.6, cap / 2 + 2.6
                local g = H2 * 0.9
                local off = 2 + g + H2
                local cx, cy = px - sx * off, py - sy * off
                if cx > C.x0 - 8 and cx < C.x0 + C.cw + 8 and cy > C.y0 - 8 and cy < C.y0 + C.ch + 8 then
                    local bx, by = -sy, sx -- Grundlinie
                    local function P(X, Y) return { cx + bx * X + sx * Y, cy + by * X + sy * Y } end
                    local Ex
                    for _, q in ipairs(apt.ends) do if q.id == id then Ex = q end end
                    local sel = Ex ~= nil and S.rwy == Ex
                    local col = sel and MC.sel or MC.white
                    local house = { P(-W2, -H2), P(W2, -H2), P(W2, H2), P(0, H2 + g), P(-W2, H2) }
                    poly_fill(house, MC.bg)
                    if sel then
                        poly_line(house, col, 0.9)
                    else
                        -- nicht gewaehlte Bahn wie im A350: unten offen (Seiten und Spitze, ohne Grundlinie)
                        for k = 2, 4 do
                            local a, b = house[k], house[k + 1]
                            DL.line(a[1], a[2], b[1], b[2], col, 0.9)
                        end
                        DL.line(house[5][1], house[5][2], house[1][1], house[1][2], col, 0.9)
                    end
                    local outer = { P(-W2 - 2.2, -H2 - 2.2), P(W2 + 2.2, -H2 - 2.2), P(W2 + 2.2, H2 + g + 2.2), P(-W2 - 2.2, H2 + g + 2.2) }
                    if sel then poly_line(outer, MC.sel, 0.9) end
                    DL.text_rot(cx, cy, math.atan2(by, bx), col, id, cap)
                    if Ex and Ex == (landE or fe) then
                        -- gruenes Dreieck wie im A350: direkt unter dem Kasten (Spitze am Kasten), fast so breit wie der
                        -- Kasten, flach, mit weisser Grundlinie (aus einem Bild vermessen)
                        local th, hw = W2 * 0.67, W2 * 0.9
                        local t1, t2, t3 = P(0, -H2 - 0.4), P(-hw, -H2 - 0.4 - th), P(hw, -H2 - 0.4 - th)
                        DL.tri(t1[1], t1[2], t2[1], t2[2], t3[1], t3[2], MC.green)
                        local b1, b2 = P(-hw - 0.5, -H2 - th - 1.2), P(hw + 0.5, -H2 - th - 1.2)
                        DL.line(b1[1], b1[2], b2[1], b2[2], MC.white, 0.9)
                    end
                    if Ex then hits[#hits + 1] = { poly = outer, qfu = Ex } end
                    local mnx, mny, mxx, mxy = math.huge, math.huge, -math.huge, -math.huge
                    for _, p in ipairs(outer) do
                        mnx, mxx = math.min(mnx, p[1]), math.max(mxx, p[1])
                        mny, mxy = math.min(mny, p[2]), math.max(mxy, p[2])
                    end
                    placed[#placed + 1] = { mnx, mny, mxx, mxy }
                end
            end
        end
    end
end

function MAPX.labels(C)
    local G, XY, INV, XYr, sc, E, nd = C.G, C.XY, C.INV, C.XYr, C.sc, C.E, C.nd
    local poly_fill = MAPX.poly_fill
    local x0, y0, cw, ch = C.x0, C.y0, C.cw, C.ch
    local placed = C.placed
    local mode = C.mode
    -- liegt ein Bildschirmpunkt (mit Rand in Pixeln) auf einer Piste?
    local function on_runway(px, py, mpx)
        local wx, wy = INV(px, py)
        local m = mpx / sc
        for _, r in ipairs(G.rwys) do
            if r.L then
                local qx, qy = wx - r.x1, wy - r.y1
                local al, cr = qx * r.ux + qy * r.uy, -qx * r.uy + qy * r.ux
                if al > -m and al < r.L + m and math.abs(cr) < r.w / 2 + m then return true end
            end
        end
        return false
    end
    local function place_box(cxp, cyp, str, col, frame_col, force, bg_col, allow_rwy)
        local tw = text_w(str)
        local r = { cxp - tw / 2 - 3, cyp - FONT_H / 2 - 2, cxp + tw / 2 + 3, cyp + FONT_H / 2 + 2 }
        if not force and DL.n > 57000 then return nil end -- Eckpunkt-Grenze des Fensters
        if not force and not allow_rwy and on_runway(cxp, cyp, tw / 2 + 4) then return nil end
        if force or not overlap(r, placed) then
            placed[#placed + 1] = r
            box_text(r[1] + 3, r[2] + 2, str, col, frame_col, bg_col)
            return r
        end
    end
    -- BTV-Anzeigen (Exits, DRY/WET): im ND sobald eine Landebahn gewaehlt ist (in der Luft / waehrend BTV),
    -- im Fenster in der BTV-Ansicht
    local show_btv = E ~= nil and ((nd and btv_ui()) or (not nd and mode == "btv"))

    -- Pistenkennungen
    if nd then
        MAPX.qfu_labels(C)
    else
        for _, r in ipairs(C.vis) do
            local landing = E and ((r.id1 == E.id) or (r.id2 == E.id))
            local ax1, ay1 = XY(r.x1 - r.ux * 40, r.y1 - r.uy * 40)
            local bx1, by1 = XY(r.x2 + r.ux * 40, r.y2 + r.uy * 40)
            for _, l in ipairs({ { ax1, ay1, r.id1 }, { bx1, by1, r.id2 } }) do
                if landing then place_box(l[1], l[2], l[3], MC.white, MC.white, true)
                elseif l[1] > x0 and l[1] < x0 + cw and l[2] > y0 and l[2] < y0 + ch then
                    place_box(l[1], l[2], l[3], MC.white, MC.white)
                end
            end
        end
    end

    local ts, tname, tx = current_target()
    -- DRY / WET (ROW): Bremsstrecke trocken / nass, magenta - ueber 500 ft vorausberechnet, darunter in Echtzeit
    -- (S.row_stops). Liegt WET hinter dem Pistenende: WET amber; liegt DRY dahinter: beide rot. Sobald BTV bremst,
    -- zeigt statt dessen der Haltebalken den vorhergesagten Haltepunkt.
    if CFG.rops and E and (show_btv or S.row_on) and not (S.mode == "ENGAGED" or S.mode == "RELEASE") then
        local lines = S.row_lines(E)
        local short = false
        for _, l in ipairs(lines) do if l[3] == "pri" and l[2] > E.len then short = true end end
        local hw = E.width / 2
        local ext = math.max(8, 9 / sc) -- ueber den Pistenrand hinaus (mindestens 9 Bildpunkte)
        local list = {}
        for _, l in ipairs(lines) do
            local col = short and MC.short or ((l[2] > E.len) and MC.amber or MC.mag)
            list[#list + 1] = { l[1], l[2], col, (l[3] == "pri") and 300 or 600 }
        end
        for _, b in ipairs(list) do
            local sb = math.min(b[2], E.len + b[4]) -- (hinter dem Pistenende begrenzt, DRY vor WET)
            local lcol = b[3]
            do
                local p1x, p1y = XYr(sb, hw + ext)
                local p2x, p2y = XYr(sb, -hw - ext)
                DL.line(p1x, p1y, p2x, p2y, lcol, nd and 2.0 or 3)
                -- Beschriftung am Ende der Linie (ausserhalb der Piste), magenta
                local tw = text_w(b[1])
                local function at(px, py, qx, qy)
                    local dx, dy = px - qx, py - qy
                    local dl = math.sqrt(dx * dx + dy * dy)
                    if dl < 1e-6 then return nil end
                    local k = (tw / 2 + 5) / dl
                    return place_box(px + dx * k, py + dy * k, b[1], lcol, nil, false, MC.bg, true)
                end
                if not at(p1x, p1y, p2x, p2y) and not at(p2x, p2y, p1x, p1y) then
                    local dx, dy = p1x - p2x, p1y - p2y
                    local dl = math.max(math.sqrt(dx * dx + dy * dy), 1e-6)
                    place_box(p1x + dx / dl * (tw / 2 + 5), p1y + dy / dl * (tw / 2 + 5), b[1], lcol, nil, true)
                end
            end
        end
    end
    -- BTV-Weg (ND): cyan auf der Mittellinie von der Schwelle bis zum Exit/Stoppunkt
    if nd and show_btv and ts then
        local ax1, ay1 = XYr(E.disp, 0)
        local bx1, by1 = XYr(ts, 0)
        DL.line(ax1, ay1, bx1, by1, S.rop and MC.short or MC.sel, 1.6) -- (ROP: Weg und Haltebalken rot)
    end
    -- Stoppunkt ohne Exit: Querbalken
    if ts and E and not tx and (show_btv or S.mode == "ENGAGED" or S.mode == "RELEASE") then
        local hw = E.width / 2
        local p1x, p1y = XYr(ts, hw + 10)
        local p2x, p2y = XYr(ts, -hw - 10)
        DL.line(p1x, p1y, p2x, p2y, MC.sel, nd and 3 or 4)
    end
    -- vorhergesagter Haltepunkt waehrend der BTV-Bremsung: gruener Balken
    if E and (S.mode == "ENGAGED" or S.mode == "RELEASE") then
        local s = rwy_sd(E, DR.lat[0], DR.lon[0])
        local v = DR.gs[0]
        local a = S.braking and math.max(S.a_cmd, S.a_meas, 0.35) or math.max(S.a_meas, 0.35)
        local sp = math.min(s + v * v / (2 * a), E.len + 150)
        local hw = E.width / 2
        local p1x, p1y = XYr(sp, hw + 6)
        local p2x, p2y = XYr(sp, -hw - 6)
        DL.line(p1x, p1y, p2x, p2y, S.rop and MC.short or MC.green, nd and 3 or 4)
    end

    -- Exits der Landebahn: mit BTV, und im ND wie im A350 ab 1 NM Reichweite (und kleiner) immer
    local marks = {}
    local near_x = nd and sc >= 0.09
    if E and (show_btv or tx or near_x) then
        local function prio(x, i)
            if x == tx then return 0 end
            if i == S.rec_idx then return 1 end
            if not x.too_short then return 2 end
            return 3
        end
        local order = {}
        for i, x in ipairs(S.exits) do
            if (show_btv or x == tx or near_x) and (not C.appr or x == tx or i == S.rec_idx) then order[#order + 1] = { x = x, i = i } end
        end
        table.sort(order, function(a, b)
            local pa, pb = prio(a.x, a.i), prio(b.x, b.i)
            if pa ~= pb then return pa < pb end
            return a.x.s < b.x.s
        end)
        for _, o in ipairs(order) do
            local x = o.x
            local selected = (x == tx)
            local col = selected and MC.sel or (o.i == S.rec_idx and MC.rec) or (x.too_short and MC.short) or MC.label
            local tw = text_w(x.name)
            local found, ft
            for _, px in ipairs(nd and { 10, 18, 28, 40, 55 } or { 16, 30, 46, 64, 88, 115 }) do
                local t = x.t0 + px / sc
                local ps, pd, us, ud = path_point(x, t)
                local qx, qy = XYr(ps, pd)
                local q2x, q2y = XYr(ps + us, pd + ud)
                local dxs, dys = q2x - qx, q2y - qy
                local dl = math.sqrt(dxs * dxs + dys * dys)
                local nx, ny = 0, 0
                if dl > 0 then nx, ny = -dys / dl, dxs / dl end
                for _, n in ipairs({ 0, 12, -12 }) do
                    local lx, ly = qx + nx * n, qy + ny * n
                    local r = { lx - tw / 2 - 3, ly - FONT_H / 2 - 2, lx + tw / 2 + 3, ly + FONT_H / 2 + 2 }
                    if not overlap(r, placed) and not on_runway(lx, ly, tw / 2 + 3) then found, ft = r, t; break end
                end
                if found then break end
            end
            if not found and selected then
                ft = x.t0 + 20 / sc
                local qx, qy = XYr(path_point(x, ft))
                found = { qx - tw / 2 - 3, qy - FONT_H / 2 - 2, qx + tw / 2 + 3, qy + FONT_H / 2 + 2 }
            end
            local m = { x = x, i = o.i, rect = found, alt = {} }
            if selected then
                m.pts = path_screen(x, 0, math.max(x.plen, (ft or 0) + 10 / sc), XYr)
                draw_poly(m.pts, MC.sel, nd and ((sc >= 0.9) and 2 or ((sc >= 0.35) and 1.5 or 1)) or 3) -- (ND: so breit wie die Rollweg-Mittellinien)
                -- Rollweg geht auf beiden Seiten weiter: beide Wege zeigen
                for _, a in ipairs(x.alt or {}) do
                    local pp = path_screen(a, 0, a.plen, XYr)
                    draw_poly(pp, MC.sel, nd and ((sc >= 0.9) and 2 or ((sc >= 0.35) and 1.5 or 1)) or 3) -- (ND: so breit wie die Rollweg-Mittellinien)
                    m.alt[#m.alt + 1] = pp
                end
            else
                m.pts = path_screen(x, x.t0, ft or (x.t0 + 40 / sc), XYr)
                draw_poly(m.pts, col, nd and ((col ~= MC.label) and 1.6 or 1.2) or 1.5)
                for _, a in ipairs(x.alt or {}) do m.alt[#m.alt + 1] = path_screen(a, a.t0, a.t0 + 40 / sc, XYr) end
            end
            if found then
                placed[#placed + 1] = found
                if nd then
                    -- OANS: gelbe Schrift auf Schwarz, gewaehlter Exit cyan mit Rahmen;
                    -- empfohlener Exit gruen, nicht mehr erreichbare rot hinterlegt (wie in der BTV-Ansicht)
                    if selected then box_text(found[1] + 3, found[2] + 2, x.name, MC.sel, MC.sel, MC.bg)
                    elseif col ~= MC.label then box_text(found[1] + 3, found[2] + 2, x.name, MC.bg, nil, col)
                    else box_text(found[1] + 3, found[2] + 2, x.name, MC.label, nil, MC.bg) end
                elseif selected then box_text(found[1] + 3, found[2] + 2, x.name, MC.sel, MC.sel, MC.bg)
                else box_text(found[1] + 3, found[2] + 2, x.name, MC.bg, nil, col) end
            end
            marks[#marks + 1] = m
        end
    end
    C.marks = marks

    -- Pistenpaar in der Mitte ("07C-25C"): im ND entlang der Piste, im Fenster ausser in der BTV-Ansicht
    if nd and C.appr then
        -- (Endanflug: keine)
    elseif nd then
        for _, r in ipairs(C.vis) do
            local t = r.id1 .. "-" .. r.id2
            local tw = DL.text_rot_w(t, TXT.small)
            local mp = C.mid[r]
            if mp and mp[1] > x0 + 15 and mp[1] < x0 + cw - 15 and mp[2] > y0 + 15 and mp[2] < y0 + ch - 15
                and r.L * sc > tw * 2.5 then
                local ax1, ay1 = XY(r.x1, r.y1)
                local bx1, by1 = XY(r.x2, r.y2)
                local ux, uy = bx1 - ax1, by1 - ay1
                local ul = math.sqrt(ux * ux + uy * uy)
                ux, uy = ux / ul, uy / ul
                if ux < 0 then ux, uy = -ux, -uy end
                local vx, vy = -uy, ux
                local W2, H2 = tw / 2 + 2, TXT.small / 2 + 2
                local cxp, cyp = mp[1], mp[2]
                local box = { { cxp - ux * W2 - vx * H2, cyp - uy * W2 - vy * H2 }, { cxp + ux * W2 - vx * H2, cyp + uy * W2 - vy * H2 },
                              { cxp + ux * W2 + vx * H2, cyp + uy * W2 + vy * H2 }, { cxp - ux * W2 + vx * H2, cyp - uy * W2 + vy * H2 } }
                local mnx, mny, mxx, mxy = math.huge, math.huge, -math.huge, -math.huge
                for _, p in ipairs(box) do
                    mnx, mxx = math.min(mnx, p[1]), math.max(mxx, p[1])
                    mny, mxy = math.min(mny, p[2]), math.max(mxy, p[2])
                end
                local rr = { mnx, mny, mxx, mxy }
                if not overlap(rr, placed) then
                    placed[#placed + 1] = rr
                    poly_fill(box, MC.bg)
                    DL.text_rot(cxp, cyp, math.atan2(uy, ux), MC.white, t, TXT.small)
                end
            end
        end
    elseif mode ~= "btv" then
        for _, r in ipairs(G.rwys) do
            local mp = C.mid and C.mid[r]
            if mp and mp[1] > x0 + 20 and mp[1] < x0 + cw - 20 and mp[2] > y0 + 20 and mp[2] < y0 + ch - 20 then
                place_box(mp[1], mp[2], r.id1 .. "-" .. r.id2, MC.white, MC.white, false, MC.bg, true)
            end
        end
    end

    -- Abrollpunkte aller Pisten (wie im A350-OANS bis 1 NM): Rollweg-Kennung gelb auf Schwarz neben jeder Abzweigung,
    -- dazu ein kurzer weisser Strich an der Pistenkante. Die gewaehlte BTV-Bahn hat ihre eigenen Exit-Marken (oben).
    if nd and sc >= 0.09 and not C.appr and G.vac then
        local vminx, vmaxx, vminy, vmaxy = C.vminx, C.vmaxx, C.vminy, C.vmaxy
        local edge = 22.5 * sc + 1.5   -- (halbe Pistenbreite 45 m)
        for _, v in ipairs(G.vac) do
            local skip = v.x < vminx or v.x > vmaxx or v.y < vminy or v.y > vmaxy
            if not skip and E then
                local rx, ry = v.x - E.ox, v.y - E.oy
                local along = rx * E.ux + ry * E.uy
                skip = math.abs(rx * E.uy - ry * E.ux) < 45 and along > -60 and along < E.len + 60
            end
            if not skip then
                local px, py = XY(v.x, v.y)
                local qx, qy = XY(v.x + v.ux / sc, v.y + v.uy / sc)
                local dx, dy = qx - px, qy - py
                local dl = math.sqrt(dx * dx + dy * dy)
                if dl > 0 then
                    dx, dy = dx / dl, dy / dl
                    local tw = text_w(v.name)
                    local off = edge + 6 + math.abs(dx) * (tw / 2 + 3) + math.abs(dy) * (FONT_H / 2 + 2)
                    local lx, ly = px + dx * off, py + dy * off
                    local r = { lx - tw / 2 - 3, ly - FONT_H / 2 - 2, lx + tw / 2 + 3, ly + FONT_H / 2 + 2 }
                    if not overlap(r, placed) then
                        placed[#placed + 1] = r
                        DL.line(px + dx * edge, py + dy * edge, px + dx * (edge + 4), py + dy * (edge + 4), MC.white, 1.2)
                        box_text(r[1] + 3, r[2] + 2, v.name, MC.label, nil, MC.bg)
                    end
                end
            end
        end
    end

    -- Rollweg-Namen (gelb), gleiche Namen mit Abstand (nicht in der Endanflug-Karte; wie im A350 nur bis 0,5 NM)
    if not C.appr and sc >= 0.12 then
        local per, count = {}, 0
        for _, m in ipairs(marks) do
            if m.rect then
                local nm = m.x.name
                per[nm] = per[nm] or {}
                table.insert(per[nm], { (m.rect[1] + m.rect[3]) / 2, (m.rect[2] + m.rect[4]) / 2 })
            end
        end
        local minlen = 18 / sc
        local gap2 = 170 * 170
        local vminx, vmaxx, vminy, vmaxy = C.vminx, C.vmaxx, C.vminy, C.vmaxy
        for _, e in ipairs(G.lbl) do
            if count >= 110 or e.len < minlen then break end
            local wx, wy = (e.x1 + e.x2) / 2, (e.y1 + e.y2) / 2
            if wx > vminx and wx < vmaxx and wy > vminy and wy < vmaxy then
                local px, py = XY(wx, wy)
                if px > x0 + 12 and px < x0 + cw - 12 and py > y0 + 10 and py < y0 + ch - 10 then
                    local lst = per[e.name]
                    local ok = true
                    if lst then
                        for _, q in ipairs(lst) do
                            if (q[1] - px) ^ 2 + (q[2] - py) ^ 2 < gap2 then ok = false; break end
                        end
                    end
                    if ok and place_box(px, py, e.name, MC.bg, nil, false, MC.label) then
                        count = count + 1
                        if not lst then lst = {}; per[e.name] = lst end
                        lst[#lst + 1] = { px, py }
                    end
                end
            end
        end
    end

    -- Namen der Parkpositionen (MARS-Gruppen gemeinsam, z.B. "29_29A_29B"); im ND bei 0,2 NM (ARC, ROSE, PLAN)
    if sc >= (nd and 0.33 or 0.6) then
        local vminx, vmaxx, vminy, vmaxy = C.vminx, C.vmaxx, C.vminy, C.vmaxy
        for _, l in ipairs(G.stand_lbls) do
            if not nd then l.px = nil end
            if l.x > vminx and l.x < vmaxx and l.y > vminy and l.y < vmaxy then
                local px, py = XY(l.x, l.y)
                if place_box(px, py, l.text, MC.stand_txt, MC.stand_frame, false, MC.stand_bg) then
                    C.nstand = (C.nstand or 0) + 1
                    if not nd then l.px, l.py = px, py end
                end
            end
        end
    elseif not nd then
        for _, l in ipairs(G.stand_lbls) do l.px = nil end
    end

    MAPX.draw_marks(C)
end

-- Kreuze (gelb) und Fahnen (magenta) des Piloten
function MAPX.draw_marks(C)
    local mk = S.apt and S.marks and S.marks[S.apt.icao]
    if not mk then return end
    local XY = C.XY
    local x0, y0, x1, y1 = C.x0, C.y0, C.x0 + C.cw, C.y0 + C.ch
    for _, c in ipairs(mk.crosses) do
        local px, py = XY(c[1], c[2])
        if px > x0 - 10 and px < x1 + 10 and py > y0 - 10 and py < y1 + 10 then
            for pass = 1, 2 do
                local col, th = (pass == 1) and MC.bg or MC.cross, (pass == 1) and 3.4 or 1.8
                DL.line(px - 6, py - 6, px + 6, py + 6, col, th)
                DL.line(px - 6, py + 6, px + 6, py - 6, col, th)
            end
        end
    end
    for _, c in ipairs(mk.flags) do
        local px, py = XY(c[1], c[2])
        if px > x0 - 12 and px < x1 + 12 and py > y0 - 20 and py < y1 + 6 then
            DL.line(px, py + 0.5, px, py - 15, MC.bg, 3)
            DL.line(px, py + 0.5, px, py - 15, MC.mag, 1.4)
            DL.tri(px + 0.6, py - 15, px + 10, py - 11.5, px + 0.6, py - 8, MC.mag)
            DL.circlef(px, py, 1.6, MC.mag, 8)
        end
    end
end

-------------------------------------------------------------------------------
-- OANS-Bedienung im ND (Aufbau und Funktionen wie das OANS im A380 von FlyByWire):
-- - oben links GS/TAS (wie im FF-ND), darunter RWY / EXIT / ROT / TURNAROUND (BTV), sonst der Wind
-- - oben rechts Stadt, ICAO und IATA des Flughafens
-- - Kompass nach EFIS-Modus, Reichweite in cyan
-- - Pistenkennung antippen = BTV-Landebahn, Exit antippen = BTV-Exit (nur in der Luft)
-- - Doppelklick auf die Karte = Kontextmenue: ADD CROSS, ADD FLAG, MAP DATA, ERASE ALL CROSSES/FLAGS, CENTER ON ACFT
-- - Doppelpfeil unten rechts = Bedienfeld: MAP DATA (Kreuze/Fahnen, CENTER MAP ON, BTV STOP DISTANCE),
--   ARPT SEL (Flughafen nach ICAO / IATA / Stadt, DISPLAY AIRPORT, FMS-Flughaefen), STATUS
-- - "RWY AHEAD": am Boden unter 40 kt eine Piste im Weg (Spannweite breit, 7 s voraus) - Kasten im ND und PFD
-- - Mauszeiger ueber der Karte: gelbes Fadenkreuz wie im FF-Cockpit
-------------------------------------------------------------------------------
local NDUI = { kb = nil, focus = nil, blink = 0 }
do
-- Grundlinien der Kopfzeilen: Zeile 1 wie im FF-ND (13 Panel-Pixel unter der Oberkante), dann alle 21 Pixel
local HDR_BASE = { 7.76, 20.3, 31.4, 42.5, 53.6 }
local UI_COL = 2.7     -- linker Rand der Kopfzeilen
local PANEL_Y = 294.1  -- Oberkante des Bedienfelds: Oberkante der Reiter (+1) auf der langen Linie des PFD daneben

local function in_rect(r, x, y) return x ~= nil and r ~= nil and x >= r[1] and x <= r[3] and y >= r[2] and y <= r[4] end
NDUI.in_rect = in_rect
local function hit_add(C, r, fn, ex)
    local h = { r = r, fn = fn, ui = true }
    if ex then for k, v in pairs(ex) do h[k] = v end end
    C.hit[#C.hit + 1] = h
    return h
end
local function hov(C, r) return C.hovered and in_rect(r, C.mx, C.my) end

-- oberstes Element unter einem Punkt (ui_only: nur Bedienelemente)
function NDUI.hit_at(list, x, y, ui_only)
    if not list or not x then return nil end
    for i = #list, 1, -1 do
        local h = list[i]
        if (not ui_only or h.ui) and ((h.r and in_rect(h.r, x, y)) or (h.poly and MAPX.in_poly(h.poly, x, y))) then return h end
    end
end

-- Kreuze und Fahnen des aktuellen Flughafens
function NDUI.marks()
    if not S.apt then return { crosses = {}, flags = {} } end
    S.marks = S.marks or {}
    local m = S.marks[S.apt.icao]
    if not m then m = { crosses = {}, flags = {} }; S.marks[S.apt.icao] = m end
    return m
end

function NDUI.center_acft(V)
    V.free = {}
    V.plan_acft = true
end

-- Tastatureingabe (Flughafen, Stoppstrecke): Fokus beim Mausfenster des Skripts
function NDUI.kb_begin(side, field, text)
    NDUI.kb = { side = side, field = field, text = text or "" }
    if NDUI.focus then NDUI.focus(true) end
end
function NDUI.kb_end()
    if NDUI.kb and NDUI.focus then NDUI.focus(false) end
    NDUI.kb = nil
end
function NDUI.kb_on(side, field)
    local kb = NDUI.kb
    return kb ~= nil and kb.side == side and kb.field == field
end

-- Stuecke einer Zeile { Text, Farbe, x (optional, sonst direkt anschliessend) }
function NDUI.segs_end(segs, cap)
    local x = UI_COL
    for _, sg in ipairs(segs) do
        x = (sg[3] or x) + text_w(sg[1], cap)
    end
    return x
end
function NDUI.draw_segs(segs, base, cap)
    local x = UI_COL
    for _, sg in ipairs(segs) do
        x = txt_b(sg[3] or x, base, sg[2], sg[1], cap)
    end
    return x
end

-- Bremsabkuehlzeit [min] mit voller / Leerlauf-Umkehr (Schaetzung): Bremsarbeit bis zum Exit -> Spitzentemperatur
-- der Kohlenstoffbremsen -> Abkuehlen (ohne Luefter) bis 150 Grad
function NDUI.turnaround(E, ts, x)
    local v = S.v_est
    local ve = exit_speed(x)
    local D = math.max(ts - (E.disp + P.tdd(E)), 100)
    local a_req = math.max(0, (v * v - ve * ve) / (2 * D))
    local m = DR.mass and DR.mass[0] or 0
    if m < 60000 then m = 190000 end
    local oat = DR.oat and DR.oat[0] or 15
    local function minutes(a_rev)
        local a_brk = math.max(0, a_req - 0.3 - a_rev)
        local T = oat + 50 + m * a_brk * D / 1.25e6
        if T <= 150 or oat >= 140 then return 0 end
        return math.floor(60 * math.log((T - oat) / (150 - oat)) + 0.5)
    end
    return minutes(0.7), minutes(0.15)
end

-- Kopfzeilen 2..5 fuer BTV (in der Luft bzw. waehrend BTV, sobald eine Landebahn gewaehlt ist)
function NDUI.header_lines(C)
    local lines = {}
    local E = C.E
    if not (E and btv_ui()) then return lines end
    local cap = TXT.cap
    local adv = text_w("M", cap)
    local function col(n) return UI_COL + n * adv end
    local G, W, Y = MC.green, MC.white, MC.sel
    local lda = math.floor(E.len - E.disp + 0.5)
    local var = DR.psi[0] - DR.mag[0]
    local hdg = math.floor((E.hdg - var) % 360 + 0.5)
    if hdg == 0 then hdg = 360 end
    lines[1] = { { "RWY", W, col(0) }, { E.id, G, col(5) }, { tostring(lda), G, col(10) }, { "M", Y }, { " -", W },
                 { string.format(" %03d", hdg), G }, { "\194\176", Y }, { "  " .. (S.cond_txt or "DRY"), G } }
    local ts, tname, tx = current_target()
    if ts then
        local dist = math.floor(ts - E.disp + 0.5)
        if tx then lines[2] = { { "EXIT", W, col(0) }, { tname, G, col(5) }, { tostring(dist), G, col(10) }, { "M", Y } }
        else lines[2] = { { "STOP", W, col(0) }, { tostring(dist), G, col(10) }, { "M", Y } } end
        local rot = rot_estimate(ts, tx)
        if rot then lines[#lines + 1] = { { "ROT", W, col(0) }, { tostring(math.floor(rot + 0.5)), G, col(5) }, { "\"", Y } } end
        local tf, ti = NDUI.turnaround(E, ts, tx)
        lines[#lines + 1] = { { "TURNAROUND", W, col(0) }, { tostring(tf), G, col(11) }, { "'", Y }, { "/", W }, { tostring(ti), G }, { "'", Y } }
    else
        lines[2] = { { "FOR BTV:SELECT EXIT", W, col(0) } }
    end
    return lines
end

-- Stadt / Codes des geladenen Flughafens (oben rechts)
function NDUI.apt_texts(apt)
    -- Stadt / IATA fehlen in manchen Szenerien: aus den anderen Quellen des Flughafens (Global Airports)
    if not apt.meta_done then
        apt.meta_done = true
        for _, e in ipairs(IDX.by_icao[apt.icao] or {}) do
            local m = META.get(e)
            apt.city = apt.city or m.city
            apt.iata = apt.iata or m.iata
        end
    end
    local city = apt.city or ((apt.name ~= "" and apt.name) or apt.icao)
    city = city:upper()
    if #city > 24 then city = city:sub(1, 24) end
    local iata = apt.iata or (apt.geo and apt.geo.iata)
    return city, apt.icao .. (iata and ("   " .. iata:upper()) or "")
end

-- Bereiche, die Beschriftungen der Karte frei lassen muessen (vor dem Zeichnen der Beschriftungen)
function NDUI.prepare(C, V, sd)
    local placed = C.placed
    C.hdr = NDUI.header_lines(C)
    local cap = TXT.cap
    placed[#placed + 1] = { 0, 0, 84, HDR_BASE[1] + 2.3 }
    if #C.hdr > 0 then
        for i, segs in ipairs(C.hdr) do
            placed[#placed + 1] = { 0, HDR_BASE[i + 1] - 10.3, NDUI.segs_end(segs, cap) + 2, HDR_BASE[i + 1] + 2.3 }
        end
    else
        placed[#placed + 1] = { 0, HDR_BASE[2] - 10.3, 48, HDR_BASE[2] + 2.3 }
    end
    local city, codes = NDUI.apt_texts(S.apt)
    local w = math.max(text_w(city, cap), text_w(codes, cap))
    placed[#placed + 1] = { 297 - w - 2, 0, ND_VW, HDR_BASE[2] + 2.3 }
    if RWYA.text then placed[#placed + 1] = { 95, 40, 205, 81 } end
    -- (keine Kartenbeschriftung unter den Meldungskaesten, dem Rollweg-Kaestchen und dem Bedienfeld)
    local _, _, _, y_hi = NDUI.lay(V.ui.panel)
    placed[#placed + 1] = { 0, y_hi, ND_VW, ND_VH }
end

-- Kompass je EFIS-Modus
function NDUI.compass(C)
    local mode, ax, ay, R = C.mode, C.ax, C.ay, C.Rr
    local cap = TXT.cap
    -- gestrichelter Ring wie im FF-ND (aus einem Bild vermessen): Striche 4,5 lang, Luecken 5,7 - auf jedem Ring gleich
    -- lang -, ein Strich oben in der Mitte, duenner als der durchgezogene Bogen
    local function dashed(r)
        if r < 5 then return end
        local n = math.max(8, math.floor(2 * math.pi * r / 10.2 + 0.5))
        local per, on = 2 * math.pi / n, 4.5 / r
        for k = 0, n - 1 do
            local a1 = k * per - on / 2
            local a2 = a1 + on
            DL.line(ax + r * math.sin(a1), ay - r * math.cos(a1), ax + r * math.sin(a2), ay - r * math.cos(a2), MC.ring, 0.8)
        end
    end
    -- Beschriftung wie im A350: ganze Zahlen ohne Nachkommastellen, sonst ohne Nullen am Ende (2.5, 0.25, 0.1)
    local function fmt(v)
        if math.abs(v - math.floor(v + 0.5)) < 1e-6 then return string.format("%d", math.floor(v + 0.5)) end
        return (string.format("%.2f", v):gsub("0+$", ""))
    end
    -- ARC: der aeussere (durchgezogene) Bogen = Reichweite; ROSE/PLAN: der Kreis = halbe Reichweite (= Durchmesser)
    local hs = fmt(C.rng / 2)
    local rs = fmt(C.rng)
    -- Reichweite wie im A350 in einem kleinen schwarzen Kasten (die Karte darunter verdeckt sie nicht)
    local function rlabel(x, y, s)
        local w = text_w(s, cap)
        DL.rectf(x - w / 2 - 1.5, y - cap / 2 - 2, x + w / 2 + 1.5, y + cap / 2 + 2, MC.bg)
        DL.text(x - w / 2, y - FONT_H / 2, MC.sel, s, cap)
    end
    if mode == "plan" then
        -- Kreis mit N / E / S / W (Norden oben; in der BTV-Ansicht mit der Landebahn gedreht),
        -- gestrichelter Halbkreis, Reichweite unten links
        DL.circle(ax, ay, R, MC.ring, 120, 1.0)
        dashed(R / 2)
        local rot = (C.hv or 0) * D2R
        for k, l in ipairs({ "N", "E", "S", "W" }) do
            local a = (k - 1) * math.pi / 2 - rot
            local sx, sy = math.sin(a), -math.cos(a)
            local tx, ty = ax + sx * (R - 11), ay + sy * (R - 11)
            DL.text(tx - text_w(l, cap) / 2, ty - FONT_H / 2, MC.white, l, cap)
            -- kleines Dreieck aussen am Buchstaben (zeigt nach aussen)
            local o = (k % 2 == 1) and 8 or 7.5
            local px, py = tx + sx * o, ty + sy * o
            local qx, qy = -sy, sx
            DL.tri(px + sx * 2.8, py + sy * 2.8, px - sx * 1.8 + qx * 2.6, py - sy * 1.8 + qy * 2.6,
                px - sx * 1.8 - qx * 2.6, py - sy * 1.8 - qy * 2.6, MC.white)
        end
        local th = 225 * D2R
        rlabel(ax + (R - 16) * math.sin(th), ay - (R - 16) * math.cos(th), hs)
        return
    end
    local hdg = DR.mag[0]
    if mode == "arc" then
        -- Bogen wie im FF-ND: Striche alle 5 Grad, Zahlen alle 10 Grad (alle 30 Grad groesser)
        local a0 = -62
        local px, py
        for a = a0, -a0, 2 do
            local t = a * D2R
            local qx, qy = ax + R * math.sin(t), ay - R * math.cos(t)
            if px then DL.line(px, py, qx, qy, MC.ring, 1.1) end
            px, py = qx, qy
        end
        local d0 = math.ceil((hdg + a0) / 5) * 5
        for deg = d0, hdg - a0, 5 do
            local t = (deg - hdg) * D2R
            local sx, sy = math.sin(t), -math.cos(t)
            local l = (deg % 10 == 0) and 8 or 4.5
            DL.line(ax + R * sx, ay + R * sy, ax + (R - l) * sx, ay + (R - l) * sy, MC.ring, 1.0)
            local dd = deg % 360
            if dd % 10 == 0 then
                local s = tostring(math.floor(dd / 10))
                local big = dd % 30 == 0
                local c2 = big and 10.75 or 7.76 -- 18 / 13 Pixel wie im FF-ND
                local lx, ly = ax + (R + (big and 15 or 13)) * sx, ay + (R + (big and 15 or 13)) * sy
                DL.text_rot(lx, ly, t, MC.white, s, c2)
            end
        end
        dashed(R / 2)
        -- Reichweite links und rechts innen am aeusseren Bogen (der Bogen ist die gewaehlte Reichweite)
        for _, a in ipairs({ -52, 52 }) do
            local t = a * D2R
            rlabel(ax + (R - 14) * math.sin(t), ay - (R - 14) * math.cos(t), rs)
        end
    else
        -- Rose: Striche alle 5 Grad, Zahlen alle 30 Grad, Dreiecke alle 45 Grad
        DL.circle(ax, ay, R, MC.ring, 120, 1.0)
        for deg = 0, 355, 5 do
            local t = (deg - hdg) * D2R
            local sx, sy = math.sin(t), -math.cos(t)
            local l = (deg % 10 == 0) and 7 or 4
            DL.line(ax + R * sx, ay + R * sy, ax + (R - l) * sx, ay + (R - l) * sy, MC.ring, 0.9)
            if deg % 30 == 0 then
                local s = tostring(math.floor(deg / 10))
                local lx, ly = ax + (R - 15) * sx, ay + (R - 15) * sy
                DL.text_rot(lx, ly, t, MC.white, s, (deg % 90 == 0) and TXT.big or cap)
            end
            if deg % 45 == 0 then
                local px, py = ax + (R + 2) * sx, ay + (R + 2) * sy
                DL.tri(px, py, px + sx * 6 - sy * 3, py + sy * 6 + sx * 3, px + sx * 6 + sy * 3, py + sy * 6 - sx * 3, MC.ring)
            end
        end
        dashed(R / 2)
        -- Reichweite (Radius des Kreises) innen am Kreis unten links, wie im A350-OANS
        local th = 225 * D2R
        rlabel(ax + (R - 16) * math.sin(th), ay - (R - 16) * math.cos(th), hs)
    end
    -- Steuerstrich (gelb) oben
    if mode == "arc" then DL.line(ax, ay - R - 9, ax, ay - R + 9, MC.lubber, 1.6)
    else DL.line(ax, ay - R - 9, ax, ay - R + 7, MC.lubber, 1.6) end
end

-- Kopfzeilen, Flughafen, Hinweise
function NDUI.header(C)
    local cap = TXT.cap
    local b1 = HDR_BASE[1]
    -- Zeile 1: GS / TAS genau wie im FF-ND
    local gs, tas = math.floor(DR.gs[0] / KT + 0.5), math.floor(DR.tas[0] / KT + 0.5)
    DL.rectf(0, 0, 84, b1 + 2.3, MC.bg)
    txt_b(UI_COL, b1, MC.white, "GS", cap)
    txt_b(37.9, b1, MC.green, tostring(gs), TXT.big, true)
    txt_b(41.3, b1, MC.white, "TAS", cap)
    if tas >= 60 then
        txt_b(80.1, b1, MC.green, tostring(tas), TXT.big, true)
    else
        for k = 0, 2 do DL.rectf(61.2 + 7.1 * k, 2.5, 65.5 + 7.1 * k, 3.7, MC.green) end
    end
    if #C.hdr > 0 then
        for i, segs in ipairs(C.hdr) do
            local base = HDR_BASE[i + 1]
            DL.rectf(0, base - 10.3, NDUI.segs_end(segs, cap) + 1.5, base + 2.3, MC.bg)
            NDUI.draw_segs(segs, base, cap)
        end
    else
        -- Wind wie im FF-ND ("---/--" ohne gueltigen Wind)
        local b = HDR_BASE[2]
        local wd, wsp
        if DR.wspd and DR.tpsi and DR.tas[0] > 60 * KT then
            wd = math.floor((DR.wdir[0] + DR.tpsi[0] - DR.mpsi[0]) % 360 + 0.5) % 360
            wsp = math.floor(DR.wspd[0] + 0.5)
        end
        local xe = 46
        if wd then xe = math.max(xe, 33 + text_w(tostring(wsp), cap) + 1) end
        DL.rectf(0, b - 10.3, xe, b + 2.3, MC.bg)
        if wd then
            txt_b(3.9, b, MC.green, string.format("%03d", wd), cap)
            txt_b(33.0, b, MC.green, tostring(wsp), cap)
        else
            for _, xx in ipairs({ { 4.4, 8.3 }, { 11.0, 14.6 }, { 17.3, 21.2 }, { 33.0, 36.5 }, { 39.3, 43.2 } }) do
                DL.rectf(xx[1], 15.7, xx[2], 16.9, MC.green)
            end
        end
        DL.line(25.7, 20.1, 30.2, 12.8, MC.white, 0.9)
    end
    -- oben rechts: Stadt, ICAO und IATA (nicht waehrend ARPT MAP LOADING - da ist noch kein Flughafen geladen)
    if S.apt then
        local city, codes = NDUI.apt_texts(S.apt)
        for i, t in ipairs({ city, codes }) do
            local w = text_w(t, cap)
            DL.rectf(297 - w - 1.5, HDR_BASE[i] - 10.3, ND_VW, HDR_BASE[i] + 2.3, MC.bg)
            txt_b(297, HDR_BASE[i], MC.white, t, cap, true)
        end
        -- wie im A350: gezeigter Flughafen weder FROM, TO noch ALTN des Flugplans -> oben mittig in zwei Zeilen
        -- (Lage aus einem Bild des A350-ND; nur mit lesbarem FMS und ohne BTV-Kopfzeilen links)
        local ic = S.apt.icao
        if FMS.st ~= "not available" and #C.hdr == 0 and ic ~= FMS.from and ic ~= FMS.dest and ic ~= FMS.altn then
            local l1, l2 = "ARPT NOT IN", "ACTIVE F-PLN"
            local w = math.max(text_w(l1, cap), text_w(l2, cap))
            DL.rectf(150 - w / 2 - 1.5, 22.4 - 9, 150 + w / 2 + 1.5, 31.4 + 2.3, MC.bg)
            txt_b(150 - text_w(l1, cap) / 2, 22.4, MC.white, l1, cap)
            txt_b(150 - text_w(l2, cap) / 2, 31.4, MC.white, l2, cap)
        end
    end
    -- Piste im Weg (Runway Proximity Advisory): die Piste blinkt auf der Karte, ihr Name pulsiert (1 Hz)
    local blink = math.floor(DR.time[0] * 2) % 2 == 0
    if RWYA.r and C.XY then
        local r = RWYA.r
        local dx, dy = r.x2 - r.x1, r.y2 - r.y1
        local L = math.sqrt(dx * dx + dy * dy)
        if L > 1 and blink then
            local nx, ny = -dy / L * r.w / 2, dx / L * r.w / 2
            local P1 = { C.XY(r.x1 + nx, r.y1 + ny) }
            local P2 = { C.XY(r.x2 + nx, r.y2 + ny) }
            local P3 = { C.XY(r.x2 - nx, r.y2 - ny) }
            local P4 = { C.XY(r.x1 - nx, r.y1 - ny) }
            for _, sgm in ipairs({ { P1, P2 }, { P2, P3 }, { P3, P4 }, { P4, P1 } }) do
                DL.line(sgm[1][1], sgm[1][2], sgm[2][1], sgm[2][2], MC.cross, 2.2)
            end
        end
    end
    if RWYA.text and blink then
        local w = text_w(RWYA.text, cap) + 10
        local yc = (C.mode == "arc") and 72 or 49 -- unter dem Bogen und Steuerstrich (ARC) bzw. ueber dem Kreis
        local r = { 150 - w / 2, yc - 7, 150 + w / 2, yc + 7 }
        DL.rectf(r[1], r[2], r[3], r[4], MC.bg)
        DL.rect(r[1], r[2], r[3], r[4], MC.cross, 0, 0, 1.0)
        nd_key_text(r, MC.cross, RWYA.text, cap)
    end
end

-- Element unter dem Flugzeug (Piste, Position oder Rollweg) unten in der Mitte, gelb umrahmt
function NDUI.on_label(C, V)
    local G = C.G
    local acx, acy = C.acx, C.acy
    local t
    for _, r in ipairs(G.rwys) do
        if r.L then
            local qx, qy = acx - r.x1, acy - r.y1
            local al, cr = qx * r.ux + qy * r.uy, -qx * r.uy + qy * r.ux
            if al > 0 and al < r.L and math.abs(cr) < r.w / 2 + 5 then t = r.id1 .. "-" .. r.id2; break end
        end
    end
    if not t then
        local bd = 30 * 30
        for _, l in ipairs(G.stand_lbls) do
            local d2 = (l.x - acx) ^ 2 + (l.y - acy) ^ 2
            if d2 < bd then bd, t = d2, l.text end
        end
    end
    if not t then
        local bd = 22
        for _, e in ipairs(G.edges) do
            if not e.rwy and e.name ~= "" and e.name ~= "--" then
                local vx, vy = e.x2 - e.x1, e.y2 - e.y1
                local L2 = vx * vx + vy * vy
                local f = L2 > 0 and clamp(((acx - e.x1) * vx + (acy - e.y1) * vy) / L2, 0, 1) or 0
                local dd = math.sqrt((e.x1 + vx * f - acx) ^ 2 + (e.y1 + vy * f - acy) ^ 2)
                if dd < bd then bd, t = dd, e.name end
            end
        end
    end
    if not t then return end
    if #t > 26 then t = t:sub(1, 26) end
    -- wie im A350: im schwarzen Balken unter der Zunge (Bedienfeld zu) bzw. direkt auf den Reitern (offen)
    local cap = TXT.small
    local w = text_w(t, cap) + 6
    local _, yl = NDUI.lay(V.ui.panel)
    local r = { 150 - w / 2, yl, 150 + w / 2, yl + NDUI.NOTCH.lbl_h }
    DL.rectf(r[1], r[2], r[3], r[4], MC.bg)
    DL.rect(r[1], r[2], r[3], r[4], MC.label, 0, 0, 0.9)
    nd_key_text(r, MC.label, t, cap)
end

-- Bedienelemente des Bedienfelds
function NDUI.button(C, r, t, en, fn, col)
    local h = en and hov(C, r)
    DL.rectf(r[1], r[2], r[3], r[4], MC.key_bg)
    DL.rect(r[1], r[2], r[3], r[4], h and MC.sel or MC.key_frame, 0, 0, 0.9)
    local cap = TXT.cap
    if text_w(t, cap) > r[3] - r[1] - 4 then cap = TXT.small end
    nd_key_text(r, en and (col or MC.white) or MC.dis, t, cap)
    hit_add(C, r, en and fn or function() end)
end
function NDUI.radio(C, x, yc, t, on, fn)
    local r = { x, yc - 5, x + 78, yc + 5 }
    DL.circle(x + 5, yc, 3.6, hov(C, r) and MC.sel or MC.white, 16, 0.9)
    if on then DL.circlef(x + 5, yc, 2.2, MC.sel, 12) end
    DL.text(x + 12, yc - FONT_H / 2, MC.white, t)
    hit_add(C, r, fn)
end
-- Auswahlfeld: Text (cyan) und Pfeil; fn_text = Feld antippen, fn_arrow = Pfeil antippen
function NDUI.ddfield(C, r, t, focused, fn_text, fn_arrow)
    DL.rectf(r[1], r[2], r[3], r[4], MC.bg)
    DL.rect(r[1], r[2], r[3], r[4], focused and MC.sel or MC.key_frame, 0, 0, 0.9)
    local ar = { r[3] - 12, r[2], r[3], r[4] }
    DL.rectf(ar[1], ar[2], ar[3], ar[4], MC.key_bg)
    DL.rect(ar[1], ar[2], ar[3], ar[4], hov(C, ar) and MC.sel or MC.key_frame, 0, 0, 0.9)
    DL.tri(ar[1] + 2.5, r[2] + 4.5, ar[3] - 2.5, r[2] + 4.5, (ar[1] + ar[3]) / 2, r[4] - 4, MC.white)
    DL.text(r[1] + 3, (r[2] + r[4]) / 2 - FONT_H / 2, MC.sel, t)
    hit_add(C, { r[1], r[2], ar[1] - 0.5, r[4] }, fn_text)
    hit_add(C, ar, fn_arrow)
end

-- Auswahlliste (oberhalb des Felds, weil das Bedienfeld unten liegt)
function NDUI.dd_open(V, kind, r, items)
    V.ui.dd = { kind = kind, field = r, items = items, scroll = 0 }
end
function NDUI.dd_draw(C, V)
    local dd = V.ui.dd
    if not dd then return end
    local items = dd.items
    local lh = 11
    local rows = math.min(#items, 9)
    local w = 0
    for _, it in ipairs(items) do w = math.max(w, text_w(it.t) + (it.t2 and (text_w(it.t2) + 10) or 0)) end
    w = math.max(w + 10, dd.field[3] - dd.field[1])
    local x1 = dd.field[1]
    local y1 = dd.field[2] - 1
    local y0 = y1 - math.max(rows, 1) * lh - 4
    local r = { x1, y0, x1 + w, y1 }
    dd.rect = r
    dd.scroll = clamp(dd.scroll, 0, math.max(0, #items - rows))
    DL.rectf(r[1], r[2], r[3], r[4], MC.menu_bg)
    DL.rect(r[1], r[2], r[3], r[4], MC.key_frame, 0, 0, 0.9)
    hit_add(C, r, function() end, { keep = true })
    if #items == 0 then
        DL.text(x1 + 4, y0 + 2, MC.dis, "NO MATCH")
        return
    end
    for k = 1, rows do
        local it = items[k + dd.scroll]
        if it then
            local ry = y0 + 2 + (k - 1) * lh
            local rr = { x1 + 1, ry, x1 + w - 1, ry + lh }
            if hov(C, rr) then DL.rectf(rr[1], rr[2], rr[3], rr[4], MC.menu_hi) end
            DL.text(x1 + 4, ry + (lh - FONT_H) / 2, it.c or MC.sel, it.t)
            if it.t2 then DL.text(x1 + w - 4 - text_w(it.t2), ry + (lh - FONT_H) / 2, MC.grey, it.t2) end
            hit_add(C, rr, function() V.ui.dd = nil; it.fn() end, { keep = true })
        end
    end
    -- Rollbalken
    if #items > rows then
        local f0, f1 = dd.scroll / #items, (dd.scroll + rows) / #items
        local ty0, ty1 = y0 + 2, y1 - 2
        DL.rectf(r[3] - 2.5, ty0 + (ty1 - ty0) * f0, r[3] - 1, ty0 + (ty1 - ty0) * f1, MC.key_frame)
    end
end

-- Code eines Flughafens je Auswahl (ICAO / IATA / Stadt)
function NDUI.code_of(e, amode)
    if not e then return "" end
    if amode == "ICAO" then return e.icao end
    local m = META.get(e)
    if amode == "IATA" then return m.iata or "---" end
    return (m.city or m.name or e.icao):sub(1, 9)
end

-- Flughaefen fuer die Auswahlliste: FMS-Flughaefen, dann die naechsten; mit Eingabe gefiltert
-- (ICAO: alle Flughaefen; IATA/Stadt: im Umkreis von 300 km)
function NDUI.arpt_cands(ui, filt)
    local key = ui.amode .. "|" .. filt
    if ui.cands and ui.cands_key == key and #jobs == 0 and ui.cands_apt == (S.apt and S.apt.icao) then return ui.cands end
    local lat, lon = DR.lat[0], DR.lon[0]
    local out, seen = {}, {}
    local function add(e, tag)
        if not e or seen[e.icao] then return end
        seen[e.icao] = true
        out[#out + 1] = { e = e, d = dist_m(lat, lon, e.lat, e.lon), tag = tag }
    end
    if not IDX.ready then return out end
    local fms = {}
    for _, f in ipairs({ { FMS.from, "ORIG" }, { FMS.dest, "DEST" }, { FMS.altn, "ALTN" } }) do
        local l = f[1] and IDX.by_icao[f[1]]
        if l then fms[#fms + 1] = { l[1], f[2] } end
    end
    if ui.amode == "ICAO" and filt ~= "" then
        local n = #filt
        for _, e in ipairs(IDX.list) do
            if e.icao:sub(1, n) == filt then add(e) end
        end
    else
        for _, f in ipairs(fms) do add(f[1], f[2]) end
        for _, e in ipairs(IDX.list) do
            if math.abs(e.lat - lat) < 3 and not seen[e.icao] then
                local d = dist_m(lat, lon, e.lat, e.lon)
                if d < 300000 then
                    local len = 0
                    for _, q in ipairs(IDX.by_icao[e.icao]) do len = math.max(len, q.len) end
                    if len >= 900 then out[#out + 1] = { e = e, d = d }; seen[e.icao] = true end
                end
            end
        end
    end
    table.sort(out, function(a, b)
        if (a.tag ~= nil) ~= (b.tag ~= nil) then return a.tag ~= nil end
        return a.d < b.d
    end)
    while #out > 60 do out[#out] = nil end
    if ui.amode ~= "ICAO" then
        local res = {}
        for _, c in ipairs(out) do
            local code = NDUI.code_of(c.e, ui.amode)
            local m = META.get(c.e)
            local full = (ui.amode == "IATA") and (m.iata or "") or (m.city or m.name or "")
            if (ui.amode == "CITY" or m.iata) and (filt == "" or full:sub(1, #filt) == filt) then
                res[#res + 1] = c
                c.code = code
            end
        end
        out = res
    end
    ui.cands, ui.cands_key, ui.cands_apt = out, key, S.apt and S.apt.icao
    return out
end

-- Flughafen laden ("DISPLAY AIRPORT"); automatische Wahl bleibt an, wenn es das FMS-Ziel oder der naechste ist
-- im aktiven Flugplan (FMS FROM / TO / ALTN)?
function NDUI.in_fpln(icao)
    return icao ~= nil and (icao == FMS.from or icao == FMS.dest or icao == FMS.altn)
end
function NDUI.display(e, V)
    if not e then return end
    if S.locked() then message(LOCK_MSG); return end
    if V then V.show_apt = e.icao end -- Karte auf den Flughafen, sobald er geladen ist
    if S.apt and S.apt.icao == e.icao then return end
    if S.mode == "ENGAGED" or S.mode == "RELEASE" then return end
    for _, j in ipairs(jobs) do if j.name == "load " .. e.icao then return end end -- laedt schon
    -- von Hand gewaehlt: bleibt bis zum naechsten Start bzw. Aufsetzen (dann wieder automatisch)
    load_airport(e, true)
    message("OANS " .. e.icao)
end

-- Eingabe mit der Tastatur
function NDUI.key(V, sd, ev)
    local kb = NDUI.kb
    if not kb or kb.side ~= sd.side then return end
    local ch, vk = ev.ch or 0, ev.vk or 0
    if ch == 8 or vk == 0x08 then
        kb.text = kb.text:sub(1, -2)
    elseif ch == 27 or vk == 0x1B then
        NDUI.kb_end()
        V.ui.dd = nil
    elseif ch == 13 or vk == 0x0D or vk == 0x9C then
        NDUI.kb_apply(V, sd)
    elseif ch >= 32 and ch < 127 then
        local c = string.char(ch):upper()
        if kb.field == "stop" then
            if c:match("%d") and #kb.text < 4 then kb.text = kb.text .. c end
        elseif c:match("[%w %-/]") and #kb.text < 14 then
            kb.text = kb.text .. c
        end
    end
end

function NDUI.kb_apply(V, sd)
    local kb = NDUI.kb
    if not kb then return end
    local ui = V.ui
    if kb.field == "arpt" then
        local t = kb.text
        local e
        if ui.amode == "ICAO" and #t == 4 and IDX.by_icao[t] then e = IDX.by_icao[t][1] end
        if not e and t ~= "" then
            local c = NDUI.arpt_cands(ui, t)[1]
            e = c and c.e
        end
        if e then ui.asel = e end
        if t ~= "" and not e then message("ARPT " .. t .. " NOT FOUND") end
        ui.dd = nil
    elseif kb.field == "stop" then
        local n = tonumber(kb.text)
        local E = S.rwy
        if S.locked() then
            message(LOCK_MSG)
        elseif n and E and btv_ui() then
            local lda = E.len - E.disp
            if n >= 300 and n <= lda then
                S.manual_dist, S.exit = math.floor(n), nil
                message(string.format("BTV STOP DISTANCE %d M", S.manual_dist))
            else
                message(string.format("STOP DISTANCE 300..%d M", math.floor(lda)))
            end
        end
    end
    NDUI.kb_end()
end

-- Bedienfeld: Reiter und Inhalt
function NDUI.panel(C, V, sd)
    local ui = V.ui
    local y0 = PANEL_Y
    DL.rectf(0, y0, ND_VW, ND_VH, MC.bg)
    local tabs = { { "MAP DATA", "MAP" }, { "LANDING", "LDG" }, { "ARPT SEL", "ARPT" }, { "STATUS", "STAT" } }
    local tw, th, sl = 66, 15, 6
    local selx
    for i, t in ipairs(tabs) do
        local x = 2 + (i - 1) * (tw + 2)
        local sel = ui.tab == t[2]
        local pts = { { x, y0 + th }, { x + sl, y0 + 1 }, { x + tw - sl, y0 + 1 }, { x + tw, y0 + th } }
        MAPX.poly_fill(pts, sel and MC.bg or MC.tab_bg)
        local r = { x, y0, x + tw, y0 + th }
        local hv2 = hov(C, r)
        DL.line(pts[1][1], pts[1][2], pts[2][1], pts[2][2], sel and MC.sel or (hv2 and MC.white or MC.key_frame), 0.9)
        DL.line(pts[2][1], pts[2][2], pts[3][1], pts[3][2], sel and MC.sel or (hv2 and MC.white or MC.key_frame), 0.9)
        DL.line(pts[3][1], pts[3][2], pts[4][1], pts[4][2], sel and MC.sel or (hv2 and MC.white or MC.key_frame), 0.9)
        nd_key_text(r, sel and MC.sel or MC.white, t[1])
        if sel then selx = { x, x + tw } end
        hit_add(C, r, function() ui.tab = t[2]; ui.dd = nil; NDUI.kb_end() end)
    end
    -- Rahmen des Inhalts (oben offen unter dem gewaehlten Reiter)
    local yt = y0 + th
    if selx then
        DL.line(0.5, yt, selx[1], yt, MC.key_frame, 0.9)
        DL.line(selx[2], yt, 276, yt, MC.key_frame, 0.9)
    else
        DL.line(0.5, yt, 276, yt, MC.key_frame, 0.9)
    end
    DL.line(0.6, yt, 0.6, ND_VH - 0.6, MC.key_frame, 0.9)
    DL.line(0.6, ND_VH - 0.6, ND_VW - 0.6, ND_VH - 0.6, MC.key_frame, 0.9)
    DL.line(ND_VW - 0.6, yt, ND_VW - 0.6, ND_VH - 0.6, MC.key_frame, 0.9)
    -- Doppelpfeil (zuklappen)
    local tr = { 278, y0, 297, y0 + th }
    NDUI.toggle_key(C, tr, false, function() ui.panel = false; ui.dd = nil; NDUI.kb_end() end)
    hit_add(C, { 0, yt, ND_VW, ND_VH }, function() ui.dd = nil end) -- Hintergrund: schliesst Listen
    if ui.tab == "ARPT" then NDUI.tab_arpt(C, V, sd)
    elseif ui.tab == "MAP" then NDUI.tab_map(C, V, sd)
    elseif ui.tab == "LDG" then NDUI.tab_ldg(C, V, sd)
    else NDUI.tab_status(C, V, sd) end
end

-- Taste mit Doppelpfeil (auf = Bedienfeld oeffnen, ab = schliessen)
function NDUI.toggle_key(C, r, up, fn)
    local h = hov(C, r)
    DL.rectf(r[1], r[2], r[3], r[4], MC.key_bg)
    DL.rect(r[1], r[2], r[3], r[4], h and MC.sel or MC.key_frame, 0, 0, 0.9)
    local cx, cy = (r[1] + r[3]) / 2, (r[2] + r[4]) / 2
    local s = up and -1 or 1
    for k = 0, 1 do
        local yy = cy - s * 3.3 + s * k * 4.2
        DL.tri(cx - 5, yy - s * 1.9, cx + 5, yy - s * 1.9, cx, yy + s * 2.6, MC.white)
    end
    hit_add(C, r, fn)
end

-- Reiter ARPT SEL
function NDUI.tab_arpt(C, V, sd)
    local ui = V.ui
    local y0 = PANEL_Y + 15
    if not ui.asel and S.apt and IDX.by_icao[S.apt.icao] then ui.asel = IDX.by_icao[S.apt.icao][1] end
    local sel = ui.asel
    local fr = { 4, y0 + 3, 84, y0 + 15 }
    local typing = NDUI.kb_on(sd.side, "arpt")
    local t = typing and (NDUI.kb.text .. ((math.floor(NDUI.blink * 2) % 2 == 0) and "_" or " ")) or NDUI.code_of(sel, ui.amode)
    local function open_list(filt)
        local items = {}
        for _, c in ipairs(NDUI.arpt_cands(ui, filt or "")) do
            local e = c.e
            local code = (ui.amode == "ICAO") and e.icao or (c.code or NDUI.code_of(e, ui.amode))
            items[#items + 1] = { t = code, t2 = c.tag or string.format("%dNM", math.floor(c.d / 1852 + 0.5)),
                                  c = NDUI.in_fpln(e.icao) and MC.green or MC.sel,
                                  fn = function() ui.asel = e; NDUI.kb_end() end }
        end
        NDUI.dd_open(V, "arpt", fr, items)
    end
    local lk = S.locked()
    NDUI.ddfield(C, fr, t, typing,
        function() if lk then message(LOCK_MSG) else NDUI.kb_begin(sd.side, "arpt", ""); open_list("") end end,
        function()
            if lk then message(LOCK_MSG)
            elseif ui.dd and ui.dd.kind == "arpt" then ui.dd = nil else open_list(typing and NDUI.kb.text or "") end
        end)
    if typing and ui.dd and ui.dd.kind == "arpt" and ui.dd.filt ~= NDUI.kb.text then
        open_list(NDUI.kb.text)
        ui.dd.filt = NDUI.kb.text
    end
    for i, m in ipairs({ { "ICAO", "ICAO" }, { "IATA", "IATA" }, { "CITY NAME", "CITY" } }) do
        NDUI.radio(C, 4, y0 + 25 + (i - 1) * 10, m[1], ui.amode == m[2], function()
            if ui.amode ~= m[2] then ui.amode, ui.cands, ui.dd = m[2], nil, nil end
        end)
    end
    -- Angaben zum gewaehlten Flughafen: gruen, wenn er im Flugplan steht (FROM / TO / ALTN), sonst gelb
    if sel then
        local infp = NDUI.in_fpln(sel.icao)
        local m = META.get(sel)
        local l1 = (m.city or m.name or ""):sub(1, 22)
        local l2 = sel.icao .. (m.iata and ("  " .. m.iata) or "")
        for i, l in ipairs({ l1, l2, META.latlon(sel.lat, sel.lon) }) do
            local w = text_w(l)
            txt_b(151 - w / 2, y0 + 10.5 + (i - 1) * 9.5, infp and MC.green or MC.label, l)
        end
        local far = dist_m(DR.lat[0], DR.lon[0], sel.lat, sel.lon) > 15 * 1852
        if lk then
            txt_b(151 - text_w(LOCK_MSG) / 2, y0 + 41, MC.amber, LOCK_MSG)
        elseif far and EFIS.mode[sd.side] ~= "plan" then
            local s = "SET PLAN MODE"
            txt_b(151 - text_w(s) / 2, y0 + 41, MC.white, s)
        end
    end
    local can = sel ~= nil and IDX.ready and not lk
    NDUI.button(C, { 90, y0 + 49.5, 212, y0 + 61.5 }, "DISPLAY AIRPORT", can, function() NDUI.display(sel, V) end)
    DL.line(218, y0 + 3, 218, ND_VH - 3, MC.key_frame, 0.8)
    -- FMS AIRPORTS (Start, Ziel, Ausweich): gruen, der gewaehlte gelb
    txt_b(260 - text_w("FMS AIRPORTS", TXT.small) / 2, y0 + 8.5, MC.white, "FMS AIRPORTS", TXT.small)
    for i, f in ipairs({ { FMS.from, "ORIGIN" }, { FMS.dest, "DEST" }, { FMS.altn, "ALTN" } }) do
        local r = { 224, y0 + 10.5 + (i - 1) * 12.5, 296, y0 + 21.5 + (i - 1) * 12.5 }
        local l = f[1] and IDX.by_icao[f[1]]
        local e = l and l[1]
        NDUI.button(C, r, e and e.icao or f[2], e ~= nil and not lk, function() ui.asel = e; ui.dd = nil; NDUI.kb_end() end,
            (e and sel and sel.icao == e.icao) and MC.label or MC.green)
    end
end

-- Elemente der Karte fuer MAP DATA (Pisten, Rollwege, Positionen)
function NDUI.md_items(kind)
    local apt = S.apt
    local G = apt and apt.geo
    if not G then return {} end
    G.md = G.md or {}
    if G.md[kind] then return G.md[kind] end
    local out = {}
    if kind == "RWY" then
        for _, E in ipairs(apt.ends) do
            out[#out + 1] = { t = E.id, x = E.ox + E.ux * E.disp, y = E.oy + E.uy * E.disp, E = E }
        end
    elseif kind == "TWY" then
        local seen = {}
        for _, e in ipairs(G.lbl) do
            if not seen[e.name] then
                seen[e.name] = true
                out[#out + 1] = { t = e.name, x = (e.x1 + e.x2) / 2, y = (e.y1 + e.y2) / 2 }
            end
        end
        table.sort(out, function(a, b) return a.t < b.t end)
    elseif kind == "STAND" then
        for _, l in ipairs(G.stand_lbls) do out[#out + 1] = { t = l.text, x = l.x, y = l.y } end
        table.sort(out, function(a, b) return a.t < b.t end)
    end
    G.md[kind] = out
    return out
end

-- Reiter MAP DATA
function NDUI.tab_map(C, V, sd)
    local ui = V.ui
    local y0 = PANEL_Y + 15
    local items = NDUI.md_items(ui.kind)
    local sel = ui.msel
    if sel and sel.kind ~= ui.kind then sel = nil end
    local fr = { 4, y0 + 3, 84, y0 + 15 }
    local function open_list()
        local li = {}
        for _, it in ipairs(items) do
            li[#li + 1] = { t = it.t, fn = function() ui.msel = { kind = ui.kind, t = it.t, x = it.x, y = it.y, E = it.E } end }
        end
        NDUI.dd_open(V, "map", fr, li)
    end
    local lk = S.locked()
    local function toggle()
        if lk then message(LOCK_MSG) elseif ui.dd and ui.dd.kind == "map" then ui.dd = nil else open_list() end
    end
    NDUI.ddfield(C, fr, sel and sel.t:sub(1, 10) or "", false, toggle, toggle)
    for i, k in ipairs({ "RWY", "TWY", "STAND", "OTHER" }) do
        NDUI.radio(C, 4, y0 + 24 + (i - 1) * 10, k, ui.kind == k, function()
            if ui.kind ~= k then ui.kind, ui.dd = k, nil end
        end)
    end
    -- Kreuz / Fahne am gewaehlten Element (zweites Antippen: loeschen)
    local mk = NDUI.marks()
    local function find(list)
        if not sel then return nil end
        for i, c in ipairs(list) do if (c[1] - sel.x) ^ 2 + (c[2] - sel.y) ^ 2 < 15 * 15 then return i end end
    end
    local ci, fi = find(mk.crosses), find(mk.flags)
    NDUI.button(C, { 90, y0 + 3, 152, y0 + 15 }, ci and "DEL CROSS" or "ADD CROSS", sel ~= nil and not lk, function()
        if ci then table.remove(mk.crosses, ci) else mk.crosses[#mk.crosses + 1] = { sel.x, sel.y } end
    end)
    NDUI.button(C, { 156, y0 + 3, 214, y0 + 15 }, fi and "DEL FLAG" or "ADD FLAG", sel ~= nil and not lk, function()
        if fi then table.remove(mk.flags, fi) else mk.flags[#mk.flags + 1] = { sel.x, sel.y } end
    end)
    NDUI.button(C, { 218, y0 + 3, 296, y0 + 15 }, "LDG SHIFT", false)
    local ct = "CENTER MAP ON" .. (sel and (" " .. sel.t:sub(1, 12)) or "")
    NDUI.button(C, { 90, y0 + 19, 296, y0 + 31 }, ct, sel ~= nil and not lk, function()
        V.free = V.free or {}
        V.free.plan = { sel.x, sel.y }
        if EFIS.vmode(sd.side) ~= "plan" then message("SET PLAN MODE") end
    end)
    -- Angaben zum Element (bzw. Hinweis, solange BTV scharf ist)
    if lk then
        txt_b(90, y0 + 43, MC.amber, LOCK_MSG)
    elseif sel then
        local segs
        if sel.E then
            local E = sel.E
            segs = { { "RWY ", MC.white, 90 }, { E.id, MC.green }, { "  TORA ", MC.white }, { tostring(math.floor(E.len + 0.5)), MC.green },
                     { "M", MC.sel }, { "  LDA ", MC.white }, { tostring(math.floor(E.len - E.disp + 0.5)), MC.green }, { "M", MC.sel } }
        else
            segs = { { ui.kind .. " ", MC.white, 90 }, { sel.t:sub(1, 30), MC.green } }
        end
        local x = 90
        for _, sg in ipairs(segs) do x = txt_b(sg[3] or x, y0 + 43, sg[2], sg[1]) end
    end
end

-- Reiter LANDING (Aufteilung wie im A350): MODIFY RWY und LDG PERF CODE oben, darunter Bahn mit Laenge und die
-- ROW-Landestrecke, unten BTV EXIT AT ... M FROM THRESHOLD (Stoppstrecke ohne Exit; scharf wird BTV mit der A/BRK-Taste)
function NDUI.tab_ldg(C, V, sd)
    local ui = V.ui
    local y0 = PANEL_Y + 15
    local E = S.rwy
    local lk = S.locked()
    local fly = btv_ui()
    -- MODIFY RWY: andere Landebahn des Flughafens waehlen
    local mr = { 6, y0 + 3, 78, y0 + 15 }
    local function open_rwy()
        if lk then message(LOCK_MSG); return end
        if ui.dd and ui.dd.kind == "rwy" then ui.dd = nil; return end
        local items = {}
        for _, q in ipairs(S.apt and S.apt.ends or {}) do
            items[#items + 1] = { t = q.id, t2 = string.format("%dM", math.floor(q.len - q.disp + 0.5)),
                                  c = (q == E) and MC.green or MC.sel,
                                  fn = function() select_runway(q, true, "MAN"); message("BTV RWY " .. q.id) end }
        end
        NDUI.dd_open(V, "rwy", { 6, y0 + 3, 120, y0 + 15 }, items)
    end
    NDUI.button(C, mr, "MODIFY RWY", fly and S.apt ~= nil and not lk, open_rwy)
    -- LDG PERF CODE (Pistenzustand: AUTO = aus dem Wetter, sonst 6 DRY ... 1 POOR)
    txt_b(112, y0 + 12.5, MC.white, "LDG PERF CODE")
    local fr = { 196, y0 + 3, 296, y0 + 15 }
    local function code(k) return P.RWYC[k][1] .. "-" .. k end
    local ct = S.cond_sel and code(S.cond_sel) or ("AUTO " .. (S.cond_auto or "DRY"))
    local function open_cond()
        if lk then message(LOCK_MSG); return end
        if ui.dd and ui.dd.kind == "cond" then ui.dd = nil; return end
        local items = { { t = "AUTO", t2 = "WX", fn = function() S.cond_sel = nil end } }
        for k = 6, 1, -1 do
            items[#items + 1] = { t = code(k), c = (S.cond_sel == k) and MC.green or MC.sel, fn = function() S.cond_sel = k end }
        end
        NDUI.dd_open(V, "cond", fr, items)
    end
    NDUI.ddfield(C, fr, ct, ui.dd and ui.dd.kind == "cond", open_cond, open_cond)
    local yb = y0 + 31
    if not fly then
        txt_b(6, yb, MC.dis, "BTV: SELECTION IN FLIGHT ONLY")
        return
    elseif not E then
        txt_b(6, yb, MC.amber, "SELECT LANDING RUNWAY")
        return
    end
    -- RWY 03   3800 M                ROW LDG DIST: 1573 M
    local x = txt_b(6, yb, MC.white, "RWY ")
    txt_b(x, yb, MC.green, E.id)
    x = txt_b(58, yb, MC.green, tostring(math.floor(E.len - E.disp + 0.5)))
    txt_b(x + 3, yb, MC.sel, "M")
    local okr, rl = pcall(S.row_lines, E)
    local ld = okr and rl and rl[1] and (rl[1][2] - E.disp)
    local lt = ld and tostring(math.floor(ld + 0.5)) or "----"
    local xr = 296 - text_w("M")
    txt_b(xr, yb, MC.sel, "M")
    xr = xr - 3 - text_w(lt)
    txt_b(xr, yb, MC.green, lt)
    txt_b(xr - 4 - text_w("ROW LDG DIST:"), yb, MC.white, "ROW LDG DIST:")
    -- BTV EXIT AT [____] M FROM THRESHOLD (Exit auf der Karte gewaehlt: dessen Strecke)
    local ye = y0 + 49
    x = txt_b(6, ye, MC.white, "BTV EXIT AT")
    local r = { x + 5, ye - 9, x + 51, ye + 4 }
    local typing = NDUI.kb_on(sd.side, "stop")
    local val = typing and (NDUI.kb.text .. ((math.floor(NDUI.blink * 2) % 2 == 0) and "_" or " "))
        or (S.manual_dist and tostring(S.manual_dist))
        or (S.exit and tostring(math.floor(S.exit.dist + 0.5))) or "----"
    DL.rectf(r[1], r[2], r[3], r[4], MC.bg)
    DL.rect(r[1], r[2], r[3], r[4], (typing or hov(C, r)) and MC.sel or MC.key_frame, 0, 0, 0.9)
    txt_b(r[3] - 3, ye, MC.sel, val, nil, true)
    hit_add(C, r, function() if lk then message(LOCK_MSG) else NDUI.kb_begin(sd.side, "stop", "") end end)
    x = txt_b(r[3] + 3, ye, MC.sel, "M")
    txt_b(x + 4, ye, MC.white, "FROM THRESHOLD")
    if S.abrk and not S.arm_request then txt_b(6, y0 + 61, MC.amber, "AUTO BRK SET - BTV NOT AVAIL") end
end

-- Reiter STATUS
function NDUI.tab_status(C, V, sd)
    local y0 = PANEL_Y + 15
    local apt = S.apt
    local G = apt and apt.geo
    local function src_name(p)
        if not p then return "-" end
        local n = p:match("([^/\\]+)[/\\]Earth nav data") or p
        if n == "Global Airports" then n = "X-PLANE GLOBAL AIRPORTS" end
        return n:upper():sub(1, 30)
    end
    local lines = {
        { "DATABASE", IDX.ready and string.format("APT.DAT  %d ARPTS", #IDX.list) or ("LOADING " .. (IDX.progress or "")) },
        { "MAP DATA", apt and src_name(apt.src) or "NO AIRPORT" },
        { "TAXI NET", (apt and #apt.nets > 0) and (string.format("%d NODES ", apt.n_nodes) .. ((S.exit_src and S.exit_src ~= apt.src) and "(DEFAULT)" or "")) or "NONE" },
        { "BUILDINGS", (G and G.osm) and string.format("OSM %d TERM %d BLDG", #G.osm.terms, #G.osm.bldg) or (S.osm_status or (CFG.osm and "NONE" or "OFF")):upper():sub(1, 30) },
        { "BTV", (AC_READY and (BRAKE_MODE == "toliss" and "TOLISS BRAKES" or "X-PLANE PEDALS") or "NOT CONNECTED") .. "  RWY " .. (S.cond_txt or "DRY") },
        { "EFIS", string.format("%s%s  %sNM  FMS %s", (EFIS.mode[sd.side] or "?"):upper(),
            EFIS.force[sd.side] and " (TAXI)" or (C.btv and " (BTV)" or ""),
            tostring(RANGES[V.rng[EFIS.vmode(sd.side)] or 2] or "-"), FMS.st == "connected" and "OK" or (FMS.dest and "OK" or "-")) },
    }
    -- Grundlagen der BTV-Rechnung: Aufsetzpunkt (Markierungen), VAPP (FMS oder Gewicht), Dichte, Gegenwind
    local E = S.rwy
    lines[#lines + 1] = { "LDG CALC", E and string.format("TD %dM VAPP %d%s TAS%+d%% HW %+d", math.floor(P.tdd(E) + 0.5),
        math.floor((S.vapp_used or 0) + 0.5), (S.vapp_src == "WEIGHT") and "(WT)" or "", math.floor(((S.tas_k or 1) - 1) * 100 + 0.5),
        math.floor((S.hw or 0) + 0.5)) or "NO RUNWAY" }
    for i, l in ipairs(lines) do
        local b = y0 + 9 + (i - 1) * 8.1
        txt_b(6, b, MC.white, l[1], TXT.small)
        txt_b(64, b, MC.green, l[2], TXT.small)
    end
    txt_b(6, y0 + 9 + #lines * 8.1, MC.grey, "A350 BTV V" .. VERSION .. "  (C) OPENSTREETMAP CONTRIBUTORS", TXT.small)
end

-- Kontextmenue (Doppelklick auf die Karte)
function NDUI.ctx_open(C, V, MV, mx, my, head)
    local wx, wy = C.INV(mx, my)
    local mk = NDUI.marks()
    local items = {}
    for _, it in ipairs(head or {}) do items[#items + 1] = it end
    -- (auf einem Kreuz / einer Fahne: einzeln loeschen, wie im A350)
    for _, l in ipairs({ { mk.crosses, "DELETE CROSS" }, { mk.flags, "DELETE FLAG" } }) do
        for i, c in ipairs(l[1]) do
            local px, py = C.XY(c[1], c[2])
            if (px - mx) ^ 2 + (py - my) ^ 2 < 10 * 10 then
                items[#items + 1] = { l[2], true, function() table.remove(l[1], i) end }
                break
            end
        end
    end
    for _, it in ipairs({
        { "ADD CROSS", true, function() mk.crosses[#mk.crosses + 1] = { wx, wy } end },
        { "ADD FLAG", true, function() mk.flags[#mk.flags + 1] = { wx, wy } end },
        { "MAP DATA", true, function() V.ui.panel, V.ui.tab = true, "MAP" end },
        { "ERASE ALL CROSSES", #mk.crosses > 0, function() mk.crosses = {} end },
        { "ERASE ALL FLAGS", #mk.flags > 0, function() mk.flags = {} end },
        { "CENTER ON ACFT", true, function() NDUI.center_acft(V) end },
    }) do items[#items + 1] = it end
    local w = text_w("ERASE ALL CROSSES") + 12
    for _, it in ipairs(items) do w = math.max(w, text_w(it[1]) + 12) end
    local h = #items * 12.5 + 4
    MV.ctx = { x = clamp(mx, 2, ND_VW - w - 2), y = clamp(my, 2, ND_VH - h - 2), w = w, h = h, items = items }
end
function NDUI.ctx_draw(C, MV)
    local m = MV.ctx
    if not m then return end
    local r = { m.x, m.y, m.x + m.w, m.y + m.h }
    DL.rectf(r[1], r[2], r[3], r[4], MC.menu_bg)
    DL.rect(r[1], r[2], r[3], r[4], MC.key_frame, 0, 0, 0.9)
    hit_add(C, r, function() end, { keep = true })
    for k, it in ipairs(m.items) do
        local ry = m.y + 2 + (k - 1) * 12.5
        local rr = { m.x + 1, ry, m.x + m.w - 1, ry + 12.5 }
        if it[2] and hov(C, rr) then DL.rectf(rr[1], rr[2], rr[3], rr[4], MC.menu_hi) end
        DL.text(m.x + 5, ry + (12.5 - FONT_H) / 2, it[2] and MC.white or MC.dis, it[1])
        if it.sep then DL.line(m.x + 3, ry + 12.5, m.x + m.w - 3, ry + 12.5, MC.key_frame, 0.8) end
        if it[2] then hit_add(C, rr, function() MV.ctx = nil; it[3]() end, { keep = true }) end
    end
end

-- Mauszeiger auf dem ND wie der KCCU-Cursor des A350 auf der OANS-Karte (aus einem Bild vermessen): magenta,
-- oben ein V und unten ein umgekehrtes V (Arme 45 Grad), dazwischen frei, Punkt in der Mitte. ND-Einheiten:
-- Arme bis 16,4 seitlich und 21,7 nach oben/unten, Spitzen 5,3 ueber/unter der Mitte, Linien wie die Kastenlinien
function NDUI.cursor(x, y)
    local w, h, a, th = 16.4, 21.7, 5.3, 1.0
    DL.line(x - w, y - h, x, y - a, MC.cursor, th)
    DL.line(x + w, y - h, x, y - a, MC.cursor, th)
    DL.line(x - w, y + h, x, y + a, MC.cursor, th)
    DL.line(x + w, y + h, x, y + a, MC.cursor, th)
    DL.rectf(x - 0.7, y - 0.7, x + 0.7, y + 0.7, MC.cursor)
end


function NDUI.bottom(C, V, sd)
    local ui = V.ui
    NDUI.toggle_key(C, { 267.4, 357.5, 288.3, 374.7 }, true, function() ui.panel = true end)
end

-- Kartenausschnitt wie im A350 (aus Bildern des echten ND vermessen, ARC und NAV gleich): oben ein schwarzes
-- Band fuer die Kopfzeilen, seitlich schmale Raender, unten schwarze Balken mit Kerben - links (Chrono) tiefer
-- als rechts, je 45 Grad abgeschraegt, dazwischen eine Zunge so breit wie die Meldungskaesten des FF-ND.
-- Alles haengt am Rollweg-Kaestchen (Unterkante NDUI.NOTCH.B): Bedienfeld zu ganz unten, offen sitzt es direkt
-- auf den Reitern - der ganze Unterteil rueckt hoch. Darueber zwei Meldungszeilen in FF-Groesse (unten
-- GPS PRIMARY, darueber TA ONLY), darueber mit festem Abstand das Flugzeug (ARC)
-- (Meldungskaesten und Chrono aus einem Bild des FF-ND ARC 10 NM mit VD vermessen: TA ONLY 69,8..229,6 x 265,7..278,6,
-- Chrono 3,65..48,65 x 243,6..256,1 - offen genau dort. Meldungen wie im FF-ND: TA ONLY oben, GPS PRIMARY darunter.
-- Feste Plaetze (A350-Bild: TA ONLY allein eine Zeile ueber dem Rollweg-Kaestchen), das Flugzeug buendig ueber
-- zweien. Bedienfeld offen: dieselben Kerben, Kaesten und Chrono (auf der linken Schulter), nur nach oben geschoben)
NDUI.NOTCH = { top = 24.5, side = 7.5, B = 360.25, l_y = 312.4, l_x = 48.7, t_x1 = 69.8, t_x2 = 229.6, t_y = 349.5,
               r_y = 287.2, r_x = 251.8, lbl_h = 9.3, msg_h = 12.9, acft_gap = 21.6,
               ch_x = 3.65, ch_w = 45, ch_h = 12.5 }
-- Verschiebung des Unterteils und Oberkanten von Rollweg-Kaestchen, unterer und oberer Meldungszeile
function NDUI.lay(pnl)
    local N = NDUI.NOTCH
    local B = pnl and PANEL_Y or N.B
    local yl = B - N.lbl_h
    return B - N.B, yl, yl - N.msg_h, yl - 2 * N.msg_h
end
function NDUI.notches(pnl)
    local N, bg = NDUI.NOTCH, MC.bg
    -- (Bedienfeld offen: dieselben Kerben ueber dem Bedienfeld, um dy nach oben geschoben)
    local dy = NDUI.lay(pnl)
    local xl, xr = N.side, ND_VW - N.side
    local ly, ty, ry = N.l_y + dy, N.t_y + dy, N.r_y + dy
    DL.rectf(0, 0, ND_VW, N.top, bg)
    DL.rectf(0, N.top, xl, ND_VH, bg)
    DL.rectf(xr, N.top, ND_VW, ND_VH, bg)
    DL.rectf(xl, ly, N.l_x, ND_VH, bg)
    DL.quadf(N.l_x, ly, N.t_x1, ly + (N.t_x1 - N.l_x), N.t_x1, ND_VH, N.l_x, ND_VH, bg)
    DL.rectf(N.t_x1, ty, N.t_x2, ND_VH, bg)
    DL.quadf(N.r_x, ry, N.r_x, ND_VH, N.t_x2, ND_VH, N.t_x2, ry + (N.r_x - N.t_x2), bg)
    DL.rectf(N.r_x, ry, xr, ND_VH, bg)
end

-------------------------------------------------------------------------------
-- Anderer Verkehr im OANS (aus der TCAS-Schnittstelle von X-Plane: KI, xPilot, LiveTraffic ...).
-- NDUI.trf wird im Panel-Teil gefuellt: list = Ziele, show[Seite] = anzeigen?, sig = Aenderungs-Kennung.
-- Symbol in Flugrichtung (wie ATSAW): hohl = anderer Verkehr, gefuellt = nah, amber = TA, rot = RA.
-- Kennung ab 1 NM Reichweite;
-- Flugzeuge in der Luft mit relativer Hoehe (100 ft) und Steig-/Sinkpfeil wie TCAS.
-------------------------------------------------------------------------------
NDUI.trf = { list = {}, show = {}, sig = "" }
local TRF_RED = 0xFF2828FF

-- kleines Richtungssymbol (Einheiten): Spitze vorn, Kerbe hinten
local function trf_arrow(px, py, ang, col, filled)
    local fx, fy = math.sin(ang), -math.cos(ang)
    local rx, ry = math.cos(ang), math.sin(ang)
    local function P(x, y) return px + rx * x + fx * y, py + ry * x + fy * y end
    local ax, ay = P(0, 7)
    local bx, by = P(4.6, -5)
    local cx, cy = P(0, -2.4)
    local dx, dy = P(-4.6, -5)
    if filled then
        DL.tri(ax, ay, bx, by, cx, cy, col)
        DL.tri(ax, ay, cx, cy, dx, dy, col)
    end
    DL.line(ax, ay, bx, by, col, 1.1)
    DL.line(bx, by, cx, cy, col, 1.1)
    DL.line(cx, cy, dx, dy, col, 1.1)
    DL.line(dx, dy, ax, ay, col, 1.1)
end

-- sichtbar? im Flug kein Verkehr am Boden (wie TCAS); am Boden Verkehr am Boden und in der Luft bis 2700 ft
-- darueber/darunter. Rueckgabe: sichtbar, am Boden, relative Hoehe [ft]
function NDUI.trf_vis(t, air, own, field)
    local onground = t.wow or (t.ele - field) < 30
    local rel = (t.ele - own) / 0.3048
    local vis = (air and not onground and math.abs(rel) <= 2700) or (not air and (onground or math.abs(rel) <= 2700))
    return vis, onground, rel
end

function NDUI.traffic(C, sd)
    local T = NDUI.trf
    if not (T.show[sd.side] and #T.list > 0) then return end
    local G, sc, rng = C.G, C.sc, C.rng or 1
    local own = DR.elev and DR.elev[0] or 0
    local air = in_air()
    local field = ((S.apt and S.apt.elev_ft) or 0) * 0.3048
    for _, t in ipairs(T.list) do
        local vis, onground, rel = NDUI.trf_vis(t, air, own, field)
        if vis then
            local px, py = C.XY(G.W(t.lat, t.lon))
            if px > -20 and px < ND_VW + 20 and py > -20 and py < ND_VH + 20 then
                local col = (t.threat >= 3) and TRF_RED or ((t.threat == 2) and MC.amber or MC.white)
                local ang = (t.psi - C.hv) * D2R
                trf_arrow(px, py, ang, col, t.threat >= 1 or onground)
                local ty = py + 6
                if rng <= 1 and t.id ~= "" then
                    DL.text(px + 8, ty - FONT_H / 2, col, t.id, TXT.small)
                    ty = ty + FONT_H
                end
                if not onground then
                    local h = math.floor(math.abs(rel) / 100 + 0.5)
                    local s = string.format("%s%02d", (rel >= 0) and "+" or "-", h)
                    local tx = px - text_w(s, TXT.small) / 2
                    local yy = (rel >= 0) and (py - 11 - FONT_H / 2) or (py + 9 - FONT_H / 2)
                    DL.text(tx, yy, col, s, TXT.small)
                    if math.abs(t.vs or 0) >= 500 then
                        local ax = tx + text_w(s, TXT.small) + 3
                        local ay = yy + FONT_H / 2
                        local d = (t.vs > 0) and -1 or 1
                        DL.line(ax, ay - 3.5 * d, ax, ay + 3.5 * d, col, 1.0)
                        DL.tri(ax, ay + 4.5 * d, ax - 2, ay + 1.5 * d, ax + 2, ay + 1.5 * d, col)
                    end
                end
            end
        end
    end
end

-- Meldungen und Chrono des FF-ND (TCAS-Modus, Chrono je Seite, GPS PRIMARY) - Lage siehe NDUI.NOTCH / NDUI.lay
NDUI.ffx = {}
for k, n in pairs({ tcas = "1-sim/surv/TCASmode", ch0 = "AirbusFBW/ChronoTimeND1", ch1 = "AirbusFBW/ChronoTimeND2",
                    gp0 = "AirbusFBW/GPSPrimMessCapt", gp1 = "AirbusFBW/GPSPrimMessCo" }) do
    if dr_exists(n) then NDUI.ffx[k] = dataref_table(n) end
end
-- FF-Meldung GPS PRIMARY: 1 = GPS PRIMARY (im FF-ND nachgeprueft), 2 = GPS PRIMARY LOST (vermutet;
-- jede Aenderung steht im Log.txt)
NDUI.gp_last = {}
function NDUI.gps_primary(side)
    local r = NDUI.ffx["gp" .. side]
    local v = r and math.floor((tonumber(r[0]) or 0) + 0.5) or 0
    if NDUI.gp_last[side] ~= v then
        NDUI.gp_last[side] = v
        pcall(log, string.format("ND %s: AirbusFBW/GPSPrimMess = %d", (side == 0) and "CAPT" or "F/O", v))
    end
    if v == 1 then return "GPS PRIMARY", MC.ffw end
    if v == 2 then return "GPS PRIMARY LOST", MC.amber end
end
function NDUI.msgs(C, V, sd)
    local X = NDUI.ffx
    local N = NDUI.NOTCH
    local dy, _, y_lo, y_hi = NDUI.lay(V.ui.panel)
    -- Chrono: in der linken Kerbe direkt unter ihrer Oberkante (auch mit offenem Bedienfeld)
    local yc = N.l_y + dy
    -- Meldungskaesten wie im FF-ND: feiner warmweisser Rahmen, Schrift gross (TXT.big), Grundlinie 10,1 unter der
    -- Oberkante, mittig bei 150; Breite = Zunge zwischen den Kerben
    local function box(y, s, col)
        DL.rectf(N.t_x1, y, N.t_x2, y + N.msg_h, MC.bg)
        DL.rect(N.t_x1, y, N.t_x2, y + N.msg_h, MC.ffw, 0, 0, 0.6)
        txt_b(150 - text_w(s, TXT.big) / 2, y + 10.1, col, s, TXT.big)
    end
    -- FF: TCASmode 1 TA/RA, 2 TA ONLY, 3 Standby. TA ONLY wie im A350/FF-ND: gewaehlt, oder TA/RA unter 1000 ft
    -- Radiohoehe (dort sind RAs gesperrt - auch am Boden)
    local tc = X.tcas and math.floor((X.tcas[0] or 0) + 0.5)
    local ta = tc == 2 or (tc == 1 and (DR.ra[0] or 0) < 1000)
    local gm, gcol = NDUI.gps_primary(sd.side or 0)
    -- GPS PRIMARY unten, TA ONLY oben (feste Plaetze, auch mit offenem Bedienfeld)
    if gm then box(y_lo, gm, gcol) end
    if ta then box(y_hi, "TA ONLY", MC.ffw) end
    -- Chrono der Seite (FF: Sekunden, 0 = aus) - gruen auf warmem Grau
    -- (Kasten, Schrift und Farbe aus einem Bild neben dem FF-ND vermessen)
    local ch = X["ch" .. (sd.side or 0)]
    local t = ch and math.floor(tonumber(ch[0]) or 0) or 0
    if t > 0 then
        -- genau das Format des FF-ND: %2i'%02i" bzw. %2iH%02i' (Minuten/Stunden zweistellig mit Leerzeichen)
        local txt
        if t < 3600 then txt = string.format("%2d'%02d\"", math.floor(t / 60), t % 60)
        else txt = string.format("%2dH%02d'", math.floor(t / 3600), math.floor(t / 60) % 60) end
        -- Kasten, Schrift und Lage wie im FF-ND: feste Breite, rechtsbuendig 3 vor dem rechten Rand (wie FF),
        -- Grundlinie 11,35; passt der Text nicht in den Kasten, wird er etwas kleiner (bleibt immer drin)
        local x1, cap = N.ch_x, 10.2
        txt = txt:gsub("^ +", "") -- (fuehrendes Leerzeichen: unsichtbar, die Lage kommt aus der Rechtsbuendigkeit)
        local w = text_w(txt, cap)
        local room = N.ch_w - 3 - 1.5
        if w > room then cap, w = cap * room / w, room end
        DL.rectf(x1, yc, x1 + N.ch_w, yc + N.ch_h, 0xFF727D85)
        txt_b(x1 + N.ch_w - 3 - w, yc + 11.35 - (10.2 - cap) * 0.5, MC.green, txt, cap)
    end
end

-- alles ueber der Karte
function NDUI.overlay(C, V, MV, sd)
    NDUI.notches(V.ui.panel)
    NDUI.compass(C)
    NDUI.traffic(C, sd)
    do
        local px, py = C.XY(C.acx, C.acy)
        if px > -20 and px < ND_VW + 20 and py > -20 and py < ND_VH + 20 then
            draw_plane(px, py, (DR.psi[0] - C.hv) * D2R, MC.acft, 0.8, true)
        end
    end
    NDUI.header(C)
    NDUI.msgs(C, V, sd)
    if V.ui.panel then NDUI.panel(C, V, sd) else NDUI.bottom(C, V, sd) end
    NDUI.on_label(C, V)
    NDUI.dd_draw(C, V)
    -- Exit unter der Maus hervorheben (in der Luft), mit Abstand ab Schwelle
    if C.hovered and not MV.ctx and btv_ui() and not S.locked() then
        local h = NDUI.hit_at(C.hit, C.mx, C.my)
        if h and h.exit and h.exit.x ~= S.exit then
            local x = h.exit.x
            draw_poly(h.exit.pts, MC.white, 1.8)
            for _, pp in ipairs(h.exit.alt or {}) do draw_poly(pp, MC.white, 1.8) end
            local r = h.exit.rect
            DL.rect(r[1], r[2], r[3], r[4], MC.white, 0, 0, 0.9)
            local tip = string.format("%s %dM%s", x.name, math.floor(x.dist + 0.5),
                x.too_short and " TOO SHORT" or ((h.exit.i == S.rec_idx) and " RECOMMENDED" or ""))
            box_text(clamp(C.mx + 10, 2, ND_VW - text_w(tip) - 6), C.my + 10, tip, MC.white, MC.key_frame)
        elseif h and h.qfu and h.qfu ~= S.rwy then
            MAPX.poly_line(h.poly, MC.white, 0.9)
        end
    end
    NDUI.ctx_draw(C, MV)
    if CFG.nd_frame then DL.rect(1, 1, ND_VW - 1, ND_VH - 1, MC.mag, 0, 0, 2) end
    if C.hovered and C.mx then NDUI.cursor(C.mx, C.my) end
end

-- vor der Ansicht: Druck/Mausrad auf einer Taste oder Liste? (Stand des letzten Bildes)
function NDUI.pre_input(V, MV, inp)
    MV.wheel_ui = false
    if (inp.wheel or 0) ~= 0 then
        local dd = V.ui.dd
        if dd and dd.rect and in_rect(dd.rect, inp.mx, inp.my) then
            dd.scroll = dd.scroll - ((inp.wheel > 0) and 1 or -1) * math.max(1, math.floor(math.abs(inp.wheel) + 0.5))
            MV.wheel_ui = true
        elseif NDUI.hit_at(MV.hit_prev, inp.mx, inp.my, true) then
            MV.wheel_ui = true
        end
    end
    if inp.clicked and inp.hovered then
        MV.press, MV.drag = true, 0
        MV.press_ui = NDUI.hit_at(MV.hit_prev, inp.mx, inp.my, true) ~= nil
    end
end

-- Antippen / Doppelklick / Tastatur
function NDUI.input(C, V, MV, inp, sd)
    local mx, my = inp.mx, inp.my
    local ui = V.ui
    local before = { MV.ctx, ui.dd, ui.panel, ui.tab }
    for _, k in ipairs(inp.keys or {}) do NDUI.key(V, sd, k) end
    if (inp.keys and #inp.keys > 0) or inp.released then NDUI.dirty = true end -- naechstes Bild neu aufbauen
    if MV.press and inp.released then
        MV.press = false
        local tap = MV.drag < 6
        MV.press_ui = false
        if tap then
            local h = NDUI.hit_at(C.hit, mx, my)
            if h and not h.keep then MV.ctx = nil end
            if h and h.ui then
                -- Tippen ausserhalb einer Eingabe beendet sie (das Eingabefeld selbst setzt den Fokus neu)
                if NDUI.kb and not h.keep then NDUI.kb_end() end
                h.fn(mx, my)
            elseif h and h.exit then
                MV.ctx, V.ui.dd = nil, nil
                if S.locked() then
                    message(LOCK_MSG)
                elseif btv_ui() then
                    -- wie im A350: Exit antippen -> SET BTV EXIT / UNSET BTV EXIT; Exits vor der DRY-Linie
                    -- (zu kurz) lassen sich nicht waehlen
                    local x = h.exit.x
                    local it
                    if x == S.exit and not S.manual_dist then
                        it = { "UNSET BTV EXIT", true, function() S.exit = nil; message("BTV EXIT " .. x.name .. " UNSET") end }
                    else
                        it = { x.too_short and "SET BTV EXIT (TOO SHORT)" or "SET BTV EXIT", not x.too_short, function()
                            S.exit, S.manual_dist = x, nil
                            message(string.format("BTV EXIT %s  %d M", x.name, math.floor(x.dist + 0.5)))
                        end }
                    end
                    -- darunter wie im A350 DECLUTTER RWY (nicht verfuegbar) und das normale Kartenmenue
                    if C.INV then NDUI.ctx_open(C, V, MV, mx, my, { it, { "DECLUTTER RWY", false, nil, sep = true } }) end
                end
            elseif h and h.qfu then
                MV.ctx, V.ui.dd = nil, nil
                if S.locked() then
                    message(LOCK_MSG)
                elseif not btv_ui() then
                    message("BTV: RWY SELECTION IN FLIGHT")
                elseif S.rwy ~= h.qfu then
                    select_runway(h.qfu, true, "MAN")
                    message("BTV RWY " .. h.qfu.id)
                end
            else
                local had = MV.ctx ~= nil or V.ui.dd ~= nil
                MV.ctx, V.ui.dd = nil, nil
                if NDUI.kb and NDUI.kb.side == sd.side then NDUI.kb_end(); had = true end
                if not had and C.INV then
                    local now = inp.t or 0
                    local lt = MV.last_tap
                    if lt and now - lt[1] < 0.5 and math.abs(mx - lt[2]) < 8 and math.abs(my - lt[3]) < 8 then
                        MV.last_tap = nil
                        NDUI.ctx_open(C, V, MV, mx, my)
                    else
                        MV.last_tap = { now, mx, my }
                    end
                end
            end
        end
    end
    -- Bedienelemente fuer das naechste Bild (Druck/Mausrad dort); hat sich die Anordnung geaendert, gilt sie nicht mehr
    if before[1] ~= MV.ctx or before[2] ~= ui.dd or before[3] ~= ui.panel or before[4] ~= ui.tab then
        MV.hit_prev = nil
    else
        MV.hit_prev = C.hit
    end
end

-- ND ohne Flughafen (Karte an): Hinweis und Bedienfeld ARPT SEL
-- Seite ohne Karte: MAP NOT AVAIL (rot; kein Heading / IRS richten aus), ARPT MAP LOADING oder kein Flughafen.
-- Der Doppelpfeil unten rechts (bzw. das Bedienfeld) bleibt immer bedienbar
function NDUI.noapt(sd, inp, na)
    local V = sd.view
    local MV = sd.mv
    local C = { hit = {}, mx = inp.mx, my = inp.my, hovered = inp.hovered, x0 = 0, y0 = 0, cw = ND_VW, ch = ND_VH, hdr = {} }
    NDUI.pre_input(V, MV, inp)
    DL.rectf(0, 0, ND_VW, ND_VH, MC.bg)
    NDUI.header(C)    -- GS / TAS und Wind bleiben oben links, wie im A350 (auch waehrend ARPT MAP LOADING)
    -- Flughafen wird geladen (Index, Daten, Karte): wie im A350 nur "ARPT MAP LOADING"
    local loading = not IDX.ready or (S.status ~= "" and not S.status:find("^Error")) or btv_osm_waiting(S.apt)
    if na or loading then
        local tl = na or "ARPT MAP LOADING"
        DL.text(ND_VW / 2 - text_w(tl) / 2, ND_VH / 2 - FONT_H / 2, na and MC.red or MC.white, tl)
    else
        local t1 = "OANS"
        local t2 = (S.status ~= "" and S.status) or "NO AIRPORT DISPLAYED"
        DL.text(ND_VW / 2 - text_w(t1) / 2, 120, MC.white, t1)
        DL.text(ND_VW / 2 - text_w(t2) / 2, 138, MC.white, t2)
        if not V.ui.panel_auto then V.ui.panel, V.ui.tab, V.ui.panel_auto = true, "ARPT", "open" end
    end
    if V.ui.panel then NDUI.panel(C, V, sd) else NDUI.bottom(C, V, sd) end
    NDUI.dd_draw(C, V)
    NDUI.ctx_draw(C, MV)
    if C.hovered and C.mx then NDUI.cursor(C.mx, C.my) end
    MV.ctx = nil
    NDUI.input(C, V, MV, inp, sd)
end

end -- (NDUI)

-------------------------------------------------------------------------------
-- Fenster (Makro): Kompassring, Kopfzeilen, Bedienung wie bisher
-------------------------------------------------------------------------------
function MAPX.reserve_win(C)
    local x0, y0, cw, ch = C.x0, C.y0, C.cw, C.ch
    local placed = C.placed
    placed[#placed + 1] = { x0, y0, x0 + 230, y0 + 4 * (FONT_H + 3) + 8 }
    placed[#placed + 1] = { x0 + cw - 190, y0, x0 + cw, y0 + 2 * (FONT_H + 3) + 8 }
    placed[#placed + 1] = { x0, y0 + ch - FONT_H - 10, x0 + 260, y0 + ch }
    if C.mode == "btv" and C.E then
        local lx = C.XYr(0, C.E.width / 2)
        local rx = C.XYr(0, -C.E.width / 2)
        placed[#placed + 1] = { lx - 3, y0 - 5000, rx + 3, y0 + ch + 5000 }
    end
end

function MAPX.overlay_win(C, apt, V, MV, inp)
    local G, XY, INV, sc, E, mode, hv = C.G, C.XY, C.INV, C.sc, C.E, C.mode, C.hv
    local x0, y0, cw, ch = C.x0, C.y0, C.cw, C.ch
    local ax, ay = C.ax, C.ay
    local mx, my, hovered = inp.mx, inp.my, inp.hovered
    local marks = C.marks
    local ts, tname, tx = current_target()
    -- Quellenangabe der Gebaeude
    if G.osm then
        local t = "\194\169 OpenStreetMap"
        DL.text(x0 + cw - text_w(t) - 6, y0 + ch - FONT_H - 4, MC.grey, t)
    elseif OSM.busy and S.osm_status then
        DL.text(x0 + cw - text_w(S.osm_status) - 6, y0 + ch - FONT_H - 4, MC.grey, S.osm_status)
    end
    -- Kompassrose: Gradskala, Zahlen alle 30 Grad, Dreiecke alle 45 Grad, gestrichelter Innenring, Reichweite cyan
    do
        local R = math.min(cw, ch) * 0.42
        DL.circle(ax, ay, R, MC.ring, 96, 1.5)
        for deg = 0, 355, 5 do
            local th = (deg - hv) * D2R
            local sx, sy2 = math.sin(th), -math.cos(th)
            local l = (deg % 10 == 0) and 9 or 5
            DL.line(ax + R * sx, ay + R * sy2, ax + (R - l) * sx, ay + (R - l) * sy2, MC.ring, 1.2)
            if deg % 30 == 0 then
                local s = tostring(math.floor(deg / 10))
                local lx, ly = ax + (R - 20) * sx, ay + (R - 20) * sy2
                DL.text(lx - text_w(s) / 2, ly - FONT_H / 2, MC.white, s)
            end
            if deg % 45 == 0 then
                local px, py = ax + (R + 2) * sx, ay + (R + 2) * sy2
                DL.tri(px, py, px + sx * 8 - sy2 * 4, py + sy2 * 8 + sx * 4, px + sx * 8 + sy2 * 4, py + sy2 * 8 - sx * 4, MC.ring)
            end
        end
        if mode == "taxi" then DL.line(ax, ay - R - 12, ax, ay - R + 5, MC.taxiline, 3) end
        local n = 48
        for k = 0, n - 1, 2 do
            local a1, a2 = k / n * 2 * math.pi, (k + 1) / n * 2 * math.pi
            DL.line(ax + R / 2 * math.sin(a1), ay - R / 2 * math.cos(a1), ax + R / 2 * math.sin(a2), ay - R / 2 * math.cos(a2), MC.ring, 1.5)
        end
        local rng = R / sc / 1852
        local rs = rng >= 1 and string.format("%.1f", rng) or string.format("%.2f", rng)
        local th = 225 * D2R
        local tx0, ty0 = ax + R * 0.66 * math.sin(th), ay - R * 0.66 * math.cos(th)
        DL.text(tx0 - text_w(rs) / 2, ty0 - FONT_H / 2, MC.sel, rs)
    end
    -- eigenes Flugzeug
    do
        local px, py = XY(C.acx, C.acy)
        if px > x0 - 20 and px < x0 + cw + 20 and py > y0 - 20 and py < y0 + ch + 20 then
            draw_plane(px, py, (DR.psi[0] - hv) * D2R, MC.acft, 1)
        end
    end
    -- Kopfzeilen
    local lh = FONT_H + 3
    local gs, tas = math.floor(DR.gs[0] / KT + 0.5), math.floor(DR.tas[0] / KT + 0.5)
    DL.rectf(x0, y0, x0 + 230, y0 + 4 * lh + 8, MC.bg)
    local yy = y0 + 4
    segs_text(x0 + 6, yy, { { "GS ", MC.white }, { tostring(gs), MC.green }, { "  TAS ", MC.white }, { tostring(tas), MC.green } })
    yy = yy + lh
    if not E then
        local wtxt = "---/---"
        if DR.wspd and DR.tpsi then
            local wd = (DR.wdir[0] + DR.tpsi[0] - DR.mpsi[0]) % 360
            wtxt = string.format("%03d/%d", math.floor(wd + 0.5), math.floor(DR.wspd[0] + 0.5))
        end
        segs_text(x0 + 6, yy, { { wtxt, MC.green } })
        yy = yy + lh
    else
        segs_text(x0 + 6, yy, { { "RWY  ", MC.white }, { E.id, MC.green },
            { string.format("  %dM - %03d\194\176", math.floor(E.len - E.disp + 0.5), math.floor(E.hdg + 0.5)), MC.green } })
        yy = yy + lh
        if ts then
            segs_text(x0 + 6, yy, { { "EXIT ", MC.white }, { tname, MC.green },
                { string.format("  %dM", math.floor(ts - E.disp + 0.5)), MC.green } })
            yy = yy + lh
            local rot = rot_estimate(ts, tx)
            if rot then segs_text(x0 + 6, yy, { { "ROT  ", MC.white }, { string.format("%d\"", math.floor(rot + 0.5)), MC.green } }) end
        else
            segs_text(x0 + 6, yy, { { "BTV : SELECT EXIT", MC.white } })
        end
    end
    local nm = (apt.name or ""):upper()
    if #nm > 26 then nm = nm:sub(1, 26) end
    DL.rectf(x0 + cw - 190, y0, x0 + cw, y0 + 2 * lh + 8, MC.bg)
    DL.text(x0 + cw - text_w(nm) - 6, y0 + 4, MC.white, nm)
    local codes = apt.icao .. (G.iata and ("   " .. G.iata) or "")
    DL.text(x0 + cw - text_w(codes) - 6, y0 + 4 + lh, MC.white, codes)
    for _, r in ipairs(G.rwys) do
        if r.L then
            local qx, qy = C.acx - r.x1, C.acy - r.y1
            local al, cr = qx * r.ux + qy * r.uy, -qx * r.uy + qy * r.ux
            if al > 0 and al < r.L and math.abs(cr) < r.w / 2 + 5 then
                local t = r.id1 .. " - " .. r.id2
                box_text(x0 + cw / 2 - text_w(t) / 2, y0 + ch - lh - 6, t, MC.taxiline, MC.taxiline)
                break
            end
        end
    end
    local by = y0 + ch - lh - 4
    DL.rectf(x0, by - 2, x0 + 260, y0 + ch, MC.bg)
    segs_text(x0 + 6, by, { { MODE_TXT[S.mode], MODE_COL[S.mode] }, { "    VIEW ", MC.white },
        { MODE_NAME[mode] .. ((V.mode == "auto") and " (AUTO)" or ""), MC.sel } })

    -- Maus: Hinweis, Antippen = Exit / Piste (Auswahl), Rechtsklick = Stoppunkt
    local menu = MV.menu
    local hm
    if hovered and #marks > 0 and not menu and btv_ui() then
        for _, m in ipairs(marks) do
            local r = m.rect
            if r and mx >= r[1] and mx <= r[3] and my >= r[2] and my <= r[4] then hm = m; break end
        end
        if not hm then
            local bd = 9
            for _, m in ipairs(marks) do
                local dd = dist_to_poly(m.pts, mx, my)
                for _, pp in ipairs(m.alt or {}) do dd = math.min(dd, dist_to_poly(pp, mx, my)) end
                if dd < bd then hm, bd = m, dd end
            end
        end
    end
    if hm then
        local x = hm.x
        if not (x == tx) then
            draw_poly(hm.pts, MC.white, 2.5)
            for _, pp in ipairs(hm.alt or {}) do draw_poly(pp, MC.white, 2.5) end
        end
        local tip = string.format("%s  %dM  %s%s", x.name, math.floor(x.dist + 0.5), x.ret and "HIGH SPEED EXIT" or "90 DEG EXIT",
            x.too_short and "  TOO SHORT" or ((hm.i == S.rec_idx) and "  RECOMMENDED" or ""))
        box_text(math.max(x0 + 4, math.min(mx + 14, x0 + cw - text_w(tip) - 8)), my + 12, tip, MC.white, MC.grey)
    elseif hovered and not menu then
        -- Rollweg / Parkposition unter der Maus anzeigen
        local bd, bname = 8, nil
        for _, e in ipairs(G.edges) do
            if not e.rwy and e.name ~= "" then
                local px, py = XY(e.x1, e.y1)
                local qx, qy = XY(e.x2, e.y2)
                if (px > x0 or qx > x0) and (px < x0 + cw or qx < x0 + cw) and (py > y0 or qy > y0) and (py < y0 + ch or qy < y0 + ch) then
                    local dd = dist_to_poly({ { px, py }, { qx, qy } }, mx, my)
                    if dd < bd then bd, bname = dd, "TWY " .. e.name end
                end
            end
        end
        for _, l in ipairs(G.stand_lbls) do
            if l.px then
                local dd = math.sqrt((l.px - mx) ^ 2 + (l.py - my) ^ 2)
                if dd < bd + 4 then bd, bname = dd, "STAND " .. l.text:gsub("_", " / ") end
            end
        end
        if bname then
            box_text(math.max(x0 + 4, math.min(mx + 14, x0 + cw - text_w(bname) - 8)), my + 12, bname, MC.label, MC.grey)
        end
    end
    -- Piste unter einem Bildschirmpunkt (fuer "SEL RWY")
    local function rwy_at(px, py)
        local wx, wy = INV(px, py)
        local best, bq
        for _, r in ipairs(G.rwys) do
            if r.L then
                local qx, qy = wx - r.x1, wy - r.y1
                local al, cr = qx * r.ux + qy * r.uy, -qx * r.uy + qy * r.ux
                local lim = r.w / 2 + math.max(6, 8 / sc)
                -- an Kreuzungen: die Bahn, deren Mittellinie am naechsten liegt
                if al > -20 and al < r.L + 20 and math.abs(cr) < lim and (not bq or math.abs(cr) / lim < bq) then
                    best, bq = r, math.abs(cr) / lim
                end
            end
        end
        return best, wx, wy
    end
    local function end_by_id(id)
        for _, Ex in ipairs(apt.ends) do if Ex.id == id then return Ex end end
    end
    local B = { x0, y0, cw, ch }
    local busy = S.mode == "ENGAGED" or S.mode == "RELEASE"
    if hovered and inp.clicked then MV.press, MV.drag = true, 0 end
    if MV.press and inp.released then
        MV.press = false
        local tap = MV.drag < 6
        if menu then
            if tap then
                local k = MENU.at(menu, mx, my)
                MV.menu = nil
                if k and k > 0 and menu.items[k].fn then menu.items[k].fn() end
            end
        elseif tap and hm and not busy and btv_ui() and not S.locked() then
            if hm.x.too_short then
                message("BTV EXIT " .. hm.x.name .. " TOO SHORT")
            else
                S.exit, S.manual_dist = hm.x, nil
                message(string.format("BTV EXIT %s  %d M", hm.x.name, math.floor(hm.x.dist + 0.5)))
            end
        elseif tap and not busy and btv_ui() and not S.locked() then
            -- Piste angetippt: Landebahn waehlen (beide Richtungen), auf der gewaehlten Bahn auch Stoppunkt
            local r, wx, wy = rwy_at(mx, my)
            if r then
                local items = {}
                for _, id in ipairs({ r.id1, r.id2 }) do
                    local Ex = end_by_id(id)
                    if Ex then
                        items[#items + 1] = {
                            cols = { { "SEL RWY", 0, MC.white }, { Ex.id, 56, (S.rwy == Ex) and MC.green or MC.sel },
                                     { string.format("%dM", math.floor(Ex.len - Ex.disp + 0.5)), 90, MC.white } },
                            fn = function() select_runway(Ex, true); message("BTV RWY " .. Ex.id) end }
                    end
                end
                if E and (E.id == r.id1 or E.id == r.id2) then
                    local dx, dy = wx - E.ox, wy - E.oy
                    local sa = dx * E.ux + dy * E.uy
                    local dist = math.floor(sa - E.disp + 0.5)
                    if dist > 300 and sa < E.len then
                        items[#items + 1] = { cols = { { "STOP HERE", 0, MC.white }, { string.format("%dM", dist), 90, MC.sel } },
                            fn = function()
                                S.manual_dist, S.exit = dist, nil
                                message(string.format("BTV STOP POINT %d M", dist))
                            end }
                    end
                end
                MENU.open(MV, items, mx + 8, my + 8, B, { title = "RWY " .. r.id1 .. "/" .. r.id2 })
            end
        end
    end
    if E and hovered and inp.rclicked and not menu and not busy and btv_ui() and not S.locked() then
        local wx, wy = INV(mx, my)
        local dx, dy = wx - E.ox, wy - E.oy
        local s, d = dx * E.ux + dy * E.uy, -dx * E.uy + dy * E.ux
        local dist = s - E.disp
        if math.abs(d) < E.width / 2 + 20 and dist > 300 and s < E.len then
            S.manual_dist, S.exit = math.floor(dist + 0.5), nil
            message(string.format("BTV STOP POINT %d M", S.manual_dist))
        end
    end
    -- Auswahlliste zuletzt (liegt ueber allem)
    if MV.menu then MENU.draw(MV.menu, hovered and mx or nil, my) end
end

-------------------------------------------------------------------------------
-- Kern der Karte: zeichnet nach [x0,x0+cw]x[y0,y0+ch] und verarbeitet die Eingaben
-- inp = { mx, my, hovered, active, wheel, dx, dy, clicked, released, rclicked, keys, t }
-- nd = Zustand der ND-Seite, wenn die Karte ins Cockpit-Display gezeichnet wird (sonst nil = Fenster)
-------------------------------------------------------------------------------
local function map_core(apt, x0, y0, cw, ch, V, MV, inp, nd)
    local C = { G = apt.geo, x0 = x0, y0 = y0, cw = cw, ch = ch, nd = nd, placed = {}, hit = {},
                mx = inp.mx, my = inp.my, hovered = inp.hovered }
    if nd then
        NDUI.pre_input(V, MV, inp)
        MAPX.view_nd(C, apt, V, MV, inp, nd.side)
        V.eff, V.btv, V.hv = C.mode, C.btv, C.hv
        -- Ein-/Ausblenden: die Karte zoomt hinein bzw. heraus (nd.fade 0..1, weich)
        local f = nd.fade or 1
        if f < 1 then
            local e = f * f * (3 - 2 * f)
            C.sc = C.sc * (0.55 + 0.45 * e)
            C.fading = true
        end
    else
        MAPX.view_win(C, apt, V, MV, inp)
    end
    MAPX.setup(C)
    MAPX.surfaces(C)
    if nd then NDUI.prepare(C, V, nd) else MAPX.reserve_win(C) end
    MAPX.labels(C)
    MV.marks = C.marks
    if nd then
        for _, m in ipairs(C.marks) do
            if m.rect then C.hit[#C.hit + 1] = { r = m.rect, exit = m } end
        end
        NDUI.overlay(C, V, MV, nd)
        NDUI.input(C, V, MV, inp, nd)
    else
        MAPX.overlay_win(C, apt, V, MV, inp)
    end
end

-- Karte im Fenster (ImGui): Eingaben sammeln, dann map_core
local function draw_map(apt, w, h)
    local vis = imgui.BeginChild("##btvmapchild", w, h, false, 8) -- 8 = ohne Scrollbalken
    if vis then
        local x0, y0 = imgui.GetWindowPos()
        local cw, ch = imgui.GetWindowSize()
        if type(x0) == "number" and type(cw) == "number" then
            imgui.InvisibleButton("##btvmap", cw, ch * 3)
            local inp = { hovered = imgui.IsItemHovered(), active = imgui.IsItemActive(), wheel = 0, dx = 0, dy = 0 }
            -- Mausrad: der Kartenbereich ist scrollbar, die Scrollbewegung wird zu Zoom
            local sy = imgui.GetScrollY() or 0
            local mid = math.floor((imgui.GetScrollMaxY() or 0) / 2)
            if MV.sy_set and math.abs(ch - MV.h_last) < 1 then
                local delta = sy - MV.sy_set
                if math.abs(delta) > 0.5 and math.abs(delta) < ch then inp.wheel = -delta / (5 * FONT_H) end
            end
            if math.abs(sy - mid) > 0.5 then imgui.SetScrollY(mid) end
            MV.sy_set, MV.h_last = mid, ch
            local mx, my = imgui.GetMousePos()
            if type(mx) ~= "number" then mx, my = -9999, -9999 end
            inp.mx, inp.my = mx, my
            if inp.active and imgui.IsMouseDragging(0) then
                local dx, dy = imgui.GetMouseDragDelta(0)
                if type(dx) == "number" and (dx ~= 0 or dy ~= 0) then
                    inp.dx, inp.dy = dx, dy
                    imgui.ResetMouseDragDelta(0)
                end
            end
            inp.clicked = imgui.IsMouseClicked(0)
            inp.released = imgui.IsMouseReleased(0)
            inp.rclicked = imgui.IsMouseClicked(1)
            DL.H = nil
            FONT_H = 13
            MENU.lh = FONT_H + 6
            map_core(apt, x0, y0, cw, ch, S.view, MV, inp, nil)
        end
    end
    imgui.EndChild()
end

-------------------------------------------------------------------------------
-- ND im Cockpit, in Schichten (fuer ruhige Bildraten):
--   Karte (Flaechen, Pisten, Beschriftungen): nur bei Bedarf neu aufgebaut, verteilt auf mehrere Bilder (Zeitbudget);
--     dazwischen verschiebt / dreht / skaliert die Grafikkarte das letzte Bild mit einer Matrix auf die aktuelle Ansicht
--   Overlay (Kompass, Kopfzeilen, Tasten, Flugzeug, Bedienfeld, Maus): jedes Bild, wenn sich etwas aendert
-------------------------------------------------------------------------------
MAPX.ND_M = 28 -- Rand um das ND [Einheiten]: beim Drehen/Verschieben zwischen zwei Aufbauten keine Luecken

-- aktuelle Ansicht (billig, jedes Bild)
function MAPX.nd_view(apt, V, MV, inp, sd)
    local C = { G = apt.geo, x0 = 0, y0 = 0, cw = ND_VW, ch = ND_VH, nd = sd, placed = {}, hit = {},
                mx = inp.mx, my = inp.my, hovered = inp.hovered }
    NDUI.pre_input(V, MV, inp)
    MAPX.view_nd(C, apt, V, MV, inp, sd.side)
    V.eff, V.btv, V.hv = C.vk or C.mode, C.btv, C.hv
    -- Ein-/Ausblenden: die Karte zoomt hinein bzw. heraus (sd.fade 0..1, weich)
    local f = sd.fade or 1
    if f < 1 then
        local e = f * f * (3 - 2 * f)
        C.sc = C.sc * (0.55 + 0.45 * e)
        C.fading = true
    end
    MAPX.setup(C)
    return C
end

-- Aufbau-Zusammenhang: wie die aktuelle Ansicht, aber mit Rand (lokal um ND_M verschoben)
function MAPX.nd_build_ctx(Cv, apt, sd)
    local m = MAPX.ND_M
    local Cb = { G = Cv.G, x0 = 0, y0 = 0, cw = ND_VW + 2 * m, ch = ND_VH + 2 * m, nd = sd, placed = {}, hit = {},
                 mode = Cv.mode, hv = Cv.hv, wcx = Cv.wcx, wcy = Cv.wcy, sc = Cv.sc, ax = Cv.ax + m, ay = Cv.ay + m,
                 Rr = Cv.Rr, rng = Cv.rng, hc = Cv.hc, hs = Cv.hs, acx = Cv.acx, acy = Cv.acy, btv = Cv.btv,
                 taxi = Cv.taxi, fading = Cv.fading, shift = m, appr = Cv.appr }
    MAPX.setup(Cb)
    return Cb
end

-- Karte aufbauen: Schritte (verteilt auf mehrere Bilder oder alle sofort)
function MAPX.nd_build_steps(Cb, V)
    local steps = MAPX.surface_steps(Cb)
    -- Beschriftungen: Bereiche frei lassen (Kopfzeilen, Tasten), dann Pisten-/Rollweg-/Exit-Namen
    steps[#steps + 1] = function()
        local n0 = #Cb.placed
        NDUI.prepare(Cb, V, Cb.nd)
        local m = Cb.shift
        for i = n0 + 1, #Cb.placed do
            local r = Cb.placed[i]
            r[1], r[2], r[3], r[4] = r[1] + m, r[2] + m, r[3] + m, r[4] + m
        end
        return true
    end
    steps[#steps + 1] = function()
        MAPX.labels(Cb)
        for _, mk in ipairs(Cb.marks or {}) do
            if mk.rect then Cb.hit[#Cb.hit + 1] = { r = mk.rect, exit = mk } end
        end
        return true
    end
    return steps
end

-- Abbildung Aufbau (lokal mit Rand) -> aktuelle Ansicht (lokal): affin aus drei Weltpunkten
function MAPX.nd_delta(Cb, Cv)
    local wx, wy = Cb.wcx, Cb.wcy
    local p0x, p0y = Cb.XY(wx, wy)
    local p1x, p1y = Cb.XY(wx + 100, wy)
    local p2x, p2y = Cb.XY(wx, wy + 100)
    local q0x, q0y = Cv.XY(wx, wy)
    local q1x, q1y = Cv.XY(wx + 100, wy)
    local q2x, q2y = Cv.XY(wx, wy + 100)
    local a, b, c, d = p1x - p0x, p2x - p0x, p1y - p0y, p2y - p0y
    local det = a * d - b * c
    if math.abs(det) < 1e-12 then return { 1, 0, 0, 1, 0, 0 } end
    local e, f, g, h = q1x - q0x, q2x - q0x, q1y - q0y, q2y - q0y
    local A11, A12 = (e * d - f * c) / det, (f * a - e * b) / det
    local A21, A22 = (g * d - h * c) / det, (h * a - g * b) / det
    return { A11, A12, A21, A22, q0x - (A11 * p0x + A12 * p0y), q0y - (A21 * p0x + A22 * p0y) }
end
function MAPX.nd_apply(D, x, y) return D[1] * x + D[2] * y + D[5], D[3] * x + D[4] * y + D[6] end

-- wie weit ist die aktuelle Ansicht vom Aufbau weg? (Drehung [Grad], Verschiebung [Einheiten], Massstab)
function MAPX.nd_moved(D)
    local m = MAPX.ND_M
    local cx, cy = MAPX.nd_apply(D, ND_VW / 2 + m, ND_VH / 2 + m)
    local rot = math.abs(math.atan2(D[3], D[1])) / D2R
    local scl = math.sqrt(D[1] * D[1] + D[3] * D[3])
    return rot, math.sqrt((cx - ND_VW / 2) ^ 2 + (cy - ND_VH / 2) ^ 2), scl
end

-- Treffer und Exits des letzten Aufbaus in die aktuelle Ansicht umrechnen
function MAPX.nd_hits_now(mb, D)
    local ap = MAPX.nd_apply
    local function pts_now(p)
        if not p then return nil end
        local out = {}
        for i, q in ipairs(p) do
            local x, y = ap(D, q[1], q[2])
            out[i] = { x, y }
        end
        return out
    end
    local function rect_poly(r)
        local a1, b1 = ap(D, r[1], r[2])
        local a2, b2 = ap(D, r[3], r[2])
        local a3, b3 = ap(D, r[3], r[4])
        local a4, b4 = ap(D, r[1], r[4])
        local poly = { { a1, b1 }, { a2, b2 }, { a3, b3 }, { a4, b4 } }
        local bb = { math.min(a1, a2, a3, a4), math.min(b1, b2, b3, b4), math.max(a1, a2, a3, a4), math.max(b1, b2, b3, b4) }
        return poly, bb
    end
    local hits, marks = {}, {}
    local done = {}
    for _, mk in ipairs(mb.marks or {}) do
        local mt = { x = mk.x, i = mk.i, pts = pts_now(mk.pts), alt = {} }
        for k, a in ipairs(mk.alt or {}) do mt.alt[k] = pts_now(a) end
        local poly
        if mk.rect then poly, mt.rect = rect_poly(mk.rect) end
        mt.poly = poly
        marks[#marks + 1] = mt
        done[mk] = mt
    end
    for _, h in ipairs(mb.hits or {}) do
        if h.exit then
            local mt = done[h.exit]
            if mt and mt.poly then hits[#hits + 1] = { poly = mt.poly, exit = mt } end
        elseif h.poly then
            hits[#hits + 1] = { poly = pts_now(h.poly), qfu = h.qfu }
        elseif h.r then
            local poly = rect_poly(h.r)
            local nh = {}
            for k, v in pairs(h) do nh[k] = v end
            nh.r, nh.poly = nil, poly
            hits[#hits + 1] = nh
        end
    end
    return hits, marks
end

-------------------------------------------------------------------------------
-- Fensterinhalt
-------------------------------------------------------------------------------
local function settings_panel()
    local function section(t)
        imgui.Separator()
        text(t, COL.cyan)
    end
    -- Erklaerung unter einer Option (grau, eingerueckt)
    local function hint(t, col) text("      " .. t, col or COL.grey) end
    text("A350 Systems Mod " .. VERSION, COL.white)

    -- OANS
    section("OANS")
    local c1, v1 = imgui.Checkbox("Airport map on the ND##nd_on", CFG.nd_on)
    if c1 then CFG.nd_on = v1; save_config() end
    hint("range knob to ZOOM (ROSE NAV, ARC or PLAN)")
    local c2, v2 = imgui.Checkbox("Buildings from OpenStreetMap##osm_on", CFG.osm)
    if c2 then CFG.osm = v2; save_config(); if v2 and S.apt then osm_attach(S.apt) end end
    if CFG.osm then
        hint("downloaded once per airport  -  \194\169 OpenStreetMap contributors (ODbL)")
    end
    -- (nur nach einem Absturz beim Zeichnen von der Absturzsicherung abgeschaltet: hier wieder einschalten)
    if not CFG.nd_layers then
        local c10, v10 = imgui.Checkbox("Layered ND drawing (switched off after a crash)##layers", CFG.nd_layers)
        if c10 then CFG.nd_layers = v10; save_config() end
        hint("less CPU; takes effect after reloading the scripts")
    end

    -- Ansagen (ROW / ROP, BTV)
    section("CALLOUTS")
    label("Voice:")
    imgui.SameLine()
    opt("SetNextItemWidth", 190)
    local cur = AUD.VOICES[CFG.voice] or AUD.VOICES[1]
    if imgui.BeginCombo("##voice", cur.name) then
        for k, v in ipairs(AUD.VOICES) do
            if imgui.Selectable(v.name .. "##voice" .. k, CFG.voice == k) then CFG.voice = k; save_config() end
        end
        imgui.EndCombo()
    end
    imgui.SameLine()
    if imgui.Button("Test##aural") then AUD.test() end
    if imgui.SliderFloat then
        label("Volume:")
        imgui.SameLine()
        opt("SetNextItemWidth", 190)
        local chv, nvv = imgui.SliderFloat("##avol", tonumber(CFG.avol) or 1, 0, 2, "%.1f")
        if chv and tonumber(nvv) then CFG.avol = clamp(tonumber(nvv), 0, 2); save_config() end
    end

    -- Datalink
    section("DATALINK (Hoppie)")
    local hs, hcode, hcs, herr
    if btv_hoppie_status then hs, hcode, hcs, herr = btv_hoppie_status() end
    if not hs then
        hint("ATCCOM files not installed (Scripts/A350_ATCCOM)")
    else
        label("Logon code:")
        imgui.SameLine()
        opt("SetNextItemWidth", 170)
        local ch, nv = imgui.InputText("##hoplogon", S.hop_input or "", 40)
        if ch then S.hop_input = nv end
        imgui.SameLine()
        if imgui.Button("Save##hop") then btv_hoppie_set_logon(S.hop_input or ""); S.hop_input = "" end
        hint(hcode and "saved - type a new one and Save to replace it" or "free at www.hoppie.nl/acars/system/register.html")
        hint("Status: " .. hs .. (hcs and ("   " .. hcs) or "") .. (herr and ("   (" .. herr .. ")") or ""))
    end

    -- Cockpit-Modell
    section("COCKPIT")
    local cx, vx = imgui.Checkbox("ZOOM on the range knob and A/BRK pushbutton##extras", CFG.cockpit_extras)
    if cx then CFG.cockpit_extras = vx; save_config(); btv_extras_apply() end
    hint("off = the aircraft's original cockpit model and textures (from the next aircraft load)")
end

-- Fenster: nur Einstellungen (bedient wird alles im ND)
function btv_build(wnd_, x, y)
    settings_panel()
end

function btv_on_close()
    wnd = nil
end

function btv_show_window()
    if wnd then return end
    wnd = float_wnd_create(720, 640, 1, true)
    float_wnd_set_title(wnd, "A350 Systems Mod")
    float_wnd_set_imgui_builder(wnd, "btv_build")
    float_wnd_set_onclose(wnd, "btv_on_close")
    if float_wnd_set_resizing_limits then pcall(float_wnd_set_resizing_limits, wnd, 480, 300, 2400, 2400) end
end

function btv_hide_window()
    if wnd then float_wnd_destroy(wnd) end
    wnd = nil
end

function btv_toggle_window()
    if wnd then btv_hide_window() else btv_show_window() end
end

-------------------------------------------------------------------------------
-- Karte im ND des 3D-Cockpits
-- Die Anzeigen der A350 sind Teil der Cockpit-Textur ("Panel": ATTR_cockpit in objects/screens.obj).
-- Das Skript zeichnet nach den Flugzeug-Plugins in dieselbe Textur (OpenGL ueber LuaJIT-FFI, Zeichenphase
-- "Gauges, danach"). X-Plane bildet sie wie die echten Anzeigen ab: richtige Perspektive, verdeckt von
-- Haenden/Rahmen, scharf auch beim Heranzoomen. Ist die Karte aus, wird nur das Kaestchen "OANS" gezeichnet.
-- Bedienung: ein unsichtbares Fenster ueber dem Bildschirm (nur sichtbar, solange ein ND auf 10 steht)
-- nimmt Klicks auf der Karte und dem Kaestchen an - alle anderen Klicks gehen unveraendert ans Cockpit.
-------------------------------------------------------------------------------
-- ND-Haelften: Textur-Koordinaten (u links, v unten, u rechts, v oben) und Ecken im Raum
-- (m relativ zum Schwerpunkt: x rechts, y oben, z hinten; oben links, oben rechts, unten rechts, unten links).
-- Die inneren Bildschirme zeigen PFD und ND nebeneinander: Kapitaen = rechte Haelfte, Copilot = linke.
-- Die eigentliche Anzeigeflaeche ist ATTR_cockpit in A350_xp11_cockpit.obj (gross: ND = 504 x 637 Panel-Pixel).
-- (objects/screens.obj liegt 0,8 mm davor und nutzt nur einen winzigen Texturbereich - dort waere die Karte
--  88 x 111 Pixel klein und pixelig.)
local ND_UV = { [0] = { 0.24810, 0.68910, 0.49429, 0.99989 }, [1] = { 0.49680, 0.68910, 0.74296, 0.99989 } }
-- PFD daneben (gleiche Groesse; Kapitaen links vom ND, Copilot rechts) - fuer "RWY AHEAD"
-- Kasten "RWY AHEAD" im PFD (lokal 300 x 380 wie das ND): ueber dem Flugzeugsymbol in der Mitte des Horizonts
local PFD = { uv = { [0] = { 0.00190, 0.68910, 0.24810, 0.99989 }, [1] = { 0.74296, 0.68910, 0.98910, 0.99989 } }, box = { 125, 138 } }
-- etwas ueber den Rand hinaus zeichnen (Panel-Pixel: links, unten, rechts, oben), damit keine FF-Pixel durchscheinen;
-- ausserhalb der Anzeigeflaeche ist die Textur unsichtbar. Zur Mitte (PFD bzw. Kapitaens-Anzeige) nichts.
local ND_PAD = { [0] = { 0, 0.5, 2, 2 }, [1] = { 2, 0.5, 0, 2 } }
local ND_GEOM = {
    [0] = { { -0.4402, 1.9664, -28.7178 }, { -0.2727, 1.9664, -28.7178 }, { -0.2727, 1.7573, -28.6809 }, { -0.4402, 1.7573, -28.6809 } },
    [1] = { { 0.2735, 1.9664, -28.7178 }, { 0.4410, 1.9664, -28.7178 }, { 0.4410, 1.7573, -28.6809 }, { 0.2735, 1.7573, -28.6809 } },
}
-- MASTER-CAUT-Tasten im Glareshield (Kapitaen, Copilot): Manipulator 1-sim/130/button in A350_xp11_cockpit.obj
-- (Objekt -> Flugzeug wie beim ND: y - 2,4384 m, z - 30,3024 m). (Feld von ND_GEOM: das Hauptprogramm hat keine
-- freien lokalen Variablen mehr - Lua erlaubt 200)
ND_GEOM.mc = {
    { { -0.3190, 2.0849, -28.5264 }, { -0.3015, 2.0849, -28.5264 }, { -0.3015, 2.0679, -28.5213 }, { -0.3190, 2.0679, -28.5213 } },
    { { 0.2996, 2.0849, -28.5264 }, { 0.3171, 2.0849, -28.5264 }, { 0.3171, 2.0679, -28.5213 }, { 0.2996, 2.0679, -28.5213 } },
}
-- Range-Knoepfe im FCU (Drehpunkt von 1-sim/fcu/ndZoomLeft/Right/switch in A350_xp11_cockpit.obj, umgerechnet wie oben)
-- und Fangradius [m] fuer das Mausrad (Stellung ZOOM)
-- A/BRK-Taste (1-sim/225/button) neben dem Fahrwerkshebel: Mitte und halbe Breite des Fangquadrats [m] (Taste
-- 17,5 mm breit - auch Klicks an Rand und Ecken duerfen im Flug nicht beim FF als RTO landen)
ND_GEOM.abrk = { 0.2213, 1.8885, -28.6993, r = 0.0105 }
ND_GEOM.knob = { [0] = { -0.1465, 2.0714, -28.5265 }, [1] = { 0.1508, 2.0714, -28.5265 }, r = 0.013 }

local PNL = {
    st = "not started yet", -- Zustand (Einstellungen)
    ok = false, tried = false, dead = false,
    frame = 0, ft = -100,
    sd = { [0] = { side = 0, q = {} }, [1] = { side = 1, q = {} } },
    seen = {},       -- panel_render_type -> Anzahl (Diagnose)
    nerr = 0, diag = 0, nclick = 0,
}
local ffi, XP, GL -- LuaJIT-FFI, XPLM- und OpenGL-Bibliothek (oder Testersatz)

local PNL_CDEF = [[
typedef int (*btv_draw_f)(int, int, void *);
typedef void (*btv_wdraw_f)(void *, void *);
typedef int (*btv_wclick_f)(void *, int, int, int, void *);
typedef void (*btv_wkey_f)(void *, char, int, char, void *, int);
typedef int (*btv_wcursor_f)(void *, int, int, void *);
typedef int (*btv_wwheel_f)(void *, int, int, int, int, void *);
typedef struct {
    int structSize; int left; int top; int right; int bottom; int visible;
    btv_wdraw_f drawWindowFunc; btv_wclick_f handleMouseClickFunc; btv_wkey_f handleKeyFunc;
    btv_wcursor_f handleCursorFunc; btv_wwheel_f handleMouseWheelFunc; void *refcon;
    int decorateAsFloatingWindow; int layer; btv_wclick_f handleRightClickFunc;
} btv_wnd_t;
typedef void (*btv_u1_f)(unsigned int);
typedef void (*btv_u2_f)(unsigned int, unsigned int);
typedef void (*btv_genbuf_f)(int, unsigned int *);
typedef void (*btv_bufdata_f)(unsigned int, ptrdiff_t, const void *, unsigned int);
typedef void *(*btv_mapbuf_f)(unsigned int, ptrdiff_t, ptrdiff_t, unsigned int);
typedef unsigned char (*btv_unmap_f)(unsigned int);
typedef void *(*btv_fence_f)(unsigned int, unsigned int);
typedef unsigned int (*btv_wait_f)(void *, unsigned int, uint64_t);
typedef void (*btv_delsync_f)(void *);
int btv_XPLMRegisterDrawCallback(btv_draw_f, int, int, void *) __asm__("XPLMRegisterDrawCallback");
int btv_XPLMUnregisterDrawCallback(btv_draw_f, int, int, void *) __asm__("XPLMUnregisterDrawCallback");
void btv_XPLMSetGraphicsState(int, int, int, int, int, int, int) __asm__("XPLMSetGraphicsState");
void *btv_XPLMFindDataRef(const char *) __asm__("XPLMFindDataRef");
int btv_XPLMGetDatai(void *) __asm__("XPLMGetDatai");
float btv_XPLMGetDataf(void *) __asm__("XPLMGetDataf");
void *btv_XPLMCreateWindowEx(btv_wnd_t *) __asm__("XPLMCreateWindowEx");
void btv_XPLMDestroyWindow(void *) __asm__("XPLMDestroyWindow");
void btv_XPLMSetWindowGeometry(void *, int, int, int, int) __asm__("XPLMSetWindowGeometry");
void btv_XPLMSetWindowIsVisible(void *, int) __asm__("XPLMSetWindowIsVisible");
void btv_XPLMGetScreenBoundsGlobal(int *, int *, int *, int *) __asm__("XPLMGetScreenBoundsGlobal");
void btv_XPLMGetMouseLocationGlobal(int *, int *) __asm__("XPLMGetMouseLocationGlobal");
void btv_XPLMTakeKeyboardFocus(void *) __asm__("XPLMTakeKeyboardFocus");
float btv_XPLMGetElapsedTime(void) __asm__("XPLMGetElapsedTime");
void btv_glGetIntegerv(unsigned int, int *) __asm__("glGetIntegerv");
void btv_glGetFloatv(unsigned int, float *) __asm__("glGetFloatv");
void btv_glEnable(unsigned int) __asm__("glEnable");
void btv_glDisable(unsigned int) __asm__("glDisable");
void btv_glBlendFunc(unsigned int, unsigned int) __asm__("glBlendFunc");
void btv_glPolygonMode(unsigned int, unsigned int) __asm__("glPolygonMode");
void btv_glPushAttrib(unsigned int) __asm__("glPushAttrib");
void btv_glPopAttrib(void) __asm__("glPopAttrib");
void btv_glPushClientAttrib(unsigned int) __asm__("glPushClientAttrib");
void btv_glPopClientAttrib(void) __asm__("glPopClientAttrib");
void btv_glEnableClientState(unsigned int) __asm__("glEnableClientState");
void btv_glDisableClientState(unsigned int) __asm__("glDisableClientState");
void btv_glVertexPointer(int, unsigned int, int, const void *) __asm__("glVertexPointer");
void btv_glColorPointer(int, unsigned int, int, const void *) __asm__("glColorPointer");
void btv_glDrawArrays(unsigned int, int, int) __asm__("glDrawArrays");
void btv_glReadPixels(int, int, int, int, unsigned int, unsigned int, void *) __asm__("glReadPixels");
void btv_glScissor(int, int, int, int) __asm__("glScissor");
void btv_glMatrixMode(unsigned int) __asm__("glMatrixMode");
void btv_glPushMatrix(void) __asm__("glPushMatrix");
void btv_glPopMatrix(void) __asm__("glPopMatrix");
void btv_glMultMatrixf(const float *) __asm__("glMultMatrixf");
int btv_XPLMGetDataRefTypes(void *) __asm__("XPLMGetDataRefTypes");
int btv_XPLMGetDatab(void *, void *, int, int) __asm__("XPLMGetDatab");
int btv_XPLMGetDatavi(void *, int *, int, int) __asm__("XPLMGetDatavi");
int btv_XPLMGetDatavf(void *, float *, int, int) __asm__("XPLMGetDatavf");
double btv_XPLMGetDatad(void *) __asm__("XPLMGetDatad");
void btv_XPLMBindTexture2d(int, int) __asm__("XPLMBindTexture2d");
void btv_XPLMGenerateTextureNumbers(int *, int) __asm__("XPLMGenerateTextureNumbers");
void btv_glTexImage2D(unsigned int, int, int, int, int, int, unsigned int, unsigned int, const void *) __asm__("glTexImage2D");
void btv_glTexParameteri(unsigned int, unsigned int, int) __asm__("glTexParameteri");
void btv_glTexEnvi(unsigned int, unsigned int, int) __asm__("glTexEnvi");
void btv_glTexCoordPointer(int, unsigned int, int, const void *) __asm__("glTexCoordPointer");
void btv_glPixelStorei(unsigned int, int) __asm__("glPixelStorei");
void btv_glDeleteTextures(int, const unsigned int *) __asm__("glDeleteTextures");
void btv_bridge_set_(btv_draw_f) __asm__("btv_bridge_set");
int btv_bridge_info_(void) __asm__("btv_bridge_info");
int btv_bridge_calls_(void) __asm__("btv_bridge_calls");
]]
local PNL_CDEF_WIN = [[ void *btv_wglGetProcAddress(const char *) __asm__("wglGetProcAddress"); ]]
local PNL_CDEF_LIN = [[ void *btv_glXGetProcAddressARB(const char *) __asm__("glXGetProcAddressARB"); ]]

local function pnl_log(s) pcall(log, "ND map: " .. s) end

-- OpenGL-Erweiterungsfunktion (nur zum Sichern/Loesen fremder Bindungen); nil, wenn nicht vorhanden
local function pnl_getproc(name, ctype)
    local ok, f = pcall(function()
        local p
        if PNL.T and PNL.T.getproc then
            p = PNL.T.getproc(name)
        elseif PNL.procaddr then
            p = PNL.procaddr(name)
        else
            return GL["btv_" .. name] -- macOS: direkt exportiert (dann ohne cdef nicht vorhanden -> nil)
        end
        if p == nil then return nil end
        local a = tonumber(ffi.cast("intptr_t", p))
        if a == 0 or a == 1 or a == 2 or a == 3 or a == -1 then return nil end
        return ffi.cast(ctype, p)
    end)
    if ok then return f end
    return nil
end

-------------------------------------------------------------------------------
-- Eingaben (Maus) je ND-Seite -> Eingabe-Tabelle fuer map_core
-------------------------------------------------------------------------------
local function pnl_input(sd)
    local inp = { mx = -9999, my = -9999, hovered = false, active = sd.held or false, wheel = 0, dx = 0, dy = 0 }
    if sd.hin then inp.mx, inp.my, inp.hovered = sd.hx, sd.hy, true end
    -- Verschiebung aus zwei Bildschirmpunkten mit derselben (aktuellen) Abbildung: so wandert die Karte nicht,
    -- wenn sich nur der Kopf/die Kamera bewegt (Vibrationen, Kopfbewegung)
    local function delta(e)
        if sd.inv and e.sx and sd.lsx then
            local ax, ay = sd.inv(sd.lsx, sd.lsy)
            local bx, by = sd.inv(e.sx, e.sy)
            if ax > -9000 and bx > -9000 then return bx - ax, by - ay end
        end
        return 0, 0
    end
    for _, e in ipairs(sd.q) do
        if e.t == "down" then
            inp.clicked, inp.hovered, inp.mx, inp.my, inp.active = true, true, e.x, e.y, true
            sd.held, sd.lsx, sd.lsy = true, e.sx, e.sy
        elseif e.t == "drag" then
            if sd.held then
                local ddx, ddy = delta(e)
                inp.dx, inp.dy = inp.dx + ddx, inp.dy + ddy
                sd.lsx, sd.lsy = e.sx, e.sy
                inp.mx, inp.my, inp.active = e.x, e.y, true
            end
        elseif e.t == "up" then
            if sd.held then
                local ddx, ddy = delta(e)
                inp.dx, inp.dy = inp.dx + ddx, inp.dy + ddy
            end
            inp.released, inp.mx, inp.my = true, e.x, e.y
            inp.hovered = e.x >= 0 and e.x <= ND_VW and e.y >= 0 and e.y <= ND_VH
            sd.held = false
        elseif e.t == "rdown" then
            inp.rclicked, inp.hovered, inp.mx, inp.my = true, true, e.x, e.y
        elseif e.t == "wheel" then
            inp.wheel = inp.wheel + e.n
            if not inp.clicked then inp.mx, inp.my, inp.hovered = e.x, e.y, true end
        elseif e.t == "key" then
            inp.keys = inp.keys or {}
            inp.keys[#inp.keys + 1] = e
        end
        if e.tm then inp.t = e.tm end
    end
    sd.q = {}
    return inp
end


-- eine ND-Seite zeichnen (lokale Koordinaten 0..ND_VW x 0..ND_VH): Karte als eigene Schicht (PNL.nd_map),
-- daruber Kompass / Kopfzeilen / Tasten (Overlay, sd.ob) - bzw. ohne Flughafen den Hinweis, bei
-- ausgeschalteter Karte nur die Taste OANS. geo = Lage der Seite im Panel (PNL.geo)
local function pnl_side(sd, geo)
    -- Zeichenflaeche zuerst: Textbreiten (text_w) brauchen sie. Ohne fragt text_w ImGui - ausserhalb eines
    -- ImGui-Fensters stuerzt X-Plane dabei ab (Absturz von v2.9 / v2.9.1)
    DL.begin(geo.H, ND_VW, ND_VH)
    local inp = pnl_input(sd)
    if not sd.vm and (sd.fade or 0) > 0.02 then
        inp = { hovered = false, active = false, wheel = 0, dx = 0, dy = 0, clicked = false, released = false, mx = -9999, my = -9999 }
    end
    local apt = S.apt
    sd.mv = sd.mv or { press = false, drag = 0 }
    if not sd.view then
        sd.view = new_view()
        S.nd_views = S.nd_views or {}
        S.nd_views[sd.side] = sd.view
    end
    sd.ob = sd.ob or PNL.newbuf(PNL.OVL_CAP)
    sd.draw_map = false
    local fa = ((sd.fade or 1) < 1) and PNL.ease(sd.fade) or nil
    if sd.vm or (sd.fade or 0) > 0.02 then
        local V = sd.view
        if sd.nomap then
            sd.mb, sd.job, sd.osig = nil, nil, nil
            PNL.ovl_begin(sd, geo, fa)
            NDUI.noapt(sd, inp, "MAP NOT AVAIL")
            PNL.ovl_end(sd)
        elseif apt and apt.geo and not btv_osm_waiting(apt) then
            if sd.view_apt ~= apt then
                -- anderer Flughafen: Ansicht zuruecksetzen, Bedienfeld bleibt (nur das selbst geoeffnete schliesst sich)
                sd.view_apt = apt
                if V.ui.panel_auto == "open" then V.ui.panel, V.ui.panel_auto = false, "done" end
                V.free, V.plan_acft = {}, false
                V.plan_key, V.ui.dd, V.ui.msel, V.ui.asel, V.ui.cands = nil, nil, nil, nil, nil
                sd.mv.ctx = nil
                sd.mb, sd.job, sd.osig = nil, nil, nil
            end
            PNL.nd_map(sd, apt, V, inp, geo, fa)
        else
            sd.mb, sd.job, sd.osig = nil, nil, nil
            PNL.ovl_begin(sd, geo, fa)
            NDUI.noapt(sd, inp)
            PNL.ovl_end(sd)
        end
        return
    end
    -- Karte aus: nichts darueber (das FF-ND bleibt sichtbar)
    sd.mv.ctx, sd.osig = nil, nil
    PNL.ovl_begin(sd, geo, nil)
    PNL.ovl_end(sd)
end

-- BTV im FMA: cyan = scharf, gruen = bremst (wie "BRK LO" des FF, aus dessen PFD vermessen)
function PFD.fma()
    if S.mode == "ARMED" then return PFD.cyan end
    if S.mode == "ENGAGED" or S.mode == "RELEASE" then return MC.green end
end
-- (Groesse wie die FF-Schrift, mittig unter SPEED; scharf: zweite Zeile wie "G/S", bremst: erste Zeile wie "SPEED")
PFD.fma_x, PFD.fma_base, PFD.fma_cap = 28.0, 23.6, 8.2
PFD.fma_base1 = 10.8
-- Cyan wie das "G/S" des FF (am Bildschirm abgeglichen)
PFD.cyan = 0xFFFFFD36

-- Meldung im PFD (Kasten unter dem Horizont): ROP vor ROW vor AUTO BRK OFF vor RWY AHEAD
function PFD.msg()
    if S.rop then return "MAX REVERSE", MC.short end
    if S.mrop then return (S.mrop == "brk") and "MAX BRAKING" or "MAX REVERSE", MC.short end
    if S.row == "dry" then return "RWY TOO SHORT", MC.short end
    if S.row == "wet" then return "IF WET : RWY TOO SHORT", MC.amber end
    local t = DR.time[0]
    if S.abo_t and t >= S.abo_t and t - S.abo_t < 10 then return "AUTO BRK OFF", MC.amber end
    if RWYA.text then return "RWY AHEAD", MC.cross end
end
-- Meldung und "BTV" in ein PFD (lokale Koordinaten wie das ND)
function PFD.draw(s)
    local col = PFD.fma()
    local line = (S.mode == "ENGAGED" or S.mode == "RELEASE") and 1 or 2
    if col and not (PNL.fma_busy and PNL.fma_busy[line]) then
        local t, cap = "BTV", PFD.fma_cap
        TF.reg = true
        local base = (line == 1) and PFD.fma_base1 or PFD.fma_base
        pcall(function() txt_b(PFD.fma_x - text_w(t, cap) / 2, base, col, t, cap) end)
        TF.reg = false
    end
    local t, mc = PFD.msg()
    if not t then return end
    local cap = (#t > 12) and TXT.cap or TXT.big
    local w = text_w(t, cap) + 10
    PFD.last_hw = w / 2 + 2
    local cx, cy = PFD.box[1], PFD.box[2]
    local r = { cx - w / 2, cy - 8, cx + w / 2, cy + 8 }
    DL.rectf(r[1], r[2], r[3], r[4], MC.bg)
    DL.rect(r[1], r[2], r[3], r[4], mc, 0, 0, 1.1)
    DL.text(cx - (w - 10) / 2, cy - FONT_H / 2, mc, t, cap)
end

-------------------------------------------------------------------------------
-- Zeichnen (Zeichenphase Gauges, nach X-Plane und den Flugzeug-Plugins)
-------------------------------------------------------------------------------
local GLC = {
    ENABLE_BIT = 0x2000, POLYGON_BIT = 0x0008, COLOR_BUFFER_BIT = 0x4000, CLIENT_VERTEX_ARRAY_BIT = 0x0002,
    SCISSOR_TEST = 0x0C11, SCISSOR_BOX = 0x0C10, SCISSOR_BIT = 0x00080000, TRANSFORM_BIT = 0x1000, MODELVIEW = 0x1700,
    CULL_FACE = 0x0B44, BLEND = 0x0BE2, COLOR_LOGIC_OP = 0x0BF2, FRONT_AND_BACK = 0x0408, FILL = 0x1B02,
    SRC_ALPHA = 0x0302, ONE_MINUS_SRC_ALPHA = 0x0303, FLOAT = 0x1406, UNSIGNED_BYTE = 0x1401, TRIANGLES = 0x0004,
    VERTEX_ARRAY = 0x8074, NORMAL_ARRAY = 0x8075, COLOR_ARRAY = 0x8076, INDEX_ARRAY = 0x8077,
    TEXTURE_COORD_ARRAY = 0x8078, EDGE_FLAG_ARRAY = 0x8079, FOG_COORD_ARRAY = 0x8457, SECONDARY_COLOR_ARRAY = 0x845E,
    TEXTURE0 = 0x84C0, CURRENT_PROGRAM = 0x8B8D, VERTEX_ARRAY_BINDING = 0x85B5, ARRAY_BUFFER_BINDING = 0x8894,
    ARRAY_BUFFER = 0x8892, VIEWPORT = 0x0BA2, PROJECTION_MATRIX = 0x0BA7, MODELVIEW_MATRIX = 0x0BA6,
    MAX_VERTEX_ATTRIBS = 0x8869, MAX_TEXTURE_COORDS = 0x8871,
    READ_FRAMEBUFFER_BINDING = 0x8CAA, DRAW_FRAMEBUFFER_BINDING = 0x8CA6, READ_FRAMEBUFFER = 0x8CA8,
    PIXEL_PACK_BUFFER = 0x88EB, PIXEL_PACK_BUFFER_BINDING = 0x88ED, RGBA = 0x1908,
    TEXTURE_BIT = 0x00040000, TEXTURE_ENV = 0x2300, TEXTURE_ENV_MODE = 0x2200, MODULATE = 0x2100,
    TEXTURE_2D = 0x0DE1, RGBA8 = 0x8058, CLIENT_PIXEL_STORE_BIT = 0x0001,
    PIXEL_UNPACK_BUFFER = 0x88EC, PIXEL_UNPACK_BUFFER_BINDING = 0x88EF,
}

-- Schriftatlas lesen (beim Start) und als Textur anlegen (im ersten Zeichen-Durchgang, dort gilt der GL-Kontext)
-- ND-Schrift aus der Displayschrift des Flugzeugs: cockpit_3d/OpenGLTextures/SymbolTex.tlstex (12 Byte Kopf, dann
-- Paare Anzahl/Wert, 2048 x 1024 Grauwerte, Zeilen von unten) -> A350_BTV_data/font_tls_px2.bin (Format wie
-- font_b612_px2.bin), einmalig beim ersten Start. Die Schrift wird nicht mitgeliefert - sie entsteht aus den Dateien
-- des eigenen Flugzeugs. Zeichen, die sie nicht hat, aus B612 (an der Grundlinie ausgerichtet).
function TF.build_tls()
    local out = SCRIPTS .. "A350_BTV_data/font_tls_px2.bin"
    local fo = io.open(out, "rb")
    if fo then fo:close(); return true end
    local a = AIRCRAFT_PATH or ""
    if a ~= "" and not a:find("[/\\]$") then a = a .. "/" end
    local f = io.open(a .. "cockpit_3d/OpenGLTextures/SymbolTex.tlstex", "rb")
    if not f then return false, "SymbolTex.tlstex not found in the aircraft" end
    local d = f:read("*a"); f:close()
    local fb = io.open(SCRIPTS .. "A350_BTV_data/font_b612_px2.bin", "rb")
    if not fb then return false, "font_b612_px2.bin not found" end
    local od = fb:read("*a"); fb:close()
    local TW, TH = 2048, 1024
    local N = TW * TH
    local img, o = {}, 0
    for i = 13, #d - 1, 2 do
        local cnt, val = string.byte(d, i, i + 1)
        local e = math.min(o + cnt, N)
        for k = o, e - 1 do img[k] = val end
        o = o + cnt
        if o >= N then break end
    end
    for k = o, N - 1 do img[k] = 0 end
    local function px(x, y) return img[(TH - 1 - y) * TW + x] end
    -- Saetze (Zelle ab x0, Breite, Versalhoehe, Zeilen oben): fett und normal
    local SETS = { bold = { x0 = 10, pitch = 32, cap = 36, tops = { 342, 382, 422, 462 } },
                   reg = { x0 = 10, pitch = 28, cap = 30, tops = { 860, 900, 940, 980 } } }
    local ink = 0
    for y = 462, 462 + 35 do for x = 10, 10 + 32 * 15 - 1 do if px(x, y) > 100 then ink = ink + 1 end end end
    if ink < 2000 then return false, "SymbolTex.tlstex: unexpected layout" end
    local ROWS = { { 60, 46, 62, 32, 47, 176, 42, 34, 39 },
                   { 85, 86, 87, 88, 89, 90, 37, 40, 41, 45, 32, 43, 61, 0, 58 },
                   { 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84 },
                   { 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 65, 66, 67, 68, 69 } }
    local POS = {}
    for r, row in ipairs(ROWS) do
        for c, code in ipairs(row) do if code ~= 32 and code ~= 0 then POS[code] = { r - 1, c - 1 } end end
    end
    local function weights(ns, nd)
        local k, w = nd / ns, {}
        for dd = 0, nd - 1 do
            local a, b = dd / k, (dd + 1) / k
            local t = {}
            for s0 = math.floor(a), math.ceil(b) - 1 do
                local ov = math.min(b, s0 + 1) - math.max(a, s0)
                if ov > 0 and s0 >= 0 and s0 < ns then t[#t + 1] = { s0, ov * k } end
            end
            w[dd] = t
        end
        return w
    end
    -- alter Atlas (B612): Kopf, Groessen, DATA
    local function nextline(pos)
        local e = od:find("\n", pos, true)
        return od:sub(pos, e - 1):gsub("\r$", ""), e + 1
    end
    local line, pos = nextline(1)
    local h = {}
    for w in line:gmatch("%S+") do h[#h + 1] = w end
    local oW, ons = tonumber(h[2]), tonumber(h[4])
    local old = {}
    for _ = 1, ons do
        line, pos = nextline(pos)
        local a, k = {}, 0
        for w in line:gmatch("%S+") do k = k + 1; if k > 1 then a[k - 1] = tonumber(w) end end
        old[a[1]] = a   -- cap adv cw ch base left x0 y0 cols
    end
    line, pos = nextline(pos)
    if line ~= "DATA" then return false, "font_b612_px2.bin: unknown format" end
    local dbase = pos   -- Byte des Pixels q: dbase + q
    local W, H = 1024, 2048
    local data, lines, y0 = {}, {}, 0
    local keys = {}
    for c = 8, 22 do keys[#keys + 1] = { c, SETS.bold } end
    for c = 8, 14 do keys[#keys + 1] = { 100 + c, SETS.reg } end
    for _, kk in ipairs(keys) do
        local key, S0 = kk[1], kk[2]
        local cap = key % 100
        local adv = math.max(1, math.floor(S0.pitch * cap / S0.cap + 0.5))
        local ob = old[key]
        if not ob then return false, "font_b612_px2.bin: size " .. key .. " missing" end
        local oAdv, oCw, oCh, oBase, oLeft, oX0, oY0, oCols = ob[2], ob[3], ob[4], ob[5], ob[6], ob[7], ob[8], ob[9]
        local left, cw = 2, adv + 4
        local topPad = math.max(oBase - cap, 2)
        local ch = math.max(oCh, topPad + cap + (oCh - oBase))
        local bas = topPad + cap
        local cols = math.floor((W - 8) / cw)
        local nrow = math.floor((96 + cols - 1) / cols)
        if y0 + nrow * ch > H - 16 then return false, "font atlas full" end
        local wx, wy = weights(S0.pitch, adv), weights(S0.cap, cap)
        for idx = 0, 95 do
            local code = (idx == 95) and 176 or (idx + 32)
            local up = (code >= 97 and code <= 122) and (code - 32) or code
            local cx, cy = 4 + (idx % cols) * cw, y0 + math.floor(idx / cols) * ch
            local pp = POS[up]
            if pp then
                local sx, sy = S0.x0 + pp[2] * S0.pitch, S0.tops[pp[1] + 1]
                for dy = 0, cap - 1 do
                    local row = (cy + topPad + dy) * W + cx + left
                    for dx = 0, adv - 1 do
                        local v = 0
                        for _, a in ipairs(wy[dy]) do
                            local yy = sy + a[1]
                            for _, b in ipairs(wx[dx]) do v = v + px(sx + b[1], yy) * a[2] * b[2] end
                        end
                        data[row + dx] = math.min(255, math.floor(v + 0.5))
                    end
                end
            elseif idx > 0 then
                local ox, oy = oX0 + (idx % oCols) * oCw, oY0 + math.floor(idx / oCols) * oCh
                local dyo, dxo = bas - oBase, left - oLeft + math.floor((adv - oAdv) / 2)
                for gy = 0, oCh - 1 do
                    local ty = cy + gy + dyo
                    if ty >= cy and ty < cy + ch then
                        for gx = 0, oCw - 1 do
                            local tx = cx + gx + dxo
                            if tx >= cx and tx < cx + cw then
                                data[ty * W + tx] = string.byte(od, dbase + (oy + gy) * oW + ox + gx)
                            end
                        end
                    end
                end
            end
        end
        lines[#lines + 1] = string.format("S %d %d %d %d %d %d 4 %d %d", key, adv, cw, ch, bas, left, y0, cols)
        y0 = y0 + nrow * ch + 2
    end
    for yy = 2036, 2044 do for xx = 1012, 1020 do data[yy * W + xx] = 255 end end
    local parts = { "BTVFONT2 " .. W .. " " .. H .. " " .. #lines .. " 32 96 1016.0 2040.0 0.960 ToLissA350DU(FF SymbolTex)Bold+Regular\n",
                    table.concat(lines, "\n"), "\nDATA\n" }
    local chunk = {}
    for k = 0, W * H - 1 do
        chunk[#chunk + 1] = data[k] or 0
        if #chunk == 4000 then parts[#parts + 1] = string.char(unpack(chunk)); chunk = {} end
    end
    if #chunk > 0 then parts[#parts + 1] = string.char(unpack(chunk)) end
    local blob = table.concat(parts)
    -- Pruefung: mit der bekannten Displayschrift (FF A350 1.7.4) muss genau der gepruefte Atlas entstehen - sonst
    -- lieber B612 als eine verdorbene Schrift
    local function hash(str)
        local x = 0
        for k = 1, #str do x = (x * 31 + string.byte(str, k)) % 4294967296 end
        return x
    end
    if #d == 151076 and hash(d) == 1957888551 and (#blob ~= 2097862 or hash(blob) ~= 194877638) then
        return false, "font check failed"
    end
    local fw = io.open(out, "wb")
    if not fw then return false, "cannot write " .. out end
    fw:write(blob); fw:close()
    return true, "built"
end

local function pnl_font_load()
    -- (einmalig: Schrift aus der Displayschrift des Flugzeugs erzeugen; sonst bleibt B612)
    local okb, okt, eb = pcall(TF.build_tls)
    if not (okb and okt) then pnl_log("display font from the aircraft: " .. tostring(okb and eb or okt) .. " - using B612")
    elseif eb == "built" then pnl_log("display font built from the aircraft (font_tls_px2.bin)") end
    -- v2: pixelgenaue Groessen (font_b612_px.bin)
    -- (font_b612_px2.bin: dazu B612 Mono Regular, Groessen 108..114 = Versalhoehe 8..14)
    -- (font_tls_px2.bin: gleiche Form, aus der Displayschrift des FF/ToLiss-A350 (SymbolTex.tlstex) erzeugt - wie
    --  die uebrigen Anzeigen im Cockpit; Zeichen, die diese Schrift nicht hat, aus B612. Fehlt sie: B612)
    local f2 = io.open(SCRIPTS .. "A350_BTV_data/font_tls_px2.bin", "rb") or io.open(SCRIPTS .. "A350_BTV_data/font_b612_px2.bin", "rb")
        or io.open(SCRIPTS .. "A350_BTV_data/font_b612_px.bin", "rb")
    if f2 then
        local t = {}
        for w in (f2:read("*l") or ""):gmatch("%S+") do t[#t + 1] = w end
        if t[1] == "BTVFONT2" and #t >= 9 then
            local W, H, ns = tonumber(t[2]), tonumber(t[3]), tonumber(t[4])
            TF.W, TF.H, TF.count = W, H, tonumber(t[6])
            TF.wu, TF.wv = tonumber(t[7]) / W, tonumber(t[8]) / H
            TF.by_cap, TF.cmin, TF.cmax = {}, 999, 0
            TF.by_cap_r, TF.rmin, TF.rmax = {}, nil, nil
            for _ = 1, ns do
                local a = {}
                for w in (f2:read("*l") or ""):gmatch("%S+") do a[#a + 1] = w end
                for k = 2, #a do a[k] = tonumber(a[k]) end
                local sz = { cap = a[2], adv = a[3], cw = a[4], ch = a[5], base = a[6], left = a[7], x0 = a[8], y0 = a[9], cols = a[10] }
                if sz.cap and sz.cols and sz.cap >= 100 then
                    local c = sz.cap - 100
                    TF.by_cap_r[c] = sz
                    TF.rmin, TF.rmax = math.min(TF.rmin or c, c), math.max(TF.rmax or c, c)
                elseif sz.cap and sz.cols then
                    TF.by_cap[sz.cap] = sz
                    TF.cmin, TF.cmax = math.min(TF.cmin, sz.cap), math.max(TF.cmax, sz.cap)
                end
            end
            local tag = f2:read("*l")
            local data = f2:read("*a")
            f2:close()
            local full = true
            for k = TF.cmin, TF.cmax do if not TF.by_cap[k] then full = false end end
            if tag == "DATA" and data and #data >= W * H and full then
                TF.data, TF.v, TF.levels, TF.pending = data, 2, 1, true
                return
            end
            pnl_log("font_b612_px.bin: file incomplete - trying font_b612.bin")
        else
            f2:close()
        end
    end
    local path = SCRIPTS .. "A350_BTV_data/font_b612.bin"
    local f = io.open(path, "rb")
    if not f then pnl_log("font_b612.bin not found - using the stroke font"); return end
    local hdr = f:read("*l") or ""
    local t = {}
    for w in hdr:gmatch("%S+") do t[#t + 1] = w end
    if t[1] ~= "BTVFONT1" or #t < 16 then f:close(); pnl_log("font_b612.bin: unknown format"); return end
    local W, H, nl = tonumber(t[2]), tonumber(t[3]), tonumber(t[4])
    TF.W, TF.H, TF.levels = W, H, nl
    TF.cw, TF.chh, TF.adv = tonumber(t[5]), tonumber(t[6]), tonumber(t[7])
    TF.cols, TF.count = tonumber(t[9]), tonumber(t[11])
    TF.cap_px, TF.pad, TF.cap_top = tonumber(t[14]), tonumber(t[15]), tonumber(t[16])
    TF.wu, TF.wv = tonumber(t[12]) / W, tonumber(t[13]) / H
    local data = f:read("*a")
    f:close()
    local need, w, h = 0, W, H
    for _ = 1, nl do
        need = need + w * h
        w, h = math.max(1, math.floor(w / 2)), math.max(1, math.floor(h / 2))
    end
    if not data or #data < need then pnl_log("font_b612.bin: file incomplete"); return end
    TF.data = data
    TF.pending = true
end

local function pnl_font_upload()
    TF.pending = false
    local ok, err = pcall(function()
        local ib = PNL.ib
        ib[0] = 0
        XP.btv_XPLMGenerateTextureNumbers(ib, 1)
        local id = ib[0]
        if id <= 0 then error("no texture number") end
        XP.btv_XPLMBindTexture2d(id, 0)
        GL.btv_glPushClientAttrib(GLC.CLIENT_PIXEL_STORE_BIT)
        local pbo = 0
        if PNL.glBindBuffer then
            GL.btv_glGetIntegerv(GLC.PIXEL_UNPACK_BUFFER_BINDING, ib); pbo = ib[0]
            if pbo ~= 0 then PNL.glBindBuffer(GLC.PIXEL_UNPACK_BUFFER, 0) end
        end
        GL.btv_glPixelStorei(0x0CF5, 1) -- UNPACK_ALIGNMENT
        GL.btv_glPixelStorei(0x0CF2, 0) -- UNPACK_ROW_LENGTH
        GL.btv_glPixelStorei(0x0CF3, 0) -- UNPACK_SKIP_ROWS
        GL.btv_glPixelStorei(0x0CF4, 0) -- UNPACK_SKIP_PIXELS
        local W, H = TF.W, TF.H
        local src = TF.data
        local sp = ffi.cast("const uint8_t *", src)
        local buf = ffi.new("uint8_t[?]", W * H * 4)
        local off = 0
        for l = 0, TF.levels - 1 do
            local n = W * H
            for k = 0, n - 1 do
                local j = 4 * k
                buf[j], buf[j + 1], buf[j + 2], buf[j + 3] = 255, 255, 255, sp[off + k]
            end
            GL.btv_glTexImage2D(GLC.TEXTURE_2D, l, GLC.RGBA8, W, H, 0, GLC.RGBA, GLC.UNSIGNED_BYTE, buf)
            off = off + n
            W, H = math.max(1, math.floor(W / 2)), math.max(1, math.floor(H / 2))
        end
        if pbo ~= 0 then PNL.glBindBuffer(GLC.PIXEL_UNPACK_BUFFER, pbo) end
        GL.btv_glPopClientAttrib()
        GL.btv_glTexParameteri(GLC.TEXTURE_2D, 0x2801, (TF.v == 2 or TF.levels == 1) and 0x2601 or 0x2703) -- MIN_FILTER (v2 ohne Mip-Stufen)
        GL.btv_glTexParameteri(GLC.TEXTURE_2D, 0x2800, 0x2601) -- MAG_FILTER = LINEAR
        GL.btv_glTexParameteri(GLC.TEXTURE_2D, 0x2802, 0x812F) -- WRAP_S = CLAMP_TO_EDGE
        GL.btv_glTexParameteri(GLC.TEXTURE_2D, 0x2803, 0x812F) -- WRAP_T
        GL.btv_glTexParameteri(GLC.TEXTURE_2D, 0x813D, TF.levels - 1) -- MAX_LEVEL
        TF.id = id
    end)
    TF.data = nil
    if ok then
        TF.ok = true
        GLB.wu, GLB.wv = TF.wu, TF.wv
        pnl_log("font " .. (TF.label or "B612 Mono") .. " loaded (" .. ((TF.v == 2) and string.format("pixel exact %d..%d px", TF.cmin, TF.cmax) or "mipmapped") ..
            ", texture " .. tostring(TF.id) .. ")")
    else
        pnl_log("font texture failed: " .. tostring(err) .. " - using the stroke font")
    end
end

-- Puffer v (x,y) / c (Farbe) mit n Eckpunkten zeichnen; alle fremden Zustaende werden gesichert und zurueckgesetzt
local function pnl_ext()
    local ib = PNL.ib
    if not PNL.ext then
        -- erst hier: Erweiterungen und Grenzen brauchen den OpenGL-Kontext der Zeichenphase
        PNL.ext = true
        PNL.glUseProgram = pnl_getproc("glUseProgram", "btv_u1_f")
        PNL.glBindVertexArray = (ffi.os == "OSX" and pnl_getproc("glBindVertexArrayAPPLE", "btv_u1_f"))
            or pnl_getproc("glBindVertexArray", "btv_u1_f")
        PNL.glBindBuffer = pnl_getproc("glBindBuffer", "btv_u2_f")
        PNL.glClientActiveTexture = pnl_getproc("glClientActiveTexture", "btv_u1_f")
        PNL.glDisableVertexAttribArray = pnl_getproc("glDisableVertexAttribArray", "btv_u1_f")
        ib[0] = 0; GL.btv_glGetIntegerv(GLC.MAX_TEXTURE_COORDS, ib)
        PNL.ntex = (ib[0] >= 1 and ib[0] <= 32) and ib[0] or 8
        ib[0] = 0; GL.btv_glGetIntegerv(GLC.MAX_VERTEX_ATTRIBS, ib)
        PNL.nattr = (ib[0] >= 1 and ib[0] <= 64) and ib[0] or 16
        local miss = {}
        for _, k in ipairs({ "glUseProgram", "glBindVertexArray", "glBindBuffer", "glClientActiveTexture", "glDisableVertexAttribArray" }) do
            if not PNL[k] then miss[#miss + 1] = k end
        end
        if #miss > 0 then pnl_log("OpenGL extensions missing: " .. table.concat(miss, ", ")) end
        PNL.glBindFramebuffer = pnl_getproc("glBindFramebuffer", "btv_u2_f")
    end
end

local function pnl_submit(v, c, n, t)
    PNL.submit_list({ { v = v, c = c, n = n, t = t } })
end

-- Geraetepixel je Panel-Einheit aus Viewport und Matrizen (fuer Linienbreite und weichen Rand)
local function pnl_dpu()
    local ib, fb = PNL.ib, PNL.fb
    GL.btv_glGetIntegerv(GLC.VIEWPORT, ib)
    local vw, vh = ib[2], ib[3]
    GL.btv_glGetFloatv(GLC.PROJECTION_MATRIX, fb)
    local p0, p1, p4, p5 = fb[0], fb[1], fb[4], fb[5]
    GL.btv_glGetFloatv(GLC.MODELVIEW_MATRIX, fb)
    local m0, m1 = fb[0], fb[1]
    local ex, ey = p0 * m0 + p4 * m1, p1 * m0 + p5 * m1 -- Bild einer Panel-Einheit in x (NDC)
    local d = math.sqrt((ex * vw / 2) ^ 2 + (ey * vh / 2) ^ 2)
    return d, vw, vh
end

-- Panel-Punkt -> Geraetepixel (fuer das Zuruecklesen)
local function pnl_devxy(px, py)
    local ib, fb = PNL.ib, PNL.fb
    GL.btv_glGetIntegerv(GLC.VIEWPORT, ib)
    local vx, vy, vw, vh = ib[0], ib[1], ib[2], ib[3]
    GL.btv_glGetFloatv(GLC.MODELVIEW_MATRIX, fb)
    local ex = fb[0] * px + fb[4] * py + fb[12]
    local ey = fb[1] * px + fb[5] * py + fb[13]
    local ez = fb[2] * px + fb[6] * py + fb[14]
    local ew = fb[3] * px + fb[7] * py + fb[15]
    GL.btv_glGetFloatv(GLC.PROJECTION_MATRIX, fb)
    local cx = fb[0] * ex + fb[4] * ey + fb[8] * ez + fb[12] * ew
    local cy = fb[1] * ex + fb[5] * ey + fb[9] * ez + fb[13] * ew
    local cw = fb[3] * ex + fb[7] * ey + fb[11] * ez + fb[15] * ew
    if math.abs(cw) < 1e-9 then return nil end
    return math.floor(vx + (cx / cw * 0.5 + 0.5) * vw), math.floor(vy + (cy / cw * 0.5 + 0.5) * vh)
end

-- ein Pixel des gerade gezeichneten Panels lesen (nur waehrend der Reihenfolge-Pruefung, wenige Bilder lang)
local function pnl_read_pixel(px, py)
    pnl_ext()
    local dx, dy = pnl_devxy(px, py)
    if not dx then return nil end
    local ib = PNL.ib
    local rfb, dfb, pbo = 0, 0, 0
    if PNL.glBindFramebuffer then
        GL.btv_glGetIntegerv(GLC.READ_FRAMEBUFFER_BINDING, ib); rfb = ib[0]
        GL.btv_glGetIntegerv(GLC.DRAW_FRAMEBUFFER_BINDING, ib); dfb = ib[0]
    end
    if PNL.glBindBuffer then GL.btv_glGetIntegerv(GLC.PIXEL_PACK_BUFFER_BINDING, ib); pbo = ib[0] end
    if rfb ~= dfb then PNL.glBindFramebuffer(GLC.READ_FRAMEBUFFER, dfb) end
    if pbo ~= 0 then PNL.glBindBuffer(GLC.PIXEL_PACK_BUFFER, 0) end
    local px4 = PNL.px4
    px4[0], px4[1], px4[2], px4[3] = 0, 0, 0, 0
    GL.btv_glReadPixels(dx, dy, 1, 1, GLC.RGBA, GLC.UNSIGNED_BYTE, px4)
    if pbo ~= 0 then PNL.glBindBuffer(GLC.PIXEL_PACK_BUFFER, pbo) end
    if rfb ~= dfb then PNL.glBindFramebuffer(GLC.READ_FRAMEBUFFER, rfb) end
    return px4[0], px4[1], px4[2]
end

-------------------------------------------------------------------------------
-- Was zeigt das FF gerade? (nach dem FF, vor der Karte gelesen - ohne die Grafikkarte warten zu lassen:
-- glReadPixels in einen Pixelpuffer, ein paar Bilder spaeter auslesen, wenn die Fence erreicht ist)
--   ND: "GS" oben links weiss = Bildschirm an; Kompassring rot (HDG-Flagge) = IRS nicht ausgerichtet / kein Heading
--   PFD (Kapitaen): cyane Schrift in FMA-Spalte 1, Zeile 2 = Autobrake gesetzt ("BRK LO/2/3/4/HI")
-------------------------------------------------------------------------------
PNL.ND_STATE_TXT = { ok = "on", flag = "heading flag (IRS not aligned)", off = "off" }
function PNL.async_init()
    if PNL.as_ok ~= nil then return PNL.as_ok end
    PNL.as_ok = false
    local g = {
        gen = pnl_getproc("glGenBuffers", "btv_genbuf_f"), data = pnl_getproc("glBufferData", "btv_bufdata_f"),
        map = pnl_getproc("glMapBufferRange", "btv_mapbuf_f"), unmap = pnl_getproc("glUnmapBuffer", "btv_unmap_f"),
        fence = pnl_getproc("glFenceSync", "btv_fence_f"), wait = pnl_getproc("glClientWaitSync", "btv_wait_f"),
        del = pnl_getproc("glDeleteSync", "btv_delsync_f"),
    }
    for _, k in ipairs({ "gen", "data", "map", "unmap", "fence", "wait", "del" }) do
        if not g[k] then pnl_log("display check off (OpenGL pixel buffers not available)"); return false end
    end
    if not PNL.glBindBuffer then return false end
    local id = ffi.new("unsigned int[1]")
    g.gen(1, id)
    if id[0] == 0 then return false end
    PNL.as_g, PNL.as_buf, PNL.as_size = g, id[0], 0
    PNL.as_ok = true
    return true
end

-- Pixel klassifizieren: weiss, rot, cyan
local function as_count(b, off, n)
    local nw, nr, nc = 0, 0, 0
    for i = 0, n - 1 do
        local k = off + 4 * i
        local r, g, bb = b[k], b[k + 1], b[k + 2]
        local mx = math.max(r, g, bb)
        if mx > 35 then -- (auch bei heruntergedrehter Display-Helligkeit)
            local mn = math.min(r, g, bb)
            if mn > mx * 0.6 then nw = nw + 1
            elseif g < r * 0.45 and bb < r * 0.45 then nr = nr + 1
            elseif r < math.min(g, bb) * 0.6 and g > 40 and bb > 40 then nc = nc + 1 end
        end
    end
    return nw, nr, nc
end

-- Auftrag: Bereiche (Geraetepixel) in den Pixelpuffer lesen
function PNL.sample_req(pl, pb, pr, pt)
    if PNL.as_pend or not PNL.async_init() then return end
    local g = PNL.as_g
    local regs, total = {}, 0
    local function add(uv, lx, ly, hw, hh, kind, side)
        local px = pl + (uv[1] + (uv[3] - uv[1]) * lx / ND_VW) * (pr - pl)
        local py = pb + (uv[4] - (uv[4] - uv[2]) * ly / ND_VH) * (pt - pb)
        local dx, dy = pnl_devxy(px, py)
        if not dx then return end
        dx, dy = math.floor(dx - hw + 0.5), math.floor(dy - hh + 0.5)
        if dx < 0 or dy < 0 then return end
        local w, h = 2 * hw + 1, 2 * hh + 1
        regs[#regs + 1] = { dx = dx, dy = dy, w = w, h = h, off = total, kind = kind, side = side }
        total = total + w * h * 4
    end
    for s = 0, 1 do
        local sd = PNL.sd[s]
        if sd.show then
            local uv = ND_UV[s]
            add(uv, 7.0, 4.5, 4, 4, "gs", s)
            local mode = EFIS.mode[s] or "arc"
            local gm = EFIS.GFF[mode]
            for _, a in ipairs((mode == "arc") and { -48, -30, 30, 48 } or { 45, 135, 225, 315 }) do
                local t = a * D2R
                add(uv, gm[1] + gm[3] * math.sin(t), gm[2] - gm[3] * math.cos(t), 4, 4, "ring", s)
            end
            -- PFD derselben Seite: Kursband unten, rote "HDG"-Flagge (aus dem PFD-Bild des FF vermessen)
            add(PFD.uv[s], 131, 283.5, 50, 12, "hdg", s)
        end
    end
    add(PFD.uv[0], 31.9, 23.9, 44, 9, "fma", 0)
    -- ATCCOM: is FF's display lit? MFD: FF's system key text (top left, never covered); upper centre display: FF's
    -- divider line just left of the mailbox
    local A = PNL.atc
    for _, w in ipairs({ { "lower", A.page or A.initkey, 60, 16, 25, 6 }, { "upper", A.mail, 0, 120, 2, 12 } }) do
        local r = A.regions[w[1]]
        if w[2] and r then
            add({ r[1] / 2048, r[2] / 2048, r[3] / 2048, r[4] / 2048 }, w[3] * ND_VW / r[5], w[4] * ND_VH / r[6], w[5], w[6], "atc_" .. w[1], 0)
        end
    end
    -- die beiden Plaetze von "BTV" in der FMA (Zeile 1 = bremst, Zeile 2 = scharf): zeigt das FF dort selbst etwas
    -- (z.B. MAN TOGA im Kasten), hat das Vorrang - im FMA ueberlagern sich Anzeigen nie
    add(PFD.uv[0], PFD.fma_x, PFD.fma_base1 - PFD.fma_cap / 2, 16, 6, "slot1", 0)
    add(PFD.uv[0], PFD.fma_x, PFD.fma_base - PFD.fma_cap / 2, 16, 6, "slot2", 0)
    if #regs == 0 then return end
    local ib = PNL.ib
    local rfb, dfb, pbo = 0, 0, 0
    if PNL.glBindFramebuffer then
        GL.btv_glGetIntegerv(GLC.READ_FRAMEBUFFER_BINDING, ib); rfb = ib[0]
        GL.btv_glGetIntegerv(GLC.DRAW_FRAMEBUFFER_BINDING, ib); dfb = ib[0]
    end
    GL.btv_glGetIntegerv(GLC.PIXEL_PACK_BUFFER_BINDING, ib); pbo = ib[0]
    if rfb ~= dfb then PNL.glBindFramebuffer(GLC.READ_FRAMEBUFFER, dfb) end
    PNL.glBindBuffer(GLC.PIXEL_PACK_BUFFER, PNL.as_buf)
    if PNL.as_size < total then
        g.data(GLC.PIXEL_PACK_BUFFER, total, nil, 0x88E1) -- STREAM_READ
        PNL.as_size = total
    end
    for _, r in ipairs(regs) do
        GL.btv_glReadPixels(r.dx, r.dy, r.w, r.h, GLC.RGBA, GLC.UNSIGNED_BYTE, ffi.cast("void *", r.off))
    end
    local fence = g.fence(0x9117, 0) -- SYNC_GPU_COMMANDS_COMPLETE
    PNL.glBindBuffer(GLC.PIXEL_PACK_BUFFER, pbo)
    if rfb ~= dfb then PNL.glBindFramebuffer(GLC.READ_FRAMEBUFFER, rfb) end
    if fence == nil then return end
    PNL.as_pend = { regs = regs, total = total, fence = fence, frame = PNL.frame, first = true }
end

-- spaeter: fertig? Dann auslesen und auswerten (kein Warten - sonst naechstes Bild erneut)
function PNL.sample_poll()
    local p = PNL.as_pend
    if not p then return end
    local g = PNL.as_g
    local r = g.wait(p.fence, p.first and 1 or 0, 0) -- beim ersten Mal die Befehle abschicken (FLUSH)
    p.first = false
    if r == 0x911B then -- TIMEOUT_EXPIRED: noch nicht fertig
        if PNL.frame - p.frame > 120 then g.del(p.fence); PNL.as_pend = nil end
        return
    end
    g.del(p.fence)
    PNL.as_pend = nil
    if r ~= 0x911A and r ~= 0x911C then return end -- ALREADY_SIGNALED / CONDITION_SATISFIED
    local ib = PNL.ib
    GL.btv_glGetIntegerv(GLC.PIXEL_PACK_BUFFER_BINDING, ib)
    local pbo = ib[0]
    PNL.glBindBuffer(GLC.PIXEL_PACK_BUFFER, PNL.as_buf)
    local ptr = g.map(GLC.PIXEL_PACK_BUFFER, 0, p.total, 0x0001) -- MAP_READ_BIT
    if ptr ~= nil then
        if not PNL.sbuf or PNL.sbuf_n < p.total then PNL.sbuf, PNL.sbuf_n = ffi.new("uint8_t[?]", p.total), p.total end
        ffi.copy(PNL.sbuf, ptr, p.total)
        g.unmap(GLC.PIXEL_PACK_BUFFER)
    end
    PNL.glBindBuffer(GLC.PIXEL_PACK_BUFFER, pbo)
    if ptr == nil then return end
    -- auswerten
    local b = PNL.sbuf
    local gs, reds, fma = { [0] = 0, [1] = 0 }, { [0] = 0, [1] = 0 }, nil
    local hdg = { [0] = 0, [1] = 0 }
    local seen = {}
    local slot = {}
    for _, q in ipairs(p.regs) do
        local nw, nr, nc = as_count(b, q.off, q.w * q.h)
        if q.kind == "slot1" or q.kind == "slot2" then
            -- beliebige helle Pixel (weiss, gruen, cyan, amber ...) des FF an diesem Platz
            local n = 0
            for i = 0, q.w * q.h - 1 do
                local k = q.off + 4 * i
                if math.max(b[k], b[k + 1], b[k + 2]) > 60 then n = n + 1 end
            end
            slot[(q.kind == "slot1") and 1 or 2] = n >= 10
        elseif q.kind == "atc_lower" or q.kind == "atc_upper" then
            -- any lit pixel of FF's (also with the display brightness turned down); twice the same before it counts
            local n = 0
            for i = 0, q.w * q.h - 1 do
                local k = q.off + 4 * i
                if math.max(b[k], b[k + 1], b[k + 2]) > 35 then n = n + 1 end
            end
            local which, A = q.kind:sub(5), PNL.atc
            local lit = n >= 6
            if lit == A.lit_new[which] and A.lit[which] ~= lit then
                A.lit[which], A.cache = lit, {}
                pnl_log("ATCCOM: FF's " .. (which == "lower" and "MFD" or "upper centre display") .. (lit and " lit" or " dark - nothing of ours drawn"))
            end
            A.lit_new[which] = lit
        elseif q.kind == "gs" then gs[q.side] = nw; seen[q.side] = true
        elseif q.kind == "ring" then if nr >= 3 then reds[q.side] = reds[q.side] + 1 end
        elseif q.kind == "hdg" then hdg[q.side] = nr
        else fma = nc end
    end
    for s = 0, 1 do
        local sd = PNL.sd[s]
        if seen[s] then
            local st = (reds[s] >= 2 or hdg[s] >= 15) and "flag" or ((gs[s] >= 4) and "ok" or "off")
            -- zweimal gleich (bzw. beim ersten Mal sofort) -> uebernehmen
            if sd.ffst == nil or st == sd.st_new then
                if sd.ffst ~= st then
                    pnl_log(string.format("FF ND %s: %s", (s == 0) and "CAPT" or "F/O", PNL.ND_STATE_TXT[st]))
                end
                sd.ffst = st
            end
            sd.st_new = st
        end
    end
    -- belegt erst nach zwei gleichen Messungen (wie oben), damit nichts flackert
    PNL.fma_busy, PNL.fma_new = PNL.fma_busy or {}, PNL.fma_new or {}
    for k = 1, 2 do
        if slot[k] ~= nil then
            if slot[k] == PNL.fma_new[k] then PNL.fma_busy[k] = slot[k] end
            PNL.fma_new[k] = slot[k]
        end
    end
    if fma then
        local ab = fma >= 15
        if ab == PNL.ab_new or S.abrk == nil then
            if S.abrk ~= ab then pnl_log("FF auto brake " .. (ab and "set" or "off")) end
            if S.abrk == true and not ab and S.td_t and DR.time[0] - S.td_t < 180 and main_gear_on_ground() then
                say("AUTO BRAKE OFF")
            end
            S.abrk = ab
        end
        PNL.ab_new = ab
    end
end

-- Reihenfolge-Pruefung: Wer zeichnet nach dem FF-ND? FlyWithLua (frueh) setzt ein kleines Testkaestchen ins ND;
-- jede Bruecke liest es zurueck. Ist es uebermalt, kam das FF-ND dazwischen -> diese Bruecke ist spaet genug.
local MARK = { { 0xFFFF00FF, 255, 0, 255 }, { 0xFF80FF00, 0, 255, 128 } }
local function pnl_mark(pl, pb, pr, pt)
    local any = PNL.sd[0].show or PNL.sd[1].show
    for s = 0, 1 do
        local sd = PNL.sd[s]
        if sd.show or (s == 0 and not any) then
            local uv = ND_UV[s]
            local x0, x1 = pl + uv[1] * (pr - pl), pl + uv[3] * (pr - pl)
            local y1 = pb + uv[4] * (pt - pb)
            local cx, cy = x0 + (x1 - x0) * 0.2, y1 - (pt - pb) * 0.006
            local mk = MARK[(PNL.frame % 2) + 1]
            local v, c = PNL.av, PNL.ac
            v[0], v[1], v[2], v[3], v[4], v[5] = cx - 2, cy - 2, cx + 2, cy - 2, cx + 2, cy + 2
            v[6], v[7], v[8], v[9], v[10], v[11] = cx - 2, cy - 2, cx + 2, cy + 2, cx - 2, cy + 2
            for k = 0, 5 do c[k] = mk[1] end
            pnl_submit(v, c, 6)
            PNL.mk_frame, PNL.mk_x, PNL.mk_y, PNL.mk = PNL.frame, cx, cy, mk
            return
        end
    end
end

local function pnl_probe(br)
    if PNL.mk_frame ~= PNL.frame or br.probed == PNL.frame then return end
    br.probed = PNL.frame
    local r, g, b = pnl_read_pixel(PNL.mk_x, PNL.mk_y)
    if not r then return end
    local mk = PNL.mk
    local same = math.abs(r - mk[2]) < 12 and math.abs(g - mk[3]) < 12 and math.abs(b - mk[4]) < 12
    br.n = (br.n or 0) + 1
    if not same then br.late = (br.late or 0) + 1 end
end

-- nach der Pruefung: Entscheidung
local function pnl_decide()
    -- eine Bruecke, die das Testkaestchen auch nur ein paar Mal uebermalt sieht, liegt nach dem FF-ND (eine Bruecke
    -- davor kann es nie uebermalt sehen); sieht es keine, zeichnet das FF-ND evtl. noch nicht (Start, Strom aus)
    local done, best = true, nil
    for _, br in ipairs(PNL.bridges) do
        if br.ok then
            if (br.n or 0) < 12 then done = false
            elseif (br.late or 0) >= 3 and (not best or br.late > best.late) then best = br end
        end
    end
    local waited = PNL.frame - (PNL.det_start or PNL.frame)
    if (best and (done or waited > 16)) or done or waited > 300 then
        local t = {}
        for _, br in ipairs(PNL.bridges) do
            t[#t + 1] = string.format("%s: %s (%d/%d overdrawn)", br.name, (not br.ok) and "not loaded" or
                ((br == best) and "AFTER the FF ND" or "before the FF ND"), br.late or 0, br.n or 0)
        end
        if best then
            pnl_log("draw order: " .. table.concat(t, ", "))
            PNL.hook, PNL.fails = best, 0
            PNL.st = "ready (draws via " .. best.name .. ")"
        else
            PNL.hook = false
            PNL.fails = (PNL.fails or 0) + 1
            -- bald erneut pruefen (2 s, dann seltener bis alle 10 s) - kein Neuladen der Skripte noetig
            PNL.retry = PNL.frame + math.min(120 * PNL.fails, 600)
            local any_ok = false
            for _, br in ipairs(PNL.bridges) do if br.ok and (br.n or 0) > 0 then any_ok = true end end
            PNL.st = any_ok and "waiting for the FF ND to draw (display off?) - retrying"
                or "cannot draw over the FF ND - helper plugin not active (restart X-Plane)"
            if PNL.fails == 1 or PNL.fails % 30 == 0 then
                pnl_log("draw order: " .. table.concat(t, ", "))
                pnl_log(PNL.st)
            end
        end
    end
end


-------------------------------------------------------------------------------
-- ND in Schichten (ruhige Bildraten):
--   Karte: nur bei Bedarf neu aufgebaut - sofort bei harten Wechseln (Flughafen, EFIS-Modus, Reichweite, Bahn),
--     sonst verteilt auf mehrere Bilder (Schritte, ~1,5 ms je Bild); dazwischen bewegt eine Matrix das letzte
--     Bild auf die aktuelle Ansicht (Drehen, Folgen, Zoom), das Scherenrechteck haelt es im ND
--   Overlay (Kompass, Kopfzeilen, Tasten, Flugzeug, Maus): nur, wenn sich etwas aendert
-------------------------------------------------------------------------------
PNL.MAP_CAP, PNL.OVL_CAP, PNL.PFD_CAP = 260000, 40000, 6000
-- Ersatz fuer ImGui waehrend des Zeichnens ins Cockpit: jeder Aufruf tut nichts (statt X-Plane abstuerzen zu lassen)
PNL.NOIMGUI = setmetatable({}, { __index = function() return function() return nil end end })
PNL.budget, PNL.budget_fade = 0.0015, 0.003
PNL.stat = { sync = 0, jobs = 0, ovl = 0, ms = 0, frames = 0, maxms = 0 }

-- Absturz-Sicherung: in den ersten Bildern mit Karte liegt A350_BTV_drawcheck.txt im Scripts-Ordner (beim
-- normalen Beenden / Neuladen wird sie geloescht). Findet der naechste Start sie noch vor, ist X-Plane beim
-- Zeichnen abgestuerzt: dann wird das Zeichnen in Schichten ausgeschaltet (wie v2.8), nach einem Absturz auch
-- ohne Schichten die OANS-Karte. Beides laesst sich im Einstellungsfenster wieder einschalten.
PNL.GUARD = SCRIPTS .. "A350_BTV_drawcheck.txt"
function PNL.guard_init()
    local f = io.open(PNL.GUARD, "r")
    if f then
        local t = (f:read("*l") or ""):gsub("%s", "")
        f:close()
        os.remove(PNL.GUARD)
        if t == "layers" then
            CFG.nd_layers = false
            save_config()
            pnl_log("X-Plane crashed while drawing the ND map last time - layered drawing switched off (settings window)")
        elseif t == "safe" then
            CFG.nd_on = false
            save_config()
            pnl_log("X-Plane crashed while drawing the ND map last time (also without layers) - OANS map switched off (settings window)")
        end
    end
    if not CFG.nd_layers then PNL.nomat, MAPX.ND_M = true, 0 end
end
function PNL.guard_draw()
    local d = PNL.dcount or 0
    if d == 1 then
        local f = io.open(PNL.GUARD, "w")
        if f then f:write(PNL.nomat and "safe\n" or "layers\n"); f:close() end
    elseif d == 120 then
        os.remove(PNL.GUARD) -- (120 Bilder ohne Absturz)
    end
end

-- genaue Uhr (Sekunden) fuer das Zeitbudget; os.clock ist unter Linux die CPU-Zeit aller Threads
function PNL.clock_init()
    PNL.now = os.clock
    pcall(function()
        local now
        if ffi.os == "Windows" then
            pcall(ffi.cdef, [[ int btv_qpc(int64_t *) __asm__("QueryPerformanceCounter");
                               int btv_qpf(int64_t *) __asm__("QueryPerformanceFrequency"); ]])
            local k = ffi.load("kernel32")
            local b = ffi.new("int64_t[1]")
            assert(k.btv_qpf(b) ~= 0)
            local f = tonumber(b[0])
            assert(f and f > 0)
            now = function() k.btv_qpc(b); return tonumber(b[0]) / f end
        else
            pcall(ffi.cdef, [[ typedef struct { long s; long ns; } btv_tspec;
                               int btv_cgt(int, btv_tspec *) __asm__("clock_gettime"); ]])
            local t = ffi.new("btv_tspec")
            local id = (ffi.os == "OSX") and 6 or 1 -- CLOCK_MONOTONIC
            assert(ffi.C.btv_cgt(id, t) == 0)
            now = function() ffi.C.btv_cgt(id, t); return tonumber(t.s) + tonumber(t.ns) * 1e-9 end
        end
        local a = now()
        assert(type(a) == "number" and a > 0)
        PNL.now = now
    end)
end

-- Puffer einer ND-Seite (in der Bild-Schleife angelegt, nicht im Zeichen-Aufruf)
function PNL.alloc_side(sd)
    if sd.mbuf then return true end
    local ok, err = pcall(function()
        local a, b, o = PNL.newbuf(PNL.MAP_CAP), PNL.newbuf(PNL.MAP_CAP), sd.ob or PNL.newbuf(PNL.OVL_CAP)
        sd.mbuf, sd.ob = { a, b }, o
    end)
    if not ok then pnl_log("map buffers: " .. tostring(err)) end
    return ok
end

function PNL.trace(msg)
    if (PNL.dcount or 0) <= 2 then pnl_log("draw " .. tostring(PNL.dcount) .. ": " .. msg) end
end

function PNL.newbuf(cap)
    return { cap = cap, n = 0, v = ffi.new("float[?]", 2 * cap), c = ffi.new("uint32_t[?]", cap), t = ffi.new("float[?]", 2 * cap) }
end
function PNL.use(b)
    GLB.v, GLB.c, GLB.t, GLB.cap, GLB.n = b.v, b.c, b.t, b.cap, 0
end
function PNL.use0()
    local b = PNL.gb0
    if b then GLB.v, GLB.c, GLB.t, GLB.cap, GLB.n = b.v, b.c, b.t, b.cap, 0 end
end
function PNL.ovl_begin(sd, geo, fa)
    PNL.use(sd.ob)
    DL.begin(geo.H, ND_VW, ND_VH)
    GLB.ts, GLB.fa, DL.n = geo.ts, fa, 0
end
function PNL.ovl_end(sd)
    sd.ob.n = GLB.n
    GLB.fa, DL.H = nil, nil
    PNL.use0()
end

-- Lage einer ND-Seite im Panel: Abbildung lokal -> Panel, Rand, Scherenrechteck (Geraetepixel)
function PNL.geo(s, pl, pb, pr, pt)
    local uv, pd = ND_UV[s], ND_PAD[s]
    local x0, x1 = pl + uv[1] * (pr - pl), pl + uv[3] * (pr - pl)
    local y0, y1 = pb + uv[2] * (pt - pb), pb + uv[4] * (pt - pb)
    local sx, sy = (x1 - x0) / ND_VW, (y1 - y0) / ND_VH
    local g = { H = { sx, 0, x0, 0, -sy, y1, 0, 0 }, ts = (sx + sy) / 2,
                pad = { x0 - pd[1], y0 - pd[2], x1 + pd[3], y1 + pd[4] },
                key = string.format("%.1f/%.1f/%.1f/%.1f", x0, y0, x1, y1) }
    local ax, ay = pnl_devxy(g.pad[1], g.pad[2])
    local bx, by = pnl_devxy(g.pad[3], g.pad[4])
    if ax and bx then
        local lx, ly = math.min(ax, bx), math.min(ay, by)
        g.scis = { lx, ly, math.max(ax, bx) - lx + 1, math.max(ay, by) - ly + 1 }
    end
    return g
end

-- harte Wechsel: sofort neu aufbauen (ein Reichweitenwechsel nicht: er zoomt stetig, siehe MAPX.view_nd; den
-- Massstab prueft PNL.nd_map selbst)
function PNL.map_hard(apt, Cv, geo)
    return table.concat({ apt.icao, tostring(apt.geo), Cv.mode, Cv.vk or "", tostring(Cv.btv), tostring(Cv.appr),
        geo.key, tostring(TF.ok), S.rwy and S.rwy.id or "-" }, "|")
end
-- weiche Wechsel (Exits, BTV-Werte, Kreuze ...): im Hintergrund neu aufbauen
function PNL.map_sig(apt, Cv, V, fa)
    local ck = 0
    local mk = S.marks and S.marks[apt.icao]
    if mk then
        for _, c in ipairs(mk.crosses) do ck = ck + c[1] * 3.1 + c[2] * 1.7 + 1 end
        for _, c in ipairs(mk.flags) do ck = ck + c[1] * 5.3 + c[2] * 7.9 + 2 end
    end
    local fe = FMS.rwy_end(apt)
    local ts, tname, tx = current_target()
    return table.concat({ tostring(tname), tostring(ts and math.floor(ts)), tostring(tx ~= nil), tostring(S.rec_idx), S.mode,
        string.format("%.1f", math.floor((S.v_est or 0) * 2 + 0.5) / 2), tostring(S.exits), fe and fe.id or "-",
        tostring(RWYA.text), tostring(V.ui.panel), tostring(btv_ui()), tostring(apt.geo.osm), string.format("%.1f", ck),
        string.format("%.4f", GLB.px), fa and tostring(math.floor(fa * 10)) or "-", PNL.row_sig(), tostring(S.rop),
        tostring(S.cond_txt) }, "|")
end
-- DRY/WET in Echtzeit (unter 500 ft / am Boden): auf 10 m gerundet in die Signatur der Karte
function PNL.row_sig()
    local E = S.rwy
    if not CFG.rops or not (E and (btv_ui() or S.row_on)) or S.mode == "ENGAGED" or S.mode == "RELEASE" then return "-" end
    local lines = S.row_lines(E)
    local t = { S.row_rt and "r" or "p" }
    for _, l in ipairs(lines) do t[#t + 1] = l[1] .. (S.row_rt and math.floor(l[2] / 10) or "") end
    return table.concat(t, "/")
end
-- Overlay neu zeichnen, wenn sich das aendert (sonst spaetestens alle 10 Bilder)
function PNL.ovl_sig(sd, Cv, V, MV, fa)
    local px, py = Cv.XY(Cv.acx, Cv.acy)
    local ui, kb = V.ui, NDUI.kb
    local ts, tname = current_target()
    -- (Flugzeugsymbol auf 0,25 Einheiten = 0,4 Panel-Pixel genau)
    return table.concat({ Cv.mode, Cv.vk or "", string.format("%.1f/%d/%d/%d/%.5f", Cv.hv, math.floor(px * 4 + 0.5),
        math.floor(py * 4 + 0.5), math.floor(DR.psi[0] * 5 + 0.5), Cv.sc),
        tostring(math.floor(DR.gs[0] / KT + 0.5)), tostring(math.floor(DR.tas[0] / KT + 0.5)),
        S.mode, tostring(S.arm_request), tostring(S.abrk), tostring(AC_READY), S.rwy and S.rwy.id or "-",
        tostring(tname), tostring(ts and math.floor(ts)), tostring(S.rec_idx),
        tostring(ui.panel), tostring(ui.tab), tostring(ui.dd), tostring(MV.ctx), tostring(ui.msel), tostring(ui.asel),
        sd.hin and string.format("%.2f/%.2f", sd.hx, sd.hy) or "-", tostring(RWYA.text),
        RWYA.text and tostring(math.floor(DR.time[0] * 2) % 2) or "",
        fa and tostring(math.floor(fa * 60)) or "-", NDUI.trf.show[sd.side] and NDUI.trf.sig or "",
        kb and (kb.side .. kb.field .. kb.text .. (math.floor(NDUI.blink * 2) % 2)) or "" }, "|")
end

-- Aufbau-Auftrag: Karte der Ansicht Cv (mit Rand) in den Puffer buf
function PNL.job_new(sd, Cv, apt, V, geo, buf, hard, soft, fa)
    local m = MAPX.ND_M
    local Cb = MAPX.nd_build_ctx(Cv, apt, sd)
    local H = geo.H
    local j = { buf = buf, C = Cb, hard = hard, soft = soft, fa = fa, n = 0, dln = 0, px = GLB.px, ts = geo.ts,
                Hb = { H[1], H[2], H[3] - m * H[1] - m * H[2], H[4], H[5], H[6] - m * H[4] - m * H[5], 0, 0 },
                W = ND_VW + 2 * m, Hh = ND_VH + 2 * m }
    j.steps, j.i = MAPX.nd_build_steps(Cb, V), 1
    PNL.stat.jobs = PNL.stat.jobs + 1
    return j
end
-- weiterbauen (budget in s; nil = bis zum Ende). Rueckgabe true = fertig
function PNL.job_run(sd, j, budget)
    local o_v, o_c, o_t, o_cap, o_n = GLB.v, GLB.c, GLB.t, GLB.cap, GLB.n
    local o_px, o_ts, o_fa = GLB.px, GLB.ts, GLB.fa
    local o_H, o_W, o_Hh, o_clip, o_dn = DL.H, DL.W, DL.Hh, DL.clip, DL.n
    local o_ppx, o_ppu, o_bud = TF.ppx, TF.ppu, MAPX.bud
    local b = j.buf
    GLB.v, GLB.c, GLB.t, GLB.cap, GLB.n = b.v, b.c, b.t, b.cap, j.n
    GLB.px, GLB.ts, GLB.fa = j.px, j.ts, j.fa
    DL.begin(j.Hb, j.W, j.Hh)
    DL.n = j.dln
    MAPX.bud = budget and { k = 0, deadline = PNL.now() + budget, clk = PNL.now } or nil
    local ok, err = pcall(PNL.job_step, j)
    j.n, j.dln = GLB.n, DL.n
    GLB.v, GLB.c, GLB.t, GLB.cap, GLB.n = o_v, o_c, o_t, o_cap, o_n
    GLB.px, GLB.ts, GLB.fa = o_px, o_ts, o_fa
    DL.H, DL.W, DL.Hh, DL.clip, DL.n = o_H, o_W, o_Hh, o_clip, o_dn
    TF.ppx, TF.ppu, MAPX.bud = o_ppx, o_ppu, o_bud
    if not ok then sd.job = nil; error(err, 0) end
    return j.i > #j.steps
end
-- Schritte bis zum Ende des Zeitbudgets (ein unterbrochener Schritt macht beim naechsten Mal weiter)
function PNL.job_step(j)
    local st = j.steps
    while j.i <= #st do
        if not st[j.i]() then return end
        j.i = j.i + 1
        if MAPX.late() then return end
    end
end
function PNL.job_done(sd)
    local j = sd.job
    sd.job = nil
    j.buf.n = j.n
    local Cb = j.C
    sd.mb = { buf = j.buf, C = Cb, hard = j.hard, soft = j.soft, Hb = j.Hb, marks = Cb.marks or {}, hits = Cb.hit }
    return sd.mb
end

-- Matrix Aufbau-Puffer (Panel) -> aktuelle Ansicht (Panel): H_jetzt * D * Hb^-1, spaltenweise fuer OpenGL
function PNL.nd_matrix(sd, mb, D, H)
    local B = mb.Hb
    local a, b, c, d, e, f = B[1], B[2], B[3], B[4], B[5], B[6]
    local det = a * e - b * d
    local i11, i12, i13 = e / det, -b / det, (b * f - e * c) / det
    local i21, i22, i23 = -d / det, a / det, (d * c - a * f) / det
    local k11, k12 = D[1] * i11 + D[2] * i21, D[1] * i12 + D[2] * i22
    local k13 = D[1] * i13 + D[2] * i23 + D[5]
    local k21, k22 = D[3] * i11 + D[4] * i21, D[3] * i12 + D[4] * i22
    local k23 = D[3] * i13 + D[4] * i23 + D[6]
    local m = sd.matf
    if not m then m = ffi.new("float[16]"); sd.matf = m end
    m[0], m[1], m[2], m[3] = H[1] * k11 + H[2] * k21, H[4] * k11 + H[5] * k21, 0, 0
    m[4], m[5], m[6], m[7] = H[1] * k12 + H[2] * k22, H[4] * k12 + H[5] * k22, 0, 0
    m[8], m[9], m[10], m[11] = 0, 0, 1, 0
    m[12], m[13], m[14], m[15] = H[1] * k13 + H[2] * k23 + H[3], H[4] * k13 + H[5] * k23 + H[6], 0, 1
    return m
end

-- Karte und Overlay einer Seite mit Flughafen
function PNL.nd_map(sd, apt, V, inp, geo, fa)
    local MV = sd.mv
    local Cv = MAPX.nd_view(apt, V, MV, inp, sd)
    if not sd.mbuf and not PNL.alloc_side(sd) then error("no memory for the map buffers", 0) end
    local hard = PNL.map_hard(apt, Cv, geo)
    local soft = PNL.map_sig(apt, Cv, V, fa)
    local mb = sd.mb
    local brk = S.mode == "ENGAGED" or S.mode == "RELEASE" -- (gruener Haltepunkt-Balken wandert)
    -- (war die Seite im letzten Bild nicht zu sehen: sofort neu, das alte Bild ist veraltet)
    local sync = not mb or mb.hard ~= hard or sd.mlast ~= PNL.dcount - 1
    local why = sync and ((not mb) and "new" or (mb.hard ~= hard and ("hard " .. mb.hard .. " -> " .. hard) or "shown again"))
    sd.mlast = PNL.dcount
    local want = false
    if not sync then
        local rot, sh, scl = MAPX.nd_moved(MAPX.nd_delta(mb.C, Cv))
        local ds = math.abs(scl - 1)
        if PNL.nomat then
            sync = rot > 1e-3 or sh > 1e-2 or ds > 1e-5 or mb.soft ~= soft or brk
        -- (waehrend des Zooms zwischen zwei Reichweiten: das alte Bild skalieren, im Hintergrund neu aufbauen)
        elseif rot > 5 or sh > 20 or (ds > (fa and 0.3 or 0.1) and not V.zanim) then
            sync = true -- zu weit weg: am Rand fehlte sonst Karte
            why = string.format("moved rot %.1f shift %.1f scale %.3f", rot, sh, scl)
        else
            -- (beim Ein-/Ausblenden zoomt die Matrix; neu nur fuer die naechste Deckkraft-Stufe)
            want = mb.soft ~= soft or brk or (not fa and (rot > 0.8 or sh > 5 or ds > 0.01))
        end
    end
    local function start()
        local free = (mb and mb.buf == sd.mbuf[1]) and sd.mbuf[2] or sd.mbuf[1]
        sd.job = PNL.job_new(sd, Cv, apt, V, geo, free, hard, soft, fa)
    end
    if sync then
        start()
        PNL.trace("map build side " .. sd.side)
        PNL.job_run(sd, sd.job, nil)
        mb = PNL.job_done(sd)
        PNL.trace(string.format("map built (%d vertices)", mb.buf.n))
        PNL.stat.sync, PNL.stat.why = PNL.stat.sync + 1, why
    else
        if not sd.job and want then start() end
        if sd.job and PNL.job_run(sd, sd.job, fa and PNL.budget_fade or PNL.budget) then mb = PNL.job_done(sd) end
    end
    local D = MAPX.nd_delta(mb.C, Cv)
    sd.D = D
    sd.mat = PNL.nd_matrix(sd, mb, D, geo.H)
    sd.draw_map = true
    -- Overlay: bei Eingaben, Aenderungen, neuem Kartenbild, sonst alle 10 Bilder
    local ev = inp.clicked or inp.released or inp.rclicked or (inp.wheel or 0) ~= 0 or (inp.keys ~= nil and #inp.keys > 0)
        or (inp.dx or 0) ~= 0 or (inp.dy or 0) ~= 0 or MV.press
    local osig = PNL.ovl_sig(sd, Cv, V, MV, fa)
    if ev or osig ~= sd.osig or mb ~= sd.ob_mb or PNL.dirty or PNL.frame - (sd.oframe or -1000) >= 10 then
        sd.osig, sd.oframe, sd.ob_mb = osig, PNL.frame, mb
        local hits, marks = MAPX.nd_hits_now(mb, D)
        Cv.hit, Cv.marks = hits, marks
        MV.marks = marks
        PNL.ovl_begin(sd, geo, fa)
        Cv.hdr = NDUI.header_lines(Cv)
        NDUI.overlay(Cv, V, MV, sd)
        NDUI.input(Cv, V, MV, inp, sd)
        PNL.ovl_end(sd)
        PNL.stat.ovl = PNL.stat.ovl + 1
        PNL.trace(string.format("overlay (%d vertices)", sd.ob.n))
    end
end

-- Liste von Puffern in einem Zug zeichnen; je Eintrag: v, c, t, n, mat (Matrix) und scis (Scherenrechteck)
function PNL.submit_list(list)
    local any = false
    for _, it in ipairs(list) do if it.n >= 3 then any = true end end
    if not any then return end
    local ib = PNL.ib
    pnl_ext()
    -- mit Texturkoordinaten und geladener Schrift: Textur 0 = Schriftatlas (Flaechen nutzen dessen weissen Fleck).
    -- X-Planes Zustand zuerst setzen, danach sichern: so stimmt X-Planes Zustands-Speicher nach dem Zuruecksetzen.
    local tex = list[1].t ~= nil and TF.ok and TF.id ~= nil
    local mat = not PNL.nomat
    local clip = mat
    PNL.f_sgs(0, tex and 1 or 0, 0, 0, 1, 0, 0)
    if tex then XP.btv_XPLMBindTexture2d(TF.id, 0) end
    local prog, vao, vbo = 0, 0, 0
    if PNL.glUseProgram then GL.btv_glGetIntegerv(GLC.CURRENT_PROGRAM, ib); prog = ib[0] end
    if PNL.glBindVertexArray then GL.btv_glGetIntegerv(GLC.VERTEX_ARRAY_BINDING, ib); vao = ib[0] end
    if PNL.glBindBuffer then GL.btv_glGetIntegerv(GLC.ARRAY_BUFFER_BINDING, ib); vbo = ib[0] end
    if PNL.diag_bind == nil then PNL.diag_bind = string.format("program %d, VAO %d, VBO %d", prog, vao, vbo) end
    -- Scherenrechteck von X-Plane (falls gesetzt): unseres liegt darin, ohne eigenes gilt es weiter
    local sc_on, sb = false, nil
    if clip then
        ib[0] = 0; GL.btv_glGetIntegerv(GLC.SCISSOR_TEST, ib); sc_on = ib[0] ~= 0
        if sc_on then GL.btv_glGetIntegerv(GLC.SCISSOR_BOX, ib); sb = { ib[0], ib[1], ib[2], ib[3] } end
    end
    if prog ~= 0 then PNL.glUseProgram(0) end
    if vao ~= 0 then PNL.glBindVertexArray(0) end
    if vbo ~= 0 then PNL.glBindBuffer(GLC.ARRAY_BUFFER, 0) end
    GL.btv_glPushAttrib(GLC.ENABLE_BIT + GLC.POLYGON_BIT + GLC.COLOR_BUFFER_BIT + (tex and GLC.TEXTURE_BIT or 0) +
        (clip and GLC.SCISSOR_BIT or 0) + (mat and GLC.TRANSFORM_BIT or 0))
    if tex then GL.btv_glTexEnvi(GLC.TEXTURE_ENV, GLC.TEXTURE_ENV_MODE, GLC.MODULATE) end
    GL.btv_glDisable(GLC.CULL_FACE)
    GL.btv_glDisable(GLC.COLOR_LOGIC_OP)
    GL.btv_glPolygonMode(GLC.FRONT_AND_BACK, GLC.FILL)
    GL.btv_glEnable(GLC.BLEND)
    GL.btv_glBlendFunc(GLC.SRC_ALPHA, GLC.ONE_MINUS_SRC_ALPHA)
    GL.btv_glPushClientAttrib(GLC.CLIENT_VERTEX_ARRAY_BIT)
    for _, a in ipairs({ GLC.NORMAL_ARRAY, GLC.INDEX_ARRAY, GLC.EDGE_FLAG_ARRAY, GLC.FOG_COORD_ARRAY, GLC.SECONDARY_COLOR_ARRAY }) do
        GL.btv_glDisableClientState(a)
    end
    if PNL.glClientActiveTexture then
        for t = PNL.ntex - 1, 0, -1 do
            PNL.glClientActiveTexture(GLC.TEXTURE0 + t)
            GL.btv_glDisableClientState(GLC.TEXTURE_COORD_ARRAY)
        end
    else
        GL.btv_glDisableClientState(GLC.TEXTURE_COORD_ARRAY)
    end
    if PNL.glDisableVertexAttribArray then
        for a = 0, PNL.nattr - 1 do PNL.glDisableVertexAttribArray(a) end
    end
    GL.btv_glEnableClientState(GLC.VERTEX_ARRAY)
    GL.btv_glEnableClientState(GLC.COLOR_ARRAY)
    -- (die Schleife oben laesst Einheit 0 als aktive Client-Textureinheit zurueck)
    if tex then GL.btv_glEnableClientState(GLC.TEXTURE_COORD_ARRAY) end
    if mat then GL.btv_glMatrixMode(GLC.MODELVIEW) end
    for _, it in ipairs(list) do
        if it.n >= 3 then
            if clip then
                local r = it.scis
                if r then
                    local x0, y0, x1, y1 = r[1], r[2], r[1] + r[3], r[2] + r[4]
                    if sb then
                        x0, y0 = math.max(x0, sb[1]), math.max(y0, sb[2])
                        x1, y1 = math.min(x1, sb[1] + sb[3]), math.min(y1, sb[2] + sb[4])
                    end
                    GL.btv_glEnable(GLC.SCISSOR_TEST)
                    GL.btv_glScissor(x0, y0, math.max(0, x1 - x0), math.max(0, y1 - y0))
                elseif sc_on then
                    GL.btv_glEnable(GLC.SCISSOR_TEST)
                    GL.btv_glScissor(sb[1], sb[2], sb[3], sb[4])
                else
                    GL.btv_glDisable(GLC.SCISSOR_TEST)
                end
            end
            if mat and it.mat then GL.btv_glPushMatrix(); GL.btv_glMultMatrixf(it.mat) end
            if tex then GL.btv_glTexCoordPointer(2, GLC.FLOAT, 0, it.t) end
            GL.btv_glVertexPointer(2, GLC.FLOAT, 0, it.v)
            GL.btv_glColorPointer(4, GLC.UNSIGNED_BYTE, 0, it.c)
            GL.btv_glDrawArrays(GLC.TRIANGLES, 0, it.n)
            if mat and it.mat then GL.btv_glPopMatrix() end
        end
    end
    GL.btv_glPopClientAttrib()
    GL.btv_glPopAttrib()
    if vbo ~= 0 then PNL.glBindBuffer(GLC.ARRAY_BUFFER, vbo) end
    if vao ~= 0 then PNL.glBindVertexArray(vao) end
    if prog ~= 0 then PNL.glUseProgram(prog) end
end

-- BEGIN ATCCOM
-------------------------------------------------------------------------------
-- ATCCOM: our A350 ATCCOM page on the captain's MFD, the ATC mailbox at the top of the upper centre display, the
-- (inactive) CPNY WIND REQUEST key on FF's INIT page and the ATC MSG pushbuttons. The page is A350_ATCCOM/ui.lua,
-- the datalink A350_ATCCOM/hoppie.lua; this part puts the page into FF's display texture, feeds it the mouse and
-- keyboard and connects it to the cockpit.
--
-- Display areas: panel texture rectangle {u0, v0, u1, v1, page width, page height} (page units, y down) and the
-- four corners of the glass in aircraft coordinates (top left, top right, bottom right, bottom left), both from
-- FF's A350_xp11_cockpit.obj (OBJ -> aircraft: y - 2.4384 m, z - 30.3024 m). The captain's MFD is the left half of
-- the lower centre display; with OIS TO CENTER (1-sim/misc/oisCycle) it moves to the outer display, with the MFD
-- swap (1-sim/misc/mfdCycle) to the right half (FF's MFD click zones move the same way in the OBJ).
-------------------------------------------------------------------------------
PNL.atc = {
    cache = {}, inv = {}, quad = {}, lit = {}, lit_new = {},
    places = {
        [0] = { -- lower centre display
            [0] = { { 3.9, 771.1, 508.1, 1410.1, 679, 884 },
                { { -0.1675, 1.7298, -28.6643 }, { 0, 1.7298, -28.6643 }, { 0, 1.5797, -28.5142 }, { -0.1675, 1.5797, -28.5142 } } },
            [1] = { { 508.1, 771.1, 1012.3, 1410.1, 679, 884 },
                { { 0, 1.7298, -28.6643 }, { 0.1675, 1.7298, -28.6643 }, { 0.1675, 1.5797, -28.5142 }, { 0, 1.5797, -28.5142 } } } },
        [1] = { -- captain's outer display
            [0] = { { 3.7, 120.0, 508.1, 759.4, 679, 884 },
                { { -0.9330, 1.9817, -28.5407 }, { -0.78795, 1.9745, -28.6241 }, { -0.77855, 1.7652, -28.58965 }, { -0.9236, 1.7724, -28.5062 } } },
            [1] = { { 508.1, 120.0, 1012.5, 759.4, 679, 884 },
                { { -0.78795, 1.9745, -28.6241 }, { -0.6429, 1.9673, -28.7075 }, { -0.6335, 1.7580, -28.6731 }, { -0.77855, 1.7652, -28.58965 } } } },
    },
    -- the mailbox: top 236 page units of the right half of the upper centre display (the memo area stays FF's)
    mailbox = { { 1521.25, 1243.1, 2025.5, 1410.1, 679, 236 },
        { { 0.0004, 1.9664, -28.7178 }, { 0.1679, 1.9664, -28.7178 }, { 0.1679, 1.91175, -28.70815 }, { 0.0004, 1.91175, -28.70815 } } },
    -- ATC MSG pushbuttons (CAPT, F/O) and their lamps
    msg_buttons = { "1-sim/127/button", "1-sim/171/button" },
    msg_lamps = { "1-sim/lamp/219", "1-sim/lamp/288" },
    -- ATC MSG chime: caspar92UA's CPDLC.wav when present (see LICENSE.txt), else the mod's own chime
    chimes = { "A350_ATCCOM/CPDLC.wav", "A350_ATCCOM/atc_msg_chime.wav" },
}
PNL.atc.regions = { lower = PNL.atc.places[0][0][1], upper = PNL.atc.mailbox[1] }
PNL.atc.geom = { lower = PNL.atc.places[0][0][2], upper = PNL.atc.mailbox[2] }

-- load the page and the datalink once
function PNL.atc_load()
    local a = PNL.atc
    if a.loaded then return end
    a.loaded = true
    local ok, page = pcall(dofile, SCRIPTS .. "A350_ATCCOM/ui.lua")
    if not ok or type(page) ~= "table" then pnl_log("ATCCOM page not loaded: " .. tostring(page)); return end
    a.ui = page
    local okh, Hoppie = pcall(dofile, SCRIPTS .. "A350_ATCCOM/hoppie.lua")
    if okh and type(Hoppie) == "table" then
        local okn, link = pcall(Hoppie.new, { dir = SCRIPTS .. "A350_ATCCOM/", log = pnl_log, version = VERSION,
            now = function() return XP.btv_XPLMGetElapsedTime() end })
        if okn then a.link = link else pnl_log("Hoppie client not started: " .. tostring(link)) end
    else
        pnl_log("Hoppie client not loaded: " .. tostring(Hoppie))
    end
    page.dir = SCRIPTS .. "A350_ATCCOM/"
    pnl_log("ATCCOM page loaded" .. (a.link and " with the Hoppie client" or ", no datalink"))
end

-- settings window, DATALINK section
function btv_hoppie_status()
    local L = PNL.atc.link
    if L then return L.status, L.logon ~= "", L.callsign, L.err end
end
function btv_hoppie_set_logon(code)
    if PNL.atc.link then PNL.atc.link:set_logon(code) end
end

-- leave text entry on the page (and give the keyboard back to the ND input, if that had it)
function PNL.atc_blur()
    local ui = PNL.atc.ui
    if ui and ui.focus then
        ui.blur()
        if NDUI.focus then NDUI.focus(false) end
    end
end

-------------------------------------------------------------------------------
-- ATC MSG: a new uplink lights both ATC MSG pushbuttons and sounds the chime every 3 s until a pilot presses one of
-- them or the avionics go off. FF drives the lamps itself, so they are written again after FF in every panel draw.
-------------------------------------------------------------------------------
PNL.atc.msg = { on = false, seq = 0, chime_at = 0, pressed = {}, refs = nil }

function PNL.atc_msg_refs()
    local m = PNL.atc.msg
    if m.refs then return m.refs end
    if not XFM.init() then return nil end
    local refs = { buttons = {}, lamps = {} }
    for i, n in ipairs(PNL.atc.msg_buttons) do refs.buttons[i] = XFM.dref(n) end
    for i, n in ipairs(PNL.atc.msg_lamps) do
        local d = XFM.dref(n)
        refs.lamps[i] = (d and d.can) and d or nil
        if not refs.lamps[i] then pnl_log("ATC MSG lamp not writable: " .. n) end
    end
    m.refs = refs
    return refs
end

function PNL.atc_msg_lamps(v)
    local refs = PNL.atc.msg.refs
    if not refs then return end
    for _, d in pairs(refs.lamps) do pcall(XFM.set, d, v) end
end

function PNL.atc_msg_chime()
    local snd = nil
    for _, f in ipairs(PNL.atc.chimes) do
        snd = AUD.load(SCRIPTS .. f)
        if snd then break end
    end
    if not snd then return end
    -- through X-Plane's cockpit sound (FMOD interior bus, or FlyWithLua's sound), at the sim's own volume
    local ok, err = pcall(snd.fmod and play_sound_on_interior_bus or play_sound, snd.id)
    if not ok and not PNL.atc.msg.chime_err then PNL.atc.msg.chime_err = true; pnl_log("ATC MSG chime: " .. tostring(err)) end
end

function PNL.atc_msg_off(why)
    local m = PNL.atc.msg
    if not m.on then return end
    m.on = false
    PNL.atc_msg_lamps(0)
    pnl_log("ATC MSG off (" .. why .. ")")
end

-- every frame: new uplinks (the page counts them in ui.chime_seq), the pushbuttons, lamps and chime
function PNL.atc_msg_tick()
    local a, m = PNL.atc, PNL.atc.msg
    if not a.ui then return end
    local refs = PNL.atc_msg_refs()
    if not refs then return end
    -- a pushbutton press = its value going up through 0.5
    local pressed = nil
    for i, d in pairs(refs.buttons) do
        local v = XFM.get(d) or 0
        if v > 0.5 and (m.pressed[i] or 0) <= 0.5 then pressed = i end
        m.pressed[i] = v
    end
    local seq = a.ui.chime_seq or 0
    local powered = a.ui.enabled and a.refs and XP.btv_XPLMGetDatai(a.refs.power) == 1
    if not powered then
        m.seq = seq
        PNL.atc_msg_off("avionics off")
        return
    end
    if seq > m.seq then
        m.seq = seq
        if not m.on then m.on, m.chime_at = true, PNL.ft; pnl_log("ATC MSG on: new message") end
    end
    if m.on and pressed then PNL.atc_msg_off(pressed == 1 and "CAPT pushbutton" or "F/O pushbutton"); return end
    if not m.on then return end
    PNL.atc_msg_lamps(1)
    if PNL.ft >= m.chime_at then PNL.atc_msg_chime(); m.chime_at = PNL.ft + 3 end
end

-- after FF's own lamp logic in the panel draw
function PNL.atc_msg_late()
    if PNL.atc.msg.on then PNL.atc_msg_lamps(1) end
end

-------------------------------------------------------------------------------
-- What is ours right now (read from FF's datarefs; also called from the panel draw, so our page is there from the
-- first frame FF shows ATCCOM):
--   a.page    the MFD shows ATCCOM (1-sim/mfd/guage 2)   -> the whole MFD page
--   a.initkey the MFD shows FMS INIT (guage 1, page 24)   -> only the CPNY WIND REQUEST key
--   a.mail    avionics on                                -> the mailbox
-------------------------------------------------------------------------------
-- int or float dataref as a number (0 when missing)
function PNL.atc_num(r)
    if r == nil then return 0 end
    local ty = XP.btv_XPLMGetDataRefTypes(r)
    if bit.band(ty, 2) ~= 0 then return XP.btv_XPLMGetDataf(r) end
    if bit.band(ty, 1) ~= 0 then return XP.btv_XPLMGetDatai(r) end
    return 0
end

function PNL.atc_mode()
    local a = PNL.atc
    if not a.ui or a.failed or not a.ui.enabled then return end
    if not a.refs then
        local refs = {
            power = XP.btv_XPLMFindDataRef("sim/cockpit/electrical/avionics_on"),
            system = XP.btv_XPLMFindDataRef("1-sim/mfd/guage"),
            mfdpage = XP.btv_XPLMFindDataRef("1-sim/mfd/page"),
            ois = XP.btv_XPLMFindDataRef("1-sim/misc/oisCycle"),
            mfdc = XP.btv_XPLMFindDataRef("1-sim/misc/mfdCycle"),
        }
        if refs.power == nil or refs.system == nil then return end
        a.refs = refs
    end
    local powered = XP.btv_XPLMGetDatai(a.refs.power) == 1
    -- which display / half shows the captain's MFD
    local place = a.places[PNL.atc_num(a.refs.ois) > .5 and 1 or 0][PNL.atc_num(a.refs.mfdc) > .5 and 1 or 0]
    if a.regions.lower ~= place[1] then
        a.regions.lower, a.geom.lower, a.cache, a.cursor = place[1], place[2], {}, nil
        pnl_log(string.format("ATCCOM page area: panel %.0f..%.0f / %.0f..%.0f", place[1][1], place[1][3], place[1][2], place[1][4]))
    end
    -- FF's MFD CONTROL option: WHEEL + POPUP (1-sim/options/Popups 1) = the cockpit MFD only opens FF's popup
    if a.refs.popups == nil then a.refs.popups = XP.btv_XPLMFindDataRef("1-sim/options/Popups") or false end
    local pm = false
    if a.refs.popups then pm = XP.btv_XPLMGetDatai(a.refs.popups) == 1 end
    if pm ~= (a.popmode or false) then
        a.popmode, a.cursor = pm, nil
        pnl_log("ATCCOM: MFD control " .. (pm and "wheel + popup (cockpit MFD not touchable)" or "touch screen"))
    end
    local sys = XP.btv_XPLMGetDatai(a.refs.system)
    local page = powered and sys == 2 or nil
    local initkey = powered and sys == 1 and a.refs.mfdpage ~= nil and XP.btv_XPLMGetDatai(a.refs.mfdpage) == 24
    if page ~= a.page or initkey ~= (a.initkey or false) then
        PNL.atc_blur()
        a.page, a.initkey = page, initkey
        a.ui.page_changed()
        a.cache, a.capture, a.cursor, a.next_backend = {}, nil, nil, nil
        pnl_log("ATCCOM: " .. (page and "ATCCOM page" or (initkey and "INIT page key" or "not shown")))
    end
    if powered ~= (a.mail or false) then
        a.mail, a.cache = powered, {}
        pnl_log("ATC mailbox " .. (powered and "on" or "off"))
    end
end

-- a choice in our MFD system list: FF's MFD is switched directly (FF's own list does not get clicks through the
-- mod's mouse window; FF switches and repaints the whole MFD when its system value is written)
function PNL.atc_sys_set(n)
    local a = PNL.atc
    if not n or n == 2 or not a.refs or not XFM.init() then return end
    XFM.xp.btv_s_XPLMSetDatai(a.refs.system, n)
    pnl_log("ATCCOM system menu: MFD system " .. n .. " chosen")
end

function PNL.atc_any() local a = PNL.atc; return (a.page or a.initkey or a.mail) and true or false end
-- FF's display lit? (sampled from FF's own pixels: nil = not measured yet, then shown)
function PNL.atc_lit(which) return PNL.atc.lit[which] ~= false end
-- the part of a display that is ours: "full", "key" (lower: INIT key only) or nil
function PNL.atc_active(which)
    local a = PNL.atc
    if which == "upper" then
        if a.mail and PNL.atc_lit("upper") then return "full" end
        return nil
    end
    if not PNL.atc_lit("lower") then return nil end
    return a.page and "full" or (a.initkey and "key" or nil)
end
-- the part of a display that takes the mouse: the cockpit MFD not in FF's popup mode (clicks there open FF's popup)
function PNL.atc_input(which)
    if which == "lower" and PNL.atc.popmode then return nil end
    return PNL.atc_active(which)
end

-------------------------------------------------------------------------------
-- once a second: datalink, flight number and route for the page
-------------------------------------------------------------------------------
-- text dataref (FF's FMS values), nil when missing; size asked first (FF may write its whole value whatever length
-- is asked for), nothing over 64 KB
function PNL.atc_text(name)
    local a = PNL.atc
    local r = XP.btv_XPLMFindDataRef(name)
    if r == nil or bit.band(XP.btv_XPLMGetDataRefTypes(r), 32) == 0 then return nil end
    local size = XP.btv_XPLMGetDatab(r, nil, 0, 0)
    if size <= 0 then return "" end
    if size > 65536 then return nil end
    if not a.tbuf or a.tlen < size then a.tlen = math.max(size, 128); a.tbuf = ffi.new("char[?]", a.tlen + 1) end
    local n = XP.btv_XPLMGetDatab(r, a.tbuf, 0, size)
    if n < 0 then return nil end
    local s = ffi.string(a.tbuf, math.min(n, size)):gsub("%z.*$", "")
    return (s:gsub("^%s+", ""):gsub("%s+$", ""):upper())
end

-- FLT NBR from FF's INIT page (FF keeps it in more than one place; "BAW 9165" -> "BAW9165")
PNL.atc.flight_refs = { "AirbusFBW/fms/init/flightNumber", "1-sim/mfd/fltNum", "1-sim/checklist/flightNumber" }

function PNL.atc_frame()
    local a = PNL.atc
    PNL.atc_load()
    if not a.ui or a.failed then return end
    PNL.atc_mode()
    local okp, ep = pcall(PNL.atp_tick)
    if not okp then PNL.atp_err(ep) end
    if not (a.ui.enabled and a.refs and a.link) or (a.next_backend and PNL.ft < a.next_backend) then return end
    a.next_backend = PNL.ft + 1
    local ok, err = pcall(function()
        local powered = XP.btv_XPLMGetDatai(a.refs.power) == 1
        local flight = nil
        for _, n in ipairs(a.flight_refs) do
            local v = PNL.atc_text(n)
            v = v and v:gsub("%s+", "")
            if v and #v >= 3 and v:match("^[A-Z0-9]+$") then flight = v; break end
        end
        local L = a.link
        -- (the link runs with the avionics off too: it ends a CPDLC session when the datalink is lost)
        L:tick(flight or "", powered)
        if not powered then return end
        a.ui.route(PNL.atc_text("AirbusFBW/fms/init/fromName"), PNL.atc_text("AirbusFBW/fms/init/toName"),
            PNL.atc_text("AirbusFBW/fms/init/altnName"))
        -- the page's top line explains a missing or rejected Hoppie logon code
        if L.status ~= a.link_status then
            a.link_status = L.status
            local why = ({ ["NO LOGON CODE"] = "SET HOPPIE LOGON IN SYSTEMS MOD SETTINGS",
                ["LOGON CODE REJECTED"] = "HOPPIE LOGON CODE REJECTED" })[L.status]
            if why then a.ui.s.info, a.ui.s.info_warn, a.ui.dirty = why, true, true end
        end
        a.ui.link_tick(L, flight)
    end)
    if not ok then
        pcall(a.ui.link_tick, nil)
        if a.backend_error ~= tostring(err) then a.backend_error = tostring(err); pnl_log("ATCCOM datalink: " .. a.backend_error) end
    else
        a.backend_error = nil
    end
end

-------------------------------------------------------------------------------
-- mouse
-------------------------------------------------------------------------------
-- cursor on a page: the page point under the mouse. While the mouse does not move, small changes of that point
-- (the cockpit view shakes, the projection with it) are ignored; a moved mouse or a moved view (head turned) takes
-- the new point.
function PNL.atc_stable_cursor(old, which, inv, sx, sy, px, py)
    if old and old.which == which and math.abs(old.sx - sx) < 0.5 and math.abs(old.sy - sy) < 0.5
        and math.abs(old.x - px) < 3 and math.abs(old.y - py) < 3 then return old end
    return { which = which, x = px, y = py, sx = sx, sy = sy }
end

-- mouse moved over our window: cursor and the page's hover frame
function PNL.atc_cursor(x, y)
    local a = PNL.atc
    local hit = PNL.atc_cursor_at(x, y)
    if a.ui and a.ui.hover then
        local c = a.cursor
        if c then a.ui.hover(c.which, c.x, c.y) else a.ui.hover(nil) end
    end
    return hit
end

-- INIT page: our key's rectangle (lower page units) - only there the page is ours
function PNL.atc_key()
    local ui = PNL.atc.ui
    if ui and ui.initkey_rect then return { ui.initkey_rect() } end
    return { 512, 365, 150, 65 }
end
function PNL.atc_in_key(px, py)
    local k = PNL.atc_key()
    return px >= k[1] and px <= k[1] + k[3] and py >= k[2] and py <= k[2] + k[4]
end

-- screen rectangle (window coordinates) around the INIT key, for the mouse window
function PNL.atc_key_screen()
    local q = PNL.atc.quad.lower
    if not q then return nil end
    local r, k = PNL.atc.regions.lower, PNL.atc_key()
    -- page point -> screen: bilinear between the four projected corners
    local function screen(px, py)
        local u, v = px / r[5], py / r[6]
        local x = (1 - v) * ((1 - u) * q[1][1] + u * q[2][1]) + v * ((1 - u) * q[4][1] + u * q[3][1])
        local y = (1 - v) * ((1 - u) * q[1][2] + u * q[2][2]) + v * ((1 - u) * q[4][2] + u * q[3][2])
        return x, y
    end
    local x0, y0, x1, y1 = math.huge, math.huge, -math.huge, -math.huge
    for _, c in ipairs({ { k[1], k[2] }, { k[1] + k[3], k[2] }, { k[1], k[2] + k[4] }, { k[1] + k[3], k[2] + k[4] } }) do
        local x, y = screen(c[1], c[2])
        x0, y0, x1, y1 = math.min(x0, x), math.min(y0, y), math.max(x1, x), math.max(y1, y)
    end
    return x0, y0, x1, y1
end

-- page under the mouse (sets a.cursor); true when the mouse is on a part that is ours
function PNL.atc_cursor_at(x, y)
    local a = PNL.atc
    local old = a.cursor
    a.cursor = nil
    if not PNL.atc_ready() then return false end
    for _, which in ipairs({ "lower", "upper" }) do
        local inv, mode, r = a.inv[which], PNL.atc_input(which), a.regions[which]
        if inv and mode then
            local px, py = inv(x, y)
            if mode == "key" then
                if PNL.atc_in_key(px, py) then a.cursor = PNL.atc_stable_cursor(old, which, inv, x, y, px, py); return true end
            elseif px >= 0 and px <= r[5] and py >= 0 and py <= r[6] then
                -- the top strip of the MFD (FF's system key) stays FF's
                if which == "lower" and py < 32 then return false end
                local c = PNL.atc_stable_cursor(old, which, inv, x, y, px, py)
                c.x = math.max(0, math.min(r[5], c.x))
                c.y = math.max(which == "lower" and 32 or 0, math.min(r[6], c.y))
                a.cursor = c
                return true
            end
        end
    end
    return false
end

-------------------------------------------------------------------------------
-- drawing into FF's display texture
-------------------------------------------------------------------------------
-- run fn with the drawing helpers set up for one display area (page units, y down) and put back what was there
function PNL.atc_build(r, pl, pb, pr, pt, buf, fn, ts)
    local saved = { DL = {}, GLB = {}, TF = {} }
    for name, t in pairs({ DL = DL, GLB = GLB, TF = TF }) do for k, v in pairs(t) do saved[name][k] = v end end
    local font_h = FONT_H
    if type(buf) == "number" then buf = PNL.newbuf(buf) end
    local ok, err = pcall(function()
        local sx, sy = (pr - pl) / 2048, (pt - pb) / 2048
        local kx, ky = (r[3] - r[1]) * sx / r[5], (r[4] - r[2]) * sy / r[6]
        PNL.use(buf)
        DL.begin({ kx, 0, pl + r[1] * sx, 0, -ky, pb + r[4] * sy, 0, 0 }, r[5], r[6])
        GLB.fa, FONT_H, DL.n = nil, 20, 0
        if ts then GLB.ts = (kx + ky) / 2 end
        GLB.px = 1 / math.max(1, pnl_dpu() or 1)
        fn()
        buf.n = GLB.n
    end)
    for name, t in pairs({ DL = DL, GLB = GLB, TF = TF }) do
        for k in pairs(t) do t[k] = nil end
        for k, v in pairs(saved[name]) do t[k] = v end
    end
    FONT_H = font_h
    if not ok then error(err, 0) end
    return buf
end

-- page drawing commands (from ui.lua) into the current buffer
function PNL.atc_ops(ops, dy)
    dy = dy or 0
    for _, o in ipairs(ops) do
        local kind = o[1]
        if kind == "rect" then DL.rectf(o[2], o[3] + dy, o[2] + o[4], o[3] + dy + o[5], o[6])
        elseif kind == "text" then DL.text(o[2], o[3] + dy, o[5], o[4], o[6])
        elseif kind == "tri" then DL.tri(o[2], o[3] + dy, o[4], o[5] + dy, o[6], o[7] + dy, o[8]) end
    end
end

-- FF's MFD cursor, in yellow: four diagonal arms around a small plus (shape taken from FF's own cursor on screen)
function PNL.atc_cursor_shape(x, y)
    local col = 0xFF4CE7F5
    for _, sx in ipairs({ -1, 1 }) do
        for _, sy in ipairs({ -1, 1 }) do DL.line(x + sx * 6, y + sy * 4.2, x + sx * 36, y + sy * 25.2, col, 1.05) end
    end
    DL.rectf(x - 2.8, y - 0.6, x + 2.8, y + 0.6, col)
    DL.rectf(x - 0.6, y - 2.8, x + 0.6, y + 2.8, col)
end

function PNL.atc_draw(br)
    local a = PNL.atc
    if not a.ui or a.failed or PNL.dead or not PNL.ok or PNL.hook ~= br then return end
    pcall(PNL.atc_mode)
    local lmode, umode = PNL.atc_active("lower"), PNL.atc_active("upper")
    if a.refs and (lmode or "-") ~= (a.logged_lmode or "-") then
        a.logged_lmode = lmode
        pnl_log(string.format("ATCCOM lower display: %s (MFD system %d, page %d, OIS %d, MFD swap %d, power %d)",
            lmode or "FF's own", XP.btv_XPLMGetDatai(a.refs.system), a.refs.mfdpage and XP.btv_XPLMGetDatai(a.refs.mfdpage) or -1,
            PNL.atc_num(a.refs.ois), PNL.atc_num(a.refs.mfdc), XP.btv_XPLMGetDatai(a.refs.power)))
    end
    if not lmode and not umode then return end
    if XP.btv_XPLMGetElapsedTime() - PNL.ft > 1 then return end
    local rt = XP.btv_XPLMGetDatai(PNL.r_rt)
    if rt ~= 1 and rt ~= 2 then return end
    local lit = rt == 2 or PNL.lit_in_1
    local pl, pb = XP.btv_XPLMGetDataf(PNL.r_pl), XP.btv_XPLMGetDataf(PNL.r_pb)
    local pr, pt = XP.btv_XPLMGetDataf(PNL.r_pr), XP.btv_XPLMGetDataf(PNL.r_pt)
    if math.abs(pr - pl - 2048) > 2 or math.abs(pt - pb - 2048) > 2 then return end
    if a.ui.dirty then a.cache, a.ui.dirty, PNL.atp.stale = {}, false, true end
    -- built meshes are kept until something on the page or the panel changes
    local lkey = lmode == "key" and ("key" .. (a.ui.initkey_sig and a.ui.initkey_sig() or "")) or (lmode and a.ui.s.page or "-")
    local key = table.concat({ lkey, umode or "-", lit and 1 or 0, pl, pb, pr, pt, TF.ok and 1 or 0, TF.id or 0 }, ":")
    local mesh = a.cache[key]
    if not mesh then
        mesh = {}
        for _, which in ipairs({ "lower", "upper" }) do
            local mode = umode
            if which == "lower" then mode = lmode end
            if mode then
                local keyonly = mode == "key"
                local buf = PNL.atc_build(a.regions[which], pl, pb, pr, pt, keyonly and 20000 or 180000, function()
                    if lit then
                        PNL.atc_ops(keyonly and a.ui.initkey() or (which == "lower" and a.ui.lower() or a.ui.upper()))
                    elseif keyonly then
                        -- day pass without the lit pass: our parts black, so FF's key does not show through
                        for _, k in ipairs(a.ui.initkey_mask and a.ui.initkey_mask() or { PNL.atc_key() }) do
                            DL.rectf(k[1], k[2], k[1] + k[3], k[2] + k[4], 0xFF000000)
                        end
                    elseif which == "lower" then DL.rectf(2, 32, 677, 884, 0xFF000000)
                    else DL.rectf(0, 0, 679, 236, 0xFF000000) end
                end)
                buf.which = which
                if buf.n >= buf.cap - 100 then pnl_log("ATCCOM: mesh nearly full (" .. which .. ")") end
                mesh[#mesh + 1] = buf
            end
        end
        a.cache[key] = mesh
    end
    local list = {}
    for _, buf in ipairs(mesh) do list[#list + 1] = buf end
    if lit and a.cursor and (a.cursor.which ~= "lower" or lmode) then
        local c = a.cursor
        a.cursor_buf = a.cursor_buf or PNL.newbuf(256)
        local ok, cbuf = pcall(PNL.atc_build, a.regions[c.which], pl, pb, pr, pt, a.cursor_buf, function() PNL.atc_cursor_shape(c.x, c.y) end, true)
        if ok then cbuf.which = c.which; list[#list + 1] = cbuf else a.cursor = nil end
    end
    PNL.submit_list(list)
    if lit then
        a.rendered = PNL.frame
        if a.logged_page ~= a.ui.s.page then a.logged_page = a.ui.s.page; pnl_log("ATCCOM page " .. a.ui.s.page .. " drawn via " .. br.name) end
    end
end

function PNL.atc_ready()
    local a = PNL.atc
    return a.ui ~= nil and (a.page or a.initkey or a.mail) and not a.failed and a.rendered ~= nil
        and PNL.frame - a.rendered < 5 and PNL.hook ~= nil and not PNL.dead
end

-- draw from the panel callback; a fault is logged once and never hands the page back to FF's own ATCCOM
function PNL.atc_safe_draw(br)
    local a = PNL.atc
    local ok, err = pcall(PNL.atc_draw, br)
    if not ok and a.draw_err ~= tostring(err) then a.draw_err = tostring(err); pnl_log("ATCCOM drawing error: " .. a.draw_err) end
    if a.ui and a.ui.page_error and a.ui.page_error ~= a.page_err then a.page_err = a.ui.page_error; pnl_log("ATCCOM page error: " .. a.page_err) end
end

-------------------------------------------------------------------------------
-- clicks and keys
-------------------------------------------------------------------------------
-- a click in our mouse window; true = taken (FF does not get it)
function PNL.atc_click(x, y, st, right)
    local a = PNL.atc
    if right then return false end
    if st ~= 1 then
        -- drag / release of a click we took
        if a.capture then
            if st == 3 then a.capture = nil end
            return true
        end
        return false
    end
    a.capture = nil
    if not PNL.atc_ready() then PNL.atc_blur(); return false end
    for _, which in ipairs({ "lower", "upper" }) do
        local inv, mode, r = a.inv[which], PNL.atc_input(which), a.regions[which]
        if inv and mode then
            local px, py = inv(x, y)
            if mode == "key" then
                -- INIT page: our CPNY WIND REQUEST key is inactive - the click is taken and does nothing
                if PNL.atc_in_key(px, py) then a.capture = true; return true end
            elseif px >= 0 and px <= r[5] and py >= 0 and py <= r[6] then
                if which == "lower" and py < 32 then
                    PNL.atc_blur()
                    -- FF's "ATCCOM v" key opens our system list (FF's own list is not used)
                    if px >= 15 and px <= 195 then a.ui.sysmenu_toggle(); a.capture = true; return true end
                    if a.ui.sysmenu_open() then a.ui.sysmenu_toggle() end
                    return false
                end
                -- system list open: a row switches FF's MFD, anywhere else closes the list
                if which == "lower" and a.ui.sysmenu_open() then
                    a.ui.sys_pick = nil
                    pcall(a.ui.click, which, px, py)
                    local n = a.ui.sys_pick
                    a.ui.sys_pick = nil
                    PNL.atc_sys_set(n)
                    a.capture = true
                    return true
                end
                -- the rest of the page is ours: hit the point where the cursor is drawn (if the mouse has not moved)
                local c = a.cursor
                if c and c.which == which and math.abs(c.sx - x) < 1 and math.abs(c.sy - y) < 1 then px, py = c.x, c.y end
                local had_focus = a.ui.focus ~= nil
                local hit = a.ui.click(which, px, py)
                if a.ui.focus then
                    if NDUI.kb then NDUI.kb_end() end
                    if NDUI.focus then NDUI.focus(true) end
                elseif had_focus and NDUI.focus then
                    NDUI.focus(false)
                end
                if hit then pnl_log(string.format("ATCCOM click %s %.0f/%.0f", which, px, py)) end
                -- taken even when nothing was hit: FF's own (hidden) page must not get it
                a.capture = true
                return true
            end
        end
    end
    PNL.atc_blur()
    return false
end

-- typing into a field of the page; true = key taken
function PNL.atc_key(k, fl, vk, lf)
    local ui = PNL.atc.ui
    if not ui or not ui.focus then return false end
    if lf ~= 0 then ui.blur(); return true end
    if bit.band(fl, 8) == 0 then return true end -- only key down
    ui.key(bit.band(k, 0xFF), bit.band(vk, 0xFF))
    if not ui.focus and NDUI.focus then NDUI.focus(false) end
    return true
end

-------------------------------------------------------------------------------
-- MFD popups (FF option "wheel + popup"): FF's popup window of the captain's MFD (1-sim/misc/popupLeft) would show
-- FF's own ATCCOM page. While the MFD is on ATCCOM the mod closes FF's popup and opens its own window with our page;
-- another system chosen there (or in the cockpit) hands back to FF's popup. Our window's close button ends it until
-- FF's popup is opened again.
-------------------------------------------------------------------------------
PNL.atp = { on = false }
PNL.ATP_STRIP = 59 -- FF's "PRESS FOR KEY INPUT" strip above the page, in page units

function PNL.atp_err(e)
    local P = PNL.atp
    if P.err ~= tostring(e) then P.err = tostring(e); pnl_log("ATCCOM popup: " .. P.err) end
end

function PNL.atp_create()
    local P = PNL.atp
    if P.win ~= nil then return true end
    if P.failed or not PNL.ok or not XFM.init() then return false end
    P.failed = true
    pcall(ffi.cdef, [[
void btv_a_XPLMGetWindowGeometry(void *, int *, int *, int *, int *) __asm__("XPLMGetWindowGeometry");
int btv_a_XPLMGetWindowIsVisible(void *) __asm__("XPLMGetWindowIsVisible");
void btv_a_XPLMSetWindowTitle(void *, const char *) __asm__("XPLMSetWindowTitle");
void btv_a_XPLMSetWindowResizingLimits(void *, int, int, int, int) __asm__("XPLMSetWindowResizingLimits");
]])
    P.g = ffi.new("int[4]")
    P.cb_draw = ffi.cast("btv_wdraw_f", function(w, rc)
        XFM.cb_enter()
        local ok, e = pcall(PNL.atp_draw)
        if not ok then PNL.atp_err(e) end
        XFM.cb_leave()
    end)
    P.cb_click = ffi.cast("btv_wclick_f", function(w, x, y, st, rc)
        XFM.cb_enter()
        local ok, e = pcall(PNL.atp_click, x, y, st)
        if not ok then PNL.atp_err(e) end
        XFM.cb_leave()
        return 1
    end)
    P.cb_key = ffi.cast("btv_wkey_f", function(w, k, fl, vk, rc, lf) XFM.cb_enter(); pcall(PNL.atc_key, k, fl, vk, lf); XFM.cb_leave() end)
    P.cb_cursor = ffi.cast("btv_wcursor_f", function(w, x, y, rc)
        XFM.cb_enter()
        local ok, inside = pcall(PNL.atp_hover, x, y)
        XFM.cb_leave()
        return (ok and inside) and 1 or 0 -- over the page: FF's yellow MFD cursor instead of the arrow
    end)
    P.cb_wheel = ffi.cast("btv_wwheel_f", function() return 1 end)
    -- first place: where FF's MFD popup opens (content 495 x 690 at the left edge, 400 below the top of the screen)
    local sl, st_, sr, sb = ffi.new("int[1]"), ffi.new("int[1]"), ffi.new("int[1]"), ffi.new("int[1]")
    XP.btv_XPLMGetScreenBoundsGlobal(sl, st_, sr, sb)
    local cw = ffi.new("btv_wnd_t")
    cw.structSize = ffi.sizeof("btv_wnd_t")
    cw.left, cw.top = sl[0] + 5, st_[0] - 400
    cw.right, cw.bottom, cw.visible = cw.left + 495, cw.top - 690, 0
    -- or where the pilot left it last time (A350_ATCCOM/popup_window.txt), if that is still on the screen
    local f = io.open(SCRIPTS .. "A350_ATCCOM/popup_window.txt", "r")
    if f then
        local l, t, r, b = (f:read("*l") or ""):match("^(%-?%d+) (%-?%d+) (%-?%d+) (%-?%d+)")
        f:close()
        l, t, r, b = tonumber(l), tonumber(t), tonumber(r), tonumber(b)
        if l and r - l >= 200 and t - b >= 200 and l < sr[0] - 50 and r > sl[0] + 50 and t <= st_[0] and t > sb[0] + 50 then
            cw.left, cw.top, cw.right, cw.bottom = l, t, r, b
        end
    end
    cw.drawWindowFunc, cw.handleMouseClickFunc, cw.handleKeyFunc = P.cb_draw, P.cb_click, P.cb_key
    cw.handleCursorFunc, cw.handleMouseWheelFunc, cw.refcon = P.cb_cursor, P.cb_wheel, nil
    cw.decorateAsFloatingWindow, cw.layer, cw.handleRightClickFunc = 1, 1, P.cb_click -- X-Plane frame (move, resize, close)
    P.win = XP.btv_XPLMCreateWindowEx(cw)
    if P.win == nil then pnl_log("ATCCOM popup: window could not be created"); return false end
    pcall(function()
        XP.btv_a_XPLMSetWindowTitle(P.win, "ATCCOM")
        XP.btv_a_XPLMSetWindowResizingLimits(P.win, 260, 340, 3000, 3900)
    end)
    P.failed = nil
    return true
end

-- once per frame: ours instead of FF's popup while the MFD shows ATCCOM
function PNL.atp_tick()
    local P, a = PNL.atp, PNL.atc
    if not a.refs then return end
    if a.refs.popl == nil then a.refs.popl = XP.btv_XPLMFindDataRef("1-sim/misc/popupLeft") or false end
    local ref = a.refs.popl
    if not ref then return end
    if P.on and XP.btv_a_XPLMGetWindowIsVisible(P.win) == 0 then
        P.on = false
        PNL.atp_save()
        pnl_log("ATCCOM popup closed")
    end
    if a.page and PNL.atc_lit("lower") and XP.btv_XPLMGetDatai(ref) == 1 then
        if not PNL.atp_create() then return end
        XFM.xp.btv_s_XPLMSetDatai(ref, 0)
        if not P.on then
            P.on, P.stale = true, true
            XP.btv_XPLMSetWindowIsVisible(P.win, 1)
            pnl_log("ATCCOM popup: our window instead of FF's popup")
        end
    elseif P.on and not (a.page and PNL.atc_lit("lower")) then
        P.on = false
        PNL.atp_save()
        XP.btv_XPLMSetWindowIsVisible(P.win, 0)
        XFM.xp.btv_s_XPLMSetDatai(ref, 1)
        pnl_log("ATCCOM popup: MFD left ATCCOM - back to FF's popup")
    end
end

function PNL.atp_draw()
    local P, a = PNL.atp, PNL.atc
    if not P.on or not a.ui or a.failed then return end
    local g = P.g
    XP.btv_a_XPLMGetWindowGeometry(P.win, g, g + 1, g + 2, g + 3)
    local l, t, r, b = g[0], g[1], g[2], g[3]
    -- like FF's popup: the page at the window's width, below a strip with "PRESS FOR KEY INPUT" (59 page units)
    local k = (r - l) / 679
    local ph = 884 + PNL.ATP_STRIP
    local w, h = 679 * k, ph * k
    P.geo = { l, t, k }
    if a.ui.dirty then a.cache, a.ui.dirty, P.stale = {}, false, true end
    local key = table.concat({ l, t, r, b, TF.ok and 1 or 0, TF.id or 0, a.ui.focus and 1 or 0 }, ":")
    if P.stale or P.key ~= key or not P.buf then
        P.stale, P.key = false, key
        P.buf = PNL.atc_build({ 0, 0, w, h, 679, ph }, l, t - h, l + 2048, t - h + 2048, P.buf or 180000, function()
            DL.rectf(0, 0, 679, ph, 0xFF000000)
            DL.text(18, 10, 0xFFFFFFFF, "PRESS FOR KEY INPUT", 11 / 0.7229)
            PNL.atc_ops(a.ui.lower(), PNL.ATP_STRIP)
            PNL.atc_ops(a.ui.syskey(), PNL.ATP_STRIP)
        end)
    end
    local list = { P.buf }
    -- FF's yellow MFD cursor where the mouse is (the window got a cursor call in the last frames)
    if P.cur and PNL.frame - P.cur.f <= 2 then
        local c = P.cur
        local ok, cbuf = pcall(PNL.atc_build, { 0, 0, w, h, 679, ph }, l, t - h, l + 2048, t - h + 2048, P.cbuf or 256,
            function() PNL.atc_cursor_shape(c.x, c.y + PNL.ATP_STRIP) end, true)
        if ok then P.cbuf = cbuf; list[2] = cbuf end
    end
    PNL.submit_list(list)
end

-- the window's place and size for next time (only when the pilot changed it)
function PNL.atp_save()
    local P = PNL.atp
    if P.win == nil or not P.g then return end
    XP.btv_a_XPLMGetWindowGeometry(P.win, P.g, P.g + 1, P.g + 2, P.g + 3)
    local line = string.format("%d %d %d %d", P.g[0], P.g[1], P.g[2], P.g[3])
    if line == P.saved then return end
    local f = io.open(SCRIPTS .. "A350_ATCCOM/popup_window.txt", "w")
    if f then f:write(line, "\n"); f:close(); P.saved = line end
end

-- window position -> page units (nil outside the page)
function PNL.atp_page(x, y)
    local g = PNL.atp.geo
    if not g then return nil end
    local px, py = (x - g[1]) / g[3], (g[2] - y) / g[3] - PNL.ATP_STRIP
    if px < 0 or px > 679 or py < 0 or py > 884 then return nil end
    return px, py
end

-- true = the mouse is over the page (our cursor is drawn there)
function PNL.atp_hover(x, y)
    local P, ui = PNL.atp, PNL.atc.ui
    if not ui then return false end
    local px, py = PNL.atp_page(x, y)
    ui.hover(px and "lower" or nil, px, py)
    P.cur = px and { x = px, y = py, f = PNL.frame } or nil
    return px ~= nil
end

function PNL.atp_click(x, y, st)
    local P, a = PNL.atp, PNL.atc
    if st ~= 1 or not a.ui then return end
    local ui = a.ui
    local px, py = PNL.atp_page(x, y)
    if not px then return end
    -- system key and list: FF's MFD is switched directly (FF's own list is not on screen here)
    if py < 32 and px <= 195 then
        ui.blur(); ui.sysmenu_toggle()
        return
    end
    if ui.sysmenu_open() then
        ui.sys_pick = nil
        ui.click("lower", px, py)
        local n = ui.sys_pick
        ui.sys_pick = nil
        PNL.atc_sys_set(n)
        return
    end
    local had = ui.focus ~= nil
    ui.click("lower", px, py)
    if ui.focus then pcall(XP.btv_XPLMTakeKeyboardFocus, P.win)
    elseif had then pcall(XP.btv_XPLMTakeKeyboardFocus, nil) end
end
-- END ATCCOM

local function pnl_draw(src)
    if MCW.on then MCW.late() end
    if not PNL.ok or PNL.dead then return end
    if PNL.atc.msg.on and not PNL.atc.failed and XP.btv_XPLMGetElapsedTime() - (PNL.ft or 0) < 1 then
        -- ATC MSG-Lampen nach dem FF erneut setzen (wie die BTV-Lampe)
        pcall(PNL.atc_msg_late)
    end
    -- laeuft das Skript noch? (FlyWithLua haelt nach einem Fehler alle Skripte an - dann nichts mehr zeichnen)
    if XP.btv_XPLMGetElapsedTime() - PNL.ft > 1.0 then return end
    local sd0, sd1 = PNL.sd[0], PNL.sd[1]
    local pfd = CFG.nd_on and (PFD.msg() ~= nil or PFD.fma() ~= nil)
    if not (sd0.show or sd1.show or pfd or PNL.need_samp or PNL.atc_any()) then return end
    local rt = XP.btv_XPLMGetDatai(PNL.r_rt)
    if src == 0 then PNL.seen[rt] = (PNL.seen[rt] or 0) + 1 end
    if rt ~= 1 and rt ~= 2 then return end
    local lit = (rt == 2) or PNL.lit_in_1
    local pl, pb = XP.btv_XPLMGetDataf(PNL.r_pl), XP.btv_XPLMGetDataf(PNL.r_pb)
    local pr, pt = XP.btv_XPLMGetDataf(PNL.r_pr), XP.btv_XPLMGetDataf(PNL.r_pt)
    if not (pr - pl > 100 and pt - pb > 100) then pl, pb, pr, pt = 0, 0, 2048, 2048 end
    -- welcher Aufruf zeichnet? (FlyWithLua = 0 ist zu frueh, FF zeichnet danach; eine der Bruecken ist spaet genug)
    if PNL.hook == nil then
        if lit then
            if src == 0 then pnl_mark(pl, pb, pr, pt) else pnl_probe(src) end
        end
        return
    end
    if PNL.hook ~= src then return end
    -- was zeigt das FF gerade (Strom, IRS, Autobrake)? Gelesen nach dem FF und vor der Karte, alle 12 Bilder
    if lit and PNL.samp_f ~= PNL.frame and not PNL.samp_off then
        local ok, e = pcall(PNL.sample_poll)
        if ok and not PNL.as_pend and PNL.frame - (PNL.samp_f or -100) >= 12 then
            PNL.samp_f = PNL.frame
            ok, e = pcall(PNL.sample_req, pl, pb, pr, pt)
        end
        if ok and PNL.as_ok == false then ok, e = false, "no pixel buffers" end
        if not ok then
            PNL.samp_off = true
            -- Pruefung nicht moeglich: Bildschirme als "an" annehmen (die Karte darf daran nie scheitern)
            PNL.sd[0].ffst, PNL.sd[1].ffst, S.abrk = "ok", "ok", false
            if not PNL.samp_err then PNL.samp_err = true; pnl_log("display check: " .. tostring(e)) end
        end
    end
    if not lit then
        -- Tagestextur: schwarz hinter Karte bzw. Kaestchen (der Bildschirm leuchtet nur in der Leuchttextur)
        local v, c, n = PNL.av, PNL.ac, 0
        for s = 0, 1 do
            local sd = PNL.sd[s]
            if pfd and PFD.msg() and sd.avail ~= false and not sd.nomap then
                local uv = PFD.uv[s]
                local x0, x1 = pl + uv[1] * (pr - pl), pl + uv[3] * (pr - pl)
                local y1 = pb + uv[4] * (pt - pb)
                local fx, fy = (x1 - x0) / ND_VW, (pb + uv[2] * (pt - pb) - y1) / ND_VH
                local hw = PFD.last_hw or 45
                local bx0, bx1 = x0 + (PFD.box[1] - hw) * fx, x0 + (PFD.box[1] + hw) * fx
                local by0, by1 = y1 + (PFD.box[2] - 10) * fy, y1 + (PFD.box[2] + 10) * fy
                local i = 2 * n
                v[i], v[i + 1], v[i + 2], v[i + 3], v[i + 4], v[i + 5] = bx0, by0, bx1, by0, bx1, by1
                v[i + 6], v[i + 7], v[i + 8], v[i + 9], v[i + 10], v[i + 11] = bx0, by0, bx1, by1, bx0, by1
                for k = 0, 5 do c[n + k] = 0xFF000000 end
                n = n + 6
            end
            local mapv = sd.vm or (sd.fade or 0) > 0.02
            if sd.show and sd.avail ~= false and mapv then
                local uv, pd = ND_UV[s], ND_PAD[s]
                local x0, x1 = pl + uv[1] * (pr - pl), pl + uv[3] * (pr - pl)
                local y0, y1 = pb + uv[2] * (pt - pb), pb + uv[4] * (pt - pb)
                local bc = 0xFF000000
                if mapv and (sd.fade or 1) < 1 then bc = bit.lshift(math.floor(PNL.ease(sd.fade) * 255 + 0.5), 24) end
                x0, y0, x1, y1 = x0 - pd[1], y0 - pd[2], x1 + pd[3], y1 + pd[4]
                local i = 2 * n
                v[i], v[i + 1], v[i + 2], v[i + 3], v[i + 4], v[i + 5] = x0, y0, x1, y0, x1, y1
                v[i + 6], v[i + 7], v[i + 8], v[i + 9], v[i + 10], v[i + 11] = x0, y0, x1, y1, x0, y1
                for k = 0, 5 do c[n + k] = bc end
                n = n + 6
            end
        end
        pnl_submit(v, c, n)
        return
    end
    if TF.pending then pnl_font_upload(); PNL.lkey = nil end
    -- einmal je Bild (und Panel-Lage) aufbauen; weitere Leucht-Durchgaenge im selben Bild zeichnen dieselbe Liste
    local lkey = string.format("%d/%.1f/%.1f/%.1f/%.1f", PNL.frame, pl, pb, pr, pt)
    if PNL.lkey ~= lkey then
        PNL.lkey = lkey
        PNL.dcount = (PNL.dcount or 0) + 1
        PNL.guard_draw()
        PNL.trace("start")
        local t0 = PNL.now()
        local dpu, vw, vh = pnl_dpu()
        if not (dpu > 0.5 and dpu < 200) then dpu = 8 end
        GLB.px = 1 / dpu
        -- Schriftmass des Cockpit-Displays (das Fenster setzt sein eigenes)
        FONT_H = TXT.h
        MENU.lh = FONT_H + 4
        PNL.dirty = NDUI.dirty
        NDUI.dirty = false
        local list = {}
        local geos = {}
        PNL.ub = PNL.ub or PNL.newbuf(64)
        PNL.pb = PNL.pb or PNL.newbuf(PNL.PFD_CAP)
        -- unterste Schicht: schwarz hinter der Karte, etwas ueber den Rand des ND hinaus
        PNL.use(PNL.ub)
        for s = 0, 1 do
            local sd = PNL.sd[s]
            local mapv = sd.vm or (sd.fade or 0) > 0.02
            if sd.show and sd.avail ~= false and mapv then
                geos[s] = PNL.geo(s, pl, pb, pr, pt)
                if mapv then
                    local p = geos[s].pad
                    GLB.fa = ((sd.fade or 1) < 1) and PNL.ease(sd.fade) or nil
                    glb_tri(p[1], p[2], p[3], p[2], p[3], p[4], MC.bg)
                    glb_tri(p[1], p[2], p[3], p[4], p[1], p[4], MC.bg)
                    GLB.fa = nil
                end
            end
        end
        PNL.ub.n = GLB.n
        list[1] = { v = PNL.ub.v, c = PNL.ub.c, t = PNL.ub.t, n = PNL.ub.n }
        local nv = 0
        for s = 0, 1 do
            local sd = PNL.sd[s]
            local g = geos[s]
            if g then
                GLB.ts = g.ts
                local ok, err = pcall(pnl_side, sd, g)
                GLB.fa, DL.H, MAPX.bud = nil, nil, nil
                PNL.use0()
                if not ok then error(err, 0) end
                if sd.draw_map and sd.mb then
                    local b = sd.mb.buf
                    list[#list + 1] = { v = b.v, c = b.c, t = b.t, n = b.n, mat = sd.mat, scis = g.scis }
                    nv = nv + b.n
                end
                local o = sd.ob
                if o then list[#list + 1] = { v = o.v, c = o.c, t = o.t, n = o.n, scis = g.scis }; nv = nv + o.n end
            else
                sd.job = nil -- (unsichtbare Seite baut nicht weiter)
            end
        end
        -- PFD: "RWY AHEAD" und "BTV"
        PNL.use(PNL.pb)
        if pfd then
            for s = 0, 1 do
                if PNL.sd[s].avail ~= false and not PNL.sd[s].nomap then
                    local uv = PFD.uv[s]
                    local x0, x1 = pl + uv[1] * (pr - pl), pl + uv[3] * (pr - pl)
                    local y0, y1 = pb + uv[2] * (pt - pb), pb + uv[4] * (pt - pb)
                    local sx, sy = (x1 - x0) / ND_VW, (y1 - y0) / ND_VH
                    DL.begin({ sx, 0, x0, 0, -sy, y1, 0, 0 }, ND_VW, ND_VH)
                    GLB.ts = (sx + sy) / 2
                    local ok, err = pcall(PFD.draw, s)
                    DL.H = nil
                    if not ok then PNL.use0(); error(err, 0) end
                end
            end
        end
        PNL.pb.n = GLB.n
        PNL.use0()
        if PNL.pb.n > 0 then list[#list + 1] = { v = PNL.pb.v, c = PNL.pb.c, t = PNL.pb.t, n = PNL.pb.n } end
        PNL.list = list
        PNL.trace(string.format("list of %d buffers, layers %s", #list, PNL.nomat and "off" or "on"))
        local ms = (PNL.now() - t0) * 1000
        local st = PNL.stat
        st.ms, st.frames, st.maxms = st.ms + ms, st.frames + 1, math.max(st.maxms, ms)
        -- alle ~1800 Bilder ins Log: Rechenzeit der Karte (zum Nachsehen in Log.txt)
        st.lf = (st.lf or 0) + 1
        st.lms, st.lmax = (st.lms or 0) + ms, math.max(st.lmax or 0, ms)
        if st.lf >= 1800 then
            pnl_log(string.format("CPU %.2f ms per frame (max %.1f), %d map rebuilds", st.lms / st.lf, st.lmax, st.jobs - (st.ljobs or 0)))
            st.lf, st.lms, st.lmax, st.ljobs = 0, 0, 0, st.jobs
        end
        if PNL.diag < 1 then
            PNL.diag = 1
            pnl_log(string.format("drawing into the cockpit display: panel %.0f/%.0f-%.0f/%.0f, viewport %dx%d, %.1f pixels per panel unit, " ..
                "%d vertices, found bound: %s, layers %s", pl, pb, pr, pt, vw, vh, dpu, nv, PNL.diag_bind or "-",
                PNL.nomat and "off (no matrix functions)" or "on"))
        end
    end
    PNL.submit_list(PNL.list)
    if (PNL.dcount or 0) <= 2 and PNL.tr_done ~= PNL.dcount then PNL.tr_done = PNL.dcount; PNL.trace("drawn") end
end

-- weich 0..1 (Ein-/Ausblenden)
function PNL.ease(f) f = clamp(f or 1, 0, 1); return f * f * (3 - 2 * f) end

local function pnl_draw_error(e)
    PNL.nerr = PNL.nerr + 1
    if PNL.nerr <= 3 then pnl_log("draw error: " .. tostring(e)) end
    if PNL.nerr >= 3 then
        PNL.dead = true
        PNL.st = "disabled after errors (see Log.txt)"
    end
end

-------------------------------------------------------------------------------
-- Maus: Bildschirmpunkt -> ND-Koordinaten
-- Aus X-Planes eigener 3D-Kameramatrix (world_matrix, projection_matrix_3d - gelten auch im 2D-Durchgang),
-- Flugzeuglage und den ND-Ecken: die Ecken werden auf den Bildschirm projiziert, eine Homographie
-- bildet den Mauspunkt auf die ND-Flaeche ab. Funktioniert bei jedem Blickwinkel und Zoom.
-------------------------------------------------------------------------------
-- Abbildung Einheitsquadrat -> Viereck (p0 = oben links, p1 = oben rechts, p2 = unten rechts, p3 = unten links)
local function quad_homography(p, VW, VH)
    local x0, y0, x1, y1, x2, y2, x3, y3 = p[1][1], p[1][2], p[2][1], p[2][2], p[3][1], p[3][2], p[4][1], p[4][2]
    local dx1, dx2, dx3 = x1 - x2, x3 - x2, x0 - x1 + x2 - x3
    local dy1, dy2, dy3 = y1 - y2, y3 - y2, y0 - y1 + y2 - y3
    local a, b, c, d, e, f, g, h
    local den = dx1 * dy2 - dx2 * dy1
    if math.abs(dx3) < 1e-9 and math.abs(dy3) < 1e-9 or math.abs(den) < 1e-12 then
        a, b, c, d, e, f, g, h = x1 - x0, x3 - x0, x0, y1 - y0, y3 - y0, y0, 0, 0
    else
        g = (dx3 * dy2 - dx2 * dy3) / den
        h = (dx1 * dy3 - dx3 * dy1) / den
        a, b, c = x1 - x0 + g * x1, x3 - x0 + h * x3, x0
        d, e, f = y1 - y0 + g * y1, y3 - y0 + h * y3, y0
    end
    -- lokale Pixel (0..VW, 0..VH) statt Einheitsquadrat
    local H = { a / VW, b / VH, c, d / VW, e / VH, f, g / VW, h / VH }
    local m11, m12, m13, m21, m22, m23, m31, m32, m33 = H[1], H[2], H[3], H[4], H[5], H[6], H[7], H[8], 1
    local i11, i12, i13 = m22 * m33 - m23 * m32, m13 * m32 - m12 * m33, m12 * m23 - m13 * m22
    local i21, i22, i23 = m23 * m31 - m21 * m33, m11 * m33 - m13 * m31, m13 * m21 - m11 * m23
    local i31, i32, i33 = m21 * m32 - m22 * m31, m12 * m31 - m11 * m32, m11 * m22 - m12 * m21
    local function inv(X, Y)
        local w = i31 * X + i32 * Y + i33
        if math.abs(w) < 1e-12 then return -9999, -9999 end
        return (i11 * X + i12 * Y + i13) / w, (i21 * X + i22 * Y + i23) / w
    end
    return H, inv
end

-- Pixel -> X-Plane-Fensterkoordinaten (Boxel): Faktor und Versatz
local function pnl_ui(W)
    local d = PNL.dr
    local s, ox, oy = 1, 0, 0
    local have = d.uis and d.uis[0] > 0.4 and d.uis[0] < 5
    if have then s = d.uis[0] end
    if XPLMGetScreenBoundsGlobal then
        local ok, l, t, r, b = pcall(XPLMGetScreenBoundsGlobal)
        if ok and type(l) == "number" and type(r) == "number" and r > l then
            if not have then s = W / (r - l) end
            if math.abs((r - l) * s - W) < 4 then ox, oy = l, b end
        end
    end
    return s, ox, oy
end

-- jeden Frame im 2D-Durchgang (btv_draw): ND-Ecken auf den Bildschirm projizieren
local function pnl_cam()
    local d = PNL.dr
    for s = 0, 1 do PNL.sd[s].inv = nil end
    PNL.atc.inv, PNL.atc.quad = {}, {}
    if not (d.wm and d.pm and d.lx) then return end
    local m, p = {}, {}
    for i = 0, 15 do m[i], p[i] = d.wm[i], d.pm[i] end
    if p[15] ~= 0 or (p[0] == 0 and p[5] == 0) or (m[0] == 0 and m[5] == 0 and m[10] == 0) then return end
    local W = (d.ww and d.ww[0] > 100) and d.ww[0] or SCREEN_WIDTH
    local Hs = (d.wh and d.wh[0] > 100) and d.wh[0] or SCREEN_HIGHT
    if not W or not Hs then return end
    local us, ox, oy = pnl_ui(W)
    local lx, ly, lz = d.lx[0], d.ly[0], d.lz[0]
    local ps, th, ph = d.psi[0] * D2R, d.the[0] * D2R, d.phi[0] * D2R
    local cps, sps, cth, sth, cph, sph = math.cos(ps), math.sin(ps), math.cos(th), math.sin(th), math.cos(ph), math.sin(ph)
    -- Flugzeug -> OpenGL-Weltkoordinaten (nur Drehung): Rollen, Nicken, Kurs
    local function rot(x, y, z)
        local x1, y1, z1 = x * cph + y * sph, -x * sph + y * cph, z
        local y2, z2 = y1 * cth - z1 * sth, y1 * sth + z1 * cth
        return x1 * cps - z2 * sps, y2, x1 * sps + z2 * cps
    end
    -- Gegenprobe: die Kamera der Matrix muss beim Kopf des Piloten sitzen. Sonst (die Matrix gilt im 2D-Durchgang
    -- doch nicht) wird die Kamera aus Kopfposition und Blickwinkel gebaut.
    if d.hx and d.hpsi then
        local hx, hy, hz = rot(d.hx[0], d.hy[0], d.hz[0])
        local cx = -(m[0] * m[12] + m[1] * m[13] + m[2] * m[14])
        local cy = -(m[4] * m[12] + m[5] * m[13] + m[6] * m[14])
        local cz = -(m[8] * m[12] + m[9] * m[13] + m[10] * m[14])
        local dist = math.sqrt((cx - lx - hx) ^ 2 + (cy - ly - hy) ^ 2 + (cz - lz - hz) ^ 2)
        local head = dist > 1.5
        if head then
            local hp, ht, hr = d.hpsi[0] * D2R, d.hthe[0] * D2R, (d.hphi and d.hphi[0] or 0) * D2R
            local sp, cp, st, ct = math.sin(hp), math.cos(hp), math.sin(ht), math.cos(ht)
            local fx, fy, fz = ct * sp, st, -ct * cp
            local rx, ry, rz = cp, 0, sp
            local ux, uy, uz = -st * sp, ct, st * cp
            if hr ~= 0 then
                local sf, cf = math.sin(hr), math.cos(hr)
                rx, ry, rz, ux, uy, uz = rx * cf - ux * sf, ry * cf - uy * sf, rz * cf - uz * sf,
                                         ux * cf + rx * sf, uy * cf + ry * sf, uz * cf + rz * sf
            end
            fx, fy, fz = rot(fx, fy, fz)
            rx, ry, rz = rot(rx, ry, rz)
            ux, uy, uz = rot(ux, uy, uz)
            local ex, ey, ez = lx + hx, ly + hy, lz + hz
            m = { [0] = rx, ux, -fx, 0, ry, uy, -fy, 0, rz, uz, -fz, 0,
                  -(rx * ex + ry * ey + rz * ez), -(ux * ex + uy * ey + uz * ez), fx * ex + fy * ey + fz * ez, 1 }
        end
        if head ~= PNL.cam_head and (PNL.ncam or 0) < 4 then
            PNL.cam_head, PNL.ncam = head, (PNL.ncam or 0) + 1
            pnl_log(head and string.format("mouse: camera from head position (matrix off by %.1f m)", dist)
                or "mouse: camera from X-Plane 3D matrix")
        end
    end
    for s = 0, 1 do
        local sd = PNL.sd[s]
        if sd.show then
            local q = {}
            for k, c in ipairs(ND_GEOM[s]) do
                local X, Y, Z = rot(c[1], c[2], c[3])
                X, Y, Z = lx + X, ly + Y, lz + Z
                local ex = m[0] * X + m[4] * Y + m[8] * Z + m[12]
                local ey = m[1] * X + m[5] * Y + m[9] * Z + m[13]
                local ez = m[2] * X + m[6] * Y + m[10] * Z + m[14]
                local ew = m[3] * X + m[7] * Y + m[11] * Z + m[15]
                local cx = p[0] * ex + p[4] * ey + p[8] * ez + p[12] * ew
                local cy = p[1] * ex + p[5] * ey + p[9] * ez + p[13] * ew
                local cw = p[3] * ex + p[7] * ey + p[11] * ez + p[15] * ew
                if cw < 1e-4 then q = nil; break end
                local sx, sy = (cx / cw * 0.5 + 0.5) * W, (cy / cw * 0.5 + 0.5) * Hs
                q[k] = { ox + sx / us, oy + sy / us }
            end
            if q then
                local _, inv = quad_homography(q, ND_VW, ND_VH)
                sd.inv, sd.quad = inv, q
            end
        end
    end
    -- ATCCOM (unteres MFD, Mailbox oben): Bildschirm -> Seite
    if PNL.atc_ready() then
        for which, g in pairs(PNL.atc.geom) do
            local q = {}
            for k, c in ipairs(g) do
                local X, Y, Z = rot(c[1], c[2], c[3])
                X, Y, Z = lx + X, ly + Y, lz + Z
                local ex = m[0] * X + m[4] * Y + m[8] * Z + m[12]
                local ey = m[1] * X + m[5] * Y + m[9] * Z + m[13]
                local ez = m[2] * X + m[6] * Y + m[10] * Z + m[14]
                local ew = m[3] * X + m[7] * Y + m[11] * Z + m[15]
                local cx = p[0] * ex + p[4] * ey + p[8] * ez + p[12] * ew
                local cy = p[1] * ex + p[5] * ey + p[9] * ez + p[13] * ew
                local cw = p[3] * ex + p[7] * ey + p[11] * ez + p[15] * ew
                if cw < 1e-4 then q = nil; break end
                q[k] = { ox + (cx / cw * 0.5 + 0.5) * W / us, oy + (cy / cw * 0.5 + 0.5) * Hs / us }
            end
            if q then
                local r = PNL.atc.regions[which]
                local _, inv = quad_homography(q, r[5], r[6])
                PNL.atc.inv[which], PNL.atc.quad[which] = inv, q
            end
        end
    end
    -- MASTER-CAUT-Tasten auf dem Bildschirm (nur solange MASTER CAUT leuchtet): Rechteck mit etwas Rand
    PNL.mcq = nil
    if MCW.on then
        local r = {}
        for _, g in ipairs(ND_GEOM.mc) do
            local x0, y0, x1, y1, bad
            for _, c in ipairs(g) do
                local X, Y, Z = rot(c[1], c[2], c[3])
                X, Y, Z = lx + X, ly + Y, lz + Z
                local ex = m[0] * X + m[4] * Y + m[8] * Z + m[12]
                local ey = m[1] * X + m[5] * Y + m[9] * Z + m[13]
                local ez = m[2] * X + m[6] * Y + m[10] * Z + m[14]
                local ew = m[3] * X + m[7] * Y + m[11] * Z + m[15]
                local cx = p[0] * ex + p[4] * ey + p[8] * ez + p[12] * ew
                local cy = p[1] * ex + p[5] * ey + p[9] * ez + p[13] * ew
                local cw = p[3] * ex + p[7] * ey + p[11] * ez + p[15] * ew
                if cw < 1e-4 then bad = true; break end
                local sx = ox + (cx / cw * 0.5 + 0.5) * W / us
                local sy = oy + (cy / cw * 0.5 + 0.5) * Hs / us
                x0, y0 = math.min(x0 or sx, sx), math.min(y0 or sy, sy)
                x1, y1 = math.max(x1 or sx, sx), math.max(y1 or sy, sy)
            end
            if not bad and x0 then
                local mx, my = math.max(6, (x1 - x0) * 0.3), math.max(6, (y1 - y0) * 0.3)
                r[#r + 1] = { x0 - mx, y0 - my, x1 + mx, y1 + my }
            end
        end
        PNL.mcq = r
    end
    -- Range-Knoepfe auf dem Bildschirm (Mitte, Fangradius) fuer das Mausrad (PNL.knob_wheel)
    local function scr(bx, by, bz)
        local X, Y, Z = rot(bx, by, bz)
        X, Y, Z = lx + X, ly + Y, lz + Z
        local ex = m[0] * X + m[4] * Y + m[8] * Z + m[12]
        local ey = m[1] * X + m[5] * Y + m[9] * Z + m[13]
        local ez = m[2] * X + m[6] * Y + m[10] * Z + m[14]
        local ew = m[3] * X + m[7] * Y + m[11] * Z + m[15]
        local cx = p[0] * ex + p[4] * ey + p[8] * ez + p[12] * ew
        local cy = p[1] * ex + p[5] * ey + p[9] * ez + p[13] * ew
        local cw = p[3] * ex + p[7] * ey + p[11] * ez + p[15] * ew
        if cw < 1e-4 then return nil end
        return ox + (cx / cw * 0.5 + 0.5) * W / us, oy + (cy / cw * 0.5 + 0.5) * Hs / us
    end
    local kq, kr = {}, ND_GEOM.knob.r
    for s = 0, 1 do
        local c = ND_GEOM.knob[s]
        local x0, y0 = scr(c[1], c[2], c[3])
        local x1, y1 = scr(c[1] + kr, c[2], c[3])
        local x2, y2 = scr(c[1], c[2] + kr, c[3])
        if x0 and x1 and x2 then
            kq[s] = { x0, y0, math.max(6, math.sqrt((x1 - x0) ^ 2 + (y1 - y0) ^ 2), math.sqrt((x2 - x0) ^ 2 + (y2 - y0) ^ 2)) }
        end
    end
    PNL.knq = kq
    do
        local c, r = ND_GEOM.abrk, ND_GEOM.abrk.r
        local x0, y0 = scr(c[1], c[2], c[3])
        local x1, y1 = scr(c[1] + r, c[2], c[3])
        PNL.abq = (x0 and x1) and { x0, y0, math.max(6, math.sqrt((x1 - x0) ^ 2 + (y1 - y0) ^ 2)) } or nil
    end
end

local function pnl_hit(x, y)
    for s = 0, 1 do
        local sd = PNL.sd[s]
        if sd.show and sd.avail ~= false and sd.vm and sd.inv then
            local lx, ly = sd.inv(x, y)
            if lx >= 0 and lx <= ND_VW and ly >= 0 and ly <= ND_VH then return s, lx, ly end
        end
    end
    return nil
end


local function pnl_cursor(x, y)
    if PNL.atc_cursor(x, y) then return 1 end
    PNL.ncur = (PNL.ncur or 0) + 1
    local now = XP.btv_XPLMGetElapsedTime()
    if PNL.shake_max and now - (PNL.shake_t or 0) > 60 and (PNL.nshake or 0) < 5 then
        if PNL.shake_max > 3 then
            PNL.nshake = (PNL.nshake or 0) + 1
            pnl_log(string.format("mouse: view shakes up to %.1f ND units (cursor held still)", PNL.shake_max))
        end
        PNL.shake_t, PNL.shake_max = now, 0
    end
    local s, lx, ly = pnl_hit(x, y)
    for k = 0, 1 do if k ~= s then PNL.sd[k].hin = false end end
    if s then
        local sd = PNL.sd[s]
        -- Vibrationen des Flugzeugs (X-Plane-Kopfwackeln, FF-Effekte) bewegen die Abbildung Bildschirm -> ND, nicht
        -- die Maus: nur die Mausbewegung uebernehmen (alter und neuer Punkt mit derselben Abbildung) - steht die Maus,
        -- steht das Fadenkreuz. Die Abweichung zum berechneten Punkt schwankt beim Wackeln um ihren Mittelwert; nur
        -- dieser gemittelte Anteil (Kopf bewegt, Ansicht verschoben) wird langsam nachgezogen. Sehr weit weg
        -- (Ansicht gewechselt): neu ansetzen.
        if sd.hin and sd.hsx and sd.inv then
            local ox, oy = sd.inv(sd.hsx, sd.hsy)
            local nx, ny = sd.hx + lx - ox, sd.hy + ly - oy
            local dx, dy = lx - nx, ly - ny
            if math.abs(dx) < 25 and math.abs(dy) < 25 then
                local fx, fy = sd.fdx or 0, sd.fdy or 0
                fx, fy = fx + (dx - fx) * 0.04, fy + (dy - fy) * 0.04
                if math.abs(fx) > 0.75 then nx = nx + fx * 0.08 end
                if math.abs(fy) > 0.75 then ny = ny + fy * 0.08 end
                sd.fdx, sd.fdy = fx, fy
                lx, ly = nx, ny
                -- Diagnose: wie stark wackelt die Ansicht bei stehender Maus (ND-Einheiten)?
                if x == sd.hsx and y == sd.hsy then
                    local a = math.max(math.abs(dx - fx), math.abs(dy - fy))
                    if a > (PNL.shake_max or 0) then PNL.shake_max = a end
                end
            else
                sd.fdx, sd.fdy = 0, 0
            end
        else
            sd.fdx, sd.fdy = 0, 0
        end
        sd.hin, sd.hx, sd.hy, sd.hsx, sd.hsy = true, lx, ly, x, y
        if sd.vm and PNL.hook then return 1 end -- xplm_CursorHidden
    end
    return 0
end

-- Tastatur (nur waehrend einer Eingabe im Bedienfeld hat das Mausfenster den Fokus)
function PNL.key(k, fl, vk, lf)
    if PNL.atc_key(k, fl, vk, lf) then return end
    if lf ~= 0 then NDUI.kb = nil; return end
    local kb = NDUI.kb
    if not kb or bit.band(fl, 8) == 0 then return end -- nur "gedrueckt"
    local sd = PNL.sd[kb.side]
    sd.q[#sd.q + 1] = { t = "key", ch = bit.band(k, 0xFF), vk = bit.band(vk, 0xFF) }
end

-- Diagnose: Mausabbildung neben click_3d (Texturkoordinate unter der Maus laut X-Plane) ins Log
local function pnl_diag_click(s, lx, ly)
    PNL.nclick = PNL.nclick + 1
    if PNL.nclick > 6 then return end
    local d = PNL.dr
    local a = "-"
    if d.c3x and d.c3y then a = string.format("%.4f/%.4f", d.c3x[0], d.c3y[0]) end
    local uv = ND_UV[s]
    pnl_log(string.format("click side %d at %.0f/%.0f (u/v %.4f/%.4f), click_3d %s", s, lx, ly,
        uv[1] + lx / ND_VW * (uv[3] - uv[1]), uv[4] - ly / ND_VH * (uv[4] - uv[2]), a))
end

local function pnl_click(x, y, st, right)
    if PNL.atc_click(x, y, st, right) then return true end
    -- rechte Maustaste bleibt bei X-Plane (Umschauen); Stoppunkt per Rechtsklick nur im Fenster
    if right then return false end
    -- Klick auf eine MASTER-CAUT-Taste: MASTER CAUT aus (der Klick geht trotzdem an das FF)
    if st == 1 and MCW.on and PNL.mcq then
        for _, b in ipairs(PNL.mcq) do
            if x >= b[1] and x <= b[3] and y >= b[2] and y <= b[4] then
                pcall(MCW.clear, "MASTER CAUT pushed (click)")
                break
            end
        end
    end
    if st == 1 and PNL.knob_click(x, y) then return true end
    if st == 1 and PNL.abrk_click(x, y) then return true end
    if st == 1 then
        local s, lx, ly = pnl_hit(x, y)
        -- Klick ausserhalb des ND mit der Eingabe: Eingabe beenden (Tastatur wieder fuer X-Plane)
        if NDUI.kb and s ~= NDUI.kb.side then NDUI.kb_end() end
        if not s then return false end
        local sd = PNL.sd[s]
        if not sd.vm then return false end -- Karte aus: das FF-ND bleibt bedienbar
        if right and not sd.vm then return false end
        -- an derselben Mausstelle: der ruhig gehaltene Punkt des Fadenkreuzes (siehe pnl_cursor)
        if sd.hin and sd.hsx == x and sd.hsy == y then lx, ly = sd.hx, sd.hy end
        PNL.cap_xy = { x, y, lx, ly }
        PNL.cap, PNL.cap_right = s, right
        sd.q[#sd.q + 1] = { t = right and "rdown" or "down", x = lx, y = ly, sx = x, sy = y, tm = XP.btv_XPLMGetElapsedTime() }
        pnl_diag_click(s, lx, ly)
        return true
    end
    local s = PNL.cap
    if s == nil or PNL.cap_right ~= right then return false end
    local sd = PNL.sd[s]
    local lx, ly = -9999, -9999
    if sd.inv then lx, ly = sd.inv(x, y) end
    local c = PNL.cap_xy
    if c and math.abs(x - c[1]) < 1 and math.abs(y - c[2]) < 1 then lx, ly = c[3], c[4] end
    if st == 2 then
        if not right and lx > -9000 then sd.q[#sd.q + 1] = { t = "drag", x = lx, y = ly, sx = x, sy = y } end
    else
        if not right then sd.q[#sd.q + 1] = { t = "up", x = lx, y = ly, sx = x, sy = y, tm = XP.btv_XPLMGetElapsedTime() } end
        PNL.cap = nil
    end
    return true
end

-- Range-Knopf (Mausrad ueber dem Knopf): das Mausfenster liegt dann ueber dem Knopf (siehe pnl_frame), das FF
-- bekommt das Rad dort nicht - das Skript dreht den Knopf selbst ueber alle Stellungen: 640 .. 10 (0..6),
-- ZOOM (-1) und auf ZOOM die OANS-Reichweiten. Rad nach oben = im Uhrzeigersinn = groessere Reichweite.
-- (Frueher lauschte das Skript dem FF und lernte die Drehrichtung - das schwankte, und das Fenster ueber dem
-- ganzen Bildschirm sperrte die anderen Knoepfe des FF.)
function PNL.knob_turn(s, up)
    local d = PNL.dr
    local rr = (s == 0) and d.zl or d.zr
    if not rr then return end
    local v = math.floor(rr[0] + 0.5)
    if v < 0 then
        if PNL.zoom_step(s, not up) then rr[0] = 0 end
    elseif up then
        rr[0] = math.min(6, v + 1)
    elseif v == 0 then
        rr[0] = -1
    else
        rr[0] = v - 1
    end
end
function PNL.knob_at(x, y)
    local q = PNL.knq
    if not q then return nil end
    for s = 0, 1 do
        local k = q[s]
        if k and (x - k[1]) ^ 2 + (y - k[2]) ^ 2 <= k[3] ^ 2 then return s, k end
    end
end
function PNL.knob_wheel(x, y, clicks)
    local s = PNL.knob_at(x, y)
    if not s then return false end
    local n = math.min(3, math.abs(clicks))
    for _ = 1, n do PNL.knob_turn(s, clicks > 0) end
    return true
end
-- A/BRK-Taste im Flug: BTV scharf / aus (wie im A350; am Boden bleibt die Taste beim FF = RTO)
function PNL.abrk_click(x, y)
    local k = PNL.abq
    if not (k and in_air()) then return false end
    if math.abs(x - k[1]) > k[3] or math.abs(y - k[2]) > k[3] then return false end
    pcall(AUD.play, "click")
    btv_arm_toggle()
    return true
end
-- Klick auf den Knopf: linke Haelfte = gegen den Uhrzeigersinn, rechte = im Uhrzeigersinn
function PNL.knob_click(x, y)
    local s, k = PNL.knob_at(x, y)
    if not s then return false end
    PNL.knob_turn(s, x >= k[1])
    return true
end
-- Range-Knopf auf ZOOM wie im A350: ZOOM beginnt (von 10 kommend) mit der groessten OANS-Reichweite (5 NM),
-- jeder Schritt nach links zoomt hinein (2 - 1 - 0,5 - 0,2 NM), nach rechts wieder heraus; ein Schritt ueber
-- 5 NM hinaus stellt den Knopf zurueck auf 10 (Rueckgabe true). Gilt fuer die Reichweite des aktuellen EFIS-Modus.
function PNL.zoom_enter(s)
    local sd = PNL.sd[s]
    if not sd.view then
        sd.view = new_view()
        S.nd_views = S.nd_views or {}
        S.nd_views[s] = sd.view
    end
    local V = sd.view
    for _, k in ipairs({ "arc", "rose", "plan", "nav" }) do V.rng[k] = #RANGES end
    V.rng_cont, V.rng_user, V.rdisp = nil, true, nil
end
function PNL.zoom_step(s, dec)
    local V = PNL.sd[s].view
    if not V then return not dec end
    local vk = EFIS.vmode(s)
    if vk == "arc" and V.rng_cont then V.rng.arc = MAPX.fit_rng(V.rng_cont * 1852); V.rng_cont = nil end
    local i = V.rng[vk] or #RANGES
    if dec then
        V.rng[vk] = math.max(1, i - 1)
    else
        if i >= #RANGES then return true end
        V.rng[vk] = i + 1
    end
    V.rng_user = true
    return false
end

local function pnl_wheel(x, y, wheel, clicks)
    if wheel ~= 0 or clicks == 0 then return false end
    if PNL.knob_wheel(x, y, clicks) then return true end
    local s, lx, ly = pnl_hit(x, y)
    if not s or not PNL.sd[s].vm then return false end
    local sd = PNL.sd[s]
    if sd.hin then lx, ly = sd.hx, sd.hy end
    sd.q[#sd.q + 1] = { t = "wheel", n = clicks, x = lx, y = ly }
    return true
end

-------------------------------------------------------------------------------
-- Start / Ende
-------------------------------------------------------------------------------
local function pnl_shutdown()
    if PNL.GUARD then os.remove(PNL.GUARD) end -- (normal beendet: kein Absturz beim Zeichnen)
    if TF.id and GL then
        pcall(function() local a = ffi.new("unsigned int[1]", TF.id); GL.btv_glDeleteTextures(1, a) end)
        TF.id, TF.ok = nil, false
    end
    for _, br in ipairs(PNL.bridges or {}) do
        if br.set then pcall(br.set, nil) end
        br.set = nil
    end
    if PNL.reg and PNL.f_unreg then pcall(PNL.f_unreg, PNL.cb, 45, 0, nil) end
    PNL.reg = false
    if PNL.win and PNL.f_destroy then pcall(PNL.f_destroy, PNL.win) end
    PNL.win = nil
    -- the ATCCOM popup window too: X-Plane must never call into a closed Lua state (script or aircraft reload)
    local P = PNL.atp
    if P and P.win ~= nil then
        pcall(PNL.atp_save)
        if PNL.f_destroy then pcall(PNL.f_destroy, P.win) else pcall(XP.btv_XPLMDestroyWindow, P.win) end
        P.win, P.on = nil, false
    end
    PNL.ok = false
end

local function pnl_init()
    PNL.tried = true
    local okf, f = pcall(require, "ffi")
    if not okf or type(f) ~= "table" then PNL.st = "LuaJIT FFI missing"; pnl_log(PNL.st); return end
    ffi = f
    local okc, errc = pcall(ffi.cdef, PNL_CDEF)
    if not okc then PNL.st = "FFI: " .. tostring(errc); pnl_log(PNL.st); return end
    PNL.T = A350_BTV_TEST and A350_BTV_TEST.pnl
    if PNL.T then
        XP, GL = PNL.T.xplm, PNL.T.gl
    else
        local os_ = ffi.os
        local gn = (os_ == "Windows") and "opengl32" or ((os_ == "OSX") and "/System/Library/Frameworks/OpenGL.framework/OpenGL" or "libGL.so.1")
        local ok1, l1 = pcall(XFM.xplm_lib, ffi, "btv_XPLMRegisterDrawCallback")
        if not ok1 then PNL.st = "XPLM not loadable"; pnl_log(PNL.st .. ": " .. tostring(l1)); return end
        local ok2, l2 = pcall(ffi.load, gn)
        if not ok2 then PNL.st = "OpenGL not loadable"; pnl_log(PNL.st .. ": " .. tostring(l2)); return end
        XP, GL = l1, l2
        if os_ == "Windows" then
            pcall(ffi.cdef, PNL_CDEF_WIN)
            PNL.procaddr = function(n) return GL.btv_wglGetProcAddress(n) end
        elseif os_ == "OSX" then
            -- macOS: alle OpenGL-Funktionen exportiert das Framework selbst -> dlsym
            pcall(ffi.cdef, [[ void *btv_dlopen(const char *, int) __asm__("dlopen");
                               void *btv_dlsym(void *, const char *) __asm__("dlsym"); ]])
            local okh, h = pcall(function() return ffi.C.btv_dlopen(gn, 1) end)
            if okh and h ~= nil then PNL.procaddr = function(n) return ffi.C.btv_dlsym(h, n) end end
        else
            pcall(ffi.cdef, PNL_CDEF_LIN)
            PNL.procaddr = function(n) return GL.btv_glXGetProcAddressARB(n) end
        end
    end
    -- alle Funktionen jetzt aufloesen: fehlt eine, scheitert es hier und nicht spaeter beim Zeichnen
    local okr, errr = pcall(function()
        for _, n in ipairs({ "XPLMRegisterDrawCallback", "XPLMUnregisterDrawCallback", "XPLMSetGraphicsState", "XPLMFindDataRef",
                             "XPLMGetDatai", "XPLMGetDataf", "XPLMCreateWindowEx", "XPLMDestroyWindow", "XPLMSetWindowGeometry",
                             "XPLMSetWindowIsVisible", "XPLMGetScreenBoundsGlobal", "XPLMGetElapsedTime" }) do
            assert(XP["btv_" .. n], n)
        end
        for _, n in ipairs({ "glGetIntegerv", "glGetFloatv", "glEnable", "glDisable", "glBlendFunc", "glPolygonMode", "glPushAttrib",
                             "glPopAttrib", "glPushClientAttrib", "glPopClientAttrib", "glEnableClientState", "glDisableClientState",
                             "glVertexPointer", "glColorPointer", "glDrawArrays" }) do
            assert(GL["btv_" .. n], n)
        end
    end)
    if not okr then PNL.st = "function missing: " .. tostring(errr); pnl_log(PNL.st); return end
    PNL.f_unreg, PNL.f_destroy, PNL.f_sgs = XP.btv_XPLMUnregisterDrawCallback, XP.btv_XPLMDestroyWindow, XP.btv_XPLMSetGraphicsState
    local function fd(n)
        local r = XP.btv_XPLMFindDataRef(n)
        if r == nil then error("dataref missing: " .. n, 0) end
        return r
    end
    local okd, errd = pcall(function()
        PNL.r_rt = fd("sim/graphics/view/panel_render_type")
        PNL.r_pl, PNL.r_pb = fd("sim/graphics/view/panel_total_pnl_l"), fd("sim/graphics/view/panel_total_pnl_b")
        PNL.r_pr, PNL.r_pt = fd("sim/graphics/view/panel_total_pnl_r"), fd("sim/graphics/view/panel_total_pnl_t")
    end)
    if not okd then PNL.st = tostring(errd); pnl_log(PNL.st); return end
    -- Datarefs fuer die Bild-Schleife (FlyWithLua)
    local function find(n) if dr_exists(n) then return dataref_table(n) end end
    PNL.dr = {
        zl = find("1-sim/fcu/ndZoomLeft/switch"), zr = find("1-sim/fcu/ndZoomRight/switch"),
        vt = find("sim/graphics/view/view_type"), vr = find("sim/graphics/VR/enabled"),
        wm = find("sim/graphics/view/world_matrix"), pm = find("sim/graphics/view/projection_matrix_3d"),
        ww = find("sim/graphics/view/window_width"), wh = find("sim/graphics/view/window_height"),
        uis = find("sim/graphics/misc/user_interface_scale"),
        lx = find("sim/flightmodel/position/local_x"), ly = find("sim/flightmodel/position/local_y"),
        lz = find("sim/flightmodel/position/local_z"),
        psi = find("sim/flightmodel/position/psi"), the = find("sim/flightmodel/position/theta"),
        phi = find("sim/flightmodel/position/phi"),
        c3x = find("sim/graphics/view/click_3d_x"), c3y = find("sim/graphics/view/click_3d_y"),
        hx = find("sim/graphics/view/pilots_head_x"), hy = find("sim/graphics/view/pilots_head_y"),
        hz = find("sim/graphics/view/pilots_head_z"), hpsi = find("sim/graphics/view/pilots_head_psi"),
        hthe = find("sim/graphics/view/pilots_head_the"), hphi = find("sim/graphics/view/pilots_head_phi"),
    }
    -- EFIS-Modus (0 LS, 1 VOR, 2 NAV, 3 ARC, 4 PLAN): der Modus des FF-ND selbst (wie der gruene Pfeil am FCU) - der
    -- Knopf steht in den Rasten nicht auf ganzen Zahlen; nur ohne diesen Wert der Knopf
    for s, names in pairs({ [0] = { "AirbusFBW/NDmodeCapt", "1-sim/fcu/ndModeLeft/switch" },
                            [1] = { "AirbusFBW/NDmodeFO", "1-sim/fcu/ndModeRight/switch" } }) do
        for _, n in ipairs(names) do
            local r = find(n)
            if r then PNL.dr["mode" .. s] = r; pnl_log("EFIS mode " .. ((s == 0) and "CAPT" or "F/O") .. ": " .. n); break end
        end
    end
    if not (PNL.dr.zl and PNL.dr.zr) then PNL.st = "FlightFactor A350 only (range knob not found)"; pnl_log(PNL.st); return end
    -- Puffer
    GLB.cap = 4000 -- (Ersatz; gezeichnet wird in eigene Puffer je Schicht, PNL.newbuf)
    GLB.v = ffi.new("float[?]", 2 * GLB.cap)
    GLB.c = ffi.new("uint32_t[?]", GLB.cap)
    GLB.t = ffi.new("float[?]", 2 * GLB.cap)
    pcall(pnl_font_load)
    PNL.av, PNL.ac = ffi.new("float[?]", 64), ffi.new("uint32_t[?]", 32)
    PNL.ib, PNL.fb = ffi.new("int[16]"), ffi.new("float[16]")
    -- Zeichen-Callback
    PNL.px4 = ffi.new("uint8_t[16]")
    PNL.gb0 = { v = GLB.v, c = GLB.c, t = GLB.t, cap = GLB.cap }
    PNL.ub, PNL.pb = PNL.newbuf(64), PNL.newbuf(PNL.PFD_CAP)
    PNL.now = os.clock
    pnl_log("LuaJIT: " .. tostring(jit and jit.version) .. ", JIT " .. tostring(jit and jit.status and jit.status()))
    PNL.nomat = not pcall(function()
        assert(GL.btv_glScissor and GL.btv_glMatrixMode and GL.btv_glPushMatrix and GL.btv_glPopMatrix and GL.btv_glMultMatrixf)
    end)
    if PNL.nomat then
        MAPX.ND_M = 0
        pnl_log("OpenGL matrix functions missing - map rebuilt every frame")
    end
    PNL.guard_init()
    pcall(PNL.irs_init)
    local okt, et = pcall(PNL.trf_init)
    if not okt then pnl_log("traffic: " .. tostring(et)) end
    -- (ohne Schichten wie v2.8: auch keine eigene Uhr - nur noch, was v2.8 schon an OpenGL/FFI benutzte)
    if not PNL.nomat then PNL.clock_init() end
    pnl_log("layered drawing " .. (PNL.nomat and "off" or "on"))
    -- XP11 / FlyWithLua NG 2.7: Datarefs in den Callbacks ueber FFI lesen (siehe XFM.ffi_peek); vorher pruefen
    XFM.pk_lib, XFM.pk_f, XFM.pk_i = XP, ffi.new("float[1]"), ffi.new("int[1]")
    local okp, ep = pcall(function()
        XFM.pk_old = DATAREF_META_TABLE.__index
        local a, b = XFM.ffi_peek(DR.time, 0), DR.time[0]
        assert(type(a) == "number" and math.abs(a - b) < 5, "value " .. tostring(a) .. " / " .. tostring(b))
    end)
    if not okp then XFM.pk_lib = nil end
    pnl_log("callback dataref reads: " .. (okp and "FFI (FlyWithLua stack safe)" or ("FlyWithLua peek - " .. tostring(ep))))
    PNL.cb = ffi.cast("btv_draw_f", function(ph, bf, rc)
        local im = imgui
        XFM.cb_enter()
        DL.panel, imgui = true, PNL.NOIMGUI -- (ImGui ausserhalb eines ImGui-Fensters = Absturz: hier gesperrt)
        local ok, e = pcall(pnl_draw, 0)
        DL.panel, imgui = false, im
        if not ok then pcall(pnl_draw_error, e) end
        XFM.cb_leave()
        return 1
    end)
    -- Hilfs-Plugins ("Bruecken"): zeichnen nach den Flugzeug-Plugins und rufen dafuer pnl_draw auf.
    -- Eine liegt global in Resources/plugins, eine im Plugin-Ordner der A350 - welche spaet genug kommt,
    -- haengt davon ab, wie X-Plane die Zeichen-Callbacks ordnet; das wird beim ersten Anzeigen geprueft.
    PNL.bridges = {}
    local ext = (ffi.os == "Windows") and "win.xpl" or ((ffi.os == "OSX") and "mac.xpl" or "lin.xpl")
    local acf_dir = (type(AIRCRAFT_PATH) == "string" and AIRCRAFT_PATH ~= "") and AIRCRAFT_PATH or (XP_ROOT .. "Aircraft/addon aircraft/AirbusA350/")
    if not acf_dir:match("[/\\]$") then acf_dir = acf_dir .. "/" end
    local cand = (PNL.T and PNL.T.bridges) or {
        { name = "Bridge aircraft", path = acf_dir .. "plugins/zz_A350_BTV_bridge/64/" .. ext },
        { name = "Bridge global", path = XP_ROOT .. "Resources/plugins/zz_A350_BTV_bridge/64/" .. ext },
    }
    for i, c in ipairs(cand) do
        local br = { name = c.name, ok = false, idx = i }
        PNL.bridges[i] = br
        local path = c.path
        if ffi.os == "Windows" then path = path:gsub("/", "\\") end
        local okl, lib = false, nil
        if file_exists(path) or PNL.T then okl, lib = pcall(ffi.load, path) end
        if okl and lib then
            local okf, info = pcall(function() return lib.btv_bridge_info_() end)
            if okf and bit.band(info, 1) == 1 then
                br.lib, br.set = lib, lib.btv_bridge_set_
                br.cb = ffi.cast("btv_draw_f", function(ph, bf, rc)
                    local im = imgui
                    XFM.cb_enter()
                    DL.panel, imgui = true, PNL.NOIMGUI
                    local ok, e = pcall(pnl_draw, br)
                    DL.panel, imgui = false, im
                    if not ok then pcall(pnl_draw_error, e) end
                    if ok then PNL.atc_safe_draw(br) end
                    XFM.cb_leave()
                    return 1
                end)
                br.set(br.cb)
                br.ok = true
            else
                br.why = okf and "not loaded as plugin (restart X-Plane)" or "old version"
            end
        else
            br.why = "not installed"
        end
        pnl_log(br.name .. ": " .. (br.ok and "active" or br.why))
    end
    PNL.det_start = nil
    if XP.btv_XPLMRegisterDrawCallback(PNL.cb, 45, 0, nil) ~= 1 then
        PNL.st = "draw phase Gauges not available"; pnl_log(PNL.st); return
    end
    PNL.reg = true
    -- unsichtbares Mausfenster ueber dem ganzen Bildschirm (sichtbar im 3D-Cockpit: OANS-Karte, MASTER CAUT, Mausrad am Range-Knopf)
    PNL.cb_wdraw = ffi.cast("btv_wdraw_f", function(w, rc) end)
    PNL.cb_key = ffi.cast("btv_wkey_f", function(w, k, fl, vk, rc, lf) XFM.cb_enter(); pcall(PNL.key, k, fl, vk, lf); XFM.cb_leave() end)
    PNL.cb_click = ffi.cast("btv_wclick_f", function(w, x, y, st, rc)
        XFM.cb_enter()
        local ok, r = pcall(pnl_click, x, y, st, false)
        XFM.cb_leave()
        return (ok and r) and 1 or 0
    end)
    PNL.cb_rclick = ffi.cast("btv_wclick_f", function(w, x, y, st, rc)
        XFM.cb_enter()
        local ok, r = pcall(pnl_click, x, y, st, true)
        XFM.cb_leave()
        return (ok and r) and 1 or 0
    end)
    PNL.cb_cursor = ffi.cast("btv_wcursor_f", function(w, x, y, rc)
        XFM.cb_enter()
        local ok, r = pcall(pnl_cursor, x, y)
        XFM.cb_leave()
        return (ok and r) or 0 -- 0 = Mauszeiger wie gewohnt, 1 = aus (ueber der Karte)
    end)
    PNL.cb_wheel = ffi.cast("btv_wwheel_f", function(w, x, y, wh, cl, rc)
        XFM.cb_enter()
        local ok, r = pcall(pnl_wheel, x, y, wh, cl)
        XFM.cb_leave()
        return (ok and r) and 1 or 0
    end)
    local cw = ffi.new("btv_wnd_t")
    cw.structSize = ffi.sizeof("btv_wnd_t")
    cw.left, cw.top, cw.right, cw.bottom, cw.visible = 0, 100, 100, 0, 0
    cw.drawWindowFunc, cw.handleMouseClickFunc, cw.handleKeyFunc = PNL.cb_wdraw, PNL.cb_click, PNL.cb_key
    cw.handleCursorFunc, cw.handleMouseWheelFunc, cw.refcon = PNL.cb_cursor, PNL.cb_wheel, nil
    cw.decorateAsFloatingWindow, cw.layer, cw.handleRightClickFunc = 0, 1, PNL.cb_rclick -- ohne Rahmen, Ebene "schwebende Fenster"
    PNL.win = XP.btv_XPLMCreateWindowEx(cw)
    if PNL.win == nil then pnl_log("mouse window could not be created - map without mouse control") end
    PNL.win_vis, PNL.win_geo = false, nil
    -- Tastaturfokus fuer Eingaben im Bedienfeld (danach zurueck an X-Plane)
    NDUI.focus = function(on)
        if PNL.win then pcall(XP.btv_XPLMTakeKeyboardFocus, on and PNL.win or nil) end
    end
    -- aufraeumen, auch wenn FlyWithLua den Lua-Zustand ohne do_on_exit schliesst (z.B. nach Skriptfehlern)
    PNL.guard = ffi.gc(ffi.new("char[1]"), function() pnl_shutdown() end)
    PNL.ok = true
    PNL.st = "ready"
    PNL.rereg = PNL.frame + 300 -- nach ein paar Sekunden neu anmelden: dann zeichnet die Karte sicher nach dem Flugzeug
    pnl_log("ready (draw phase Gauges, " .. ffi.os .. ")")
end

-------------------------------------------------------------------------------
-- Verkehr: sim/cockpit2/tcas/targets/... (Index 0 = eigenes Flugzeug). Darin stehen KI-Flugzeuge oder die Ziele
-- des Plugins, das TCAS uebernommen hat (xPilot, LiveTraffic ...). Positionen jedes Bild, Kennungen alle 30 Bilder.
-- Dazu die Stellungen von XPDR / TCAS / TRAF (FF A350, ToLiss-Systeme): Aenderungen stehen im Log.
-------------------------------------------------------------------------------
PNL.TRF_N = 64
PNL.SURV_REFS = { "AirbusFBW/TCASSelectedND1", "AirbusFBW/TCASSelectedND2", "AirbusFBW/XPDRTCASMode", "AirbusFBW/XPDRPower",
    "AirbusFBW/XPDRAltitude", "AirbusFBW/XPDRTCASAltSelect", "AirbusFBW/TCASStatus", "1-sim/surv/TCAS", "1-sim/surv/TCASmode",
    "1-sim/surv/TCASdisplay", "1-sim/surv/XPDR", "1-sim/surv/XPDRmode", "sim/cockpit2/radios/actuators/transponder_mode" }
function PNL.trf_init()
    local T = { ok = false, list = {}, pool = {}, ids = {}, types = {}, idf = -1000, surv = {}, last = {} }
    PNL.trf = T
    for _, n in ipairs(PNL.SURV_REFS) do
        if dr_exists(n) then T.surv[n] = dataref_table(n) end
    end
    local function fr(n) local r = XP.btv_XPLMFindDataRef(n); if r ~= nil then return r end end
    local b = "sim/cockpit2/tcas/targets/position/"
    T.r = { lat = fr(b .. "lat"), lon = fr(b .. "lon"), ele = fr(b .. "ele"), psi = fr(b .. "psi"), vs = fr(b .. "vertical_speed"),
            wow = fr(b .. "weight_on_wheels"), threat = fr("sim/cockpit2/tcas/targets/threat"),
            id = fr("sim/cockpit2/tcas/targets/flight_id"), typ = fr("sim/cockpit2/tcas/targets/icao_type") }
    pcall(PNL.lt_find, T)
    if not (T.r.lat and T.r.lon and T.r.ele) then pnl_log("traffic: X-Plane TCAS datarefs missing"); return end
    local N = PNL.TRF_N
    T.f = { lat = ffi.new("float[?]", N), lon = ffi.new("float[?]", N), ele = ffi.new("float[?]", N),
            psi = ffi.new("float[?]", N), vs = ffi.new("float[?]", N) }
    T.i = { wow = ffi.new("int[?]", N), threat = ffi.new("int[?]", N) }
    T.cb = ffi.new("uint8_t[?]", 8 * N)
    T.ok = true
end

-- Texte im 8-Byte-Raster (je Ziel 7 Zeichen + 0)
local function trf_strings(T, ref, out)
    if not ref then return end
    local n = XP.btv_XPLMGetDatab(ref, T.cb, 0, 8 * PNL.TRF_N)
    local cb = T.cb
    for i = 1, PNL.TRF_N - 1 do
        local o, t = 8 * i, {}
        if o + 8 <= n then
            for k = 0, 7 do
                local c = cb[o + k]
                if c == 0 then break end
                if c > 32 and c < 127 then t[#t + 1] = string.char(c) end
            end
        end
        out[i] = table.concat(t):upper()
    end
end

-- Verkehr dieser Seite anzeigen? nd_traffic: 0 aus, 2 immer, 1 wie TCAS: TRAF gewaehlt und TCAS nicht STBY
-- (Werte der FF-Datarefs noch an einem Flug pruefen - sie stehen im Log, sobald sie sich aendern)
function PNL.trf_show(side)
    local T = PNL.trf
    if not (T and (T.ok or T.lt)) or (CFG.nd_traffic or 0) == 0 then return false end
    if CFG.nd_traffic == 2 then return true end
    local sv = T.surv
    local sel = sv["AirbusFBW/TCASSelectedND" .. (side + 1)]
    if sel and sel[0] == 0 then return false end
    local tm = sv["AirbusFBW/XPDRTCASMode"]
    if tm then return tm[0] >= 1 end
    local xm = sv["sim/cockpit2/radios/actuators/transponder_mode"]
    if xm then return xm[0] >= 6 end -- (X-Plane: 6 = TA ONLY, 7 = TA/RA)
    return true
end

-- je Bild, solange eine Seite die Karte zeigt
-------------------------------------------------------------------------------
-- LiveTraffic direkt lesen (Bulk-Schnittstelle wie LTAPI). Haelt ein anderes Plugin (z.B. xPilot) TCAS/AI,
-- fehlen die LiveTraffic-Flugzeuge in den TCAS-Daten. Satz je Flugzeug: Windows 168 Byte (MSVC-Bitfelder),
-- sonst 152; Groessenabsprache: XPLMGetDatab ohne Puffer, Laenge = eigene Satzgroesse -> Rueckgabe LiveTraffics.
-- Felder: keyNum @0, heading @20, vsi_ft @40, height_ft @48 (Hoehe ueber Grund), lat/lon/alt_ft als double
-- @Groesse-72/-64/-56 (ab LiveTraffic 1.22, sonst float @8/12/16). Texte (304 Byte): Typ @16, Callsign @200.
-------------------------------------------------------------------------------
PNL.LT_N = 50
function PNL.lt_find(T)
    T.lt_try = PNL.frame
    local function fr(n) local r = XP.btv_XPLMFindDataRef(n); if r ~= nil then return r end end
    local q, num = fr("livetraffic/bulk/quick"), fr("livetraffic/ac/num")
    if not (q and num) then return end
    local sz = PNL.LT_SZ or ((ffi.os == "Windows") and 168 or 152) -- (LT_SZ: nur Tests)
    T.lt = { q = q, x = fr("livetraffic/bulk/expensive"), num = num, disp = fr("livetraffic/cfg/aircrafts_displayed"),
             ai = fr("livetraffic/cfg/ai_controlled"), sz = sz, list = {}, pool = {}, txt = {}, ntxt = 0, tf = -1000,
             buf = ffi.new("uint8_t[?]", sz * PNL.LT_N + 64), tbuf = ffi.new("uint8_t[?]", 304 * PNL.LT_N + 64) }
    pnl_log("traffic: LiveTraffic found (bulk data)")
end

local function lt_str(b, o, n)
    local t = {}
    for k = 0, n - 1 do
        local c = b[o + k]
        if c == 0 then break end
        if c > 32 and c < 127 then t[#t + 1] = string.char(c) end
    end
    return table.concat(t):upper()
end

-- Liste der LiveTraffic-Flugzeuge im Umkreis von ~15 NM (L.list); leer, solange LiveTraffic TCAS/AI selbst haelt
-- (dann stehen sie schon in den TCAS-Daten) oder keine Flugzeuge zeigt
function PNL.lt_read(T)
    local L = T.lt
    local list = L.list
    for k = #list, 1, -1 do list[k] = nil end
    local ai = L.ai and XP.btv_XPLMGetDatai(L.ai) ~= 0
    if ai ~= L.ai_last then
        L.ai_last = ai
        pnl_log(ai and "traffic: LiveTraffic controls TCAS - its aircraft come via TCAS"
            or "traffic: LiveTraffic does not control TCAS - reading its aircraft directly")
    end
    if ai or (L.disp and XP.btv_XPLMGetDatai(L.disp) == 0) then return end
    local num = math.min(XP.btv_XPLMGetDatai(L.num), 1000)
    if num <= 0 then return end
    local sz, N = L.sz, PNL.LT_N
    local ltsz = XP.btv_XPLMGetDatab(L.q, nil, 0, sz)
    if ltsz < 80 then return end
    local dbl = ltsz >= sz - 48
    local la0, lo0 = DR.lat[0], DR.lon[0]
    local cosl = math.cos(la0 * D2R)
    local u = ffi.cast("uint32_t *", L.buf)
    local fp = ffi.cast("float *", L.buf)
    local dp = ffi.cast("double *", L.buf)
    local need = false
    local pi = 0
    for a = 0, num - 1, N do
        local n = XP.btv_XPLMGetDatab(L.q, L.buf, a * sz, N * sz)
        local cnt = math.min(math.floor(n / sz), N)
        if cnt <= 0 then break end
        for k = 0, cnt - 1 do
            local b = k * sz
            local la, lo, alt
            if dbl then
                la, lo, alt = dp[(b + sz - 72) / 8], dp[(b + sz - 64) / 8], dp[(b + sz - 56) / 8]
            else
                la, lo, alt = fp[(b + 8) / 4], fp[(b + 12) / 4], fp[(b + 16) / 4]
            end
            if (la ~= 0 or lo ~= 0) and math.abs(la - la0) < 0.25 and math.abs((lo - lo0) * cosl) < 0.25 then
                local key = u[b / 4] + u[b / 4 + 1] * 4294967296
                pi = pi + 1
                local t = L.pool[pi]
                if not t then t = {}; L.pool[pi] = t end
                local hgt = fp[(b + 48) / 4]
                t.lat, t.lon, t.ele = la, lo, alt * 0.3048
                t.psi, t.vs = fp[(b + 20) / 4], fp[(b + 40) / 4]
                t.wow = hgt == hgt and hgt < 15 -- (nan ~= nan)
                t.threat = 0
                local tx = L.txt[key]
                if tx then t.id, t.typ = tx.id, tx.typ else t.id, t.typ = "", ""; need = true end
                list[#list + 1] = t
            end
        end
        if cnt < N then break end
    end
    -- Callsign und Typ (alle 3 s, bei neuen Flugzeugen nach 1 s)
    if L.x and (PNL.frame - L.tf >= 90 or (need and PNL.frame - L.tf >= 30)) then
        L.tf = PNL.frame
        if L.ntxt > 3000 then L.txt, L.ntxt = {}, 0 end
        local tsz = 304
        if XP.btv_XPLMGetDatab(L.x, nil, 0, tsz) > 0 then
            local tu = ffi.cast("uint32_t *", L.tbuf)
            for a = 0, num - 1, N do
                local n = XP.btv_XPLMGetDatab(L.x, L.tbuf, a * tsz, N * tsz)
                local cnt = math.min(math.floor(n / tsz), N)
                if cnt <= 0 then break end
                for k = 0, cnt - 1 do
                    local b = k * tsz
                    local key = tu[b / 4] + tu[b / 4 + 1] * 4294967296
                    local cs = lt_str(L.tbuf, b + 200, 8)
                    if cs == "" then cs = lt_str(L.tbuf, b + 216, 8) end -- Flugnummer
                    if cs == "" then cs = lt_str(L.tbuf, b + 8, 8) end -- Kennzeichen
                    if not L.txt[key] then L.ntxt = L.ntxt + 1 end
                    L.txt[key] = { id = cs, typ = lt_str(L.tbuf, b + 16, 8) }
                end
                if cnt < N then break end
            end
        end
    end
end

-- je Bild, solange eine Seite die Karte zeigt: TCAS-Ziele und (falls noetig) LiveTraffic
function PNL.trf_read()
    local T = PNL.trf
    if not T then return end
    if not T.lt and PNL.frame - (T.lt_try or -1e9) >= 600 then PNL.lt_find(T) end
    for n, d in pairs(T.surv) do
        local v = d[0]
        if T.last[n] ~= v then
            T.last[n] = v
            pnl_log(string.format("SURV %s = %s", n, tostring(v)))
        end
    end
    local list = T.list
    for k = #list, 1, -1 do list[k] = nil end
    local h = 0
    if T.ok then
        local N, r, f, ib = PNL.TRF_N, T.r, T.f, T.i
        local n = XP.btv_XPLMGetDatavf(r.lat, f.lat, 0, N)
        if n > N then n = N end
        if n > 1 then
            XP.btv_XPLMGetDatavf(r.lon, f.lon, 0, n)
            XP.btv_XPLMGetDatavf(r.ele, f.ele, 0, n)
            if r.psi then XP.btv_XPLMGetDatavf(r.psi, f.psi, 0, n) end
            if r.vs then XP.btv_XPLMGetDatavf(r.vs, f.vs, 0, n) end
            if r.wow then XP.btv_XPLMGetDatavi(r.wow, ib.wow, 0, n) end
            if r.threat then XP.btv_XPLMGetDatavi(r.threat, ib.threat, 0, n) end
            if PNL.frame - T.idf >= 30 then
                T.idf = PNL.frame
                trf_strings(T, r.id, T.ids)
                trf_strings(T, r.typ, T.types)
            end
            for i = 1, n - 1 do
                local la, lo = f.lat[i], f.lon[i]
                if la ~= 0 or lo ~= 0 then
                    local t = T.pool[i]
                    if not t then t = {}; T.pool[i] = t end
                    t.lat, t.lon, t.ele = la, lo, f.ele[i]
                    t.psi = r.psi and f.psi[i] or 0
                    t.vs = r.vs and f.vs[i] or 0
                    t.wow = r.wow ~= nil and ib.wow[i] ~= 0
                    t.threat = r.threat and ib.threat[i] or 0
                    if not r.threat and not t.wow then
                        -- XP11 hat kein targets/threat: grob nach Abstand und Hoehe (Index 0 = eigenes Flugzeug)
                        local dn = math.sqrt(((la - f.lat[0]) * 60) ^ 2 + ((lo - f.lon[0]) * 60 * math.cos(f.lat[0] * D2R)) ^ 2)
                        local dh = math.abs(f.ele[i] - f.ele[0]) * 3.28084
                        local own_gnd = r.wow ~= nil and ib.wow[0] ~= 0
                        if dn < 2.0 and dh < 600 and not own_gnd then t.threat = 3
                        elseif dn < 3.3 and dh < 850 and not own_gnd then t.threat = 2
                        elseif dn < 6 and dh < 1200 then t.threat = 1 end
                    end
                    t.id, t.typ = T.ids[i] or "", T.types[i] or ""
                    list[#list + 1] = t
                end
            end
        end
    end
    local L = T.lt
    if L then
        -- (5 Hz; dazwischen die letzte Liste)
        if PNL.frame - (L.f or -100) >= 6 then
            L.f = PNL.frame
            local okl, el = pcall(PNL.lt_read, T)
            if not okl then
                pnl_log("traffic: LiveTraffic: " .. tostring(el) .. " - not used")
                T.lt, T.lt_try, L = nil, 1e12, nil
            end
        end
        if L then for _, t in ipairs(L.list) do list[#list + 1] = t end end
    end
    for _, t in ipairs(list) do
        h = h + math.floor(t.lat * 1e5 + 0.5) * 3 + math.floor(t.lon * 1e5 + 0.5) * 7 + math.floor(t.psi or 0) + (t.threat or 0) * 11
    end
    NDUI.trf.list = list
    -- Overlay neu fuer Verkehr hoechstens alle 6 Bilder (5 Hz; ADS-B/TCAS selbst nur 1-2 Hz), neue/weggefallene sofort
    local sig = string.format("%d/%d", #list, h % 1000000007)
    if #list ~= (T.nlast or -1) or PNL.frame - (T.sigf or -100) >= 6 then
        T.nlast, T.sigf = #list, PNL.frame
        NDUI.trf.sig = sig
    end
end

-------------------------------------------------------------------------------
-- IRS: OANS erst, wenn die IRS ganz ausgerichtet sind. Das FF zeigt das Heading schon waehrend des Ausrichtens
-- (Kompassring nicht mehr rot), das ND aber noch "MAP NOT AVAIL": AirbusFBW/CaptMAPAvail / CoMAPAvail = 0.
-- Dazu AirbusFBW/TimeToAlign: zaehlt die Restzeit herunter (zaehlt nur, wenn sie sich auch wirklich verringert).
-- Aenderungen stehen im Log ("IRS ...").
-------------------------------------------------------------------------------
PNL.IRS_MAP = { [0] = "AirbusFBW/CaptMAPAvail", [1] = "AirbusFBW/CoMAPAvail" }
function PNL.irs_init()
    local I = { map = {}, last = {}, res = {}, bad = false, since = {}, lastf = {} }
    PNL.irs = I
    for s = 0, 1 do
        if dr_exists(PNL.IRS_MAP[s]) then I.map[s] = dataref_table(PNL.IRS_MAP[s]) end
    end
    if dr_exists("AirbusFBW/TimeToAlign") then I.tta = dataref_table("AirbusFBW/TimeToAlign") end
end
function PNL.irs_ok(s)
    local I = PNL.irs
    if not I then return true end
    if I.f ~= PNL.frame then
        I.f = PNL.frame
        local t = DR.time[0]
        if I.tta then
            local v = tonumber(I.tta[0]) or 0
            if I.tv and v < I.tv - 1e-6 then I.dec = t end
            if I.tv and v > I.tv + 1e-6 then I.dec = nil end -- neu gesetzt: erst wieder, wenn sie herunterzaehlt
            I.tv = v
            local al = v > 0 and I.dec ~= nil and t - I.dec < 90
            if al ~= I.al then
                I.al = al
                pnl_log(string.format("IRS: AirbusFBW/TimeToAlign = %.1f (%s)", v, al and "aligning" or "not counting down"))
            end
        end
        for k = 0, 1 do
            local m = I.map[k]
            local v = m and m[0]
            if v ~= I.last[k] then
                I.last[k] = v
                pnl_log(string.format("IRS: %s = %s", PNL.IRS_MAP[k], tostring(v)))
            end
        end
        -- Plausibilitaet: laenger als 1 min in der Luft ist die Karte verfuegbar - sonst den Wert nicht verwenden
        if not I.bad and S.air_time > 60 then
            for k = 0, 1 do
                if I.map[k] and I.map[k][0] == 0 then
                    I.bad = true
                    pnl_log("IRS: " .. PNL.IRS_MAP[k] .. " = 0 in flight - not used")
                end
            end
        end
    end
    -- (nur aufgerufen, solange das FF-ND an ist und ein Heading hat: seit wann ununterbrochen?)
    local t = DR.time[0]
    if I.lastf[s] ~= PNL.frame - 1 and I.lastf[s] ~= PNL.frame then I.since[s] = t end
    I.lastf[s] = PNL.frame
    local ok = not I.al
    local m = I.map[s]
    if ok and m and not I.bad and m[0] == 0 then
        -- Ausrichten dauert hoechstens ~10 min: nach 15 min mit gueltigem Heading den Wert nicht mehr verwenden
        if t - (I.since[s] or t) > 900 then
            I.bad = true
            pnl_log("IRS: " .. PNL.IRS_MAP[s] .. " still 0 after 15 min with valid heading - not used")
        else
            ok = false
        end
    end
    if ok ~= I.res[s] then
        I.res[s] = ok
        pnl_log(string.format("FF ND %s: %s", (s == 0) and "CAPT" or "F/O", ok and "IRS aligned" or "IRS aligning (MAP NOT AVAIL) - no OANS"))
    end
    return ok
end

-------------------------------------------------------------------------------
-- ARC-Bild (ZOOM): Kompassradius 193 Einheiten; nutzbar seitlich 135, voraus bis 50 unter die Oberkante (Kopfzeilen),
-- hinten bis an TA ONLY (fuer die Reichweite im Endanflug, PNL.appr_rng; Flugzeug aus EFIS.G / EFIS.GP)
-------------------------------------------------------------------------------
PNL.TZ = { lr = 135, top = 50 }
-- Endanflug (ARC unter 500 ft): Reichweite mit Zwischenstufen, so dass Schwelle, Exit und ROW-Haltepunkte im
-- Bild sind - nicht die naechste A350-Stufe (von 2 auf 5 NM wird der Flughafen sonst winzig)
function PNL.appr_rng(V)
    local E = S.rwy
    if not E then return end
    local ay = ((V.ui and V.ui.panel) and EFIS.GP or EFIS.G).arc[2]
    local up, dn = ay - PNL.TZ.top, NDUI.NOTCH.acft_gap
    local s0, d0 = rwy_sd(E, DR.lat[0], DR.lon[0])
    local pts = { { E.disp, 0 } }
    local ts, _, x = current_target()
    if x and x.path and x.cum and #x.path >= 2 then
        local t0 = x.t0 or 0
        local a2, b2 = path_point(x, t0)
        local a3, b3 = path_point(x, math.min(x.plen or t0, t0 + 120))
        pts[#pts + 1] = { a2, b2 }
        pts[#pts + 1] = { a3, b3 }
    elseif ts then
        pts[#pts + 1] = { ts, 0 }
    end
    if CFG.rops then
        for _, l in ipairs(S.row_lines(E)) do pts[#pts + 1] = { math.min(l[2], E.len + 150), 0 } end
    end
    if #pts == 1 then pts[#pts + 1] = { math.min(E.len, E.disp + 2500), 0 } end
    local h = DR.psi[0] * D2R
    local hc, hs = math.cos(h), math.sin(h)
    local need = 0
    for _, p in ipairs(pts) do
        local ds, dd = p[1] - s0, p[2] - d0
        local wx, wy = ds * E.ux - dd * E.uy, ds * E.uy + dd * E.ux
        local X, Y = wx * hc - wy * hs, wx * hs + wy * hc
        local r = math.max(math.abs(X) / PNL.TZ.lr, (Y >= 0) and (Y / up) or (-Y / dn)) * 193
        if r > need then need = r end
    end
    -- Zwischenstufen mit glatten Ringwerten (halbe Reichweite 0.25 / 0.5 / 0.75 / 1 / 1.25 / 1.5 / 2 / 2.5 NM)
    local STEPS = { 0.5, 1, 1.5, 2, 2.5, 3, 4, 5 }
    local function q(m)
        for _, r in ipairs(STEPS) do if r * 1852 >= m then return r end end
        return 5
    end
    local out_r, in_r = q(need * 1.05), q(need * 1.2)
    local cur = V.rng_cont
    if not cur or out_r > cur then cur = out_r
    elseif in_r < cur then cur = in_r end
    V.rng_cont = cur
end

-- jeden Frame (btv_frame)
local function pnl_frame()
    PNL.frame = PNL.frame + 1
    if not PNL.tried then pnl_init() end
    if not PNL.ok then return end
    PNL.ft = XP.btv_XPLMGetElapsedTime()
    local atc_ok, atc_err = pcall(PNL.atc_frame)
    -- (reported once; the ATCCOM page stays ours - it never falls back to FF's own page)
    if not atc_ok and PNL.atc.frame_err ~= tostring(atc_err) then PNL.atc.frame_err = tostring(atc_err); pnl_log("ATCCOM frame error: " .. tostring(atc_err)) end
    if not PNL.atc.msg.failed then
        local msg_ok, msg_err = pcall(PNL.atc_msg_tick)
        if not msg_ok then
            PNL.atc.msg.failed = true; pcall(PNL.atc_msg_off, "error"); pnl_log("ATC MSG switched off after an error: " .. tostring(msg_err))
        end
    end
    if PNL.rereg and PNL.frame >= PNL.rereg then
        PNL.rereg = nil
        XP.btv_XPLMUnregisterDrawCallback(PNL.cb, 45, 0, nil)
        PNL.reg = XP.btv_XPLMRegisterDrawCallback(PNL.cb, 45, 0, nil) == 1
    end
    local d = PNL.dr
    local any = false
    PNL.anyui = false
    PNL.need_samp = CFG.nd_on and (in_air() or S.mode ~= "OFF" or S.arm_request) and true or false
    -- Autobrake gesetzt, waehrend BTV scharf ist (vor dem Aufsetzen): BTV aus
    if S.abrk and S.arm_request and (S.mode == "OFF" or S.mode == "ARMED") then
        S.arm_request = false
        message("BTV DISARMED: AUTO BRK SET")
    end
    -- EFIS-Modus aus dem Knopf, mit Rastung: beim Drehen steht er zwischen den Stellungen (2,6 waere gerundet schon
    -- ARC, das FF zeigt noch NAV) - ein neuer Modus gilt erst nahe der naechsten Stellung
    for s = 0, 1 do
        local ref = d["mode" .. s]
        if ref then
            local v = math.floor(ref[0] + 0.5)
            if EFIS.raw[s] and math.abs(ref[0] - v) > 0.3 then v = EFIS.raw[s] end
            local m = (v == 3) and "arc" or ((v == 4) and "plan" or "rose")
            if EFIS.raw[s] ~= v then
                EFIS.raw[s] = v
                pnl_log(string.format("EFIS %s: %d = %s", (s == 0) and "CAPT" or "F/O", v, m:upper()))
            end
            EFIS.mode[s] = m
        end
    end
    -- BTV-Landung: nach dem Aufsetzen mit scharfem BTV die Taxi-Ansicht auf den NDs (auch ohne Range 10)
    -- Kreuze und Fahnen bleiben bis zum naechsten Flug (A350: geloescht mit Flugphase 1 - hier 5 min nach dem
    -- Abstellen aller Triebwerke nach einer Landung)
    if in_air() then
        PNL.flown, PNL.eng_off_t = true, nil
    elseif PNL.flown then
        local run = DR.engrun == nil
        if DR.engrun then for i = 0, 3 do if (tonumber(DR.engrun[i]) or 0) ~= 0 then run = true end end end
        if run then
            PNL.eng_off_t = nil
        else
            PNL.eng_off_t = PNL.eng_off_t or DR.time[0]
            if DR.time[0] - PNL.eng_off_t > 300 then
                PNL.flown, PNL.eng_off_t = false, nil
                if S.marks and next(S.marks) then
                    S.marks = nil
                    pnl_log("new flight: all crosses and flags erased")
                end
            end
        end
    end
    -- (1 s nach dem Aufsetzen; ein Huepfer davor startet die Wartezeit neu)
    if in_air() then
        PNL.td_watch, PNL.td_due = true, nil
    elseif PNL.td_watch then
        PNL.td_watch = false
        if S.arm_request or S.mode == "ARMED" or S.mode == "ENGAGED" then PNL.td_due = DR.time[0] + 1 end
    end
    if PNL.td_due and DR.time[0] >= PNL.td_due then
        PNL.td_due = nil
        for s = 0, 1 do
            local sd = PNL.sd[s]
            local rr = (s == 0) and d.zl or d.zr
            if rr and not sd.force then
                -- Range-Knopf beider Seiten auf ZOOM (ZOOM-Lampe an), ohne Neustart der ZOOM-Reichweite (5 NM):
                -- die Taxi-Ansicht stellt die Reichweite selbst
                if rr[0] > -0.5 then rr[0] = -1; sd.was_zoom = true end
                sd.force = { rng = rr[0], mode = EFIS.raw[s] }
                if not sd.view then
                    sd.view = new_view()
                    S.nd_views = S.nd_views or {}
                    S.nd_views[s] = sd.view
                end
                -- fest (kein schrittweises Hineinzoomen beim Langsamerwerden): die Taxi-Ansicht ist immer ARC,
                -- Reichweite 1 NM (= Radius): aeusserer Bogen 2 NM quer, gestrichelter 1 NM - in jedem EFIS-Modus.
                -- Gesetzt erst in MAPX.view_nd, nach dem Zuruecksetzen der Reichweite bei neuem plan_key
                sd.view.td_rng = MAPX.fit_rng(1852)
                sd.view.rng_user = false
            end
        end
        pnl_log("BTV landing: taxi view")
    end
    -- ARC im kurzen Endanflug: Reichweite so, dass Exit und DRY/WET im Bild sind
    for s = 0, 1 do
        local sd = PNL.sd[s]
        if sd.arc_row and sd.view and not sd.force and not sd.view.rng_user and S.rwy then
            local okz, ez = pcall(PNL.appr_rng, sd.view)
            if not okz and not PNL.appr_err then PNL.appr_err = true; pnl_log("approach zoom: " .. tostring(ez)) end
        end
    end
    for s = 0, 1 do
        local sd = PNL.sd[s]
        local rng = (s == 0) and d.zl or d.zr
        -- erzwungene Taxi-Ansicht endet mit Range- oder EFIS-Knopf, der OANS-Taste oder beim Abheben
        local f = sd.force
        -- (ein Huepfer beim Aufsetzen beendet sie nicht, solange BTV bremst)
        local lifted = in_air() and not (S.mode == "ENGAGED" or S.mode == "RELEASE")
        if f and (lifted or math.abs(rng[0] - f.rng) > 0.5 or EFIS.raw[s] ~= f.mode) then sd.force, f = nil, nil end
        EFIS.force[s] = f ~= nil
        -- FF-Bildschirm an? Ohne gueltiges Heading bzw. waehrend die IRS ausrichten zeigt die Seite statt der Karte
        -- MAP NOT AVAIL (nomap); PFD-Meldungen nur bei wirklich verfuegbarer Karte
        sd.avail = sd.ffst ~= "off"
        sd.nomap = sd.avail and (sd.ffst == "flag" or not PNL.irs_ok(s))
        -- Range-Knopf auf ZOOM (Stellung links von 10, Wert -1): wie im A350 zeigt das ND dann die Flughafenkarte
        -- in ROSE NAV, ARC und PLAN - am Boden wie im Flug. ROSE LS / ROSE VOR: normales ND (kein OANS)
        local knob_zoom = rng[0] < -0.5
        local oans_mode = EFIS.raw[s] == 2 or EFIS.raw[s] == 3 or EFIS.raw[s] == 4
        local zoom = knob_zoom and oans_mode
        local air = in_air()
        sd.was_air = air
        if knob_zoom and not sd.was_zoom then PNL.zoom_enter(s) end
        sd.was_zoom = knob_zoom
        -- A350: bei scharfem BTV zeigt ARC (ZOOM) unter 500 ft die Karte, DRY/WET in Echtzeit
        local arc_row = zoom and air and EFIS.raw[s] == 3 and DR.ra[0] < 500 and (S.arm_request or S.mode == "ARMED" or S.row_on)
        if arc_row and not sd.arc_row then
            if sd.view then sd.view.rng_user = false end
        end
        if sd.view then
            sd.view.appr = arc_row or nil
            if not arc_row then sd.view.rng_cont = nil end
        end
        sd.arc_row = arc_row
        sd.vm = sd.avail and ((f ~= nil) or zoom)
        sd.show = CFG.nd_on and not PNL.dead and (zoom or f ~= nil)
        if sd.show and not sd.mbuf then PNL.alloc_side(sd) end
        -- Ein-/Ausblenden (0,45 s): beim Erscheinen hineinzoomen, beim Verschwinden herauszoomen
        local want = (sd.vm and sd.ffst ~= nil) and 1 or 0
        if not sd.show then
            sd.fade = 0
        else
            -- (wie im A350 ohne Ueberblenden: die Karte ist sofort da bzw. weg)
            sd.fade = want
        end
        if sd.show and sd.avail and sd.vm then PNL.anyui = true end
        -- zeigt die Seite nichts Bedienbares: keine alten Klicks aufheben (sie kaemen sonst spaeter an)
        if not (sd.show and sd.avail and sd.vm) then sd.q, sd.held = {}, false end
        if not sd.show then sd.q, sd.hin, sd.held = {}, false, false end
        if #sd.q > 64 then sd.q = {} end
        any = any or sd.show
    end
    -- Verkehr (nur, solange eine Seite die Karte zeigt)
    if (PNL.sd[0].show and PNL.sd[0].vm) or (PNL.sd[1].show and PNL.sd[1].vm) then
        local okt, et = pcall(PNL.trf_read)
        if not okt and not PNL.trf_err then PNL.trf_err = true; pnl_log("traffic: " .. tostring(et)) end
        NDUI.trf.show[0], NDUI.trf.show[1] = PNL.trf_show(0), PNL.trf_show(1)
    end
    -- Tastatureingabe beenden, wenn ihr ND nicht mehr die Karte zeigt
    local kb = NDUI.kb
    if kb and not (PNL.sd[kb.side].show and PNL.sd[kb.side].vm) then NDUI.kb_end() end
    NDUI.blink = (PNL.ft or 0)
    -- welcher Zeichen-Aufruf ist spaet genug? (einmalig pruefen, bei Misserfolg alle 30 s erneut)
    if any or (PFD.msg() and CFG.nd_on) or PNL.need_samp or PNL.atc_any() then
        local nbr = 0
        for _, br in ipairs(PNL.bridges) do if br.ok then nbr = nbr + 1 end end
        if PNL.hook == nil and nbr == 0 then
            PNL.hook = false
            PNL.st = "helper plugin missing or not loaded - the map cannot be drawn over the FF ND (restart X-Plane)"
            pnl_log(PNL.st)
        elseif PNL.hook == false and nbr > 0 and PNL.retry and PNL.frame >= PNL.retry then
            PNL.hook, PNL.det_start = nil, nil
            for _, br in ipairs(PNL.bridges) do br.n, br.late = 0, 0 end
        end
        if PNL.hook == nil then
            PNL.det_start = PNL.det_start or PNL.frame
            pnl_decide()
        end
    end
    if any or PNL.atc_any() then
        PNL.show_frames = (PNL.show_frames or 0) + 1
        -- kein Leucht-Durchgang (Typ 2) gesehen? Dann die Karte im Tages-Durchgang zeichnen
        if PNL.show_frames == 240 then
            pnl_log(string.format("panel passes: type 1 = %d, type 2 = %d", PNL.seen[1] or 0, PNL.seen[2] or 0))
            if not PNL.seen[2] and (PNL.seen[1] or 0) > 0 then PNL.lit_in_1 = true; pnl_log("no lit pass - map drawn in the day pass") end
        end
    end
    -- Mausfenster nur in der 3D-Cockpitansicht und nur dort, wo der Mauszeiger gerade ist und das Skript etwas
    -- bedient: ueber dem ND mit Karte, den leuchtenden MASTER-CAUT-Tasten oder einem Range-Knopf. Sonst versteckt -
    -- ein Fenster ueber dem ganzen Bildschirm sperrt die Knoepfe des FF (das FF nimmt keine Maus unter Fenstern).
    -- Ganzer Bildschirm nur waehrend Ziehen auf der Karte oder Eingabe im Bedienfeld.
    local cockpit = d.vt and d.vt[0] == 1026 and not (d.vr and d.vr[0] == 1)
    local rect
    if cockpit then
        local ib = PNL.ib
        local okm = pcall(XP.btv_XPLMGetMouseLocationGlobal, ib, ib + 1)
        local mx, my = okm and ib[0] or -1e6, okm and ib[1] or -1e6
        local function box(x0, y0, x1, y1, m)
            if mx >= x0 - m and mx <= x1 + m and my >= y0 - m and my <= y1 + m then
                rect = { math.floor(x0 - m), math.ceil(y1 + m), math.ceil(x1 + m), math.floor(y0 - m) }
            end
        end
        if PNL.hook then
            for s = 0, 1 do
                local sd = PNL.sd[s]
                if sd.show and sd.avail ~= false and sd.vm and sd.quad then
                    local q = sd.quad
                    local x0, x1, y0, y1 = 1e9, -1e9, 1e9, -1e9
                    for _, c in ipairs(q) do x0, x1, y0, y1 = math.min(x0, c[1]), math.max(x1, c[1]), math.min(y0, c[2]), math.max(y1, c[2]) end
                    box(x0, y0, x1, y1, 4)
                end
            end
            if MCW.on and PNL.mcq then for _, b in ipairs(PNL.mcq) do box(b[1], b[2], b[3], b[4], 2) end end
            if PNL.atc_ready() and PNL.atc.quad then
                for which, q in pairs(PNL.atc.quad) do
                    local mode = PNL.atc_input(which)
                    if mode == "key" then
                        -- (FF's INIT page: only over our CPNY WIND REQUEST key, every other key stays FF's)
                        local x0, y0, x1, y1 = PNL.atc_key_screen()
                        if x0 then box(x0, y0, x1, y1, 2) end
                        -- our key covers FF's cursor: place ours from the mouse every frame (not only from the
                        -- window's cursor calls), so the cursor is always drawn on top of the key
                        local inv = PNL.atc.inv and PNL.atc.inv.lower
                        if inv then
                            local oki, lx, ly = pcall(inv, mx, my)
                            if oki and lx and PNL.atc_in_key(lx, ly) then
                                PNL.atc.cursor = PNL.atc_stable_cursor(PNL.atc.cursor, "lower", inv, mx, my, lx, ly)
                            elseif PNL.atc.cursor and PNL.atc.cursor.which == "lower" then
                                PNL.atc.cursor = nil
                            end
                        end
                    elseif mode == "full" then
                        local x0, x1, y0, y1 = 1e9, -1e9, 1e9, -1e9
                        for _, c in ipairs(q) do x0, x1, y0, y1 = math.min(x0, c[1]), math.max(x1, c[1]), math.min(y0, c[2]), math.max(y1, c[2]) end
                        box(x0, y0, x1, y1, 2)
                    end
                end
            end
        end
        for s = 0, 1 do
            local k = PNL.knq and PNL.knq[s]
            if k and d.zl then box(k[1] - k[3], k[2] - k[3], k[1] + k[3], k[2] + k[3], 0) end
        end
        local ab = PNL.abq
        if ab and in_air() then box(ab[1] - ab[3], ab[2] - ab[3], ab[1] + ab[3], ab[2] + ab[3], 0) end
        if PNL.cap or NDUI.kb or PNL.atc.capture or (PNL.atc.ui and PNL.atc.ui.focus) then rect = "full" end
    end
    local want = rect ~= nil
    if PNL.win then
        if want then
            local l, t, r, b = 0, SCREEN_HIGHT or 1000, SCREEN_WIDTH or 1000, 0
            if XPLMGetScreenBoundsGlobal then
                local ok, l1, t1, r1, b1 = pcall(XPLMGetScreenBoundsGlobal)
                if ok and type(l1) == "number" then l, t, r, b = l1, t1, r1, b1 end
            end
            if rect ~= "full" then l, t, r, b = rect[1], rect[2], rect[3], rect[4] end
            local g = PNL.win_geo
            if not g or g[1] ~= l or g[2] ~= t or g[3] ~= r or g[4] ~= b then
                XP.btv_XPLMSetWindowGeometry(PNL.win, l, t, r, b)
                PNL.win_geo = { l, t, r, b }
            end
        end
        if want ~= PNL.win_vis then
            XP.btv_XPLMSetWindowIsVisible(PNL.win, want and 1 or 0)
            PNL.win_vis = want
            if not want then
                PNL.cap = nil; NDUI.kb_end()
                for s = 0, 1 do PNL.sd[s].hin = false end
                PNL.atc.cursor = nil
                if PNL.atc.ui and PNL.atc.ui.hover then PNL.atc.ui.hover(nil) end
                if not cockpit then PNL.atc_blur() end
            end
        end
        -- Diagnose: kommen Mausereignisse an? (sonst faengt ein anderes Plugin die Maus vorher ab)
        if want then
            PNL.vis_frames = (PNL.vis_frames or 0) + 1
            if PNL.vis_frames == 600 then
                local q = PNL.sd[0].quad or PNL.sd[1].quad
                pnl_log(string.format("mouse window %s: %d mouse events in 600 frames%s", table.concat(PNL.win_geo or {}, "/"), PNL.ncur or 0,
                    q and string.format(", ND top left corner at %.0f/%.0f", q[1][1], q[1][2]) or ", ND not in view"))
            end
        end
    end
end

function btv_nd_frame()
    pcall(PNL.abrk_light)
    pcall(PNL.ab_drive)
    pnl_frame()
end
-- A/BRK-Licht (gruener Balken): an, solange BTV scharf ist oder bremst, die klassische Bremsautomatik gewaehlt ist
-- (Helligkeit wie die Moduslampen des FCU) oder RTO scharf ist
function PNL.abrk_light()
    if not PNL.abrk_dr then
        if not dr_exists(PNL.ABRK_DR) then return end
        PNL.abrk_dr = dataref_table(PNL.ABRK_DR)
        if dr_exists("1-sim/lamp/292") then PNL.abrk_rto = dataref_table("1-sim/lamp/292") end
        PNL.abrk_lamps = {}
        for _, n in ipairs({ 239, 240, 241, 242, 243 }) do
            if dr_exists("1-sim/lamp/" .. n) then PNL.abrk_lamps[#PNL.abrk_lamps + 1] = dataref_table("1-sim/lamp/" .. n) end
        end
    end
    local on = S.arm_request or S.mode == "ARMED" or S.mode == "ENGAGED" or S.mode == "RELEASE" or PNL.ab_type() > 0
    local v = 0
    if on then
        for _, t in ipairs(PNL.abrk_lamps) do v = math.max(v, t[0] or 0) end
        if v <= 0.02 and #PNL.abrk_lamps == 0 then v = 1 end
    end
    -- RTO scharf (Licht des FF hinter dem Balken)
    local rto = PNL.abrk_rto and PNL.abrk_rto[0] or 0
    if rto > 0.05 then v = math.max(v, rto) end
    if PNL.abrk_v ~= v then PNL.abrk_dr[PNL.ABRK_IDX] = v; PNL.abrk_v = v end
end
function nd_status()
    if not PNL.ok or PNL.hook == false then return PNL.st end
    local t = {}
    for s = 0, 1 do
        local sd = PNL.sd[s]
        local nm = (s == 0) and "CAPT" or "F/O"
        if sd.show then
            t[#t + 1] = nm .. ": ZOOM, map " .. (sd.vm and "on" or "not available")
        else
            t[#t + 1] = nm .. ": map at range knob ZOOM"
        end
    end
    return ((PNL.hook == nil) and "ready" or PNL.st) .. " - " .. table.concat(t, ", ")
end
function btv_nd_cam()
    -- (Bildschirmlage von ND und Range-Knoepfen auch bei verstecktem Mausfenster: danach richtet es sich)
    local d = PNL.dr
    if PNL.ok and (PNL.win_vis or (d and d.vt and d.vt[0] == 1026)) then pnl_cam() end
end

-- Befehle: OANS-Karte ein/aus, Zoom (z.B. fuer VR oder Joystick-Tasten)
-- Range-Knopf auf ZOOM und zurueck auf die vorige Reichweite (wie den Knopf drehen)
function btv_nd_toggle(side)
    local rr = PNL.dr and ((side == 1) and PNL.dr.zr or PNL.dr.zl)
    if not rr then return end
    PNL.zoom_prev = PNL.zoom_prev or {}
    if rr[0] < -0.5 then
        rr[0] = PNL.zoom_prev[side or 0] or 0
    else
        PNL.zoom_prev[side or 0] = math.floor(rr[0] + 0.5)
        rr[0] = -1
    end
end

-- (wie der Range-Knopf auf ZOOM: heraus ueber 5 NM hinaus stellt den Knopf zurueck auf 10)
function btv_nd_zoom(f)
    for s = 0, 1 do
        local sd = PNL.sd[s]
        local rr = PNL.dr and ((s == 0) and PNL.dr.zl or PNL.dr.zr)
        if rr and rr[0] < -0.5 then
            if PNL.zoom_step(s, f > 1) then rr[0] = 0 end
        elseif sd.show and sd.vm and sd.view and sd.view.eff and sd.view.rng[sd.view.eff] then
            local V = sd.view
            V.rng[V.eff] = clamp(V.rng[V.eff] + ((f > 1) and -1 or 1), 1, #RANGES)
        end
    end
end


-------------------------------------------------------------------------------
-- FMS lesen (ToLiss-Systemplugin der A350): Zielflughafen, Startflughafen, Landebahn/Anflug.
-- Die Werte sind Text-Datarefs (Byte-Felder); gelesen ueber XPLMGetDatab, etwa jede Sekunde.
-------------------------------------------------------------------------------
do
local FMS_NAMES = {
    { key = "dest", name = "AirbusFBW/fms/init/toName" },
    { key = "dest", name = "AirbusFBW/fms/arr/to" },
    { key = "dest", name = "AirbusFBW/fms/report/destination/name" },
    { key = "rwy", name = "AirbusFBW/fms/arr/rwy" },
    { key = "rwy", name = "AirbusFBW/fms/arr/app" },
    { key = "from", name = "AirbusFBW/fms/init/fromName" },
    { key = "from", name = "AirbusFBW/fms/dep/from" },
    { key = "altn", name = "AirbusFBW/fms/init/altnName" },
    { key = "altn", name = "AirbusFBW/fms/init/altn" },
    { key = "vapp", name = "1-sim/fms/perf/vapp", num = true },
}
local FMSX = { n = 0, tries = 0, refs = nil, last = {} }

-- Zahl (int/float/double) oder nil
local function fms_num(ref)
    local ty = XP.btv_XPLMGetDataRefTypes(ref)
    if bit.band(ty, 2) ~= 0 then return XP.btv_XPLMGetDataf(ref) end
    if bit.band(ty, 4) ~= 0 then return XP.btv_XPLMGetDatad(ref) end
    if bit.band(ty, 1) ~= 0 then return XP.btv_XPLMGetDatai(ref) end
end

local function fms_read(ref)
    local ty = XP.btv_XPLMGetDataRefTypes(ref)
    local s
    if bit.band(ty, 32) ~= 0 then
        -- size first: FF may write its whole value whatever length is asked for (fixed buffers crashed X-Plane)
        -- (over 64 KB: not read at all, a capped read could still be overrun)
        local size = XP.btv_XPLMGetDatab(ref, nil, 0, 0)
        if size > 65536 then size = 0 end
        if size > FMSX.blen then FMSX.blen, FMSX.buf = size, ffi.new("char[?]", size + 1) end
        local n = size > 0 and XP.btv_XPLMGetDatab(ref, FMSX.buf, 0, size) or 0
        if n > 0 then s = ffi.string(FMSX.buf, math.min(n, size)) end
    elseif bit.band(ty, 16) ~= 0 then
        local size = XP.btv_XPLMGetDatavi(ref, nil, 0, 0)
        if size > 4096 then size = 0 end
        if size > FMSX.ilen then FMSX.ilen, FMSX.ib = size, ffi.new("int[?]", size + 1) end
        local n = size > 0 and XP.btv_XPLMGetDatavi(ref, FMSX.ib, 0, size) or 0
        local t = {}
        for i = 0, math.min(n, size) - 1 do
            local c = FMSX.ib[i]
            if c <= 0 or c > 126 then break end
            t[#t + 1] = string.char(c)
        end
        s = table.concat(t)
    end
    if s then s = s:gsub("%z.*$", ""):gsub("^%s+", ""):gsub("%s+$", ""):upper() end
    return s
end

-- "25C", "RW25C", "ILS25C", "I25C-Y", "RNAV Z 07R" -> "25C" / "07R"
local function fms_rwy(v)
    local r = v:gsub("^RWY?", "")
    local num, side = r:match("(%d%d?)([LCR]?)")
    num = tonumber(num)
    if num and num >= 1 and num <= 36 then return string.format("%02d%s", num, side) end
end

FMS.poll = function()
    if not XP or not PNL.tried then return end
    FMSX.n = FMSX.n + 1
    if FMSX.n % 45 ~= 1 then return end
    if not FMSX.refs then
        if FMSX.tries >= 20 then return end
        FMSX.tries = FMSX.tries + 1
        if not FMSX.buf then FMSX.buf, FMSX.ib, FMSX.blen, FMSX.ilen = ffi.new("char[64]"), ffi.new("int[32]"), 63, 32 end
        local refs, found = {}, {}
        for _, d in ipairs(FMS_NAMES) do
            local r = XP.btv_XPLMFindDataRef(d.name)
            if r ~= nil then
                refs[#refs + 1] = { key = d.key, name = d.name, ref = r, num = d.num }
                found[#found + 1] = d.name
            end
        end
        if #refs == 0 then
            if FMSX.tries == 1 or FMSX.tries == 20 then
                log("FMS: no flight plan datarefs (AirbusFBW/fms/...) - runway from approach or ND selection only")
            end
            return
        end
        FMSX.refs = refs
        FMS.st = "connected"
        log("FMS: reading " .. table.concat(found, ", "))
    end
    local val = {}
    for _, d in ipairs(FMSX.refs) do
        local ok, v = pcall(d.num and fms_num or fms_read, d.ref)
        if d.num then
            -- VAPP [kt] aus PERF APPR (nur plausible Werte)
            if ok and type(v) == "number" and v >= 100 and v <= 200 then val[d.key] = v end
            local key = (ok and type(v) == "number") and string.format("%.0f", v) or "?"
            if FMSX.last[d.name] ~= key then
                FMSX.last[d.name] = key
                log("FMS: " .. d.name .. " = " .. key)
            end
            ok = false
        end
        if ok and v and v ~= "" then
            if FMSX.last[d.name] ~= v then
                FMSX.last[d.name] = v
                log("FMS: " .. d.name .. " = '" .. v .. "'")
            end
            if not val[d.key] then
                if d.key == "rwy" then
                    val.rwy = fms_rwy(v)
                else
                    local icao = v:match("^(%w%w%w%w?)")
                    if icao and (not IDX.ready or IDX.by_icao[icao]) then val[d.key] = icao end
                end
            end
        end
    end
    FMS.altn = val.altn
    if val.dest ~= FMS.dest or val.rwy ~= FMS.rwy or val.from ~= FMS.from then
        FMS.dest, FMS.rwy, FMS.from = val.dest, val.rwy, val.from
        log(string.format("FMS: FROM %s  TO %s  ARR RWY %s", FMS.from or "-", FMS.dest or "-", FMS.rwy or "-"))
        last_auto = 1000 -- sofort neu bewerten
    end
    FMS.vapp = val.vapp
    local l = FMS.dest and IDX.ready and IDX.by_icao[FMS.dest]
    FMS.dest_e = l and l[1] or nil
    FMS.st = FMS.dest and ("TO " .. FMS.dest .. (FMS.rwy and (" RWY " .. FMS.rwy) or "")) or "connected, no destination"
end
end

-------------------------------------------------------------------------------
-- Kommandos
-------------------------------------------------------------------------------
-- A/BRK-Taste im Flug (wie im A350): BTV scharf -> aus; klassische Bremsautomatik scharf -> aus; sonst mit Exit oder
-- Stoppstrecke BTV, ohne BTV-Vorbereitung die klassische Bremsautomatik MED (beim FF Stufe 3 = 3 m/s2)
function btv_arm_toggle()
    if S.arm_request then S.arm_request = false; return end
    if not btv_ui() then return end
    if PNL.ab_type() > 0 then
        PNL.ab_set(0)
        message("AUTO BRK DISARMED")
        return
    end
    if S.rwy and (S.exit or S.manual_dist) then
        if S.abrk then message("BTV NOT AVAIL: AUTO BRK SET"); return end
        S.arm_request = true
        return
    end
    if PNL.ab_set(PNL.AB_MED) then message("AUTO BRK MED") end
end

-- klassische Bremsautomatik des FF (Drehknopf ueber der Taste): 1-sim/gear/autobrakeType 0 = aus, 1 LO, 2, 3, 4 HI,
-- -1 RTO. Setzen: Wert direkt schreiben; ueberschreibt ihn das FF, den Knopf (1-sim/brk/innerSwitch) Schritt fuer
-- Schritt drehen wie mit der Maus, bis der Wert stimmt (hoechstens 3 s)
PNL.AB_MED = 3
function PNL.ab_type()
    if PNL.abt == nil then
        PNL.abt = dr_exists("1-sim/gear/autobrakeType") and dataref_table("1-sim/gear/autobrakeType") or false
        PNL.abk = dr_exists("1-sim/brk/innerSwitch") and dataref_table("1-sim/brk/innerSwitch") or false
    end
    return PNL.abt and math.floor((PNL.abt[0] or 0) + 0.5) or 0
end
function PNL.ab_set(v)
    if PNL.ab_type() == v then return true end
    if not PNL.abt then message("AUTO BRK NOT AVAIL"); return false end
    PNL.abt[0] = v
    PNL.ab_goal = { v = v, t0 = DR.time[0], tn = DR.time[0] + 0.2 }
    return true
end
function PNL.ab_drive()
    local g = PNL.ab_goal
    if not g then return end
    local cur, now = PNL.ab_type(), DR.time[0]
    if cur == g.v then PNL.ab_goal = nil; return end
    if now - g.t0 > 3 or not PNL.abk then
        PNL.ab_goal = nil
        message("AUTO BRK: SELECTOR DID NOT RESPOND")
        return
    end
    if now < g.tn then return end
    -- Richtung: erst zum Ziel hin; hat der letzte Schritt vom Ziel weggefuehrt, umdrehen
    if g.last and math.abs(g.v - cur) > math.abs(g.v - g.last) then g.flip = not g.flip end
    local dir = (g.v > cur) and 1 or -1
    if g.flip then dir = -dir end
    local k = PNL.abk[0] + dir
    if k < 0 or k > 100 then k, g.flip = PNL.abk[0] - dir, not g.flip end
    PNL.abk[0] = k
    g.last, g.tn = cur, now + 0.12
end

function btv_select_exit(step)
    if S.locked() then message("NOT AVAIL DURING BTV BRAKING"); return end
    if #S.exits == 0 or not btv_ui() then return end
    local idx = 0
    for i, x in ipairs(S.exits) do if x == S.exit then idx = i end end
    if idx == 0 then idx = (S.rec_idx or 1) - step end
    local j = idx + step
    while S.exits[j] and S.exits[j].too_short do j = j + step end -- (zu kurz: nicht waehlbar, wie im A350)
    if not S.exits[j] then return end
    idx = j
    S.exit, S.manual_dist = S.exits[idx], nil
    message(string.format("BTV EXIT %s  %d M", S.exit.name, math.floor(S.exit.dist + 0.5)))
end

function btv_disconnect()
    if S.mode == "ENGAGED" or S.mode == "RELEASE" then disconnect("BTV DISCONNECT") else S.arm_request = false end
end

-------------------------------------------------------------------------------
-- Cockpit-Modell des Flugzeugs (A350_xp11_cockpit.obj, XP11 und XP12): Range-Knopf mit der Stellung ZOOM (Taste
-- links von 10, wie im A350), gruene ZOOM-Lampe neben "ZOOM" (leuchtet wie die gewaehlte Moduslampe), 10-NM-Lampe
-- aus, solange ZOOM gewaehlt ist. Wird beim Laden des Flugzeugs eingetragen (vorher Sicherung *.before_A350_BTV)
-- und mit der Einstellung "cockpit_extras" wieder entfernt (Sicherung zurueck). Wirkt ab dem naechsten Laden des
-- Flugzeugs - kein automatisches Neuladen: sim/operation/reload_aircraft aus einem Skript-Rueckruf laedt sofort neu,
-- FlyWithLua startet dabei den Lua-Zustand neu, in dem der Rueckruf noch laeuft -> Absturz von X-Plane.
-------------------------------------------------------------------------------
PNL.EXTRAS_VT = {
    "VT -0.155575 4.502521 1.781467 0.000000 0.258800 0.965900 0.884277 0.914307",
    "VT -0.153825 4.502521 1.781467 0.000000 0.258800 0.965900 0.887695 0.914307",
    "VT -0.153825 4.504211 1.781015 0.000000 0.258800 0.965900 0.887695 0.917725",
    "VT -0.155575 4.504211 1.781015 0.000000 0.258800 0.965900 0.884277 0.917725",
    "VT 0.141925 4.502521 1.781467 0.000000 0.258800 0.965900 0.884277 0.914307",
    "VT 0.143675 4.502521 1.781467 0.000000 0.258800 0.965900 0.887695 0.914307",
    "VT 0.143675 4.504211 1.781015 0.000000 0.258800 0.965900 0.887695 0.917725",
    "VT 0.141925 4.504211 1.781015 0.000000 0.258800 0.965900 0.884277 0.917725",
    -- A/BRK-Taste (0,6 mm vor der Tastenflaeche, Aufteilung wie die FD-Taste des FF): gruener Balken (Leuchttextur des FF) ...
    "VT 0.226555 4.329455 1.603308 0.000000 0.168900 0.985641 0.913086 0.914551",
    "VT 0.216041 4.332355 1.602797 0.000000 0.168900 0.985641 0.903076 0.916992",
    "VT 0.226555 4.332355 1.602797 0.000000 0.168900 0.985641 0.913086 0.916992",
    "VT 0.216041 4.329455 1.603308 0.000000 0.168900 0.985641 0.903076 0.914551",
    -- ... und aufgemaltes "A/BRK" (in main.dds eingetragen, PNL.abrk_dds)
    "VT 0.228543 4.321227 1.604758 0.000000 0.168900 0.985641 0.913086 0.886719",
    "VT 0.214053 4.326661 1.603800 0.000000 0.168900 0.985641 0.918945 0.902344",
    "VT 0.228543 4.326661 1.603800 0.000000 0.168900 0.985641 0.918945 0.886719",
    "VT 0.214053 4.321227 1.604758 0.000000 0.168900 0.985641 0.913086 0.902344",
}
PNL.EXTRAS_IDX = { 0, 2, 1, 0, 3, 2, 4, 6, 5, 4, 7, 6, 8, 9, 10, 11, 9, 8, 12, 13, 14, 15, 13, 12 }
-- Helligkeit des A/BRK-Lichts: ein allgemeiner X-Plane-Wert (gibt es schon beim Laden des Modells, die A350 nutzt
-- ihn nicht) - das Skript setzt ihn (0 = aus)
PNL.ABRK_DR, PNL.ABRK_IDX = "sim/cockpit2/switches/generic_lights_switch", 120

function PNL.extras_patch(text)
    local eol = text:find("\r\n", 1, true) and "\r\n" or "\n"
    local L = {}
    for line in (text:gsub("\r\n", "\n") .. "\n"):gmatch("(.-)\n") do L[#L + 1] = line end
    -- 1. ZOOM-Stellung beider Range-Knoepfe: Drehung bei -1, Ziehbereich -1..6
    local keys, manips, i = 0, 0, 1
    while i <= #L do
        local t = L[i]:gsub("%s+$", "")
        if t:find("^ANIM_rotate_begin") and (t:find("1%-sim/fcu/ndZoomLeft/switch$") or t:find("1%-sim/fcu/ndZoomRight/switch$")) then
            table.insert(L, i + 1, "ANIM_rotate_key -1.000000 -34.459999")
            keys, i = keys + 1, i + 1
        elseif t:find("^ATTR_manip_drag_axis") and t:find("0.000000 6.000000 1-sim/fcu/ndZoom", 1, true) then
            L[i] = L[i]:gsub("0%.000000 6%.000000 1%-sim/fcu/ndZoom", "-1.000000 6.000000 1-sim/fcu/ndZoom", 1)
            manips = manips + 1
        end
        i = i + 1
    end
    if keys ~= 2 or manips ~= 2 then error("range knobs not found (" .. keys .. "/" .. manips .. ") - unknown cockpit version", 0) end
    -- 2. Anzahlen
    local pc
    for k, l in ipairs(L) do if l:find("^POINT_COUNTS") then pc = k; break end end
    if not pc then error("POINT_COUNTS missing", 0) end
    local p = {}
    for w in L[pc]:gmatch("%S+") do p[#p + 1] = w end
    local nv, ni = tonumber(p[2]), tonumber(p[5])
    local cv, ci = 0, 0
    for _, l in ipairs(L) do
        if l:find("^VT ") then cv = cv + 1
        elseif l:find("^IDX10 ") then ci = ci + 10
        elseif l:find("^IDX ") then ci = ci + 1 end
    end
    if cv ~= nv or ci ~= ni then error("POINT_COUNTS does not match the data", 0) end
    -- 3. Lampen
    local function lamp(name)
        for k = 1, #L - 1 do
            local t = L[k]:gsub("%s+$", "")
            if t:find("^ATTR_light_level") and t:sub(-#name - 1) == " " .. name and L[k + 1]:find("^TRIS ") then return k end
        end
        error("lamp " .. name .. " not found - unknown cockpit version", 0)
    end
    local function zoom_block(at, knob, modes, tris)
        local b = { "# ZOOM range lamp square (added for A350_BTV): lit like the selected mode lamp when the range knob is at ZOOM (-1)" }
        for _, m in ipairs(modes) do
            b[#b + 1] = "ANIM_begin"
            b[#b + 1] = "ANIM_hide -0.500000 99.000000 " .. knob
            b[#b + 1] = "ANIM_hide -99.000000 0.010000 1-sim/lamp/" .. m
            b[#b + 1] = "ATTR_light_level 0.000000 1.000000 1-sim/lamp/" .. m
            b[#b + 1] = "TRIS " .. tris .. " 6"
            b[#b + 1] = "ANIM_end"
        end
        for k = #b, 1, -1 do table.insert(L, at, b[k]) end
    end
    local function wrap_ten(name, knob)
        local a = lamp(name)
        table.insert(L, a + 2, "ANIM_end")
        table.insert(L, a, "ANIM_hide -99.000000 -0.500000 " .. knob)
        table.insert(L, a, "ANIM_begin")
        table.insert(L, a, "# (A350_BTV) 10 lamp off while the range knob is at ZOOM")
    end
    local KL, KR = "1-sim/fcu/ndZoomLeft/switch", "1-sim/fcu/ndZoomRight/switch"
    local function capt()
        zoom_block(lamp("1-sim/lamp/244") + 2, KL, { 239, 240, 241, 242, 243 }, ni)
        wrap_ten("1-sim/lamp/244", KL)
    end
    local function fo()
        zoom_block(lamp("1-sim/lamp/275") + 2, KR, { 276, 277, 278, 279, 280 }, ni + 6)
        wrap_ten("1-sim/lamp/269", KR)
    end
    if lamp("1-sim/lamp/244") > lamp("1-sim/lamp/275") then capt(); fo() else fo(); capt() end
    -- A/BRK-Taste wie im A350: statt des FF-Schriftzugs "RTO ARM" (1-sim/lamp/292) ein gruener Balken (leuchtet bei
    -- RTO oder BTV scharf, PNL.abrk_light) und das aufgemalte "A/BRK"
    local ab = PNL.ABRK_DR .. "[" .. PNL.ABRK_IDX .. "]"
    local at = lamp("1-sim/lamp/292")
    local blk = { "# A350_BTV v6: A/BRK pushbutton - green bar (RTO / BTV armed), painted A/BRK", "ANIM_begin",
                  "ANIM_hide -99.000000 0.010000 " .. ab, "ATTR_light_level 0.000000 1.000000 " .. ab,
                  "TRIS " .. (ni + 12) .. " 6", "ANIM_end",
                  "ATTR_light_level 0.000000 1.000000 1-sim/lights/consile", "TRIS " .. (ni + 18) .. " 6",
                  "ATTR_light_level_reset" }
    table.remove(L, at + 1)
    table.remove(L, at)
    for k = #blk, 1, -1 do table.insert(L, at, blk[k]) end
    -- 4. Geometrie: Indizes nach dem letzten IDX, Punkte nach dem letzten VT
    local lastVt, lastIdx = 0, 0
    for k, l in ipairs(L) do
        if l:find("^VT ") then lastVt = k elseif l:find("^IDX") then lastIdx = k end
    end
    for k = #PNL.EXTRAS_IDX, 1, -1 do table.insert(L, lastIdx + 1, "IDX " .. (nv + PNL.EXTRAS_IDX[k])) end
    for k = #PNL.EXTRAS_VT, 1, -1 do table.insert(L, lastVt + 1, PNL.EXTRAS_VT[k]) end
    for k, l in ipairs(L) do if l:find("^POINT_COUNTS") then pc = k; break end end
    p[2], p[5] = tostring(nv + #PNL.EXTRAS_VT), tostring(ni + #PNL.EXTRAS_IDX)
    L[pc] = table.concat(p, " ")
    -- Selbstpruefung: genau die erwarteten Aenderungen
    local nk, nm, nz, nt, vt, na = 0, 0, 0, 0, 0, 0
    for _, l in ipairs(L) do
        if l == "ANIM_rotate_key -1.000000 -34.459999" then nk = nk + 1 end
        if l:find("-1.000000 6.000000 1-sim/fcu/ndZoom", 1, true) then nm = nm + 1 end
        if l:find("^# ZOOM range lamp square") then nz = nz + 1 end
        if l:find("^# %(A350_BTV%) 10 lamp off") then nt = nt + 1 end
        if l:find("^VT ") then vt = vt + 1 end
        if l:find("^# A350_BTV v6: A/BRK pushbutton") then na = na + 1 end
        if l:find(" 1%-sim/lamp/292%s*$") then na = na + 10 end
    end
    if nk ~= 2 or nm ~= 2 or nz ~= 2 or nt ~= 2 or na ~= 1 or vt ~= nv + #PNL.EXTRAS_VT then
        error("self-check failed - cockpit model left unchanged", 0)
    end
    return table.concat(L, eol)
end

-- Datei sicher ersetzen: erst neben die alte schreiben und pruefen, dann austauschen
function PNL.extras_write(path, text)
    local tmp = path .. ".A350_BTV_tmp"
    local f = io.open(tmp, "wb")
    if not f then error("cannot write " .. tmp, 0) end
    f:write(text); f:close()
    f = io.open(tmp, "rb")
    local chk = f and f:read("*a")
    if f then f:close() end
    if chk ~= text then os.remove(tmp); error("write check failed", 0) end
    os.remove(path)
    if not os.rename(tmp, path) then
        f = io.open(path, "wb"); f:write(text); f:close(); os.remove(tmp)
    end
end

-- einschalten (true) / ausschalten (false); Rueckgabe: geaendert?
function PNL.acf_dir()
    local a = AIRCRAFT_PATH or ""
    if a ~= "" and not a:find("[/\\]$") then a = a .. "/" end
    return a
end
function PNL.extras(on)
    local obj = PNL.acf_dir() .. "A350_xp11_cockpit.obj"
    local bak = obj .. ".before_A350_BTV"
    local f = io.open(obj, "rb")
    if not f then return false, "A350_xp11_cockpit.obj not found" end
    local text = f:read("*a"); f:close()
    local patched = text:find("A350_BTV", 1, true) ~= nil
    if not on then
        if not patched then return false, "original" end
        local b = io.open(bak, "rb")
        if not b then return false, "no backup - reinstall the aircraft to undo" end
        local orig = b:read("*a"); b:close()
        PNL.extras_write(obj, orig)
        os.remove(bak)
        return true, "original restored"
    end
    if patched and not text:find("A350_BTV v6", 1, true) then
        -- aelterer Eintrag (ohne A/BRK-Taste): vom Original aus neu eintragen
        local b = io.open(bak, "rb")
        if not b then return false, "older version installed, no backup" end
        text = b:read("*a"); b:close()
        patched = false
    end
    if patched then return false, "installed" end
    local new = PNL.extras_patch(text)
    local b = io.open(bak, "rb")
    if b then b:close() else PNL.extras_write(bak, text) end
    PNL.extras_write(obj, new)
    return true, "installed"
end

-- Aufgemaltes "A/BRK" der A/BRK-Taste: wird in eine freie, durchsichtige Stelle der FF-Textur main.dds (DXT5,
-- 4096 px; dort auch im Leuchtbild leer) eingetragen - weiss, Deckkraft aus dem Bild unten (48 x 64 px, 0..f,
-- Schrift um 90 Grad gedreht), Mip-Stufen 0 bis 2. Die
-- ueberschriebenen Bloecke stehen vorher in main.dds.A350_BTV_blocks.
PNL.ABRK_TXT = { x = 3728, y = 400, w = 48, h = 64,
    a = "0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000009a61000000000000000000000000000000000000000000006dffb7200000000000000000000000000000000000000000002feefd8400000000000000000000000000000000000000000fa048cfe9300000000000000000000000000000000000000fa00039ff900000000000000000000000000000000000000fb59dfd94000000000000000000000000000000000000037fffc730000000000000000000000000000000000000009ffb620000000000000000000000000000000000000000006510000000000000000000000000000000000000000000ea61000000000000000000000000000000000000000000006aefc730000000000000000000000000000000000000000000049dfe951000000000000000000000000000000000000000000027bffb720000000000000000000000000000000000000000000159efd800000000000000000000000000000000000000000000038c0000000000000000000000000000000000455555555555300000000000000000000000000000000000afffffffffff900000000000000000000000000000000000ae1113f7111f900000000000000000000000000000000000ae0002f6000f900000000000000000000000000000000000ae0002f6000f9000000000000000000000000000000000009f4006fa005f7000000000000000000000000000000000002efbcf6eedfe200000000000000000000000000000000000029ba4028971000000000000000000000000000000000000000000000000000000000000000000000000000000000000122222222222100000000000000000000000000000000000afffffffffff90000000000000000000000000000000000035555af6555f900000000000000000000000000000000000000007f2000f90000000000000000000000000000000000000002bf2000f900000000000000000000000000000000000005bfff5003f8000000000000000000000000000000000005dfd72dfbbee200000000000000000000000000000000000ac500029cca20000000000000000000000000000000000001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000008ccccccccccc700000000000000000000000000000000000699aefc9999960000000000000000000000000000000000000002bf80000000000000000000000000000000000000000000005efc3000000000000000000000000000000000000000005cfe7cf8100000000000000000000000000000000000005cfe71007fd3000000000000000000000000000000000009fd71000003c900000000000000000000000000000000000860000000000300000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000" }
function PNL.abrk_dds(on)
    local path = PNL.acf_dir() .. "main.dds"
    local bak = path .. ".A350_BTV_blocks"
    local f = io.open(path, "r+b")
    if not f then return "main.dds not found" end
    local h = f:read(128) or ""
    local function u32(o) local a, b, c, d = h:byte(o + 1, o + 4); return a + b * 256 + c * 65536 + d * 16777216 end
    if #h < 128 or h:sub(1, 4) ~= "DDS " or h:sub(85, 88) ~= "DXT5" or u32(12) ~= 4096 or u32(16) ~= 4096 then
        f:close()
        return "main.dds: unknown format"
    end
    local T = PNL.ABRK_TXT
    local A = {}
    for i = 1, T.w * T.h do A[i] = tonumber(T.a:sub(i, i), 16) * 17 end
    local COL = string.char(0x5D, 0xEF, 0x5D, 0xEF, 0, 0, 0, 0)    -- hellgrau-weiss (RGB565 0xEF5D)
    local rows, base, dim, w, ht, x0, y0 = {}, 128, 4096, T.w, T.h, T.x, T.y
    for _ = 0, 2 do
        local bpr = dim / 4
        for by = 0, ht / 4 - 1 do
            local parts = {}
            for bx = 0, w / 4 - 1 do
                local val = 0
                for py = 0, 3 do
                    for px = 0, 3 do
                        local st = math.floor(A[(by * 4 + py) * w + bx * 4 + px + 1] * 7 / 255 + 0.5)
                        local ix = (st == 7) and 0 or ((st == 0) and 1 or (8 - st))
                        val = val + ix * 2 ^ (3 * (py * 4 + px))
                    end
                end
                local b = { 255, 0 }
                for k = 1, 6 do b[k + 2] = math.floor(val / 256 ^ (k - 1)) % 256 end
                parts[#parts + 1] = string.char(unpack(b)) .. COL
            end
            rows[#rows + 1] = { base + ((y0 / 4 + by) * bpr + x0 / 4) * 16, table.concat(parts) }
        end
        base, dim = base + bpr * bpr * 16, dim / 2
        local B = {}
        for yy = 0, ht / 2 - 1 do
            for xx = 0, w / 2 - 1 do
                local i = 2 * yy * w + 2 * xx + 1
                B[yy * (w / 2) + xx + 1] = (A[i] + A[i + 1] + A[i + w] + A[i + w + 1]) / 4
            end
        end
        A, w, ht, x0, y0 = B, w / 2, ht / 2, x0 / 2, y0 / 2
    end
    local same = true
    for _, r in ipairs(rows) do
        f:seek("set", r[1])
        if f:read(#r[2]) ~= r[2] then same = false; break end
    end
    if on then
        if not same then
            local b = io.open(bak, "rb")
            if b then b:close() else
                local parts = {}
                for _, r in ipairs(rows) do f:seek("set", r[1]); parts[#parts + 1] = f:read(#r[2]) end
                b = io.open(bak, "wb")
                if not b then f:close(); return "cannot write " .. bak end
                b:write(table.concat(parts)); b:close()
            end
            for _, r in ipairs(rows) do f:seek("set", r[1]); f:write(r[2]) end
        end
        f:close()
        return same and "A/BRK painted" or "A/BRK painted - visible from the next aircraft load"
    end
    local b = io.open(bak, "rb")
    if not (same and b) then
        if b then b:close() end
        f:close()
        return "main.dds original"
    end
    local data = b:read("*a"); b:close()
    local k = 1
    for _, r in ipairs(rows) do f:seek("set", r[1]); f:write(data:sub(k, k + #r[2] - 1)); k = k + #r[2] end
    f:close()
    os.remove(bak)
    return "main.dds original restored"
end

-------------------------------------------------------------------------------
-- Leuchtendes "A/BRK" bei Nacht: dieselbe Schrift auch ins Leuchtbild main_LIT.png des FF (gleiche Stelle wie in
-- main.dds, dort ebenfalls leer). Das PNG wird von einem Hintergrundprogramm (PowerShell mit C#, Bordmittel von
-- Windows) verlustfrei neu geschrieben - nichts Schweres im Simulator; das Original bleibt als
-- main_LIT.png.before_A350_BTV. Die Schrift leuchtet mit dem Konsolenlicht (1-sim/lights/consile).
-------------------------------------------------------------------------------
PNL.LIT_PS = [==[
$ErrorActionPreference = 'Stop'
try {
Add-Type -TypeDefinition @'
using System;
using System.IO;
using System.IO.Compression;
public static class A350BtvAbrkLit {
    static uint U32(byte[] b, int o) { return (uint)(b[o] << 24 | b[o + 1] << 16 | b[o + 2] << 8 | b[o + 3]); }
    static uint[] T;
    static uint Crc(byte[] b) {
        if (T == null) {
            T = new uint[256];
            for (uint i = 0; i < 256; i++) { uint c = i; for (int k = 0; k < 8; k++) c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1; T[i] = c; }
        }
        uint r = 0xFFFFFFFFu;
        for (int i = 0; i < b.Length; i++) r = T[(r ^ b[i]) & 255] ^ (r >> 8);
        return r ^ 0xFFFFFFFFu;
    }
    static uint Adler(byte[] b) {
        uint a = 1, s = 0; int i = 0, n = b.Length;
        while (i < n) { int e = Math.Min(n, i + 5552); for (; i < e; i++) { a += b[i]; s += a; } a %= 65521; s %= 65521; }
        return (s << 16) | a;
    }
    static void BE(Stream s, uint v) { s.WriteByte((byte)(v >> 24)); s.WriteByte((byte)(v >> 16)); s.WriteByte((byte)(v >> 8)); s.WriteByte((byte)v); }
    static void Chunk(Stream s, string t, byte[] d) {
        byte[] td = new byte[4 + d.Length];
        for (int i = 0; i < 4; i++) td[i] = (byte)t[i];
        Buffer.BlockCopy(d, 0, td, 4, d.Length);
        BE(s, (uint)d.Length); s.Write(td, 0, td.Length); BE(s, Crc(td));
    }
    static bool Inflate(byte[] z, byte[] raw) {
        using (var ds = new DeflateStream(new MemoryStream(z, 2, z.Length - 2), CompressionMode.Decompress)) {
            int got = 0, r;
            while (got < raw.Length && (r = ds.Read(raw, got, raw.Length - got)) > 0) got += r;
            if (got != raw.Length) return false;
        }
        return Adler(raw) == U32(z, z.Length - 4);
    }
    public static string Run(string path, string bak, string mark, int tx, int ty, int tw, int th, string hex) {
        byte[] png = File.ReadAllBytes(path);
        string all = System.Text.Encoding.ASCII.GetString(png);
        if (all.Contains(mark)) return "already installed";
        int pos = 8, W = 0, H = 0;
        var keep = new MemoryStream(); var idat = new MemoryStream();
        while (pos + 8 <= png.Length) {
            int len = (int)U32(png, pos);
            string typ = System.Text.Encoding.ASCII.GetString(png, pos + 4, 4);
            if (typ == "IHDR") {
                W = (int)U32(png, pos + 8); H = (int)U32(png, pos + 12);
                if (png[pos + 16] != 8 || png[pos + 17] != 6 || png[pos + 20] != 0) return "ERROR unsupported PNG format";
            }
            if (typ == "IDAT") idat.Write(png, pos + 8, len);
            else if (typ == "IEND") break;
            else keep.Write(png, pos, len + 12);
            pos += len + 12;
        }
        if (W != 4096 || H != 4096) return "ERROR unexpected size";
        int st = W * 4 + 1, rb = W * 4;
        byte[] raw = new byte[st * H];
        if (!Inflate(idat.ToArray(), raw)) return "ERROR decode check failed";
        for (int y = 0; y < H; y++) {
            int r0 = y * st + 1, p0 = r0 - st, ft = raw[y * st];
            for (int i = 0; i < rb; i++) {
                int a = i >= 4 ? raw[r0 + i - 4] : 0, b = y > 0 ? raw[p0 + i] : 0, c = (i >= 4 && y > 0) ? raw[p0 + i - 4] : 0, pr;
                if (ft == 0) pr = 0; else if (ft == 1) pr = a; else if (ft == 2) pr = b; else if (ft == 3) pr = (a + b) >> 1;
                else if (ft == 4) {
                    int pa = Math.Abs(b - c), pb = Math.Abs(a - c), pc = Math.Abs(a + b - 2 * c);
                    pr = (pa <= pb && pa <= pc) ? a : (pb <= pc ? b : c);
                } else return "ERROR bad PNG filter";
                raw[r0 + i] = (byte)(raw[r0 + i] + pr);
            }
            raw[y * st] = 0;
        }
        // warm white like FF's panel labels, opacity from the lettering image
        for (int sy = 0; sy < th; sy++)
            for (int sx = 0; sx < tw; sx++) {
                int a = Convert.ToInt32(hex.Substring(sy * tw + sx, 1), 16) * 17;
                int o = (ty + sy) * st + 1 + (tx + sx) * 4;
                raw[o] = (byte)a; raw[o + 1] = (byte)(a * 94 / 100); raw[o + 2] = (byte)(a * 78 / 100); raw[o + 3] = (byte)a;
            }
        var zs = new MemoryStream();
        zs.WriteByte(0x78); zs.WriteByte(0x9C);
        using (var ds = new DeflateStream(zs, CompressionLevel.Optimal, true)) ds.Write(raw, 0, raw.Length);
        BE(zs, Adler(raw));
        byte[] nz = zs.ToArray();
        byte[] chk = new byte[raw.Length];
        if (!Inflate(nz, chk)) return "ERROR encode check failed";
        for (int i = 0; i < raw.Length; i++) if (chk[i] != raw[i]) return "ERROR encode compare failed";
        var o2 = new MemoryStream();
        o2.Write(png, 0, 8);
        keep.WriteTo(o2);
        Chunk(o2, "tEXt", System.Text.Encoding.ASCII.GetBytes("Comment\0" + mark));
        Chunk(o2, "IDAT", nz);
        Chunk(o2, "IEND", new byte[0]);
        File.WriteAllBytes(bak, png);
        File.WriteAllBytes(path + ".A350_BTV_tmp", o2.ToArray());
        File.Delete(path);
        File.Move(path + ".A350_BTV_tmp", path);
        return "installed";
    }
}
'@
$r = [A350BtvAbrkLit]::Run('@@PATH@@', '@@BAK@@', 'A350_BTV A/BRK', @@X@@, @@Y@@, @@W@@, @@H@@, '@@HEX@@')
} catch { $r = 'ERROR ' + $_.Exception.Message }
Set-Content -Path '@@LOG@@' -Value $r -Encoding ASCII
]==]
function PNL.abrk_lit(on)
    local path = PNL.acf_dir() .. "main_LIT.png"
    local bak = path .. ".before_A350_BTV"
    local logf = SCRIPTS .. "A350_BTV_abrk_lit.log"
    local f = io.open(path, "rb")
    if not f then return "main_LIT.png not found" end
    local png = f:read("*a"); f:close()
    local marked = png:find("A350_BTV A/BRK", 1, true) ~= nil
    if not on then
        if not marked then return "main_LIT.png original" end
        local b = io.open(bak, "rb")
        if not b then return "main_LIT.png: no backup - reinstall the aircraft to undo" end
        local orig = b:read("*a"); b:close()
        PNL.extras_write(path, orig)
        os.remove(bak)
        return "main_LIT.png original restored"
    end
    if marked then return "A/BRK night lighting installed" end
    if package.config:sub(1, 1) ~= "\\" then return "A/BRK night lighting: Windows only" end
    local lf = io.open(logf, "r")
    local last = lf and lf:read("*l")
    if lf then lf:close() end
    if last and last:find("^ERROR") then return "A/BRK night lighting failed: " .. last .. " (delete " .. logf .. " to retry)" end
    local T = PNL.ABRK_TXT
    local function q(s) return (s:gsub("/", "\\"):gsub("'", "''")) end
    local V = { PATH = q(path), BAK = q(bak), LOG = q(logf), X = T.x, Y = T.y, W = T.w, H = T.h, HEX = T.a }
    local ps = SCRIPTS .. "A350_BTV_abrk_lit.ps1"
    local pf = io.open(ps, "wb")
    if not pf then return "cannot write " .. ps end
    pf:write((PNL.LIT_PS:gsub("@@(%u+)@@", function(k) return tostring(V[k]) end):gsub("\r?\n", "\r\n")))
    pf:close()
    os.remove(logf)
    OSM.run_hidden('powershell.exe -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File "' .. q(ps) .. '"')
    return "A/BRK night lighting: started in the background - visible from the next aircraft load"
end

-- beim Laden und bei Aenderung der Einstellung: anwenden (sichtbar ab dem naechsten Laden des Flugzeugs).
-- Kein automatisches Neuladen mehr (bis 2.18.1): X-Plane fuehrt reload_aircraft als kompletten Neustart des Flugs
-- aus - das Bild steht dabei rund eine Minute, X-Plane 11 wurde in der Zeit als "reagiert nicht" geschlossen.
function PNL.extras_apply()
    local okd, wd = pcall(PNL.abrk_dds, CFG.cockpit_extras)
    log("cockpit A/BRK: " .. (okd and tostring(wd) or ("ERROR " .. tostring(wd))))
    local okl, wl = pcall(PNL.abrk_lit, CFG.cockpit_extras)
    log("cockpit A/BRK night: " .. (okl and tostring(wl) or ("ERROR " .. tostring(wl))))
    local ok, changed, why = pcall(PNL.extras, CFG.cockpit_extras)
    log("cockpit extras: " .. (ok and (tostring(why) .. (changed and " - visible from the next aircraft load" or ""))
        or ("ERROR " .. tostring(changed))))
end
-- (aus dem Einstellungsfenster, das vor PNL steht)
function btv_extras_apply() pcall(PNL.extras_apply) end

function btv_on_exit()
    pcall(PNL.atc_msg_off, "script exit")
    pcall(PNL.atp_save)
    pcall(MCW.shutdown)
    pcall(MCW.brk_shutdown)
    pcall(release_override)
    pcall(pnl_shutdown)
    save_config()
end

create_command("FlyWithLua/A350_BTV/toggle_window", "A350 Systems: Settings (show/hide)", "btv_toggle_window()", "", "")
create_command("FlyWithLua/A350_BTV/arm_toggle", "A350 BTV: ARM/DISARM", "btv_arm_toggle()", "", "")
create_command("FlyWithLua/A350_BTV/next_exit", "A350 BTV: next exit", "btv_select_exit(1)", "", "")
create_command("FlyWithLua/A350_BTV/prev_exit", "A350 BTV: previous exit", "btv_select_exit(-1)", "", "")
create_command("FlyWithLua/A350_BTV/disconnect", "A350 BTV: Disconnect", "btv_disconnect()", "", "")
add_macro("A350 Systems", "btv_toggle_window()")

create_command("FlyWithLua/A350_BTV/nd_zoom", "A350 BTV: captain range knob ZOOM / back", "btv_nd_toggle(0)", "", "")
create_command("FlyWithLua/A350_BTV/nd_zoom_fo", "A350 BTV: F/O range knob ZOOM / back", "btv_nd_toggle(1)", "", "")
create_command("FlyWithLua/A350_BTV/nd_map_in", "A350 BTV: OANS map zoom in", "btv_nd_zoom(1.5)", "", "")
create_command("FlyWithLua/A350_BTV/nd_map_out", "A350 BTV: OANS map zoom out", "btv_nd_zoom(1 / 1.5)", "", "")
do_every_frame("btv_frame()")
do_every_draw("btv_draw()")
do_on_exit("btv_on_exit()")

add_job("index", build_index_job)
-- Cockpit-Modell (ZOOM am Range-Knopf)
pcall(PNL.extras_apply)
log("version " .. VERSION .. " loaded")

-- nur fuer Offline-Tests
if A350_BTV_TEST then
    A350_BTV_TEST.S, A350_BTV_TEST.IDX, A350_BTV_TEST.P, A350_BTV_TEST.CFG = S, IDX, P, CFG
    A350_BTV_TEST.select_runway, A350_BTV_TEST.rwy_sd = select_runway, rwy_sd
    A350_BTV_TEST.load_airport, A350_BTV_TEST.PNL, A350_BTV_TEST.GLB = load_airport, PNL, GLB
    A350_BTV_TEST.FMS, A350_BTV_TEST.MENU, A350_BTV_TEST.jobs = FMS, MENU, jobs
    A350_BTV_TEST.NDUI, A350_BTV_TEST.EFIS, A350_BTV_TEST.RWYA, A350_BTV_TEST.TF = NDUI, EFIS, RWYA, TF
    A350_BTV_TEST.MAPX, A350_BTV_TEST.RANGES = MAPX, RANGES
    A350_BTV_TEST.pnl_cursor, A350_BTV_TEST.pnl_click = pnl_cursor, pnl_click
    A350_BTV_TEST.PFD, A350_BTV_TEST.P = PFD, P
    A350_BTV_TEST.AUD, A350_BTV_TEST.XFM, A350_BTV_TEST.MCW, A350_BTV_TEST.say = AUD, XFM, MCW, say
end
