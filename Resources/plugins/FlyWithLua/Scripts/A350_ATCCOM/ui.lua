-- A350 Systems Mod - ATCCOM page (lower centre MFD + mailbox on the upper display). MIT License.
-- Layout, colours and behaviour follow photos of the real A350 ATC COM pages; text in the FF display font at pixel
-- exact sizes. Draws only primitives ({'rect'|'text'|'tri', ...}) in page units (lower 679 x 884, upper 679 x 236);
-- A350_BTV.lua puts them into the cockpit display and hands it the datalink (A350_ATCCOM/hoppie.lua) in
-- M.link_tick.
-- The top 32 units of the lower page stay FF's own system selector ("ATC COM v"); only x > 190 is used there.

local M = { enabled = true, hits = {}, focus = nil, dirty = true, flight = "", link = "ACARS WAITING" }

-- font sizes: cap height in page units for pixel exact FF font sizes (1 unit = 0.723 panel pixels vertically).
-- Sizes are named 9..12 in the layout and drawn one pixel larger (BUMP): the FF MFD pages use 11-12 px capitals
-- (measured next to FF's INIT page from the same camera position).
local BUMP = 1
local function px(p) return (p + BUMP) / 0.7229 end
local F9, F10, F11, F12 = px(9), px(10), px(11), px(12)
local ADV = { [9] = 8, [10] = 9, [11] = 10, [12] = 11, [13] = 12, [14] = 12 } -- character advance in panel pixels
local function tw(t, p) return #t * (ADV[p + BUMP] or 10) / 0.7426 end -- text width in page units

-- colours (ABGR), matched to FF's own MFD pages on the same screen
local C = {
    bg = 0xFF000000, white = 0xFFFFFFFF, grey = 0xFF8C8C8C, dim = 0xFF5E5C5A, cyan = 0xFFF0DE58, green = 0xFF3DD334,
    amber = 0xFF2C92D6, btn = 0xFF403F40, btn_off = 0xFF282828, hi = 0xFF9C9894, lo = 0xFF141110, band = 0xFFF4F5F5,
    band_text = 0xFF8C8C8C, line = 0xFF6E6C6A, frame = 0xFFC8C8C8, field = 0xFF050403, emer = 0xFF0A4696,
    emer_text = 0xFF2896FF, black = 0xFF000000,
    list = 0xFF7A7A7A, -- FF's dropdown lists (measured against FF's keys in the same light: about RGB 122)
    kframe = 0xFFB4B4B4, -- FF's key border (from FF's INIT page next to ours: same brightness on screen)
}
local KB = 1.6 -- key border width

local CATS = { "VERTICAL", "LATERAL", "SPEED", "CLEARANCE", "WHEN CAN\nWE EXPECT", "OTHER" }
-- menu entries: { label, request id }
local MENUS = {
    [1] = { { "CLIMB TO", "CLIMB TO" }, { "DESCEND TO", "DESCEND TO" }, { "BLOCK ALT/FL", "BLOCK" } },
    [2] = { { "DIRECT TO", "DIRECT TO" }, { "OFFSET", "OFFSET" }, { "WX DEVIATION", "WX DEVIATION" }, { "HEADING", "HEADING" },
        { "TRACK", "TRACK" } },
    [3] = { { "SPEED", "SPEED" }, { "SPEED RANGE", "SPEED RANGE" } },
    [4] = { { "DEPARTURE", "DEPARTURE" } },
    [5] = { { "HIGHER LEVEL", "W HIGHER" }, { "LOWER LEVEL", "W LOWER" }, { "CLIMB TO", "W CLIMB" }, { "DESCEND TO", "W DESCEND" },
        { "SPEED", "W SPEED" }, { "BACK ON ROUTE", "W ROUTE" } },
    [6] = { { "FREETEXT MSG", "FREETEXT MSG" } },
}
-- ADD TEXT: { label, CPDLC text } (FREETEXT = own text)
local REASONS = { { "DUE TO WEATHER", "DUE TO WEATHER" }, { "DUE TO A/C PERFORMANCE", "DUE TO AIRCRAFT PERFORMANCE" },
    { "DUE TO TURBULENCE", "DUE TO TURBULENCE" }, { "DUE TO TECHNICAL", "DUE TO TECHNICAL" }, { "DUE TO MEDICAL", "DUE TO MEDICAL" },
    { "AT PILOTS DISCRETION", "AT PILOTS DISCRETION" }, { "FREETEXT", false } }
local TABS = { { 1, "CONNECT" }, { 3, "REQUEST" }, { 4, "REPORT\n& MODIFY" }, { 8, "MSG\nRECORD" }, { 9, "D-ATIS" }, { 10, "EMER" } }
local TITLES = { [1] = "CONNECT", [3] = "REQUEST", [4] = "REPORT & MODIFY", [8] = "MSG RECORD", [9] = "D-ATIS",
    [10] = "EMERGENCY", [11] = "MSG LIST" }

local function utc() return os.date("!%H%MZ") end
local function utc_colon() return os.date("!%H:%MZ") end
local function station(s) return type(s) == "string" and #s >= 3 and #s <= 9 and s:match("^[A-Z0-9_]+$") ~= nil end
local function icao(s) return type(s) == "string" and s:match("^[A-Z][A-Z][A-Z][A-Z]$") ~= nil end
local function nonblank(s) return type(s) == "string" and s:find("%S") ~= nil end

local function initial()
    local s = { page = 1, center = "", active = "", pending = "", net = "VATSIM", info = "", info_warn = false,
        mail = {}, idx = 1, reply = "", records = {}, recscroll = 0, open_rec = nil, stations = nil, dropdown = nil,
        atis = { { code = "", mode = "DEP" }, { code = "", mode = "ARR" }, { code = "", mode = "ARR" } },
        route = {}, hid = 0, cat = nil, sel = nil, group = nil, val = {}, scroll = 0, notified_list = {}, reasons = {}, extra = nil }
    return s
end
M.s = initial()
function M.reset() M.s = initial(); M.focus, M.net_loaded, M.net_chosen = nil, nil, nil; M.dirty = true end

local function info(t, warn) M.s.info, M.s.info_warn, M.dirty = t or "", warn and true or false, true end

-------------------------------------------------------------------------------
-- request definitions
-------------------------------------------------------------------------------
-- requests: { title, fields, shown text, wire text } - fields { label, max length, kind }; %1 / %2 = field values.
-- Wire text = Hoppie CPDLC wording: "@" before each value, "@_@" between message elements (as in the uplinks).
-- kind: level / wpt / nm / lr / deg / spd / station / icao / text / nil (free)
local LR = { { "DISTANCE NM", 3, "nm" }, { "DIRECTION L/R", 1, "lr" } }
local REQ = {
    ["CLIMB TO"] = { "CLIMB TO", { { "LEVEL", 8, "level" } }, "REQUEST CLIMB TO %1", "REQUEST CLIMB TO@%1" },
    ["DESCEND TO"] = { "DESCEND TO", { { "LEVEL", 8, "level" } }, "REQUEST DESCENT TO %1", "REQUEST DESCENT TO@%1" },
    ["BLOCK"] = { "BLOCK ALT/FL", { { "LOWER LEVEL", 8, "level" }, { "UPPER LEVEL", 8, "level" } }, "REQUEST BLOCK %1 TO %2",
        "REQUEST BLOCK@%1@TO@%2" },
    ["DIRECT TO"] = { "DIRECT TO", { { "WAYPOINT", 7, "wpt" } }, "REQUEST DIRECT TO %1", "REQUEST DIRECT TO@%1" },
    ["OFFSET"] = { "OFFSET", LR, "REQUEST OFFSET %1NM %2 OF ROUTE", "REQUEST OFFSET@%1NM@%2@OF ROUTE" },
    ["WX DEVIATION"] = { "WX DEVIATION", LR, "REQUEST WEATHER DEVIATION UP TO %1NM %2 OF ROUTE",
        "REQUEST WEATHER DEVIATION UP TO@%1NM@%2@OF ROUTE" },
    ["HEADING"] = { "HEADING", { { "DEGREES", 3, "deg" } }, "REQUEST HEADING %1", "REQUEST HEADING@%1" },
    ["TRACK"] = { "TRACK", { { "DEGREES", 3, "deg" } }, "REQUEST GROUND TRACK %1", "REQUEST GROUND TRACK@%1" },
    ["SPEED"] = { "SPEED", { { "IAS / MACH", 6, "spd" } }, "REQUEST %1", "REQUEST @%1" },
    ["SPEED RANGE"] = { "SPEED RANGE", { { "MIN SPEED", 6, "spd" }, { "MAX SPEED", 6, "spd" } }, "REQUEST %1 TO %2",
        "REQUEST @%1@TO@%2" },
    ["W HIGHER"] = { "WHEN CAN WE EXPECT", {}, "WHEN CAN WE EXPECT HIGHER ALT", "WHEN CAN WE EXPECT HIGHER ALT" },
    ["W LOWER"] = { "WHEN CAN WE EXPECT", {}, "WHEN CAN WE EXPECT LOWER ALT", "WHEN CAN WE EXPECT LOWER ALT" },
    ["W CLIMB"] = { "WHEN CAN WE EXPECT", { { "LEVEL", 8, "level" } }, "WHEN CAN WE EXPECT CLIMB TO %1", "WHEN CAN WE EXPECT CLIMB TO@%1" },
    ["W DESCEND"] = { "WHEN CAN WE EXPECT", { { "LEVEL", 8, "level" } }, "WHEN CAN WE EXPECT DESCENT TO %1",
        "WHEN CAN WE EXPECT DESCENT TO@%1" },
    ["W SPEED"] = { "WHEN CAN WE EXPECT", { { "IAS / MACH", 6, "spd" } }, "WHEN CAN WE EXPECT %1", "WHEN CAN WE EXPECT@%1" },
    ["W ROUTE"] = { "WHEN CAN WE EXPECT", {}, "WHEN CAN WE EXPECT BACK ON ROUTE", "WHEN CAN WE EXPECT BACK ON ROUTE" },
    ["DEPARTURE"] = { "DEPARTURE", { { "DEPARTURE ARPT", 4, "icao" }, { "GATE", 6 }, { "D-ATIS CODE", 1 }, { "ACFT TYPE", 4 },
        { "DESTINATION", 4, "icao" } } },
    ["FREETEXT MSG"] = { "FREETEXT MSG", { { "RECIPIENT", 7, "station" }, { "MESSAGE", 200, "text" } } },
}
local function fields(sel) return REQ[sel] and REQ[sel][2] or {} end
local function kind_of(sel)
    if sel == "FREETEXT MSG" or sel == "DEPARTURE" then return "TELEX" end
    return "CPDLC"
end
local function req_title(sel)
    local t = REQ[sel] and REQ[sel][1] or ""
    if sel == "FREETEXT MSG" or t == "WHEN CAN WE EXPECT" then return t end
    return t .. " REQUEST"
end
local function recipient(s)
    if s.sel == "FREETEXT MSG" then return s.val.RECIPIENT or "" end
    if s.sel == "DEPARTURE" then return s.val["DEPARTURE ARPT"] or "" end
    return s.active
end
local function fill(tpl, s)
    local f = fields(s.sel)
    return (tpl:gsub("%%(%d)", function(i) local d = f[tonumber(i)]; return d and (s.val[d[1]] or "") or "" end))
end
local function extra_clean(t) return (tostring(t or ""):upper():gsub("[^A-Z0-9 ./%-]", ""):gsub("^%s+", ""):gsub("%s+$", "")) end
-- text of the request as shown (DEPARTURE: Hoppie PDC telex format)
local function compose(s)
    local v = s.val
    if s.sel == "FREETEXT MSG" then return v.MESSAGE or "" end
    if s.sel == "DEPARTURE" then
        local t = "REQUEST PREDEP CLEARANCE " .. M.flight .. " " .. (v["ACFT TYPE"] or "") .. " TO " .. (v.DESTINATION or "") ..
            " AT " .. (v["DEPARTURE ARPT"] or "")
        if nonblank(v.GATE) then t = t .. " STAND " .. v.GATE end
        if nonblank(v["D-ATIS CODE"]) then t = t .. " ATIS " .. v["D-ATIS CODE"] end
        return t
    end
    local r = REQ[s.sel]
    if not r then return "" end
    local t = fill(r[3], s)
    for i, rs in ipairs(REASONS) do if rs[2] and s.reasons[i] then t = t .. "\n" .. rs[1] end end
    if nonblank(extra_clean(s.extra)) then t = t .. "\n" .. extra_clean(s.extra) end
    return t
end
-- CPDLC wire text for the Hoppie client
local function wire(s)
    local r = REQ[s.sel]
    if not r or not r[4] then return nil end
    local t = fill(r[4], s)
    for i, rs in ipairs(REASONS) do if rs[2] and s.reasons[i] then t = t .. "@_@" .. rs[2] end end
    if nonblank(extra_clean(s.extra)) then t = t .. "@_@" .. extra_clean(s.extra) end
    return t
end
local function value_ok(kind, v)
    if kind == "level" then return v:match("^FL%d%d%d$") or v:match("^%d%d%d%d%d? FT$") end
    if kind == "wpt" then return v:match("^[A-Z0-9]+$") and #v >= 2 end
    if kind == "nm" then return v:match("^%d%d?%d?$") and tonumber(v) > 0 end
    if kind == "lr" then return v == "L" or v == "R" end
    if kind == "deg" then return v:match("^%d%d%d$") and tonumber(v) >= 1 and tonumber(v) <= 360 end
    if kind == "spd" then return v:match("^%d%d%d KT$") or v:match("^M%.%d%d$") end
    return true
end
local function valid(s)
    if not s.sel or not REQ[s.sel] or not station(recipient(s)) then return false end
    for _, f in ipairs(fields(s.sel)) do
        local v = s.val[f[1]]
        local optional = s.sel == "DEPARTURE" and (f[1] == "GATE" or f[1] == "D-ATIS CODE")
        if not optional then
            if not nonblank(v) then return false end
            if f[3] == "station" and not station(v) then return false end
            if f[3] == "icao" and not icao(v) then return false end
            if not value_ok(f[3], v) then return false end
        end
    end
    return true
end

-------------------------------------------------------------------------------
-- the datalink (hoppie.lua client, M.dl): requests go to M.dl:request, answers come back through M.dl.out
-------------------------------------------------------------------------------
local function readfile(p)
    local f = io.open(p, "rb")
    if not f then return nil end
    local v = f:read("*a"); f:close()
    return v
end
local function linked() return M.link == "ACARS READY" and M.dl ~= nil end

-- returns the request id or nil
local function handoff(dest, payload, typ)
    if not linked() then info("ACARS NOT AVAILABLE", true); return nil end
    M.txseq = (M.txseq or 0) + 1
    local id = tostring(os.time()) .. "-" .. M.txseq
    local ok = pcall(M.dl.request, M.dl, id, M.flight, dest, payload, typ)
    if not ok then info("DATALINK ERROR", true); return nil end
    return id
end

local function addmail(m)
    local s = M.s
    s.hid = s.hid + 1
    m.id, m.time = s.hid, m.time or utc()
    s.mail[#s.mail + 1] = m
    s.idx, s.reply = #s.mail, ""
end
local function record(m, status)
    -- every record needs its own id (MSG RECORD entries are opened by it)
    if not m.id then M.s.hid = M.s.hid + 1; m.id = M.s.hid end
    table.insert(M.s.records, 1, { id = m.id, time = m.time or utc(), station = m.station, text = m.text, kind = m.kind,
        dir = m.dir, status = status, attr = m.attr })
    M.s.recscroll = 0
end
local function rec_of(m)
    for _, r in ipairs(M.s.records) do if r.id == m.id then return r end end
end
local function mail_by_live(id)
    for i, m in ipairs(M.s.mail) do if m.liveid == id then return i, m end end
end
local function drop_mail(i)
    local s = M.s
    table.remove(s.mail, i)
    s.idx = math.max(1, math.min(s.idx, #s.mail))
    s.reply = ""
end

local function session_lost()
    local s = M.s
    s.active, s.pending, M.cpcontrol = "", "", nil
    for _, m in ipairs(s.mail) do if m.kind == "CPDLC" and m.dir == "UP" then m.archive = true end end
end

local function event(kind, id, status, from, text)
    local s = M.s
    if kind == "CPSTATE" then
        if status == "ACTIVE" then s.active, s.pending = from, ""
        elseif status == "PENDING" then s.pending = from
        else session_lost() end
        info(text, status == "OFF" and text ~= "LOGOFF SENT - SERVER ACK")
    elseif kind == "CPRX" then
        local m = { station = from, text = text, kind = "CPDLC", dir = "UP", cpkey = id, attr = status, archive = status == "READ ONLY" }
        -- answerable without a session = departure clearance (DCL) from the station asked by PDC
        m.dcl = not m.archive and from ~= s.active
        addmail(m)
        record(m, m.archive and "RECEIVED" or "OPEN")
        info("ATC MSG")
        M.chime_seq = (M.chime_seq or 0) + 1
    elseif kind == "CPTX" then
        local ctl = M.cpcontrol
        if ctl and ctl.id == id then
            if status ~= "QUEUED" then M.cpcontrol = nil end
            if status ~= "QUEUED" then record({ station = from, text = text, kind = "CPDLC", dir = "DOWN" }, status) end
            info(status == "SERVER ACK" and (text == "REQUEST LOGON" and "NOTIFICATION SENT" or "MESSAGE SENT") or ("CPDLC " .. status), status ~= "SERVER ACK" and status ~= "QUEUED")
            return
        end
        local i, m = mail_by_live(id)
        if not m then return end
        if status == "QUEUED" then m.txstate = "SENDING"
        elseif status == "SERVER ACK" then
            if m.dir == "DOWN" then
                record(m, "SENT"); m.liveid, m.txstate, m.sent_at = nil, "SENT", os.time(); info("MESSAGE SENT")
            else
                local r = rec_of(m)
                if r then r.reply = text; r.status = (text == "STANDBY") and "STANDBY" or "CLOSED" end
                record({ station = from, text = text, kind = "CPDLC", dir = "DOWN" }, "SENT")
                if text == "STANDBY" then m.liveid, m.txstate, m.standby = nil, nil, true
                else m.liveid, m.txstate, m.sent_at = nil, "SENT", os.time() end
                info(text .. " SENT")
            end
        elseif status == "UNCONFIRMED" then
            m.txstate = "UNCONFIRMED"
            if m.dir == "UP" then m.archive = true end
            info("SEND NOT CONFIRMED - CHECK", true)
        else
            m.liveid, m.txstate = nil, nil
            info("CPDLC " .. status, true)
        end
    elseif kind == "RX" or kind == "INFO" then
        local m = { station = from, text = text, kind = kind == "RX" and "TELEX" or "INFO", dir = "UP" }
        addmail(m)
        record(m, "RECEIVED")
        info(kind == "RX" and "ACARS MSG" or "INFO RECEIVED")
        if kind == "RX" then M.chime_seq = (M.chime_seq or 0) + 1 end
    elseif kind == "WX" then
        local w = M.wxactive
        if not (w and w.id == id) then return end
        local slot = s.atis[w.slot]
        local same = slot and slot.code == w.code and slot.mode == w.mode
        if status == "QUEUED" or status == "PENDING" then return end
        if same then
            slot.pending, slot.checked = nil, os.time()
            local function norm(t) return (tostring(t or ""):gsub("%s+", " "):gsub("^ ", ""):gsub(" $", "")) end
            if (status == "ATIS" or status == "METAR") and w.auto and norm(text) == norm(slot.text) then
                -- AUTO UPDATE check: nothing new - the page stays as it is
            elseif status == "ATIS" or status == "METAR" then
                if w.auto then info("NEW D-ATIS " .. slot.code) end
                slot.text, slot.time, slot.source, slot.offset, slot.at = text, utc_colon(), status, 0, os.time()
                record({ station = slot.code, text = text, kind = "INFO", dir = "UP" }, status)
            elseif not w.auto then
                slot.status = status
                info(slot.code .. " " .. status, true)
            end
        end
        M.wxactive = nil
    elseif kind == "TX" then
        local i, m = mail_by_live(id)
        if status == "QUEUED" then if m then m.txstate = "SENDING" end; return end
        if status == "SERVER ACK" or status == "UNCONFIRMED" then
            if m then
                record(m, status == "SERVER ACK" and "SENT" or "UNCONFIRMED")
                if status == "SERVER ACK" then m.liveid, m.txstate, m.sent_at = nil, "SENT", os.time() else drop_mail(i) end
            end
            info(status == "SERVER ACK" and "MESSAGE SENT" or "SEND NOT CONFIRMED - CHECK", status ~= "SERVER ACK")
        else
            if m then m.liveid, m.txstate = nil, nil end
            info("ACARS " .. status, true)
        end
    elseif kind == "STATIONS" then
        if status == "OK" then
            -- "CODE*" = staffed on the selected network (VATSIM or IVAO): those first, each group alphabetical
            local list, vat = {}, {}
            for cs in text:gmatch("%S+") do
                local code, star = cs:match("^([^*]+)(%*?)$")
                list[#list + 1] = code
                if star == "*" then vat[code] = true end
            end
            table.sort(list, function(x, y) if (vat[x] or false) ~= (vat[y] or false) then return vat[x] == true end return x < y end)
            s.stations, s.st_staffed = list, vat
        else
            s.stations = {}
            info("STATION LIST " .. status, true)
        end
    end
    M.dirty = true
end

-- network choice (VATSIM / IVAO) into atccom_network.txt in M.dir (the A350_ATCCOM folder, set by A350_BTV.lua)
function M.save_net()
    if not M.dir then return end
    local f = io.open(M.dir .. "atccom_network.txt", "wb")
    if f then f:write(M.s.net, "\n"); f:close() end
end

-- about once a second: L = the Hoppie client (nil = no datalink), flight = FMS flight number
function M.link_tick(L, flight)
    if L ~= M.dl then
        -- datalink gone or restarted: requests in flight are uncertain, a CPDLC session is gone
        if M.dl then
            for _, m in ipairs(M.s.mail) do if m.liveid then m.txstate = "UNCONFIRMED" end end
            session_lost()
        end
        M.dl, M.wxactive, M.wxqueue, M.dirty = L, nil, {}, true
        for _, w in ipairs(M.s.atis) do w.pending = nil end
    end
    local id = type(flight) == "string" and flight or ""
    if not id:match("^[A-Z0-9]+$") or #id > 7 then id = "" end
    local status
    if not L or L.logon == "" or L.bad_logon then status = "ACARS UNAVAILABLE"
    elseif #id < 3 then status = "ENTER FLIGHT ID"
    elseif L.callsign ~= id then status = "ACARS WAITING"
    elseif L.ready then status = "ACARS READY"
    else status = "ACARS CONNECTING" end
    if M.flight ~= id or M.link ~= status then M.flight, M.link, M.dirty = id, status, true end
    if not L then return end
    -- the saved network choice (a choice made before the datalink was there wins, and is saved now)
    if not M.net_loaded and M.dir then
        M.net_loaded = true
        if M.net_chosen then M.save_net()
        else
            local n = (readfile(M.dir .. "atccom_network.txt") or ""):match("%u+")
            if n == "IVAO" or n == "VATSIM" then M.s.net = n end
        end
    end
    -- the client's answers, in order (taken first: an answer may lead to a new request)
    local q = L.out
    if #q > 0 then
        L.out = {}
        for _, e in ipairs(q) do event(e[1], e[2], e[3], e[4], e[5]) end
    end
    M.weather_tick()
end

-------------------------------------------------------------------------------
-- status from the aircraft and the datalink
-------------------------------------------------------------------------------
function M.route(from, to, alt)
    local s = M.s
    local a = { from, to, alt }
    for i = 1, 3 do
        local v = a[i]
        if v ~= nil then
            v = icao(v) and v or ""
            if s.route[i] ~= v then s.route[i] = v; M.dirty = true end
        end
    end
    -- D-ATIS slots follow the flight plan until the pilot changes them: DEP / ARR / ALTN ARR
    local auto = { { s.route[1], "DEP" }, { s.route[2], "ARR" }, { s.route[3], "ARR" } }
    for i, w in ipairs(s.atis) do
        local c = auto[i][1] or ""
        if not w.edited and w.code ~= c then w.code, w.mode, w.text, w.time, w.source, w.status = c, auto[i][2], nil, nil, nil, nil; M.dirty = true end
    end
end

-------------------------------------------------------------------------------
-- sending
-------------------------------------------------------------------------------
local function busy_tx()
    for _, m in ipairs(M.s.mail) do if m.liveid and m.txstate ~= "UNCONFIRMED" then return true end end
    return M.cpcontrol ~= nil or M.wxactive ~= nil
end

local function send_mail(m, typ, payload)
    if busy_tx() then info("DATALINK BUSY", true); return end
    payload = payload or m.text
    if not station(m.station) or not nonblank(payload) or #payload > 220 or payload:find("[{}%c]") then info("INVALID MESSAGE", true); return end
    local id = handoff(m.station, payload, typ)
    if id then m.liveid, m.txstate = id, "SENDING"; info("SENDING") end
end

local function cp_control(dest, text, typ)
    if busy_tx() then info("DATALINK BUSY", true); return end
    local id = handoff(dest, text, typ)
    if id then M.cpcontrol = { id = id, typ = typ }; info(typ == "cp_logon" and "NOTIFYING " .. dest or "SENDING") end
end

-- auto = AUTO UPDATE check in the background (the slot keeps showing its report; it only changes when the ATIS did)
local function weather(i, auto)
    local s = M.s
    local w = s.atis[i]
    if not icao(w.code) then return end
    -- keys stay active; without the datalink the reason is shown on the top line (e.g. no FLT NBR on FMS INIT)
    if not linked() then
        if not auto then info(M.link == "ENTER FLIGHT ID" and "NO FLT NBR - ENTER IT ON FMS INIT" or (M.link ~= "ACARS READY" and M.link or "ACARS NOT AVAILABLE"), true) end
        return
    end
    M.wxqueue = M.wxqueue or {}
    if M.wxactive and M.wxactive.slot == i then return end
    for _, v in ipairs(M.wxqueue) do if v.slot == i then return end end
    M.wxqueue[#M.wxqueue + 1] = { slot = i, code = w.code, mode = w.mode, auto = auto }
    if not auto then w.pending, w.status = true, nil end
    M.dirty = true
end

function M.weather_tick()
    local s = M.s
    -- a sent message stays in the mailbox (green, SENT) for a moment, then the mailbox empties
    for i = #s.mail, 1, -1 do
        local m = s.mail[i]
        if m.txstate == "SENT" and os.time() - (m.sent_at or 0) >= 4 then drop_mail(i); M.dirty = true end
    end
    -- AUTO UPDATE: look for a new ATIS every 10 minutes after the last report or check
    for i, w in ipairs(s.atis) do
        if w.auto and w.text and not w.pending and os.time() - (w.checked or w.at or 0) > 600 then
            w.checked = os.time()
            weather(i, true)
        end
    end
    if busy_tx() or not M.wxqueue or #M.wxqueue == 0 or not linked() then return end
    local w = table.remove(M.wxqueue, 1)
    local slot = s.atis[w.slot]
    if not slot or slot.code ~= w.code or slot.mode ~= w.mode then if slot then slot.pending = nil end; return end
    local id = handoff(w.code, slot.mode .. ":" .. s.net, "weather")
    if id then w.id = id; M.wxactive = w else slot.pending = nil end
    M.dirty = true
end

-------------------------------------------------------------------------------
-- drawing
-------------------------------------------------------------------------------
local function canvas(which)
    local ops, hits = {}, {}
    M.hits[which] = hits
    local hov = M.hov and M.hov.which == which and M.hov or nil
    local o = {}
    function o.rect(x, y, w, h, c) if w > 0 and h > 0 then ops[#ops + 1] = { "rect", x, y, w, h, c } end end
    function o.tri(x1, y1, x2, y2, x3, y3, c) ops[#ops + 1] = { "tri", x1, y1, x2, y2, x3, y3, c or C.white } end
    function o.border(x, y, w, h, c, t)
        t = t or 1.4
        o.rect(x, y, w, t, c); o.rect(x, y + h - t, w, t, c); o.rect(x, y, t, h, c); o.rect(x + w - t, y, t, h, c)
    end
    function o.text(x, y, t, c, size, max)
        t = tostring(t or ""):upper():gsub("[\r\n]", " ")
        size = size or F10
        if max then
            local p = math.floor(size * 0.7229 + 0.5)
            local n = math.floor(max * 0.7426 / (ADV[p] or 9))
            if #t > n then t = t:sub(1, math.max(0, n)) end
        end
        ops[#ops + 1] = { "text", x, y, t, c or C.white, size }
    end
    -- text block centred / right aligned (size given in pixels: 9..12)
    function o.ctext(cx, y, t, c, p) o.text(cx - tw(t, p) / 2, y, t, c, px(p)) end
    function o.rtext(rx, y, t, c, p) o.text(rx - tw(t, p), y, t, c, px(p)) end
    function o.hit(x, y, w, h, act) hits[#hits + 1] = { x, y, w, h, act } end
    function o.hovered(x, y, w, h) return hov and hov.x >= x and hov.x <= x + w and hov.y >= y and hov.y <= y + h end
    -- A350 key in FF's style: on = active, sel = white inner box, mark = cyan frame (selected option)
    function o.button(x, y, w, h, label, act, on, sel, p, mark)
        p = p or 10
        o.rect(x, y, w, h, on == false and C.btn_off or C.btn)
        if on ~= false then
            -- FF's keys: flat fill, even light border all round (no bevel)
            o.border(x, y, w, h, C.kframe, KB)
        else
            o.border(x, y, w, h, C.lo, 1.5)
        end
        local lines = {}
        for ln in (label .. "\n"):gmatch("(.-)\n") do lines[#lines + 1] = ln end
        -- centred on the visible capitals (the font draws them in the middle of a 20 unit line box; drawn they are
        -- about 8 % taller than px() says). Several lines in a short key: one size smaller when they would leave less
        -- than 6 units above and below, and a smaller line gap, so the white box fits between text and edge.
        if #lines > 1 and p > 8 and h - (#lines * px(p) * 1.08 + (#lines - 1) * 3) < 12 then p = p - 1 end
        local cap = px(p)
        local lh = cap + 8
        if #lines > 1 then lh = math.max(cap * 1.08 + 2, math.min(lh, (h - 12 - cap * 1.08) / (#lines - 1))) end
        local total = cap * (#lines > 1 and 1.08 or 1) + (#lines - 1) * lh
        local yv = y + (h - total) / 2
        local col = on == false and C.dim or C.white
        for i, ln in ipairs(lines) do o.ctext(x + w / 2, yv - (20 - cap) / 2 + (i - 1) * lh, ln, col, p) end
        if sel then
            -- white box around the label, always inside the key: halfway between the text and the key's edge
            local bw = 0
            for _, ln in ipairs(lines) do bw = math.max(bw, tw(ln, p)) end
            local padx = math.max(1.5, math.min(6, (w - bw) / 2 - 2))
            local pady = math.max(1.5, math.min(5, (h - total) / 2 - 2))
            local bx0, bx1 = x + w / 2 - bw / 2 - padx, x + w / 2 + bw / 2 + padx
            local by0, by1 = yv - pady, yv + total + pady
            o.border(bx0, by0, bx1 - bx0, by1 - by0, C.white, 1.3)
        end
        if (mark or (on ~= false and act and o.hovered(x, y, w, h))) then o.border(x - 1, y - 1, w + 2, h + 2, C.cyan, 2) end
        if on ~= false and act then o.hit(x, y, w, h, act) end
    end
    -- dropdown list entry like FF's: grey list, white text from the left, cyan frame under the mouse
    function o.item(x, y, w, h, label, act, on, mark)
        o.rect(x, y, w, h, C.list)
        o.text(x + 8, y + h / 2 - 10, label, on == false and C.dim or C.white, F10, w - 12)
        if mark then o.border(x + 3, y + 3, w - 6, h - 6, C.white, 1.3) end
        if on ~= false and act and o.hovered(x, y, w, h) then o.border(x + 2, y + 1, w - 4, h - 2, C.cyan, 2) end
        if on ~= false and act then o.hit(x, y, w, h, act) end
    end
    -- data field: cyan value, amber boxes when empty, cyan frame while typing
    function o.field(x, y, w, h, key, t, limit, kind, p, dashes)
        p = p or 11
        local focus = M.focus and M.focus.key == key
        o.rect(x, y, w, h, C.field)
        o.rect(x, y + h - 1.5, w, 1.5, C.grey); o.rect(x, y, 1.5, h, C.grey)
        if focus then o.border(x - 1, y - 1, w + 2, h + 2, C.cyan, 2) end
        if nonblank(t) or focus then
            o.text(x + 7, y + (h - 20) / 2, (t or "") .. (focus and "_" or ""), C.cyan, px(p), w - 10)
        elseif dashes then
            o.text(x + 7, y + (h - 20) / 2, string.rep("-", limit), C.cyan, px(p), w - 10)
        else
            local n = math.min(limit, 4)
            local bw, gap = 11, 4
            local x0 = x + 7
            for j = 0, n - 1 do o.border(x0 + j * (bw + gap), y + (h - 16) / 2, bw, 16, C.amber, 1.4) end
        end
        o.hit(x, y, w, h, { key = key, limit = limit, kind = kind })
    end
    -- small "v" key
    function o.arrow_down(x, y, w, h, act, on)
        o.button(x, y, w, h, "", act, on)
        local cx, cy = x + w / 2, y + h / 2
        o.tri(cx - 6, cy - 3, cx + 6, cy - 3, cx, cy + 4, on == false and C.dim or C.white)
    end
    o.ops = ops
    return o
end

local function radio(a, x, y, label, on, act)
    local cx, cy, r = x + 8, y + 10, 7.5
    local ri = on and 0 or r - 1.8
    local c = on and C.cyan or C.grey
    for j = 0, 23 do
        local q, w = j * math.pi / 12, (j + 1) * math.pi / 12
        local ax, ay, bx, by = cx + r * math.cos(q), cy + r * math.sin(q), cx + r * math.cos(w), cy + r * math.sin(w)
        local ix, iy, jx, jy = cx + ri * math.cos(q), cy + ri * math.sin(q), cx + ri * math.cos(w), cy + ri * math.sin(w)
        a.tri(ax, ay, bx, by, ix, iy, c)
        if ri > 0 then a.tri(bx, by, jx, jy, ix, iy, c) end
    end
    a.text(x + 21, y, label, on and C.cyan or C.white, F10)
    if act then a.hit(x, y, 30 + tw(label, 10), 20, act) end
end

local function envelope(a, x, y)
    a.border(x, y, 26, 18, C.white, 1.4)
    for j = 0, 12 do
        a.rect(x + 1 + j, y + 1 + j * 0.62, 1.2, 1.2, C.white)
        a.rect(x + 24 - j, y + 1 + j * 0.62, 1.2, 1.2, C.white)
    end
end

-- word wrap to n characters per line
local function wrap(t, n)
    local out = {}
    for para in (tostring(t or "") .. "\n"):gmatch("(.-)\n") do
        local line = ""
        for word in para:gmatch("%S+") do
            while #word > n do
                if line ~= "" then out[#out + 1] = line; line = "" end
                out[#out + 1] = word:sub(1, n); word = word:sub(n + 1)
            end
            if line == "" then line = word elseif #line + 1 + #word <= n then line = line .. " " .. word
            else out[#out + 1] = line; line = word end
        end
        out[#out + 1] = line
    end
    while #out > 1 and out[#out] == "" do out[#out] = nil end
    return out
end
local function chars(w, p) return math.floor(w * 0.7426 / (ADV[p + BUMP] or 10)) end

local function frame_page(a, s, page)
    local R, T = a.rect, a.text
    -- (ends 9 units short of the page's right edge: FF's divider line to the checklist half stays visible)
    R(2, 32, 668, 852, C.bg)
    R(190, 0, 480, 32, C.bg)
    -- top line: info / warning next to the native "ATC COM v" key, flight number on the right
    local msg, warn = s.info, s.info_warn
    if not nonblank(msg) and M.link ~= "ACARS READY" then msg, warn = M.link, true end
    if nonblank(msg) then T(198, 6, msg, warn and C.amber or C.white, F10, 370) end
    if M.flight ~= "" then a.rtext(660, 5, M.flight, C.white, 11) end
    -- tabs: same row and height as FF's ACTIVE / POSITION / SEC INDEX / DATA keys, edge to edge like FF's (one shared
    -- border between two tabs, no gap)
    local x0, gap, ty, th = 23, -KB, 36, 41
    local widths = { 104, 104, 130, 100, 100, 104 }
    local x = x0
    for i, t in ipairs(TABS) do
        local w = widths[i]
        if t[1] == 10 then
            R(x, ty, w, th, C.emer); a.border(x, ty, w, th, C.kframe, KB)
            a.ctext(x + w / 2, ty + (th - px(9)) / 2 - (20 - px(9)) / 2, "EMER", C.emer_text, 9)
            if page == 10 then
                local bx0, bx1 = math.max(x + 3, x + w / 2 - tw("EMER", 9) / 2 - 6), math.min(x + w - 3, x + w / 2 + tw("EMER", 9) / 2 + 6)
                local by0, by1 = math.max(ty + 3, ty + (th - px(9)) / 2 - 5), math.min(ty + th - 3, ty + (th + px(9)) / 2 + 5)
                a.border(bx0, by0, bx1 - bx0, by1 - by0, C.white, 1.3)
            end
            if a.hovered(x, ty, w, th) then a.border(x - 1, ty - 1, w + 2, th + 2, C.cyan, 2) end
            a.hit(x, ty, w, th, "page:10")
        else
            a.button(x, ty, w, th, t[2], "page:" .. t[1], true, page == t[1], 9)
        end
        x = x + w + gap
    end
    -- title band (FF: 80 .. 97.5, grey text)
    R(23, 80, 634, 17.5, C.band)
    local title = TITLES[page] or ""
    if page == 8 and s.recview then
        title = title .. (s.recview == "mon" and "/MONITORED MSG" or "/ALL MSG") .. (s.expand and "/EXPAND" or "")
    end
    T(29, 80 + 17.5 / 2 - 10, title, C.band_text, F9)
    -- FF's line above the bottom keys
    R(21, 811, 634, 1.5, C.line)
    -- bottom left (FF's CLEAR INFO place): CLEAR INFO while a message is shown, else MSG LIST
    if nonblank(s.info) then a.button(9, 822, 86, 58, "CLEAR\nINFO", "clear", true, false, 10)
    else a.button(9, 822, 86, 58, "MSG\nLIST", "page:11", true, page == 11, 10) end
end

-------------------------------------------------------------------------------
-- pages
-------------------------------------------------------------------------------
local function page_connect(a, s)
    local R, T = a.rect, a.text
    a.rtext(262, 136, "NOTIFY TO ATC:", C.white, 10)
    local can = linked()
    -- station field with its list
    a.field(272, 124, 128, 32, "center", s.center, 7, "station", 11, true)
    a.arrow_down(402, 124, 28, 32, "stations", can)
    a.button(486, 122, 112, 34, "NOTIFY", "notify", station(s.center) and s.active == "" and s.pending == "" and not M.cpcontrol and can, false, 10)
    radio(a, 32, 168, "VATSIM", s.net == "VATSIM", "net:VATSIM")
    radio(a, 160, 168, "IVAO", s.net == "IVAO", "net:IVAO")
    a.rtext(262, 244, "ACTIVE ATC:", C.white, 10)
    if s.active ~= "" then T(276, 241, s.active, C.green, F11) end
    a.rtext(262, 294, "NEXT ATC:", C.white, 10)
    if s.pending ~= "" then T(276, 291, s.pending, C.cyan, F11) end
    a.button(440, 238, 214, 34, "DISCONNECT ALL", "disconnect", (s.active ~= "" or s.pending ~= "") and not M.cpcontrol and can, false, 10)
    a.button(446, 290, 208, 54, "NOTIFY\nMAX UPLINK DELAY", nil, false, false, 9)
    R(48, 384, 618, 1.5, C.line)
    -- ADS-C (not available in this simulation: shown OFF)
    a.ctext(247, 412, "ADS-C", C.white, 10)
    a.border(162, 432, 170, 66, C.frame, 1.5)
    a.ctext(247, 442, "CONNECTED", C.dim, 12)
    a.ctext(247, 472, "OFF", C.white, 10)
    a.ctext(515, 406, "ADS-C EMERGENCY", C.white, 10)
    a.border(430, 426, 170, 66, C.frame, 1.5)
    a.ctext(515, 436, "ON", C.dim, 10)
    a.ctext(515, 466, "OFF", C.white, 10)
    -- notification history
    a.border(256, 552, 318, 250, C.line, 1.4)
    local n = #s.notified_list
    for i = math.max(1, n - 8), n do T(270, 560 + (i - math.max(1, n - 8)) * 26, s.notified_list[i], C.green, F10, 290) end
    -- station list (open)
    if s.dropdown == "stations" then
        local list = s.stations
        local x, y, w = 272, 158, (s.st_staffed and next(s.st_staffed)) and 200 or 158
        if not list then
            R(x, y, w, 30, C.list); a.border(x, y, w, 30, C.lo, 1.3); T(x + 8, y + 5, "LOADING...", C.white, F10)
        elseif #list == 0 then
            R(x, y, w, 30, C.list); a.border(x, y, w, 30, C.lo, 1.3); T(x + 8, y + 5, "NO STATION", C.white, F10)
        else
            local rows = math.min(#list, 12)
            local off = math.max(0, math.min(s.st_off or 0, #list - rows))
            s.st_off = off
            R(x, y, w, rows * 27 + 6, C.list); a.border(x, y, w, rows * 27 + 6, C.lo, 1.3)
            for j = 1, rows do
                local cs = list[off + j]
                local yy = y + 3 + (j - 1) * 27
                if a.hovered(x, yy, w, 27) then a.border(x + 3, yy, w - 6, 27, C.cyan, 1.6) end
                T(x + 10, yy + 3, (s.st_staffed or {})[cs] and (cs .. " (" .. (s.st_net or s.net) .. ")") or cs, C.white, F10)
                a.hit(x, yy, w, 27, "pick:" .. cs)
            end
            if #list > rows then
                a.button(x + w + 2, y, 26, 26, "", "stup", off > 0); a.tri(x + w + 9, y + 18, x + w + 21, y + 18, x + w + 15, y + 8, C.white)
                a.button(x + w + 2, y + rows * 27 - 20, 26, 26, "", "stdn", off + rows < #list)
                a.tri(x + w + 9, y + rows * 27 - 12, x + w + 21, y + rows * 27 - 12, x + w + 15, y + rows * 27 - 2, C.white)
            end
        end
    end
end

local function page_request(a, s)
    local R, T = a.rect, a.text
    -- category column
    local ys = { 116, 150, 184, 218, 252, 302 }
    for i, c in ipairs(CATS) do
        local h = (i == 5) and 44 or 30
        local en = #MENUS[i] > 0
        local sel = s.cat == i or (s.group == i and s.sel ~= nil)
        a.button(500, ys[i], 160, h, "", en and ("cat:" .. i) or nil, en, false, 10, s.cat == i)
        local col = en and (sel and C.white or C.white) or C.dim
        local cy = ys[i] + h / 2
        a.tri(510, cy, 519, cy - 5, 519, cy + 5, col)
        local lines = {}
        for ln in (c .. "\n"):gmatch("(.-)\n") do lines[#lines + 1] = ln end
        for j, ln in ipairs(lines) do T(528, ys[i] + (h - #lines * 19) / 2 - 1 + (j - 1) * 19, ln, col, F10) end
    end
    local addt = s.sel ~= nil and kind_of(s.sel) == "CPDLC"
    a.button(526, 366, 134, 30, "", addt and "cat:7" or nil, addt, false, 10, s.cat == 7)
    a.tri(536, 381, 545, 376, 545, 386, addt and C.white or C.dim)
    T(554, 371, "ADD TEXT", addt and C.white or C.dim, F10)
    -- flyout with the choices of the open category (ADD TEXT: reasons, several can be chosen) - drawn last, on top
    local function flyout()
        if not s.cat then return end
        local items, y0, right = {}, (s.cat == 7) and 366 or ys[s.cat], (s.cat == 7) and 526 or 500
        if s.cat == 7 then for _, rs in ipairs(REASONS) do items[#items + 1] = { rs[1] } end else items = MENUS[s.cat] end
        local w = 0
        for _, it in ipairs(items) do w = math.max(w, tw(it[1], 10)) end
        w = math.max(150, w + 28)
        for j, it in ipairs(items) do
            local on = s.cat == 7 and ((REASONS[j][2] and s.reasons[j]) or (not REASONS[j][2] and s.extra ~= nil))
            a.button(right - w - 2, y0 + (j - 1) * 32, w, 30, it[1], "item:" .. j, true, false, 10, on)
        end
    end
    if not s.sel then flyout(); return end
    -- request frame
    local fx, fy, fw, fh = 30, 128, 455, 584
    a.border(fx, fy, fw, fh, C.frame, 1.5)
    local head = req_title(s.sel)
    R(fx + fw / 2 - tw(head, 10) / 2 - 10, fy - 6, tw(head, 10) + 20, 12, C.bg)
    a.ctext(fx + fw / 2, fy - 12, head, C.white, 10)
    -- trash key on the frame
    R(fx + fw - 74, fy - 18, 46, 1.5, C.frame)
    a.button(fx + fw - 28, fy - 30, 26, 26, "", "trash", true)
    R(fx + fw - 21, fy - 23, 12, 2, C.white); a.border(fx + fw - 20, fy - 20, 10, 12, C.white, 1.3)
    local y = fy + 20
    T(fx + 20, y, s.sel == "FREETEXT MSG" and "TELEX TO" or "REQUEST WILL BE SENT TO", C.white, F10)
    y = y + 34
    local fl = fields(s.sel)
    local lx = fx + 220
    if kind_of(s.sel) == "CPDLC" then
        T(lx, y, s.active ~= "" and s.active or "----", s.active ~= "" and C.cyan or C.amber, F11)
        y = y + 40
    end
    for _, f in ipairs(fl) do
        if f[3] == "text" then
            T(fx + 20, y + 4, f[1], C.white, F10)
            y = y + 30
            local h = 150
            a.field(fx + 20, y, fw - 40, h, "val:" .. f[1], "", f[2], "text", 10)
            local ls = wrap(s.val[f[1]] or "", chars(fw - 60, 10))
            if M.focus and M.focus.key == "val:" .. f[1] then ls[#ls] = (ls[#ls] or "") .. "_" end
            for j = 1, math.min(#ls, 6) do T(fx + 28, y + 4 + (j - 1) * 24, ls[j], C.cyan, F10) end
            y = y + h + 12
        else
            a.rtext(lx - 12, y + 7, f[1], C.white, 10)
            local fwid = math.max(78, math.min(150, f[2] * 15 + 24))
            a.field(lx, y, fwid, 30, "val:" .. f[1], s.val[f[1]], f[2], f[3], 11)
            y = y + 40
        end
    end
    -- ADD TEXT free text (reasons appear in the preview)
    if s.extra ~= nil then
        a.rtext(lx - 12, y + 7, "FREE TEXT", C.white, 10)
        a.field(lx, y, fw - (lx - fx) - 20, 30, "extra", s.extra, 60, "text", 10)
        y = y + 40
    end
    y = y + 10
    -- message preview with scroll keys
    if s.sel ~= "FREETEXT MSG" then
        local by, bh = math.max(y + 6, 430), fy + fh - 52 - math.max(y + 6, 430)
        a.border(fx + 20, by, fw - 40, bh, C.line, 1.4)
        local ls = wrap(compose(s), chars(fw - 110, 10))
        local rows = math.floor((bh - 10) / 24)
        s.scroll = math.max(0, math.min(s.scroll or 0, math.max(0, #ls - rows)))
        for j = 1, rows do if ls[s.scroll + j] then T(fx + 30, by + 6 + (j - 1) * 24, ls[s.scroll + j], C.white, F10) end end
        a.button(fx + fw - 56, by + 6, 28, 28, "", "pvup", s.scroll > 0)
        a.tri(fx + fw - 48, by + 25, fx + fw - 36, by + 25, fx + fw - 42, by + 14, s.scroll > 0 and C.white or C.dim)
        a.button(fx + fw - 56, by + 38, 28, 28, "", "pvdn", s.scroll + rows < #ls)
        a.tri(fx + fw - 48, by + 47, fx + fw - 36, by + 47, fx + fw - 42, by + 58, s.scroll + rows < #ls and C.white or C.dim)
    end
    local note = kind_of(s.sel) == "TELEX" and "(NO NOTIFICATION REQUIRED)" or (s.active == "" and "(NO ACTIVE ATC)" or "(CPDLC)")
    a.ctext(fx + fw / 2, fy + fh - 36, note, C.white, 10)
    a.button(64, 752, 162, 54, "CANCEL", "trash", true, false, 10)
    a.button(498, 752, 162, 54, "XFR\nTO MAILBOX", "transfer", valid(s), false, 10)
    flyout()
end

local function page_datis(a, s)
    local R, T = a.rect, a.text
    for i, w in ipairs(s.atis) do
        local y = 116 + (i - 1) * 216
        a.field(30, y, 78, 32, "icao:" .. i, w.code, 4, "icao", 11)
        R(110, y, 84, 32, C.field); R(110, y + 30.5, 84, 1.5, C.grey)
        T(116, y + 6, w.mode, C.cyan, F11)
        a.arrow_down(168, y + 4, 22, 24, "mode:" .. i, true)
        if w.time then
            T(230, y + 8, w.time, C.white, F9)
            envelope(a, 312, y + 6)
        end
        -- right keys
        if w.pending then
            a.button(526, y - 4, 134, 46, "PENDING...\nREQUEST", nil, false, false, 9)
            a.button(526, y + 44, 134, 40, "AUTO\nUPDATE", "auto:" .. i, true, false, 9, w.auto)
        elseif w.text then
            a.button(526, y - 4, 134, 46, "UPDATE\nOR PRINT", "upd:" .. i, icao(w.code), false, 9, s.upd_menu == i)
            a.tri(640, y + 9, 650, y + 9, 645, y + 15, C.white)
        else
            a.button(526, y - 4, 134, 46, "SEND\nREQUEST", "weather:" .. i, icao(w.code), false, 9)
            a.button(526, y + 44, 134, 40, "AUTO\nUPDATE", "auto:" .. i, true, false, 9, w.auto)
        end
        if w.status then T(30, y + 44, w.status, C.amber, F10) end
        if w.text then
            local ls = wrap(w.text, chars(480, 10))
            local off = w.offset or 0
            for j = 1, 5 do if ls[off + j] then T(30, y + 44 + (j - 1) * 25, ls[off + j], C.white, F10) end end
            if #ls > off + 5 then a.button(452, y + 156, 56, 28, ">>>", "wxnext:" .. i, true, false, 10)
            elseif off > 0 then a.button(452, y + 156, 56, 28, "<<<", "wxtop:" .. i, true, false, 10) end
        end
        if i < 3 then R(30, y + 196, 630, 1.5, C.line) end
        -- mode menu
        if s.weather_menu == i then
            for j, m in ipairs({ "DEP", "ARR" }) do a.item(110, y + 32 + (j - 1) * 30, 84, 30, m, "setmode:" .. m, true, w.mode == m) end
        end
        if s.upd_menu == i then
            a.item(526, y + 42, 134, 30, "UPDATE", "weather:" .. i, true)
            a.item(526, y + 72, 134, 30, "PRINT", nil, false)
        end
    end
    R(30, 744, 630, 1.5, C.line)
    a.button(334, 752, 162, 54, "PRINT\nALL", nil, false, false, 10)
    a.button(498, 752, 162, 54, "UPDATE\nALL", "weatherall", true, false, 10)
end

local function page_records(a, s, only_open, monitored)
    local R, T = a.rect, a.text
    local list = {}
    for _, r in ipairs(s.records) do
        if monitored then if r.monitored then list[#list + 1] = r end
        elseif not only_open or r.status == "OPEN" or r.status == "STANDBY" then list[#list + 1] = r end
    end
    local rowh, rows = 92, 7
    s.recscroll = math.max(0, math.min(s.recscroll, math.max(0, #list - rows)))
    if #list == 0 then a.ctext(320, 300, only_open and "NO OPEN MESSAGE" or "NO MESSAGE", C.white, 10) end
    for j = 1, rows do
        local r = list[s.recscroll + j]
        if not r then break end
        local y = 116 + (j - 1) * rowh
        T(34, y + 4, r.time .. "   " .. (r.dir == "UP" and "FROM " or "TO ") .. (r.station or ""), C.green, F9)
        local ls = wrap(r.text, chars(520, 11))
        -- longer than two lines: narrower lines, so the >>> after the second line never covers text
        if #ls > 2 then ls = wrap(r.text, chars(462, 11)) end
        for k = 1, 2 do if ls[k] then T(34, y + 28 + (k - 1) * 26, ls[k], C.green, F11) end end
        -- status tag
        local tag, tc, filled = nil, C.green, false
        if r.reply then tag, filled = ({ WILCO = "ACCEPT", UNABLE = "REJECT", STANDBY = "STBY" })[r.reply] or r.reply, true; if r.reply == "UNABLE" or r.reply == "NEGATIVE" then tc = C.amber end
        elseif r.status == "OPEN" then tag, tc = "OPEN", C.cyan
        elseif r.status == "STANDBY" then tag, tc = "STANDBY", C.cyan
        elseif r.status == "UNCONFIRMED" then tag, tc = "NOT CONFIRMED", C.amber
        elseif r.status == "ATIS" or r.status == "METAR" then tag = r.status end
        if tag then
            local w = tw(tag, 10) + 20
            if filled then R(560 - w, y + 4, w, 22, tc); a.ctext(560 - w / 2, y + 5, tag, C.black, 10)
            else a.border(560 - w, y + 4, w, 22, tc, 1.4); a.ctext(560 - w / 2, y + 5, tag, tc, 10) end
        end
        if #ls > 2 then a.rtext(566, y + 56, ">>>", C.white, 10) end
        a.hit(30, y, 540, rowh - 4, (only_open and "record:" or "expand:") .. r.id)
        R(30, y + rowh - 3, 540, 1.5, C.line)
    end
    -- scroll column
    R(580, 116, 6, 616, C.line)
    local function key(yy, act, on, up, double)
        a.button(594, yy, 34, 30, "", act, on)
        local col = on and C.white or C.dim
        local function tri(dy) if up then a.tri(603, yy + 20 + dy, 619, yy + 20 + dy, 611, yy + 11 + dy, col) else a.tri(603, yy + 10 + dy, 619, yy + 10 + dy, 611, yy + 19 + dy, col) end end
        if double then tri(-4); tri(4) else tri(0) end
    end
    local more = s.recscroll + rows < #list
    key(116, "rectop", s.recscroll > 0, true, true)
    key(150, "recup", s.recscroll > 0, true, false)
    key(184, "recdown", more, false, false)
    key(218, "recend", more, false, true)
    R(30, 744, 630, 1.5, C.line)
    if not only_open and not monitored then a.button(12, 752, 162, 54, "ERASE ALL", "erase", #s.records > 0, false, 10) end
    a.button(498, 752, 162, 54, "PRINT", nil, false, false, 10)
end

-- MSG RECORD: ALL MSG / MONITORED MSG keys; a list; one message expanded with RETURN TO LIST
local function page_record_menu(a, s)
    local R, T = a.rect, a.text
    if not s.recview then
        a.button(52, 140, 140, 40, "ALL MSG", "recview:all", true, false, 9)
        a.button(52, 214, 170, 40, "MONITORED MSG", "recview:mon", true, false, 9)
        R(30, 292, 630, 1.5, C.line)
        return
    end
    local r
    if s.expand then for _, x in ipairs(s.records) do if x.id == s.expand then r = x end end end
    if not r then s.expand = nil; return page_records(a, s, false, s.recview == "mon") end
    T(34, 120, r.time .. "   " .. (r.dir == "UP" and "FROM " or "TO ") .. (r.station or ""), C.green, F9)
    local ls = wrap(r.text, chars(600, 11))
    for k = 1, math.min(#ls, 22) do T(34, 146 + (k - 1) * 26, ls[k], C.green, F11) end
    R(30, 744, 630, 1.5, C.line)
    a.button(12, 752, 162, 54, "RETURN\nTO LIST", "recback", true, false, 10)
    a.button(498, 752, 162, 54, "PRINT", nil, false, false, 10)
end

local function page_inop(a, s, t)
    a.ctext(340, 300, t, C.white, 10)
end

-- MFD system menu (FF's "ATCCOM v" key at the top left): drawn by us where FF's own list opens, so nothing of FF's
-- page has to show through. FF opens its list on the same click and still makes the choice (A350 order, FF's
-- 1-sim/mfd/guage values); the mod checks afterwards that FF switched.
M.SYSTEMS = { { "FMS 1", 1 }, { "ATCCOM", 2 }, { "SURV", 3 }, { "FCU BKUP", 4 } }
local SYSBOX = { x = 18, y = 31, w = 170, row = 33 }
function M.sysmenu_toggle() M.s.sysmenu = not M.s.sysmenu or nil; M.dirty = true end
function M.sysmenu_open() return M.s.sysmenu == true end
local function sysmenu(a)
    local b = SYSBOX
    a.rect(b.x, b.y, b.w, #M.SYSTEMS * b.row, C.list)
    for i, e in ipairs(M.SYSTEMS) do a.item(b.x, b.y + (i - 1) * b.row, b.w, b.row, e[1], "sys:" .. e[2], true, e[2] == 2) end
    a.border(b.x, b.y, b.w, #M.SYSTEMS * b.row, C.lo, 1.3)
end

-- the system key at the top left (FF draws it on the cockpit display; the mod's ATCCOM popup window has no FF page)
function M.syskey()
    local a = canvas("syskey")
    a.rect(18, 2, 168, 28, C.black)
    a.border(18, 2, 168, 28, C.kframe, 1.3)
    a.text(26, 8, "ATCCOM", C.cyan, F10, 130)
    a.tri(164, 12, 178, 12, 171, 21, C.white)
    return a.ops
end

function M.lower(page)
    local s = M.s
    page = page or s.page
    s.notified_list = s.notified_list or {}
    local a = canvas("lower")
    frame_page(a, s, page)
    -- a fault on one page must never take the ATCCOM page away: show the frame and a note, report the error
    local ok, err = pcall(function()
        if page == 1 then page_connect(a, s)
        elseif page == 3 then page_request(a, s)
        elseif page == 4 then page_inop(a, s, "NOT AVAILABLE")
        elseif page == 8 then page_record_menu(a, s)
        elseif page == 9 then page_datis(a, s)
        elseif page == 10 then page_inop(a, s, "NOT AVAILABLE")
        elseif page == 11 then page_records(a, s, true) end
    end)
    if not ok then
        M.page_error = tostring(err)
        a.ctext(340, 300, "PAGE NOT AVAILABLE - SEE LOG.TXT", C.amber, 10)
    end
    if s.sysmenu then sysmenu(a) end
    return a.ops
end

-- answer keys of a CPDLC uplink: { key label, answer sent } (ACCEPT = WILCO, REJECT = UNABLE, STBY = STANDBY)
local ANSWERS = {
    WU = { { "ACCEPT", "WILCO" }, { "REJECT", "UNABLE" }, { "STBY", "STANDBY" } },
    AN = { { "AFFIRM", "AFFIRM" }, { "NEGATIVE", "NEGATIVE" }, { "STBY", "STANDBY" } },
    R = { { "ROGER", "ROGER" }, { "STBY", "STANDBY" } },
}

-- mailbox at the top of the right half of the upper centre display (above the memo), like the A350. The frame
-- never changes: two side lines from the top of the screen, the message area, a status strip below it (split in two
-- by a short line) and the line above the memo. A message only fills it: green header (time / station), status at
-- the top right in cyan, the text (not sent yet: black on cyan, sending / sent: black on green), two scroll keys,
-- PRINT / RECALL on the left, SEND / CANCEL or the answer keys on the right, SENDING / SENT in the strip.
local MB = { x0 = 112, x1 = 539, strip = 206, base = 232 }
function M.upper()
    local s = M.s
    local a = canvas("upper")
    local R, T = a.rect, a.text
    local x0, x1, sy, by = MB.x0, MB.x1, MB.strip, MB.base
    -- (starts 8 units in: FF's divider line between the two halves of the display stays visible)
    R(8, 0, 671, 236, C.bg)
    -- the fixed frame
    R(x0, 0, 1.5, by, C.frame); R(x1, 0, 1.5, by, C.frame)
    R(x0, sy, x1 - x0 + 1.5, 1.5, C.frame)
    R((x0 + x1) / 2, sy, 1.5, by - sy, C.frame)
    R(30, by, 622, 1.5, C.frame)
    local m = s.mail[s.idx]
    if not m then return a.ops end
    local down = m.dir == "DOWN" and not m.archive
    local sending = m.txstate == "SENDING" or m.txstate == "SENT"
    local unsent = down and not m.liveid and m.txstate ~= "UNCONFIRMED" and m.txstate ~= "SENT"
    -- header and status
    T(x0 + 12, 8, m.time .. (m.dir == "UP" and "  FROM " or "  TO ") .. m.station, C.green, F10, 300)
    local state = ""
    if down then state = (not sending and m.txstate ~= "UNCONFIRMED") and "OPEN" or ""
    elseif m.kind == "CPDLC" and not m.archive then state = m.standby and "STBY" or "OPEN" end
    if state ~= "" then a.rtext(x1 - 6, 8, state, C.cyan, 10) end
    -- text
    local tx, ty, tw_, th = x0 + 7, 38, 353, 126
    local fill = (down and sending) and C.green or (unsent and C.cyan or nil)
    if fill then R(tx, ty, tw_, th, fill) end
    local ls = wrap(m.text, chars(tw_ - 16, 9))
    local rows = 4
    m.scroll = math.max(0, math.min(m.scroll or 0, math.max(0, #ls - rows)))
    for j = 1, rows do
        local ln = ls[m.scroll + j]
        if ln then T(tx + 8, ty + 8 + (j - 1) * 29, ln, fill and C.black or C.white, F9) end
    end
    -- two scroll keys (double arrows) right of the text
    local more_up, more_dn = m.scroll > 0, m.scroll + rows < #ls
    local function key(ky, act, on, up)
        local kx = x0 + 368
        a.button(kx, ky, 52, 38, "", act, on)
        local col = on and C.white or C.dim
        for _, dy in ipairs({ -5, 5 }) do
            if up then a.tri(kx + 16, ky + 24 + dy, kx + 36, ky + 24 + dy, kx + 26, ky + 14 + dy, col)
            else a.tri(kx + 16, ky + 14 + dy, kx + 36, ky + 14 + dy, kx + 26, ky + 24 + dy, col) end
        end
    end
    key(44, "mup", more_up, true)
    key(86, "mdn", more_dn, false)
    -- left of the frame: PRINT, RECALL
    a.button(31, 160, 74, 34, "PRINT", nil, false, false, 9)
    a.button(31, 198, 74, 34, "RECALL", "recall", true, false, 9)
    -- status strip: SENDING / SENT / NOT CONFIRMED on the left, further messages on the right
    local st, sc = nil, C.white
    if m.txstate == "SENDING" then st = "SENDING"
    elseif m.txstate == "SENT" then st = "SENT"
    elseif m.txstate == "UNCONFIRMED" then st, sc = "NOT CONFIRMED", C.amber
    elseif m.recalled then st = "RECALLED" end
    if st then a.ctext((x0 + (x0 + x1) / 2) / 2, sy + 5, st, sc, 9) end
    if #s.mail > 1 then
        local cx = ((x0 + x1) / 2 + x1) / 2
        a.ctext(cx, sy + 5, string.format("MSG %d/%d", s.idx, #s.mail), C.white, 9)
        a.hit((x0 + x1) / 2, sy, (x1 - x0) / 2, by - sy, "next")
    end
    -- right of the frame: what can be done with this message
    local kx, kw = x1 + 10, 124
    if down then
        a.button(kx, 6, kw, 40, "SEND", "send", unsent, false, 9)
        a.button(kx, 50, kw, 40, "CANCEL", "cancelmail", unsent or m.txstate == "UNCONFIRMED", false, 9)
    elseif m.kind == "CPDLC" and not m.archive then
        local en = not m.liveid and m.txstate ~= "SENT" and not M.cpcontrol and (m.station == s.active or m.dcl) and linked()
        local opts = ANSWERS[m.attr] or {}
        for i, o in ipairs(opts) do a.button(kx, 6 + (i - 1) * 44, kw, 40, o[1], "reply:" .. o[2], en, false, 9, s.reply == o[2]) end
        if s.reply ~= "" then a.button(kx, 6 + #opts * 44, kw, 40, "SEND", "sendreply", en, false, 9) end
    else
        a.button(kx, 6, kw, 40, "CLOSE", "ack", true, false, 9)
    end
    return a.ops
end

-------------------------------------------------------------------------------
-- input
-------------------------------------------------------------------------------
local function action(act)
    local s = M.s
    local k, v = act:match("^([^:]+):?(.*)$")
    M.focus = nil
    s.sysmenu = nil
    if k ~= "stations" and k ~= "pick" and k ~= "stup" and k ~= "stdn" then s.dropdown = nil end
    if k ~= "mode" and k ~= "setmode" then s.weather_menu = nil end
    if k ~= "upd" then s.upd_menu = nil end
    if k == "page" then
        local p = tonumber(v)
        if TITLES[p] then s.page, s.cat = p, nil; M.hits.lower = {} end
        if p == 8 then s.recview, s.expand = nil, nil end
    elseif k == "sys" then M.sys_pick = tonumber(v)
    elseif k == "clear" then info("")
    elseif k == "net" then
        -- network for D-ATIS and the staffed stations in the NOTIFY TO ATC list (Hoppie itself serves all networks)
        s.net, s.stations, s.st_staffed = v, nil, nil
        M.net_chosen = true
        M.save_net()
        info("NETWORK " .. v)
    elseif k == "stations" then
        if s.dropdown == "stations" then s.dropdown = nil
        else
            s.dropdown, s.stations, s.st_off = "stations", nil, 0
            s.st_net = s.net
            if not handoff("SERVER", "ALL-CALLSIGNS:" .. s.net, "stations") then s.dropdown = nil end
        end
    elseif k == "stup" then s.st_off = math.max(0, (s.st_off or 0) - 10)
    elseif k == "stdn" then s.st_off = (s.st_off or 0) + 10
    elseif k == "pick" then s.center, s.dropdown = v, nil
    elseif k == "cat" then
        local i = tonumber(v)
        if s.cat == i then s.cat = nil else s.cat = i end
    elseif k == "item" and s.cat == 7 then
        local rs = REASONS[tonumber(v)]
        if rs and rs[2] then s.reasons[tonumber(v)] = not s.reasons[tonumber(v)] or nil
        elseif rs then
            if s.extra == nil then s.extra = ""; M.focus = { key = "extra", limit = 60, kind = "text" } else s.extra = nil end
        end
        s.cat = nil
    elseif k == "item" then
        local it = s.cat and MENUS[s.cat][tonumber(v)]
        it = it and it[2]
        if it then
            s.sel, s.group, s.val, s.scroll, s.reasons, s.extra = it, s.cat, {}, 0, {}, nil
            if it == "DEPARTURE" then
                s.val["DEPARTURE ARPT"] = s.route[1] or ""
                s.val.DESTINATION = s.route[2] or ""
                s.val["ACFT TYPE"] = (type(PLANE_ICAO) == "string" and PLANE_ICAO ~= "") and PLANE_ICAO or "A359"
            end
        end
        s.cat = nil
    elseif k == "trash" then s.sel, s.group, s.val, s.cat, s.reasons, s.extra = nil, nil, {}, nil, {}, nil
    elseif k == "pvup" then s.scroll = math.max(0, (s.scroll or 0) - 1)
    elseif k == "pvdn" then s.scroll = (s.scroll or 0) + 1
    elseif k == "notify" and station(s.center) and s.active == "" and s.pending == "" then
        cp_control(s.center, "REQUEST LOGON", "cp_logon")
        table.insert(s.notified_list, s.center .. "  " .. utc())
    elseif k == "disconnect" then
        local dest = s.active ~= "" and s.active or s.pending
        if dest ~= "" then cp_control(dest, "LOGOFF", "cp_logoff") end
    elseif k == "transfer" and valid(s) then
        addmail({ station = recipient(s), text = compose(s), wire = wire(s), kind = kind_of(s.sel), dir = "DOWN", sel = s.sel })
        s.sel, s.group, s.val, s.reasons, s.extra = nil, nil, {}, {}, nil
        info("XFR TO MAILBOX")
    elseif k == "send" then
        local m = s.mail[s.idx]
        if m and m.dir == "DOWN" then
            if m.kind == "TELEX" then send_mail(m, "telex") else send_mail(m, "cp_request", m.wire) end
        end
    elseif k == "cancelmail" or k == "ack" then
        local m = s.mail[s.idx]
        if m and (not m.liveid or m.txstate == "UNCONFIRMED") then drop_mail(s.idx) end
    elseif k == "reply" then s.reply = v
    elseif k == "sendreply" and s.reply ~= "" then
        local m = s.mail[s.idx]
        if m and m.dir == "UP" and m.kind == "CPDLC" and not m.archive and m.cpkey and (m.station == s.active or m.dcl) then
            send_mail(m, "cp_reply", m.cpkey .. ":" .. s.reply)
            if m.liveid then m.answer = s.reply end
        end
    elseif k == "prev" then s.idx = math.max(1, s.idx - 1); s.reply = ""
    elseif k == "next" then s.idx = (s.idx % math.max(1, #s.mail)) + 1; s.reply = ""
    elseif k == "mtop" or k == "mup" or k == "mdn" or k == "mend" then
        local m = s.mail[s.idx]
        if m then
            local d = ({ mtop = -1000, mup = -1, mdn = 1, mend = 1000 })[k]
            m.scroll = math.max(0, (m.scroll or 0) + d)
        end
    elseif k == "record" or k == "recall" then
        -- open a message in the mailbox: a record (MSG RECORD), or RECALL = the last one no longer in the mailbox
        local r
        if k == "record" then
            local id = tonumber(v)
            local found = false
            for i, m in ipairs(s.mail) do if m.id == id then s.idx, s.reply, found = i, "", true end end
            if not found then for _, x in ipairs(s.records) do if x.id == id then r = x; break end end end
        else
            local open = {}
            for _, m in ipairs(s.mail) do open[m.id] = true end
            for _, x in ipairs(s.records) do if not open[x.id] then r = x; break end end
            if not r then info("NO MESSAGE TO RECALL") end
        end
        if r then
            s.mail[#s.mail + 1] = { id = r.id, time = r.time, station = r.station or "", text = r.text, kind = r.kind or "INFO",
                dir = r.dir, attr = r.attr, archive = true, recalled = true }
            s.idx, s.reply = #s.mail, ""
        end
    elseif k == "recview" then s.recview, s.expand, s.recscroll = v, nil, 0
    elseif k == "expand" then s.expand = tonumber(v)
    elseif k == "recback" then s.expand = nil
    elseif k == "rectop" then s.recscroll = 0
    elseif k == "recup" then s.recscroll = math.max(0, s.recscroll - 1)
    elseif k == "recdown" then s.recscroll = s.recscroll + 1
    elseif k == "recend" then s.recscroll = #s.records
    elseif k == "erase" then s.records, s.recscroll = {}, 0
    elseif k == "mode" then local i = tonumber(v); s.weather_menu = (s.weather_menu == i) and nil or i
    elseif k == "setmode" then
        local w = s.atis[s.weather_menu or 0]
        if w then w.mode, w.text, w.time, w.source, w.status, w.offset, w.edited = v, nil, nil, nil, nil, 0, true end
        s.weather_menu = nil
    elseif k == "weather" then weather(tonumber(v))
    elseif k == "weatherall" then for i = 1, 3 do weather(i) end
    elseif k == "auto" then local w = s.atis[tonumber(v)]; if w then w.auto = not w.auto end
    elseif k == "upd" then local i = tonumber(v); s.upd_menu = (s.upd_menu == i) and nil or i
    elseif k == "wxnext" then local w = s.atis[tonumber(v)]; if w then w.offset = (w.offset or 0) + 5 end
    elseif k == "wxtop" then local w = s.atis[tonumber(v)]; if w then w.offset = 0 end
    end
    M.dirty = true
end

function M.click(which, x, y)
    local hits = M.hits[which] or {}
    for i = #hits, 1, -1 do
        local h = hits[i]
        if x >= h[1] and x <= h[1] + h[3] and y >= h[2] and y <= h[2] + h[4] then
            if type(h[5]) == "table" then
                M.focus, M.dirty = h[5], true
                M.s.dropdown, M.s.weather_menu, M.s.upd_menu, M.s.sysmenu = nil, nil, nil, nil
            else
                action(h[5])
            end
            return true
        end
    end
    M.focus, M.dirty = nil, true
    M.s.dropdown, M.s.weather_menu, M.s.upd_menu, M.s.sysmenu = nil, nil, nil, nil
    return false
end

-- FF's INIT page: CPNY WIND REQUEST key where FF shows a second CPNY F-PLN REQUEST (A350: CPNY WIND REQUEST).
-- Shown like the A350's key but inactive: the mod has no wind uplink (FF's FMS does not use uplinked winds).
local IK = { x = 512, y = 365, w = 150, h = 65 }
function M.initkey_rect() return IK.x, IK.y, IK.w, IK.h end
-- areas to cover in black when the display is not lit
function M.initkey_mask() return { { IK.x, IK.y, IK.w, IK.h } } end
-- changes whenever the key has to be redrawn (never: it is always inactive)
function M.initkey_sig() return "inop" end
function M.initkey()
    local a = canvas("lower")
    a.rect(IK.x - 2, IK.y - 2, IK.w + 4, IK.h + 4, C.bg)
    a.button(IK.x, IK.y, IK.w, IK.h, "CPNY WIND\nREQUEST", nil, false, false, 10)
    return a.ops
end

-- mouse position for the cyan hover frame (which = nil: mouse elsewhere); redraws only when the key under it changes
function M.hover(which, x, y)
    local old = M.hov_key
    M.hov = which and { which = which, x = x, y = y } or nil
    local id = nil
    if which then
        local hits = M.hits[which] or {}
        for i = #hits, 1, -1 do
            local h = hits[i]
            if x >= h[1] and x <= h[1] + h[3] and y >= h[2] and y <= h[2] + h[4] then id = which .. i; break end
        end
    end
    M.hov_key = id
    if id ~= old then M.dirty = true end
    return id ~= old
end

function M.key(ch, vk)
    local f = M.focus
    if not f then return false end
    local s = M.s
    local k = f.key
    local t, key
    if k == "center" then t, key = s, "center"
    elseif k == "extra" then t, key = s, "extra"
    elseif k:sub(1, 4) == "val:" then t, key = s.val, k:sub(5)
    elseif k:sub(1, 5) == "icao:" then t, key = s.atis[tonumber(k:sub(6))], "code"
    else return false end
    local val = t[key] or ""
    if ch == 8 or vk == 8 or ch == 127 then val = val:sub(1, -2)
    elseif ch == 13 or ch == 27 or vk == 13 or vk == 27 then M.focus, M.dirty = nil, true; return true
    elseif ch >= 32 and ch <= 126 then
        local c = string.char(ch):upper()
        if f.kind == "station" and not c:match("[A-Z0-9_]") then return true end
        if f.kind == "icao" and not c:match("[A-Z]") then return true end
        if #val < f.limit then val = val .. c end
    else return true end
    t[key] = val
    if key == "code" then t.edited, t.text, t.time, t.source, t.status, t.offset = true, nil, nil, nil, nil, 0 end
    M.dirty = true
    return true
end

function M.blur() if M.focus then M.focus, M.dirty = nil, true end end
function M.page_changed() M.blur(); M.s.cat, M.s.weather_menu, M.s.dropdown, M.s.upd_menu, M.s.sysmenu = nil, nil, nil, nil, nil; M.hits = {}; M.dirty = true end

M._test = { compose = compose, wire = wire, valid = valid, action = action, fields = fields, station = station, REQ = REQ }
return M
