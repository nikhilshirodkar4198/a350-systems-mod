-- A350 Systems Mod - Hoppie ACARS client in Lua (X-Plane 11 and 12, FlyWithLua only). MIT License. Version 1.0
-- Written from Hoppie's public protocol (https://www.hoppie.nl/acars/system/tech.html): one HTTPS form post per
-- message to connect.html (logon, from, to, type, packet), answer "ok ..." or "error {...}". Requests run in the
-- background with curl, one at a time, so the simulator never waits for the network.
-- D-ATIS comes from the network's public data first (VATSIM ATIS list, or the IVAO controllers' ATIS) (Hoppie's vatatis only finds airports with one combined
-- ICAO_ATIS station, never separate _D_ATIS / _A_ATIS ones), then Hoppie, then the METAR.
--
-- The ATCCOM page (A350_ATCCOM/ui.lua) runs in the same script: it calls link:request(...) and collects the answers
-- from link.out ({kind, id, status, station, text} in order). Safety rules: never resend automatically, one
-- transmission at a time, CPDLC message numbers 1..63 per session, logon timeout 180 s, datalink loss ends the
-- CPDLC session.

local ffi = require("ffi")
pcall(ffi.cdef, [[ unsigned int hop_WinExec(const char *, unsigned int) __asm__("WinExec"); ]])

local URL = "https://www.hoppie.nl/acars/system/connect.html"
local VATSIM_ATIS_URL = "https://data.vatsim.net/v3/afv-atis-data.json"
local VATSIM_DATA_URL = "https://data.vatsim.net/v3/vatsim-data.json"
local IVAO_URL = "https://api.ivao.aero/v2/tracker/whazzup"
local IS_WIN = package.config:sub(1, 1) == "\\"
local POLL_SLOW, POLL_FAST = { 45, 75 }, { 20, 25 }
local TX_TIMEOUT, LOGON_TIMEOUT, WX_TIMEOUT = 45, 180, 40

local H = {}
H.__index = H

local function urlenc(s)
    return (tostring(s):gsub("[^%w%-%._~]", function(c) return string.format("%%%02X", c:byte()) end))
end
local function trim(s) return (tostring(s or ""):gsub("^%s+", ""):gsub("%s+$", "")) end
local function winpath(p) return (p:gsub("/", "\\")) end
local function readfile(p)
    local f = io.open(p, "rb")
    if not f then return nil end
    local s = f:read("*a"); f:close()
    return s
end
local function writefile(p, s)
    local f = io.open(p, "wb")
    if not f then return false end
    f:write(s); f:close()
    return true
end

-- small JSON reader for VATSIM's data feed: objects, arrays (n = length, null leaves a hole), strings, numbers,
-- true/false/null. Decodes the one value starting at start (default 1). Returns nil on any malformed input.
local function json_decode(s, start)
    if type(s) ~= "string" then return nil end
    local pos = start or 1
    local ESC = { b = "\b", f = "\f", n = "\n", r = "\r", t = "\t" }
    local function ws() pos = s:find("[^ \t\r\n]", pos) or #s + 1 end
    local function char() return s:sub(pos, pos) end
    local function str()
        local out, i = {}, pos + 1
        while true do
            local a = s:find('["\\]', i)
            if not a then error("unterminated string") end
            out[#out + 1] = s:sub(i, a - 1)
            if s:sub(a, a) == '"' then pos = a + 1; return table.concat(out) end
            local c = s:sub(a + 1, a + 1)
            if c == "u" then
                local cp = tonumber(s:sub(a + 2, a + 5), 16) or 63
                out[#out + 1] = cp < 128 and string.char(cp) or "?"
                i = a + 6
            else
                out[#out + 1] = ESC[c] or c
                i = a + 2
            end
        end
    end
    local value
    function value()
        ws()
        local c = char()
        if c == "{" then
            local t = {}
            pos = pos + 1; ws()
            if char() == "}" then pos = pos + 1; return t end
            while true do
                ws()
                if char() ~= '"' then error("key expected") end
                local k = str(); ws()
                if char() ~= ":" then error("':' expected") end
                pos = pos + 1
                t[k] = value(); ws()
                c = char(); pos = pos + 1
                if c == "}" then return t elseif c ~= "," then error("',' expected") end
            end
        elseif c == "[" then
            local t = { n = 0 }
            pos = pos + 1; ws()
            if char() == "]" then pos = pos + 1; return t end
            while true do
                t.n = t.n + 1
                t[t.n] = value(); ws()
                c = char(); pos = pos + 1
                if c == "]" then return t elseif c ~= "," then error("',' expected") end
            end
        elseif c == '"' then return str()
        end
        if s:sub(pos, pos + 3) == "true" then pos = pos + 4; return true end
        if s:sub(pos, pos + 4) == "false" then pos = pos + 5; return false end
        if s:sub(pos, pos + 3) == "null" then pos = pos + 4; return nil end
        local num = s:match("^%-?%d[%d%.eE%+%-]*", pos)
        if not num then error("value expected at " .. pos) end
        pos = pos + #num
        return tonumber(num)
    end
    local ok, r = pcall(value)
    return ok and r or nil
end

-- opts: dir (folder with trailing slash), now (seconds), log (function), version (string)
function H.new(opts)
    local self = setmetatable({}, H)
    self.dir, self.now, self.log, self.ua = opts.dir, opts.now, opts.log or print, "A350-Systems-Mod/" .. (opts.version or "?")
    self.http = opts.http -- (offline tests: function(form) -> response text)
    math.randomseed(os.time() + math.floor((os.clock() % 1) * 1e6))
    -- download files left behind when X-Plane closed or crashed during a request: FlyWithLua can list the folder
    if type(directory_to_table) == "function" then
        local ok, list = pcall(directory_to_table, self.dir)
        if ok and type(list) == "table" then
            for _, n in ipairs(list) do
                if type(n) == "string" and n:match("^http%d+%.") then os.remove(self.dir .. n) end
            end
        end
    end
    self.out = {}
    self.logon = trim(readfile(self.dir .. "hoppie_logon.txt") or "")
    self.callsign, self.ready, self.status, self.err = nil, false, "NO LOGON CODE", nil
    self.job, self.jobn, self.queue = nil, 0, {}
    self.next_poll, self.fast_until = 0, 0
    self.seen, self.rx_seq = {}, 0
    self.tx, self.wx = nil, nil
    self.cp = { active = "", pending = nil, mid = 0, open = {}, seen = {}, requests = {} }
    return self
end

-------------------------------------------------------------------------------
-- settings
-------------------------------------------------------------------------------
function H:set_logon(code)
    code = trim(code)
    if code == self.logon then return end
    self.logon = code
    writefile(self.dir .. "hoppie_logon.txt", code)
    self.err, self.ready, self.bad_logon = nil, false, nil
    self.next_poll = 0
    self.log("Hoppie: logon code " .. (code == "" and "removed" or "changed"))
end

-------------------------------------------------------------------------------
-- answers to the ATCCOM page (it takes them from self.out)
-------------------------------------------------------------------------------
function H:event(kind, id, status, station, text)
    self.out[#self.out + 1] = { kind, tostring(id), status, station or "", text or "" }
    self.log(string.format("Hoppie: %s %s %s %s", kind, tostring(id), status, tostring(station or "")))
end

-------------------------------------------------------------------------------
-- HTTP in the background (curl), one request at a time
-------------------------------------------------------------------------------
local function run_hidden(cmd)
    local ok = pcall(function() assert(ffi.load("kernel32").hop_WinExec(cmd, 0) > 31) end)
    if not ok then os.execute(cmd) end
end

function H:send(form, cb)
    self.queue[#self.queue + 1] = { form = form, cb = cb }
end

function H:http_tick()
    local j = self.job
    if j then
        local resp = readfile(j.out)
        local failed = not resp and readfile(j.err)
        if resp or failed or self.now() - j.t0 > 40 then
            for _, p in ipairs({ j.out, j.err, j.body, j.cmd }) do os.remove(p) end
            self.job = nil
            if j.get and resp then self.gcache[j.get] = { t = self.now(), text = resp } end
            local ok, e = pcall(j.cb, resp and trim(resp) or nil)
            if not ok then self.log("Hoppie: " .. tostring(e)) end
        end
        return
    end
    local q = table.remove(self.queue, 1)
    if not q then return end
    -- form.get = plain download of that URL (no logon or other data sent), else a form post to Hoppie. Downloads
    -- are kept 60 s: opening the station list again or several D-ATIS slots fetch each network feed once.
    local get = q.form.get
    self.gcache = self.gcache or {}
    local hit = get and self.gcache[get]
    if hit and self.now() - hit.t < 60 then
        local ok, e = pcall(q.cb, hit.text)
        if not ok then self.log("Hoppie: " .. tostring(e)) end
        return
    end
    local parts = {}
    for _, k in ipairs({ "logon", "from", "to", "type", "packet" }) do parts[#parts + 1] = k .. "=" .. urlenc(q.form[k] or "") end
    local body = table.concat(parts, "&")
    if self.http then -- offline test: synchronous fake server
        local ok, e = pcall(q.cb, self.http(q.form))
        if not ok then self.log("Hoppie: " .. tostring(e)) end
        return
    end
    self.jobn = self.jobn + 1
    local base = self.dir .. "http" .. self.jobn
    j = { out = base .. ".txt", err = base .. ".err", body = base .. ".body", cmd = base .. ".cmd", t0 = self.now(), cb = q.cb, get = get }
    os.remove(j.out); os.remove(j.err)
    if not get then writefile(j.body, body) end
    if IS_WIN then
        local tmp = winpath(base .. ".tmp")
        local data = get and "" or string.format('--data-binary "@%s" ', winpath(j.body))
        writefile(j.cmd, "@echo off\r\n" ..
            string.format('curl.exe -sS -m 30 -A "%s" %s-o "%s" "%s" >nul 2>&1\r\n', self.ua, data, tmp, get or URL) ..
            "if errorlevel 1 goto bad\r\n" ..
            string.format('move /y "%s" "%s" >nul\r\nexit /b\r\n:bad\r\ndel /q "%s" 2>nul\r\necho x> "%s"\r\n', tmp, winpath(j.out), tmp, winpath(j.err)))
        run_hidden(string.format('cmd.exe /c "%s"', winpath(j.cmd)))
    else
        local tmp = base .. ".tmp"
        local data = get and "" or string.format('--data-binary "@%s" ', j.body)
        os.execute(string.format('(curl -sS -m 30 -A "%s" %s-o "%s" "%s" && mv "%s" "%s" || echo x > "%s") >/dev/null 2>&1 &',
            self.ua, data, tmp, get or URL, tmp, j.out, j.err))
    end
    self.job = j
end

-- "ok {FROM type {packet}} ..." -> list of {from, type, packet}; "error {...}" -> nil, reason
local function parse(resp)
    if not resp then return nil, "NO RESPONSE" end
    local head, rest = resp:match("^(%a+)%s*(.*)$")
    if head == "error" then return nil, trim((rest or ""):match("^{(.-)}") or rest) end
    if head ~= "ok" then return nil, "UNEXPECTED RESPONSE" end
    local msgs = {}
    for from, typ, packet in rest:gmatch("{(%S+)%s+(%S+)%s+{(.-)}}") do msgs[#msgs + 1] = { from = from, type = typ:lower(), packet = packet } end
    return msgs
end
H.parse = parse

-------------------------------------------------------------------------------
-- polling and status
-------------------------------------------------------------------------------
function H:schedule(fast)
    local r = fast and POLL_FAST or POLL_SLOW
    self.next_poll = self.now() + r[1] + math.random() * (r[2] - r[1])
end

function H:want_fast()
    return self.cp.active ~= "" or self.cp.pending ~= nil or self.now() < self.fast_until
        or (self.dcl ~= nil and self.now() - self.dcl.t0 < 1800)
end

function H:poll()
    self.polling = true
    self:send({ logon = self.logon, from = self.callsign, to = "SERVER", type = "poll", packet = "" }, function(resp)
        self.polling = false
        local msgs, err = parse(resp)
        if not msgs then
            if err and err:upper():find("LOGON") then
                self.bad_logon, self.ready, self.err = true, false, "HOPPIE LOGON CODE REJECTED"
                self.log("Hoppie: logon code rejected (" .. err .. ")")
            else
                self.err = err
                self.fails = (self.fails or 0) + 1
                if self.ready then self.log("Hoppie: poll failed - " .. tostring(err)) end
                if self.fails >= 3 then self.ready = false end
            end
            self:schedule(false)
            return
        end
        if not self.ready then self.log("Hoppie: connected as " .. self.callsign) end
        self.ready, self.err, self.fails = true, nil, 0
        for _, m in ipairs(msgs) do self:incoming(m) end
        self:schedule(self:want_fast())
    end)
end

-- flight = FMS flight number (callsign), powered = avionics on. Call about once per second.
function H:tick(flight, powered)
    local t = self.now()
    flight = trim(flight):upper()
    local cs = (powered and flight:match("^[A-Z0-9]+$") and #flight >= 3 and #flight <= 7) and flight or nil
    if cs ~= self.callsign then
        if self.cp.active ~= "" or self.cp.pending then self:cp_clear("DATALINK LOST - RELOGON REQUIRED") end
        if self.tx then self:finish("UNCONFIRMED") end
        self.callsign, self.ready, self.next_poll = cs, false, 0
    end
    if self.logon == "" then self.status = "NO LOGON CODE"
    elseif self.bad_logon then self.status = "LOGON CODE REJECTED"
    elseif not cs then self.status = powered and "ENTER FLIGHT ID" or "AVIONICS OFF"
    elseif self.ready then self.status = "READY"
    else self.status = "CONNECTING" end
    if cs and self.logon ~= "" and not self.bad_logon and not self.polling and t >= self.next_poll then self:poll() end
    if self.tx and t - self.tx.t0 > TX_TIMEOUT then self:finish("UNCONFIRMED") end
    if self.cp.pending and t - self.cp.pending.t0 > LOGON_TIMEOUT then self:cp_clear("LOGON TIMEOUT") end
    if self.wx and self.wx.sent and t - self.wx.sent > WX_TIMEOUT then self:wx_advance(true) end
    self:http_tick()
end

-------------------------------------------------------------------------------
-- requests from the ATCCOM page
-------------------------------------------------------------------------------
function H:busy() return self.tx ~= nil or self.wx ~= nil end

-- id = the page's request id, sender = its flight number; the answer comes as an event with the same id
function H:request(id, sender, dest, text, typ)
    id, sender, dest, text = tostring(id), tostring(sender or ""), tostring(dest or ""), tostring(text or "")
    typ = (typ and typ ~= "") and typ or "telex"
    if self.seen[id] then return end
    self.seen[id] = true
    local ready = self.ready and sender == self.callsign
    if typ:sub(1, 3) == "cp_" then return self:cp_submit(id, dest, text, typ, ready) end
    -- online stations for the NOTIFY TO ATC list: ping ALL-CALLSIGNS (answer: everyone logged on to Hoppie, aircraft
    -- included). Only 4-character callsigns are kept (ATC logon codes); those staffed on the selected network (VATSIM
    -- or IVAO, payload "ALL-CALLSIGNS:IVAO") are marked "CODE*".
    local net = "VATSIM"
    if typ == "stations" or typ == "weather" then
        net = (text:match(":(%u+)$") == "IVAO") and "IVAO" or "VATSIM"
        text = text:gsub(":%u+$", "")
    end
    if typ == "stations" then
        if not ready then return self:event("STATIONS", id, "NOT READY", "", "") end
        return self:send({ logon = self.logon, from = self.callsign, to = "SERVER", type = "ping", packet = "ALL-CALLSIGNS" }, function(resp)
            local body = resp and resp:match("^ok%s*{(.*)}%s*$")
            if not body then return self:event("STATIONS", id, "FAILED", "", "") end
            local list = {}
            for cs in body:gmatch("%S+") do
                cs = cs:upper()
                if cs ~= self.callsign and cs:match("^[A-Z0-9][A-Z0-9][A-Z0-9][A-Z0-9]$") then list[#list + 1] = cs end
            end
            self:send({ get = net == "IVAO" and IVAO_URL or VATSIM_DATA_URL }, function(feed)
                -- (a feed that cannot be read still answers with the plain list, never leaves the page waiting)
                local okf, staffed = pcall(function() return H.staffed(H.atc_list(feed, net)) end)
                if not okf then staffed = nil end
                if not staffed then self.log("Hoppie: " .. net .. " controller list not available") end
                for i, cs in ipairs(list) do if staffed and staffed[cs] then list[i] = cs .. "*" end end
                self:event("STATIONS", id, "OK", "", table.concat(list, " "))
            end)
        end)
    end
    if typ == "weather" then
        if not ready then return self:event("WX", id, "NOT READY", dest, "") end
        if self:busy() then return self:event("WX", id, "BUSY", dest, "") end
        if not dest:match("^%u%u%u%u$") or not (text == "DEP" or text == "ARR" or text == "ENROUTE") then return self:event("WX", id, "INVALID", dest, "") end
        -- (Hoppie's vatatis only answers a plain airport code, and only for a combined ICAO_ATIS station)
        local stages = { { "NET", net:lower() .. " atis " .. dest }, { "ATIS", "vatatis " .. dest }, { "METAR", "metar " .. dest } }
        self.wx = { id = id, icao = dest, mode = text, net = net, stages = stages }
        return self:wx_advance(false)
    end
    if typ ~= "telex" then return self:event("TX", id, "INVALID", dest, text) end
    if not ready then return self:event("TX", id, "NOT READY", dest, text) end
    if not dest:match("^[A-Z0-9]+$") or #dest < 4 or #dest > 7 or #text < 1 or #text > 220 or text:find("[{}%c]") or trim(text) == "" then
        return self:event("TX", id, "INVALID", dest, text)
    end
    if self:busy() then return self:event("TX", id, "BUSY", dest, text) end
    self:transmit(id, dest, "telex", text, text, nil)
end

-- one transmission at a time; "ok" = SERVER ACK (accepted by Hoppie, not proof of delivery)
function H:transmit(id, dest, typ, packet, display, cpinfo)
    self.tx = { id = id, dest = dest, display = display, cp = cpinfo, t0 = self.now() }
    -- departure clearance (DCL): the station answers by CPDLC without a logon, its clearance must be answerable
    if typ == "telex" and packet:upper():match("^REQUEST PREDEP CLEARANCE") then self.dcl = { station = dest, t0 = self.now() } end
    if cpinfo then self:event("CPTX", id, "QUEUED", dest, display) else self:event("TX", id, "QUEUED", dest, display) end
    self:send({ logon = self.logon, from = self.callsign, to = dest, type = typ, packet = packet }, function(resp)
        if not self.tx or self.tx.id ~= id then return end
        local msgs, err = parse(resp)
        if msgs then
            self:finish("SERVER ACK")
            -- a reply may follow: poll once in about 20 s
            self.fast_until = self.now() + 120
            self.next_poll = math.min(self.next_poll, self.now() + 20)
        else
            self:finish(err == "NO RESPONSE" and "UNCONFIRMED" or "REJECTED")
            if err then self.log("Hoppie: send rejected - " .. err) end
        end
    end)
end

function H:finish(status)
    local tx = self.tx
    if not tx then return end
    self.tx = nil
    local c = tx.cp
    if not c then return self:event("TX", tx.id, status, tx.dest, tx.display) end
    self:event("CPTX", tx.id, status, tx.dest, tx.display)
    if c.typ == "cp_reply" and self.cp.open[c.key] then
        local r = self.cp.open[c.key]
        if status == "SERVER ACK" then
            r.busy = false
            if tx.display ~= "STANDBY" then self.cp.open[c.key] = nil end
        end -- (UNCONFIRMED stays locked: never answered twice)
    elseif c.typ == "cp_logoff" and status == "SERVER ACK" then
        self:cp_clear("LOGOFF SENT - SERVER ACK")
    elseif c.typ == "cp_logon" and status ~= "SERVER ACK" then
        self:cp_clear("LOGON NOT SENT")
    elseif c.typ == "cp_request" and status == "SERVER ACK" then
        self.cp.requests[c.mid] = tx.display
    end
end

-------------------------------------------------------------------------------
-- weather: VATSIM ATIS list (departure/arrival, then combined), Hoppie vatatis, then METAR
-------------------------------------------------------------------------------
local ATIS_PICK = { DEP = { "_D_ATIS", "_ATIS" }, ARR = { "_A_ATIS", "_ATIS" }, ENROUTE = { "_ATIS", "_A_ATIS", "_D_ATIS" } }

-- list = decoded afv-atis-data.json; returns text, station of the best ATIS for icao/mode, or nil
function H.vatsim_atis(list, icao, mode)
    if type(list) ~= "table" then return nil end
    local by = {}
    for i = 1, list.n or #list do
        local a = list[i]
        if type(a) == "table" and type(a.callsign) == "string" and type(a.text_atis) == "table" then
            local lines = {}
            for k = 1, a.text_atis.n or #a.text_atis do
                if type(a.text_atis[k]) == "string" then lines[#lines + 1] = a.text_atis[k] end
            end
            local text = trim((table.concat(lines, " "):gsub("%s+", " ")))
            if text ~= "" then by[a.callsign:upper()] = text end
        end
    end
    for _, suffix in ipairs(ATIS_PICK[mode] or ATIS_PICK.ENROUTE) do
        local cs = icao .. suffix
        if by[cs] then return by[cs], cs end
    end
end
H.json_decode = json_decode

local function json_array(feed, key)
    if type(feed) ~= "string" then return nil end
    local a = feed:find('"' .. key .. '"%s*:%s*%[')
    if not a then return nil end
    local list = json_decode(feed, feed:find("%[", a))
    return type(list) == "table" and list or nil
end
local function lines_of(t)
    local out = {}
    if type(t) == "table" then for k = 1, t.n or #t do if type(t[k]) == "string" then out[#out + 1] = t[k] end end end
    return out
end

-- online controllers of a network feed as { {cs = callsign, lines = {info / ATIS text}}, ... }, or nil. Only the
-- controllers part is decoded: VATSIM vatsim-data.json "controllers" (observers, facility 0, left out), IVAO whazzup
-- "atcs" (each controller's ATIS lines; the first, the voice server address, left out).
function H.atc_list(feed, net)
    local list = json_array(feed, net == "IVAO" and "atcs" or "controllers")
    if not list then return nil end
    local out = {}
    for i = 1, list.n or #list do
        local c = list[i]
        if type(c) == "table" and type(c.callsign) == "string" then
            if net == "IVAO" then
                local l = lines_of(type(c.atis) == "table" and c.atis.lines)
                if l[1] and (l[1]:find("ivao%.aero") or l[1]:match("^ts%-")) then table.remove(l, 1) end
                out[#out + 1] = { cs = c.callsign:upper(), lines = l }
            elseif (tonumber(c.facility) or 0) > 0 then
                out[#out + 1] = { cs = c.callsign:upper(), lines = lines_of(c.text_atis) }
            end
        end
    end
    return out
end

-- logon codes staffed: a code counts when it starts a controller callsign (EKCH_W_APP) or stands as a word in the
-- controller's info ("CPDLC/PDC [LONC]", IVAO "CPDLC ID EDDH")
function H.staffed(atcs)
    if not atcs then return nil end
    local set = {}
    for _, c in ipairs(atcs) do
        local head = c.cs:match("^[^_]+")
        if head then set[head] = true end
        for _, l in ipairs(c.lines) do for w in l:upper():gmatch("[A-Z0-9]+") do if #w == 4 then set[w] = true end end end
    end
    return set
end

-- IVAO has no ATIS stations: every controller carries its airport's ATIS. Text of a controller at icao that reads
-- like an ATIS (INFORMATION / ATIS), tower first, then approach, ground, delivery, others; or nil
local IVAO_RANK = { TWR = 1, APP = 2, GND = 3, DEL = 4 }
function H.ivao_atis(atcs, icao)
    local best, br
    for _, c in ipairs(atcs or {}) do
        if c.cs:match("^[^_]+") == icao then
            local text = trim((table.concat(c.lines, " "):gsub("%s+", " ")))
            local up = text:upper()
            if text ~= "" and (up:find("INFORMATION") or up:find("ATIS")) then
                local r = IVAO_RANK[c.cs:match("_(%u+)$") or ""] or 9
                if not br or r < br then best, br = { text, c.cs }, r end
            end
        end
    end
    if best then return best[1], best[2] end
end

function H:wx_finish(status, text)
    local w = self.wx
    if not w then return end
    self.wx = nil
    self:event("WX", w.id, status, w.icao, text or "")
end

function H:wx_advance(timeout)
    local w = self.wx
    if not w then return end
    if timeout then
        if w.stage == "METAR" then return self:wx_finish("NO RESPONSE") end
        -- (the network data timing out says nothing about Hoppie: its ATIS is still asked)
        if w.stage ~= "NET" then w.stages = { { "METAR", "metar " .. w.icao } } end
    end
    local s = table.remove(w.stages, 1)
    if not s then return self:wx_finish("NOT AVAILABLE") end
    w.stage, w.packet, w.sent = s[1], s[2], self.now()
    w.n = (w.n or 0) + 1
    local n = w.n
    self:event("WX", w.id, "PENDING", w.icao, w.packet)
    if w.stage == "NET" then
        return self:send({ get = w.net == "IVAO" and IVAO_URL or VATSIM_ATIS_URL }, function(resp)
            if self.wx ~= w or w.n ~= n then return end
            local text, cs
            if w.net == "IVAO" then text, cs = H.ivao_atis(H.atc_list(resp, "IVAO"), w.icao)
            else text, cs = H.vatsim_atis(json_decode(resp), w.icao, w.mode) end
            if not text then return self:wx_advance(false) end
            self.log("Hoppie: D-ATIS " .. cs .. " from the " .. w.net .. " data")
            self:wx_finish("ATIS", text)
        end)
    end
    self:send({ logon = self.logon, from = self.callsign, to = "SERVER", type = "inforeq", packet = w.packet }, function(resp)
        if self.wx ~= w or w.n ~= n then return end
        local msgs, err = parse(resp)
        if not msgs then
            if err == "NO RESPONSE" then return self:wx_advance(true) end
            return self:wx_finish("NOT AVAILABLE")
        end
        local text = ""
        for _, m in ipairs(msgs) do if m.type == "info" then text = trim(m.packet) end end
        self:wx_answer(text)
    end)
end

function H:wx_answer(text)
    local w = self.wx
    -- (Hoppie splits answers over lines, e.g. "THIS ATIS IS NOT" / "AVAILABLE": compare on one line)
    local up = text:upper():gsub("%s+", " ")
    local icao = up:match("^%s*METAR%s+(%u%u%u%u)%s+%d%d%d%d%d%dZ") or up:match("^%s*SPECI%s+(%u%u%u%u)%s+%d%d%d%d%d%dZ")
        or up:match("^%s*(%u%u%u%u)%s+%d%d%d%d%d%dZ")
    if icao then
        if icao == w.icao then return self:wx_finish("METAR", text) end
        return self:wx_advance(false)
    end
    if w.stage == "METAR" then return self:wx_finish("NOT AVAILABLE") end
    local missing = up:find("NO ATIS") or up:find("NO VATSIM") or up:find("NOT FOUND") or up:find("NOT AVAILABLE") or up:find("NOT ONLINE")
        or up:find("UNKNOWN") or up:find("UNAVAILABLE") or up:find("NO STATION") or up:find("NO DATA") or up:find("MULTIPLE MATCH")
    if text == "" or missing or not (up:find("INFORMATION") or up:find("ATIS") or up:find("INFO")) then return self:wx_advance(false) end
    self:wx_finish("ATIS", text)
end

-------------------------------------------------------------------------------
-- CPDLC (Hoppie /data2/ format: /data2/<msg no>/<ref no>/<response attribute>/<text>)
-------------------------------------------------------------------------------
function H:cp_clear(reason)
    local cp = self.cp
    cp.active, cp.pending, cp.open, cp.seen, cp.requests = "", nil, {}, {}, {}
    self:event("CPSTATE", "0", "OFF", "", reason)
end

-- the small set of requests the ATCCOM page can build, in Hoppie CPDLC wording ("@" separates text and value)
local function cp_request_text(t)
    t = trim(t):upper()
    local kind, v = t:match("^REQUEST (CLIMB TO) (.+)$")
    if not kind then kind, v = t:match("^REQUEST (DESCEND TO) (.+)$") end
    if kind and (v:match("^FL%d%d%d$") or v:match("^%d%d%d%d?%d? FT$")) then
        return "REQUEST " .. (kind == "DESCEND TO" and "DESCENT TO" or kind) .. "@" .. v
    end
    v = t:match("^REQUEST DIRECT TO ([A-Z0-9]+)$")
    if v and #v >= 2 and #v <= 7 then return "REQUEST DIRECT TO@" .. v end
    v = t:match("^REQUEST SPEED (%d%d%d KT)$") or t:match("^REQUEST SPEED (M%.%d%d)$")
    if v then return "REQUEST @" .. v end
    return nil
end
H.cp_request_text = cp_request_text
-- requests composed by the ATCCOM page in Hoppie wording ("@" before values, "@_@" between elements): only plain
-- capitals, digits and . / - and only requests or WHEN CAN WE EXPECT questions
local function cp_wire_ok(t)
    t = trim(t)
    if #t < 8 or #t > 200 or not t:match("^[A-Z0-9 .@_/%-]+$") then return nil end
    if t:sub(1, 8) == "REQUEST " or t:sub(1, 19) == "WHEN CAN WE EXPECT " then return t end
end
H.cp_wire_ok = cp_wire_ok

function H:cp_submit(id, dest, text, typ, ready)
    local cp = self.cp
    local function reject(r) self:event("CPTX", id, r, dest, text) end
    if not ready then return reject("NOT READY") end
    if not dest:match("^[A-Z0-9]+$") or #dest < 4 or #dest > 7 then return reject("INVALID STATION") end
    if self:busy() then return reject("BUSY") end
    local ref, attr, body, display, key = nil, "Y", nil, text, nil
    if typ == "cp_logon" then
        if cp.active ~= "" or cp.pending then return reject("DISCONNECT OR WAIT FIRST") end
        body = "REQUEST LOGON"
    elseif typ == "cp_logoff" then
        if dest ~= cp.active and not (cp.pending and dest == cp.pending.station) then return reject("NO SESSION") end
        body = "LOGOFF"
    elseif typ == "cp_request" then
        if dest ~= cp.active then return reject("NO ACTIVE ATC") end
        body = cp_request_text(text) or cp_wire_ok(text)
        if not body then return reject("REQUEST NOT SUPPORTED") end
    elseif typ == "cp_reply" then
        local k, r = text:match("^([^:]+):(.+)$")
        local o = k and cp.open[k]
        if not o or o.station ~= dest or (dest ~= cp.active and not o.dcl) or o.busy then return reject("NO OPEN UPLINK") end
        local allowed = ({ WU = { WILCO = 1, UNABLE = 1, STANDBY = 1 }, AN = { AFFIRM = 1, NEGATIVE = 1, STANDBY = 1 }, R = { ROGER = 1, STANDBY = 1 } })[o.attr] or {}
        if not allowed[r] then return reject("INVALID RESPONSE") end
        body, attr, ref, display, key = r, "N", o.mid, r, k
    else
        return reject("INVALID TYPE")
    end
    if cp.mid >= 63 then return reject("MESSAGE ID LIMIT; RESTART WHEN DISCONNECTED") end
    cp.mid = cp.mid + 1
    if key then cp.open[key].busy = true end
    local wire = string.format("/data2/%d/%s/%s/%s", cp.mid, ref and tostring(ref) or "", attr, body)
    if typ == "cp_logon" then
        cp.pending = { station = dest, mid = cp.mid, t0 = self.now() }
        self:event("CPSTATE", "0", "PENDING", dest, "LOGON IN PROGRESS")
    end
    self:transmit(id, dest, "cpdlc", wire, display, { typ = typ, mid = cp.mid, key = key })
end

function H:incoming(m)
    local from = m.from:upper()
    if m.type == "cpdlc" then return self:cp_incoming(from, m.packet) end
    self.rx_seq = self.rx_seq + 1
    if m.type == "telex" then
        self:event("RX", self.rx_seq, "RECEIVED", from, trim(m.packet))
    elseif trim(m.packet) ~= "" then
        self:event("INFO", self.rx_seq, "RECEIVED", from, trim(m.packet))
    end
end

function H:cp_incoming(from, raw)
    local cp = self.cp
    self.rx_seq = self.rx_seq + 1
    local key = tostring(self.rx_seq)
    local mid, ref, attr, body = trim(raw):match("^/[Dd][Aa][Tt][Aa]2/(%d+)/(%d*)/(%a+)/(.+)$")
    if not mid then return self:event("CPRX", key, "READ ONLY", from, "UNSUPPORTED CPDLC FORMAT: " .. raw) end
    mid, ref, attr = tonumber(mid), tonumber(ref), attr:upper()
    local dedup = table.concat({ from, mid, tostring(ref), attr, body }, "|")
    if cp.seen[dedup] then return end
    cp.seen[dedup] = true
    local text = trim((body:gsub("@_@", "\n"):gsub("@", " "):gsub("[ \t]+", " "):gsub(" ?\n ?", "\n")))
    local up = text:upper():gsub("%s+", " ")
    local accepted = up == "LOGON ACCEPTED" or up == "ACCEPT LOGON"
    local rejected = up == "LOGON REJECTED" or up == "LOGON DENIED" or up == "LOGON FAILED" or up == "NOTIF FAILED" or up == "UNABLE"
    local p = cp.pending
    if p and from == p.station and ref == p.mid and (accepted or rejected) then
        cp.pending = nil
        if accepted then
            cp.active = from
            self:event("CPSTATE", "0", "ACTIVE", from, "LOGON ACCEPTED")
        else
            self:cp_clear("LOGON REJECTED")
        end
        return self:event("CPRX", key, "READ ONLY", from, text)
    end
    -- DCL dialogue: the station we asked for a departure clearance (30 min) may send it without a CPDLC session
    local dcl = from ~= cp.active and self.dcl and self.dcl.station == from and self.now() - self.dcl.t0 < 1800
    if dcl then
        if attr == "WU" or attr == "AN" or attr == "R" then
            cp.open[key] = { station = from, mid = mid, attr = attr, busy = false, dcl = true }
            return self:event("CPRX", key, attr, from, text)
        end
        return self:event("CPRX", key, "READ ONLY", from, text)
    end
    if from ~= cp.active then return self:event("CPRX", key, "READ ONLY", from, "NO ACTIVE SESSION: " .. text) end
    if up == "LOGOFF" or up == "LOG OFF" or up == "END SERVICE" then
        self:cp_clear("ATC ENDED SERVICE")
        return self:event("CPRX", key, "READ ONLY", from, text)
    end
    if up:sub(1, 9) == "HANDOVER " then return self:event("CPRX", key, "READ ONLY", from, text .. " [MANUAL LOGOFF/LOGON REQUIRED]") end
    if ref and cp.requests[ref] and up ~= "STANDBY" then cp.requests[ref] = nil end
    if attr == "WU" or attr == "AN" or attr == "R" then
        cp.open[key] = { station = from, mid = mid, attr = attr, busy = false }
        return self:event("CPRX", key, attr, from, text)
    end
    self:event("CPRX", key, "READ ONLY", from, text)
end

return H
