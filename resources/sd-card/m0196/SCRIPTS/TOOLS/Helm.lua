-- TNS|HELM|TNE
---- #########################################################################
---- #                                                                       #
---- # Copyright (C) OpenTX, adapted for ExpressLRS                          #
-----#                                                                       #
---- # License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html               #
---- #                                                                       #
---- #########################################################################
local EXITVER = "-- EXIT --"
local TX_DEVICE_ID = 0xEE
local PAGE_DASHBOARD = 0
local PAGE_DEVICE = 1
local PAGE_VRX = 2
local DASHBOARD_DEVICE_ROWS = 3
local BIND_STICKY_ID = 0
local VRX_SCAN_STICKY_ID = 1
local UNBIND_STICKY_ID = 2
local ACTION_PULSE_TICKS = 300
local LQ_BAR_HEIGHTS = { 2, 3, 5, 7 }
-- Each icon row uses bit zero for its left pixel.
local ANTENNA_ICON = string.char(0x7F, 0x3E, 0x1C, 0x08, 0x08, 0x08, 0x08)
local ENCRYPTION_OFF_ICON = string.char(0x30, 0x48, 0x48, 0x1F, 0x1F, 0x1F, 0x00)
local ENCRYPTION_ON_ICON = string.char(0x0C, 0x12, 0x12, 0x3F, 0x3F, 0x3F, 0x00)
local WIFI_ICON = string.char(0x00, 0x1C, 0x22, 0x41, 0x1C, 0x22, 0x08)
local MAX_DEVICES = 8
local MAX_FIELDS = 128
local MAX_FIELD_BYTES = 512
local MAX_FIELD_RETRIES = 3
local FIELD_RETRY_DELAY = 20

local deviceId = TX_DEVICE_ID
local handsetId = 0xEA
local deviceName = ""
local lineIndex = 1
local pageOffset = 0
local edit = nil
local fieldPopup
local fieldTimeout = 0
local loadQ = {}
local fieldChunk = 0
local fieldData = nil
local fieldRetries = {}
local fields = {}
local devices = {}
local quickCommands = {}
local goodBadPkt = ""
local elrsFlags = 0
local elrsFlagsInfo = ""
local fields_count = 0
local devicesRefreshTimeout = 0
local currentFolderId = nil
local currentFolderName = nil
local commandRunningIndicator = 1
local expectChunksRemain = -1
local deviceIsELRS_TX = nil
local linkstatTimeout = 100
local titleShowWarn = nil
local titleShowWarnTimeout = 100
local exitscript = 0

local page = PAGE_DASHBOARD
local dashboardCursor = 1
local dashboardDeviceOffset = 0
local dashboardRedrawAt = 0
local txRefreshAt = 0
local rfSelected = true
local usbSelected = true
local joystickReady = false
local crsfAvailable = false
local quickConfirm = nil
local quickConfirmYes = true
local uiNotice = nil
local uiNoticeUntil = 0
local bindPulseEnd = 0
local scanPulseEnd = 0
local unbindPulseEnd = 0
local crsfCheckAt = 0
local encryptionState = nil
local encryptionStateRank = 0
local wifiState = nil
local wifiStateRank = 0
local trackStatusField
local returnToDashboard

local COL1 = 0
local COL2 = 70
local maxLineIndex = 6
local textYoffset = 3
local textSize = 8

local function allocateFields()
  -- The dashboard selects devices. This list contains fields and Back.
  fields = {}
  for i=1, fields_count do
    fields[i] = { }
  end
  fields[#fields+1] = {name=EXITVER, type=14}
end

local function reloadAllField()
  fieldTimeout = 0
  fieldChunk = 0
  fieldData = nil
  fieldRetries = {}
  -- loadQ is actually a stack
  loadQ = {}
  for fieldId = fields_count, 1, -1 do
    fields[fieldId].rejected = nil
    loadQ[#loadQ+1] = fieldId
  end
end

local function getField(line)
  local counter = 1
  for i = 1, #fields do
    local field = fields[i]
    if currentFolderId == field.parent and not field.hidden then
      if counter < line then
        counter = counter + 1
      else
        return field
      end
    end
  end
end

local function keepField(field)
  if field == fieldPopup then return true end
  if deviceId == TX_DEVICE_ID and
     (field.id == quickCommands.bind or field.id == quickCommands.unbind) then return true end
  if page == PAGE_DEVICE then
    for row = 1, maxLineIndex + 1 do
      if field == getField(pageOffset + row) then return true end
    end
  end
  return false
end

local function discardDetails(field)
  return { id=field.id, name=field.name, parent=field.parent,
           type=field.type, hidden=field.hidden,
           cachedValue=field.cachedValue }
end

local function fieldChoice(field, value)
  local start = 1
  for i = 1, value do
    start = string.find(field.values, ";", start, true)
    if not start then return "ERR" end
    start = start + 1
  end
  local stop = string.find(field.values, ";", start, true)
  return string.sub(field.values, start, stop and stop - 1 or -1)
end

local function incrField(step)
  local field = getField(lineIndex)
  local min, max = 0, 0
  if field.type <= 8 then
    min = field.min or 0
    max = field.max or 0
    step = (field.step or 1) * step
  elseif field.type == 9 then
    min = 0
    max = field.max
  end

  local newval = field.value
  repeat
    newval = newval + step
    if newval < min then
      newval = min
    elseif newval > max then
      newval = max
    end

    -- keep looping until a non-blank selection value is found
    if field.values == nil or fieldChoice(field, newval) ~= "" then
      field.value = newval
      return
    end
  until (newval == min or newval == max)
end

-- Select the next or previous editable field
local function selectField(step)
  local newLineIndex = lineIndex
  local field
  repeat
    newLineIndex = newLineIndex + step
    if newLineIndex <= 0 then
      newLineIndex = #fields
    elseif newLineIndex == 1 + #fields then
      newLineIndex = 1
      pageOffset = 0
    end
    field = getField(newLineIndex)
  until newLineIndex == lineIndex or (field and field.name)
  lineIndex = newLineIndex
  if lineIndex > maxLineIndex + pageOffset then
    pageOffset = lineIndex - maxLineIndex
  elseif lineIndex <= pageOffset then
    pageOffset = lineIndex - 1
  end
end

local function fieldGetStrOrOpts(data, offset)
  local stop = string.find(data, "\0", offset, true)
  if not stop then error("Invalid field string") end
  local text = string.sub(data, offset, stop - 1)
  if string.find(text, "\192", 1, true) then text = string.gsub(text, "\192", CHAR_UP) end
  if string.find(text, "\193", 1, true) then text = string.gsub(text, "\193", CHAR_DOWN) end
  return text, stop + 1
end

local function getDevice(id)
  for _, device in ipairs(devices) do
    if device.id == id then
      return device
    end
  end
end

local function fieldGetValue(data, offset, size)
  local result = 0
  for i=0, size-1 do
    result = bit32.lshift(result, 8) + string.byte(data, offset + i)
  end
  return result
end

local function removeRequest(id)
  for i = #loadQ, 1, -1 do
    if loadQ[i] == id then
      for j = i, #loadQ - 1 do loadQ[j] = loadQ[j+1] end
      loadQ[#loadQ] = nil
      break
    end
  end
end

local function reloadField(field)
  if not field.id then return end
  fieldTimeout = 0
  fieldChunk = 0
  fieldData = nil
  fieldRetries[field.id] = nil
  -- Keep one pending request per field, with this field at the top.
  removeRequest(field.id)
  loadQ[#loadQ+1] = field.id
end

local function reloadCurField()
  reloadField(getField(lineIndex))
end

-- UINT8/INT8/UINT16/INT16 + FLOAT + TEXTSELECT
local function fieldUnsignedLoad(field, data, offset, size, unitoffset)
  field.value = fieldGetValue(data, offset, size)
  field.min = fieldGetValue(data, offset+size, size)
  field.max = fieldGetValue(data, offset+2*size, size)
  --field.default = fieldGetValue(data, offset+3*size, size)
  field.unit = fieldGetStrOrOpts(data, offset+(unitoffset or (4*size)))
  -- Only store the size if it isn't 1 (covers most fields / selection)
  if size ~= 1 then
    field.size = size
  end
end

local function fieldUnsignedToSigned(field, size)
  local bandval = bit32.lshift(0x80, (size-1)*8)
  field.value = field.value - bit32.band(field.value, bandval) * 2
  field.min = field.min - bit32.band(field.min, bandval) * 2
  field.max = field.max - bit32.band(field.max, bandval) * 2
  --field.default = field.default - bit32.band(field.default, bandval) * 2
end

local function fieldSignedLoad(field, data, offset, size, unitoffset)
  fieldUnsignedLoad(field, data, offset, size, unitoffset)
  fieldUnsignedToSigned(field, size)
  -- signed ints are INTdicated by a negative size
  field.size = -size
end

local function fieldIntLoad(field, data, offset)
  -- Type is U8/I8/U16/I16, use that to determine the size and signedness
  local loadFn = (field.type % 2 == 0) and fieldUnsignedLoad or fieldSignedLoad
  loadFn(field, data, offset, math.floor(field.type / 2) + 1)
end

local function fieldIntSave(field)
  local value = field.value
  local size = field.size or 1
  -- Convert signed to 2s complement
  if size < 0 then
    size = -size
    if value < 0 then
      value = bit32.lshift(0x100, (size-1)*8) + value
    end
  end

  local frame = { deviceId, handsetId, field.id }
  for i = size-1, 0, -1 do
    frame[#frame + 1] = bit32.rshift(value, 8*i) % 256
  end
  crossfireTelemetryPush(0x2D, frame)
end

local function fieldIntDisplay(field, y, attr)
  lcd.drawText(COL2, y, field.value .. field.unit, attr)
end

-- -- FLOAT
local function fieldFloatLoad(field, data, offset)
  fieldSignedLoad(field, data, offset, 4, 21)
  field.prec = string.byte(data, offset+16)
  if field.prec > 3 then
    field.prec = 3
  end
  field.step = fieldGetValue(data, offset+17, 4)

  -- precompute the format string to preserve the precision
  field.fmt = "%." .. tostring(field.prec) .. "f"
  -- Convert precision to a divider
  field.prec = 10 ^ field.prec
end

local function fieldFloatDisplay(field, y, attr)
  lcd.drawText(COL2, y, string.format(field.fmt, field.value / field.prec) .. field.unit, attr)
end

-- TEXT SELECTION
local function fieldTextSelLoad(field, data, offset)
  field.values, offset = fieldGetStrOrOpts(data, offset)
  local count, nonempty = 0, 0
  for option in string.gmatch(field.values .. ";", "(.-);") do
    count = count + 1
    if option ~= "" then nonempty = nonempty + 1 end
  end
  field.max = count - 1
  field.grey = nonempty <= 1
  field.value = string.byte(data, offset)
  -- min max and default (offset+1 to 3) are not used on selections
  -- units never uses cache
  field.unit = fieldGetStrOrOpts(data, offset+4)
end

local function fieldTextSelDisplay(field, y, attr)
  lcd.drawText(COL2, y, fieldChoice(field, field.value), attr)
  lcd.drawText(lcd.getLastPos(), y, field.unit, 0)
end

-- STRING
local function fieldStringLoad(field, data, offset)
  field.value, offset = fieldGetStrOrOpts(data, offset)
  if #data >= offset then
    field.maxlen = string.byte(data, offset)
  end
end

local function fieldStringDisplay(field, y, attr)
  lcd.drawText(COL2, y, field.value, attr)
end

local function fieldFolderOpen(field)
  currentFolderId = field.id
  currentFolderName = string.match(field.name, "^(.-)%s*%(.*%)$") or field.name

  local backFld = fields[#fields]
  backFld.name = "----BACK----"
  -- Store the lineIndex and pageOffset to return to in the backFld
  backFld.li = lineIndex
  backFld.po = pageOffset
  backFld.parent = currentFolderId
  backFld.grandParent = fields[currentFolderId].parent

  lineIndex = 1
  pageOffset = 0
end

local function fieldFolderDisplay(field,y ,attr)
  lcd.drawText(COL1, y, "> " .. field.name, attr + BOLD)
end

local function fieldCommandLoad(field, data, offset)
  field.status = string.byte(data, offset)
  field.timeout = string.byte(data, offset+1)
  field.info = fieldGetStrOrOpts(data, offset+2)
end

local function startFieldCommand(field, autoConfirm)
  reloadField(field)

  if field.status ~= nil then
    if field.status < 4 then
      field.status = 1
      crossfireTelemetryPush(0x2D, { deviceId, handsetId, field.id, field.status })
      fieldPopup = field
      fieldPopup.lastStatus = 0
      fieldPopup.autoConfirm = autoConfirm
      fieldTimeout = getTime() + field.timeout
      return true
    end
  end
  return false
end

local function fieldCommandSave(field)
  startFieldCommand(field, nil)
end

local function fieldCommandDisplay(field, y, attr)
    lcd.drawText(10, y, "[" .. field.name .. "]", attr + BOLD)
end

local function fieldBackExec(field)
  if field.grandParent then -- Back from Sub-menu
    local parent = fields[field.grandParent]
    if parent and parent.type == 11 then fieldFolderOpen(parent) else returnToDashboard() end
  elseif field.parent then
    lineIndex = field.li or 1
    pageOffset = field.po or 0

    field.name = EXITVER
    field.parent = nil
    field.li = nil
    field.po = nil
    currentFolderId = nil
    currentFolderName = nil
  else
    returnToDashboard()
  end
end

local function changeDeviceId(devId) --change to selected device ID
  local device = getDevice(devId)
  if not device then return false end
  if deviceId == devId and fields_count == device.fldcnt then return end

  deviceId = devId
  quickCommands = {}
  fieldPopup = nil
  elrsFlags = 0
  encryptionState = nil
  encryptionStateRank = 0
  wifiState = nil
  wifiStateRank = 0
  currentFolderId = nil
  currentFolderName = nil
  deviceName = device.name
  fields_count = device.fldcnt
  deviceIsELRS_TX = device.isElrs and devId == TX_DEVICE_ID or nil -- ELRS and ID is TX module

  allocateFields()
  reloadAllField()
  return true
end

returnToDashboard = function()
  page = PAGE_DASHBOARD
  edit = nil
  fieldPopup = nil
  currentFolderId = nil
  currentFolderName = nil

  local tx = getDevice(TX_DEVICE_ID)
  if tx then
    changeDeviceId(TX_DEVICE_ID)
  else
    deviceId = TX_DEVICE_ID
    deviceName = ""
    deviceIsELRS_TX = nil
    fields_count = 0
    fields = {}
    loadQ = {}
  end
  txRefreshAt = 0
  dashboardRedrawAt = 0
end

local function parseDeviceInfoMessage(data)
  local id = string.byte(data, 2)
  local newName, offset = fieldGetStrOrOpts(data, 3)
  local count = string.byte(data, offset + 12)
  if not count then return end
  local device = getDevice(id)
  if device == nil then
    if #devices >= MAX_DEVICES then
      uiNotice = "Device limit: 8"
      uiNoticeUntil = getTime() + 300
      return
    end
    device = { id = id }
    devices[#devices + 1] = device
  end
  device.name = string.sub(newName, 1, 32)
  device.shortName = #newName > 18 and string.sub(newName, 1, 18) or newName
  device.fldcnt = count
  if device.fldcnt > MAX_FIELDS then
    device.fldcnt = 0
    uiNotice = "Field limit: 128"
    uiNoticeUntil = getTime() + 300
  end
  device.isElrs = fieldGetValue(data, offset, 4) == 0x454C5253 -- SerialNumber = 'E L R S'

  if deviceId == id then
    changeDeviceId(id)
  end
end

local functions = {
  { load=fieldIntLoad, save=fieldIntSave, display=fieldIntDisplay }, --1 UINT8(0)
  false, --2 INT8(1)
  false, --3 UINT16(2)
  false, --4 INT16(3)
  nil,
  nil,
  nil,
  nil,
  { load=fieldFloatLoad, save=fieldIntSave, display=fieldFloatDisplay },  --9 FLOAT(8)
  { load=fieldTextSelLoad, save=fieldIntSave, display=fieldTextSelDisplay }, --10 SELECT(9)
  { load=fieldStringLoad, save=nil, display=fieldStringDisplay }, --11 STRING(10) editing NOTIMPL
  { load=nil, save=fieldFolderOpen, display=fieldFolderDisplay }, --12 FOLDER(11)
  false, --13 INFO(12)
  { load=fieldCommandLoad, save=fieldCommandSave, display=fieldCommandDisplay }, --14 COMMAND(13)
  { load=nil, save=fieldBackExec, display=fieldCommandDisplay }, --15 back/exit(14)
}
-- Integer types and informational strings share their handlers.
functions[2] = functions[1]
functions[3] = functions[1]
functions[4] = functions[1]
functions[13] = functions[11]

local function cacheFieldValue(field)
  if field.type <= 3 then
    field.cachedValue = tostring(field.value) .. (field.unit or "")
  elseif field.type == 8 then
    field.cachedValue = string.format(field.fmt, field.value / field.prec) ..
                        (field.unit or "")
  elseif field.type == 9 then
    field.cachedValue = fieldChoice(field, field.value) .. (field.unit or "")
  elseif field.type == 10 or field.type == 12 then
    field.cachedValue = field.value
  end
end

local function decodeField(field, data)
  local parent, kind = string.byte(data, 1, 2)
  if not kind or parent > fields_count or parent == field.id then error("Invalid parent") end
  field.parent = parent ~= 0 and parent or nil
  field.type = bit32.band(kind, 0x7f)
  field.hidden = bit32.btest(kind, 0x80) or nil
  local offset
  field.name, offset = fieldGetStrOrOpts(data, 3)
  field.name = string.sub(field.name, 1, 32)
  local handler = field.type < 14 and functions[field.type+1]
  if not handler then field.hidden = true; return end
  if handler.load then handler.load(field, data, offset) end
  if field.type == 9 and (not field.value or field.value > field.max) then error("Invalid choice") end
  if field.min == 0 then field.min = nil end
  if field.max == 0 and field.type ~= 9 then field.max = nil end
end

local function rejectField(fieldId, message)
  local field = fields[fieldId]
  if field and (field.ready or field.cachedValue ~= nil) then
    field.stale = true
  else
    field = discardDetails(field or { id=fieldId })
    field.id = fieldId
    field.name = field.name or ("Field " .. fieldId)
    field.type = field.type or 12
    field.rejected = true
    fields[fieldId] = field
  end
  fieldPopup = nil
  fieldData = nil
  fieldChunk = 0
  expectChunksRemain = -1
  fieldRetries[fieldId] = nil
  removeRequest(fieldId)
  uiNotice = message
  uiNoticeUntil = getTime() + 300
end

local function retryField(fieldId, message)
  fieldData = nil
  fieldChunk = 0
  expectChunksRemain = -1

  local retries = (fieldRetries[fieldId] or 0) + 1
  fieldRetries[fieldId] = retries
  print(string.format("HELM: field %u decode failed (%u/%u): %s",
                      fieldId, retries, MAX_FIELD_RETRIES, tostring(message)))

  if retries >= MAX_FIELD_RETRIES then
    rejectField(fieldId, "Field " .. fieldId .. " refresh failed")
    return true, true
  end

  removeRequest(fieldId)
  loadQ[#loadQ+1] = fieldId
  fieldTimeout = getTime() + FIELD_RETRY_DELAY
  uiNotice = "Retry field " .. fieldId
  uiNoticeUntil = getTime() + 100
  return true, false
end

local function parseParameterInfoMessage(data)
  local fieldId = (fieldPopup and fieldPopup.id) or loadQ[#loadQ]
  if string.byte(data, 2) ~= deviceId or string.byte(data, 3) ~= fieldId then
    return nil, false
  end
  local field = fields[fieldId]
  local chunksRemain = string.byte(data, 4)
  -- If no field or the chunksremain changed when we have data, don't continue
  if not field or (fieldData and chunksRemain ~= expectChunksRemain) then
    return nil, false
  end

  if not chunksRemain then return nil, false end
  if #(fieldData or "") + #data - 4 > MAX_FIELD_BYTES then
    rejectField(fieldId, "Field exceeds 512B")
    return true, true
  end
  fieldData = (fieldData or "") .. string.sub(data, 5)

  if chunksRemain > 0 then
    fieldChunk = fieldChunk + 1
    expectChunksRemain = chunksRemain - 1
    return nil, true
  else
    -- Field data stream is now complete, process into a field
    local decoded = { id=fieldId }
    local ok, err = pcall(decodeField, decoded, fieldData)
    if not ok then
      return retryField(fieldId, err)
    end
    cacheFieldValue(decoded)
    removeRequest(fieldId)
    fieldRetries[fieldId] = nil
    if fieldPopup then
      decoded.autoConfirm = fieldPopup.autoConfirm
      decoded.lastStatus = fieldPopup.lastStatus
      fieldPopup = decoded.status ~= 0 and decoded or nil
    end
    fields[fieldId] = decoded
    field = decoded
    if deviceId == TX_DEVICE_ID and field.type == 13 then
      local name = string.lower(field.name)
      if (name == "bind" or name == "unbind") and not quickCommands[name] then
        quickCommands[name] = fieldId
      end
    end
    if trackStatusField then trackStatusField(field) end
    field.ready = true
    if not keepField(field) then fields[fieldId] = discardDetails(field) end

    fieldChunk = 0
    fieldData = nil
    expectChunksRemain = -1

    if #loadQ == 0 and page == PAGE_DASHBOARD and
       deviceId == TX_DEVICE_ID and not fieldPopup then
      txRefreshAt = getTime() + 1000
    end

    -- Return value is if the screen should be updated
    -- If deviceId is TX module, then the Bad/Good drives the update; for other
    -- devices update each new item. and always update when the queue empties
    return deviceId ~= TX_DEVICE_ID or #loadQ == 0, true
  end
end

local function parseElrsInfoMessage(data)
  if string.byte(data, 2) ~= deviceId then return end

  local badPkt = string.byte(data, 3)
  local goodPkt = fieldGetValue(data, 4, 2)
  local newFlags = string.byte(data, 6)
  -- If flags are changing, reset the warning timeout to display/hide message immediately
  if newFlags ~= elrsFlags then
    elrsFlags = newFlags
    titleShowWarnTimeout = 0
  end
  elrsFlagsInfo = fieldGetStrOrOpts(data, 7)

  local state = (bit32.btest(elrsFlags, 1) and "C") or "-"
  goodBadPkt = string.format("%u/%u   %s", badPkt, goodPkt, state)
end

local function refreshNext(skipPush)
  local command, data, forceRedraw
  -- Parameter reads refresh values after writes. 0x2D write acknowledgments need no action.
  repeat
    command, data = crossfireTelemetryPop()
    if data and (command == 0x29 or command == 0x2B or command == 0x2E) then
      local packet = ""
      for i = 1, #data do packet = packet .. string.char(data[i]) end
      data = packet
    end
    if command == 0x29 then
      pcall(parseDeviceInfoMessage, data)
    elseif command == 0x2B then
      local redraw, requestNext = parseParameterInfoMessage(data)
      if redraw then
        forceRedraw = true
      end
      if requestNext then
        if #loadQ > 0 then
          fieldTimeout = 0 -- request next chunk immediately
        elseif fieldPopup then
          fieldTimeout = getTime() + fieldPopup.timeout
        end
      end
    elseif command == 0x2E then
      pcall(parseElrsInfoMessage, data)
      forceRedraw = true
    end
  until command == nil

  if skipPush then return forceRedraw end

  if not crsfAvailable or not rfSelected then
    return forceRedraw
  end

  local time = getTime()
  if fieldPopup then
    if time > fieldTimeout and fieldPopup.status ~= 3 then
      crossfireTelemetryPush(0x2D, { deviceId, handsetId, fieldPopup.id, 6 }) -- lcsQuery
      fieldTimeout = time + fieldPopup.timeout
    end
  elseif time > devicesRefreshTimeout and not getDevice(TX_DEVICE_ID) then
    forceRedraw = true
    devicesRefreshTimeout = time + 100 -- 1s
    crossfireTelemetryPush(0x28, { 0x00, 0xEA })
  elseif page == PAGE_DASHBOARD and deviceId == TX_DEVICE_ID and
         txRefreshAt > 0 and time > txRefreshAt and #loadQ == 0 then
    reloadAllField()
    txRefreshAt = 0
    fieldTimeout = time + 20
    devicesRefreshTimeout = time + 1000
    crossfireTelemetryPush(0x28, { 0x00, 0xEA })
    forceRedraw = true
  elseif #loadQ > 0 and fields_count ~= 0 then
    if time > fieldTimeout then
      local queued = crossfireTelemetryPush(
        0x2C, { deviceId, handsetId, loadQ[#loadQ], fieldChunk })
      if queued then
        fieldTimeout = time + (deviceIsELRS_TX and 50 or 500) -- 0.5s for local / 5s for remote devices
      else
        fieldTimeout = time + 1
      end
    end
  elseif time > linkstatTimeout then
    if deviceIsELRS_TX then
      crossfireTelemetryPush(0x2D, { deviceId, handsetId, 0x0, 0x0 }) --request linkstat
    else
      goodBadPkt = ""
    end
    linkstatTimeout = time + 100
  end

  if time > titleShowWarnTimeout then
    -- if elrsFlags bit set is bit higher than bit 0 and bit 1, it is warning flags
    titleShowWarn = (elrsFlags > 3 and not titleShowWarn) or nil
    titleShowWarnTimeout = time + 100
    forceRedraw = true
  end

  return forceRedraw
end

local function updateCrsfModuleState()
  local time = getTime()
  if time < crsfCheckAt then return end
  crsfCheckAt = time + 100

  local available = false
  for modIdx = 0, 1 do
    local mod = model.getModule(modIdx)
    if mod and mod.Type == 5 then
      available = true
      break
    end
  end

  if available and not crsfAvailable then
    devicesRefreshTimeout = 0
    txRefreshAt = 0
  elseif not available then
    goodBadPkt = ""
  end
  crsfAvailable = available
end

local function updateActionPulses()
  local time = getTime()
  if bindPulseEnd > 0 and time >= bindPulseEnd then
    setStickySwitch(BIND_STICKY_ID, false)
    bindPulseEnd = 0
  end
  if scanPulseEnd > 0 and time >= scanPulseEnd then
    setStickySwitch(VRX_SCAN_STICKY_ID, false)
    scanPulseEnd = 0
  end
  if unbindPulseEnd > 0 and time >= unbindPulseEnd then
    setStickySwitch(UNBIND_STICKY_ID, false)
    unbindPulseEnd = 0
  end
end

local function stopActionPulses()
  setStickySwitch(BIND_STICKY_ID, false)
  setStickySwitch(VRX_SCAN_STICKY_ID, false)
  setStickySwitch(UNBIND_STICKY_ID, false)
  bindPulseEnd = 0
  scanPulseEnd = 0
  unbindPulseEnd = 0
end

local function dashboardDeviceCount()
  return #devices + (joystickReady and 1 or 0)
end

local function dashboardDevice(index)
  if joystickReady then
    if index == 1 then return nil, "VRX", true end
    index = index - 1
  end
  local device = devices[index]
  if device then return device, device.shortName or device.name, false end
end

local function statusValue(field)
  local value = field.value
  if field.type == 9 and field.values then
    value = fieldChoice(field, field.value or 0)
  end

  if type(value) == "number" then return value ~= 0 end
  if type(value) ~= "string" then return nil end

  local text = string.lower(value)
  if text == "0" or text == "off" or text == "disabled" or
     text == "inactive" or text == "stopped" or
     string.find(text, "disabled", 1, true) or
     string.find(text, "inactive", 1, true) or
     string.find(text, "stopped", 1, true) then
    return false
  end
  if text == "1" or text == "on" or text == "enabled" or
     text == "active" or text == "running" or
     string.find(text, "enabled", 1, true) or
     string.find(text, "running", 1, true) then
    return true
  end
  return nil
end

trackStatusField = function(field)
  if not field.name or not field.type or
     (field.type > 10 and field.type ~= 12) then
    return
  end

  local name = string.lower(field.name)
  local encryptionRank = 0
  if name == "encryption" then
    encryptionRank = 2
  elseif string.find(name, "encrypt", 1, true) then
    encryptionRank = 1
  end
  if encryptionRank >= encryptionStateRank and encryptionRank > 0 then
    encryptionStateRank = encryptionRank
    encryptionState = statusValue(field)
  end

  local newWifiRank = 0
  if name == "wifi state" then
    newWifiRank = 2
  elseif string.find(name, "wifi", 1, true) then
    newWifiRank = 1
  end
  if newWifiRank >= wifiStateRank and newWifiRank > 0 then
    wifiStateRank = newWifiRank
    wifiState = statusValue(field)
  end
end

local function receiverLqBars()
  local value = getRSSI()
  if type(value) ~= "number" then return 0 end
  if value <= 0 then return 0 end
  if value < 40 then return 1 end
  if value < 60 then return 2 end
  if value < 80 then return 3 end
  return 4
end

local function drawUnknown(x)
  lcd.drawText(x + 1, 1, "?", INVERS)
end

local function drawMaskIcon(x, y, icon)
  for row = 1, #icon do
    local mask = string.byte(icon, row)
    local runStart = nil
    for column = 0, 8 do
      local set = column < 8 and
                  bit32.btest(mask, bit32.lshift(1, column))
      if set and runStart == nil then
        runStart = column
      elseif not set and runStart ~= nil then
        lcd.drawFilledRectangle(x + runStart, y + row - 1,
                                column - runStart, 1, ERASE)
        runStart = nil
      end
    end
  end
end

local function drawJoystickIcon(x)
  lcd.drawRectangle(x, 4, 9, 4, ERASE)
  lcd.drawLine(x + 4, 1, x + 4, 5, SOLID, ERASE)
  lcd.drawRectangle(x + 3, 1, 3, 2, ERASE)
  lcd.drawFilledRectangle(x + 7, 5, 1, 1, ERASE)
end

local function drawLqIcon(x, bars)
  if not bars or bars <= 0 then return end
  for i = 1, 4 do
    local height = LQ_BAR_HEIGHTS[i]
    if i <= bars then
      lcd.drawFilledRectangle(x + (i - 1) * 3, 8 - height, 2, height, ERASE)
    else
      lcd.drawRectangle(x + (i - 1) * 3, 8 - height, 2, height, ERASE)
    end
  end
end

local function drawDashboard()
  lcd.clear()
  lcd.drawFilledRectangle(0, 0, LCD_W, 9, FORCE)

  if joystickReady then drawJoystickIcon(3) end

  if wifiState == nil then
    drawUnknown(29)
  else
    drawMaskIcon(29, 1, WIFI_ICON)
    if not wifiState then lcd.drawLine(29, 1, 36, 7, SOLID, ERASE) end
  end

  if encryptionState == nil then
    drawUnknown(55)
  else
    drawMaskIcon(55, 1,
                 encryptionState and ENCRYPTION_ON_ICON or ENCRYPTION_OFF_ICON)
  end

  drawMaskIcon(81, 1, ANTENNA_ICON)
  if not rfSelected then lcd.drawLine(81, 1, 88, 7, SOLID, ERASE) end

  drawLqIcon(104, receiverLqBars())

  lcd.drawText(7, 13, "[Bind]", BOLD + (dashboardCursor == 1 and INVERS or 0))
  lcd.drawText(67, 13, "[Unbind]", BOLD + (dashboardCursor == 2 and INVERS or 0))
  lcd.drawText(7, 22, rfSelected and "[x] RF" or "[ ] RF",
               dashboardCursor == 3 and INVERS or 0)
  lcd.drawText(67, 22, usbSelected and "[x] USB" or "[ ] USB",
               dashboardCursor == 4 and INVERS or 0)

  for row = 1, DASHBOARD_DEVICE_ROWS do
    local index = dashboardDeviceOffset + row
    local _, name = dashboardDevice(index)
    if name then
      local y = 34 + (row - 1) * 9
      local selected = dashboardCursor == index + 4
      lcd.drawText(2, y, name, selected and INVERS or 0)
    end
  end

  if uiNotice and getTime() < uiNoticeUntil then
    lcd.drawFilledRectangle(0, 55, LCD_W, 9, GREY_DEFAULT)
    lcd.drawText(LCD_W / 2, 56, uiNotice, CENTER + INVERS)
  elseif uiNotice then
    uiNotice = nil
  end
end

local function findCommand(name)
  return fields[quickCommands[string.lower(name)]]
end

local function startUsbAction(name)
  local now = getTime()
  if name == "Bind" then
    setStickySwitch(BIND_STICKY_ID, true)
    bindPulseEnd = now + ACTION_PULSE_TICKS
  else
    setStickySwitch(UNBIND_STICKY_ID, true)
    unbindPulseEnd = now + ACTION_PULSE_TICKS
  end
end

local function startQuickAction(name)
  if usbSelected then startUsbAction(name) end

  if rfSelected then
    local command = crsfAvailable and findCommand(name) or nil
    if command and command.status ~= nil then
      startFieldCommand(command, true)
    else
      print("HELM: RF " .. name .. " unavailable")
    end
  end

  if not usbSelected and not rfSelected then
    print("HELM: " .. name .. " ignored because all outputs are disabled")
  end
end

local function loadRadioSettings()
  local ok, settings = pcall(getRadioSettings)
  if not ok or type(settings) ~= "table" then
    print("HELM: failed to load radio settings: " .. tostring(settings))
    return
  end

  rfSelected = settings.internalModuleEnabled ~= false
  usbSelected = settings.usbMode ~= "serial"
end

local function saveRadioSetting(name, value)
  local values = { [name] = value }
  local ok, accepted = pcall(setRadioSettings, values)
  if not ok or not accepted then
    print("HELM: failed to save " .. name .. ": " .. tostring(accepted))
    return false
  end
  return true
end

local function updateDashboardOffset()
  if dashboardCursor <= 4 then return end
  local index = dashboardCursor - 4
  if index <= dashboardDeviceOffset then
    dashboardDeviceOffset = index - 1
  elseif index > dashboardDeviceOffset + DASHBOARD_DEVICE_ROWS then
    dashboardDeviceOffset = index - DASHBOARD_DEVICE_ROWS
  end
end

local function updateJoystickReady()
  local ready = isUsbJoystickReady() == true
  if ready == joystickReady then return false end

  if page == PAGE_DASHBOARD and dashboardCursor > 4 then
    if ready then
      dashboardCursor = dashboardCursor + 1
    elseif dashboardCursor > 5 then
      dashboardCursor = dashboardCursor - 1
    end
  end

  joystickReady = ready
  local maxCursor = 4 + dashboardDeviceCount()
  if dashboardCursor > maxCursor then dashboardCursor = maxCursor end
  updateDashboardOffset()
  dashboardRedrawAt = 0
  return true
end

local function handleDashboardEvent(event)
  local maxCursor = 4 + dashboardDeviceCount()
  if event == EVT_VIRTUAL_EXIT then
    stopActionPulses()
    exitscript = 2
  elseif event == EVT_VIRTUAL_NEXT or event == EVT_VIRTUAL_NEXT_REPT then
    dashboardCursor = dashboardCursor >= maxCursor and 1 or dashboardCursor + 1
    updateDashboardOffset()
  elseif event == EVT_VIRTUAL_PREV or event == EVT_VIRTUAL_PREV_REPT then
    dashboardCursor = dashboardCursor <= 1 and maxCursor or dashboardCursor - 1
    updateDashboardOffset()
  elseif event == EVT_VIRTUAL_ENTER then
    if dashboardCursor == 1 then
      startQuickAction("Bind")
    elseif dashboardCursor == 2 then
      quickConfirm = "Unbind"
      quickConfirmYes = true
    elseif dashboardCursor == 3 then
      local enabled = not rfSelected
      if saveRadioSetting("internalModuleEnabled", enabled) then
        rfSelected = enabled
        devicesRefreshTimeout = 0
        txRefreshAt = 0
        uiNotice = enabled and "RF ON" or "RF OFF"
      else
        uiNotice = "RF SAVE ERROR"
      end
      uiNoticeUntil = getTime() + 100
    elseif dashboardCursor == 4 then
      local enabled = not usbSelected
      local mode = enabled and "joystick" or "serial"
      if saveRadioSetting("usbMode", mode) then
        usbSelected = enabled
        uiNotice = "RECONNECT USB"
      else
        uiNotice = "USB SAVE ERROR"
      end
      uiNoticeUntil = getTime() + 100
    else
      local device, _, isVrx = dashboardDevice(dashboardCursor - 4)
      if isVrx then
        page = PAGE_VRX
      elseif device then
        changeDeviceId(device.id)
        page = PAGE_DEVICE
        lineIndex = 1
        pageOffset = 0
        edit = nil
      end
    end
  end
end

local function runDashboard(event)
  handleDashboardEvent(event)
  drawDashboard()
end

local function runVrxPage(event)
  if event == EVT_VIRTUAL_EXIT then
    returnToDashboard()
  elseif event == EVT_VIRTUAL_ENTER and joystickReady and scanPulseEnd == 0 then
    setStickySwitch(VRX_SCAN_STICKY_ID, true)
    scanPulseEnd = getTime() + ACTION_PULSE_TICKS
  end

  lcd.clear()
  lcd.drawFilledRectangle(0, 0, LCD_W, 9, GREY_DEFAULT)
  lcd.drawText(COL1, 1, "VRX", INVERS)

  if scanPulseEnd > 0 then
    lcd.drawText(COL1, 13, "Scanning", INVERS)
  else
    lcd.drawText(COL1, 13, "[ Scan ]", joystickReady and INVERS or 0)
    if not joystickReady then
      lcd.drawText(COL1, 31, "USB disconnected")
    end
  end
end

local function runQuickConfirmation(event)
  drawDashboard()

  lcd.drawFilledRectangle(11, 10, 106, 45, ERASE)
  lcd.drawRectangle(12, 11, 104, 43)
  lcd.drawText(LCD_W / 2, 14, "UNBIND", BOLD + CENTER)
  lcd.drawText(LCD_W / 2, 26, "Generate new UID?", CENTER)
  lcd.drawText(39, 42, "YES", quickConfirmYes and INVERS or 0)
  lcd.drawText(78, 42, "NO", quickConfirmYes and 0 or INVERS)

  if event == EVT_VIRTUAL_PREV or event == EVT_VIRTUAL_PREV_REPT then
    quickConfirmYes = true
  elseif event == EVT_VIRTUAL_NEXT or event == EVT_VIRTUAL_NEXT_REPT then
    quickConfirmYes = false
  elseif event == EVT_VIRTUAL_ENTER then
    local confirmed = quickConfirmYes
    quickConfirm = nil
    if confirmed then startQuickAction("Unbind") end
  elseif event == EVT_VIRTUAL_EXIT then
    quickConfirm = nil
  end
end

local function lcd_title()
  lcd.clear()
  -- B&W screen
  local barHeight = 9
  if not titleShowWarn then
    lcd.drawText(LCD_W - 1, 1, goodBadPkt, RIGHT)
    lcd.drawLine(LCD_W - 10, 0, LCD_W - 10, barHeight-1, SOLID, INVERS)
  end

  if #loadQ > 0 and fields_count > 0 then
    lcd.drawFilledRectangle(COL2, 0, LCD_W, barHeight, GREY_DEFAULT)
    lcd.drawGauge(0, 0, COL2, barHeight, fields_count - #loadQ, fields_count, 0)
  else
    lcd.drawFilledRectangle(0, 0, LCD_W, barHeight, GREY_DEFAULT)
    if titleShowWarn then
      lcd.drawText(COL1, 1, elrsFlagsInfo, INVERS)
    else
      -- Due to space, show the folder name, or if top level, deviceName
      lcd.drawText(COL1, 1, currentFolderName or deviceName, INVERS)
    end
  end
end

local function lcd_warn()
  lcd.drawText(COL1, textSize*2, "Error:")
  lcd.drawText(COL1, textSize*3, elrsFlagsInfo)
  lcd.drawText(LCD_W/2, textSize*5, "[OK]", BLINK + INVERS + CENTER)
end

local function reloadRelatedFields(field)
  -- Reload the parent folder to update the description
  if field.parent then
    reloadField(fields[field.parent])
    fields[field.parent].name = nil
  end

  -- Reload all editable fields at the same level as well as the parent item
  for fieldId = fields_count, 1, -1 do
    -- Skip this field, will be added to end
    local fldTest = fields[fieldId]
    local fldType = fldTest.type or 99 -- type could be nil if still loading
    if fieldId ~= field.id
      and fldTest.parent == field.parent
      and (fldType < 11 or fldType == 12 or fldType == 13) then -- ignores FOLDER/devices/EXIT
      reloadField(fldTest)
    end
  end

  -- Reload this field
  reloadField(field)
  -- with a short delay to allow the module EEPROM to commit
  fieldTimeout = getTime() + 20
  -- Also push the next bad/good update further out
  linkstatTimeout = fieldTimeout + 100
end

local function handleDevicePageEvent(event)
  if #fields == 0 then --if there is no field yet
    return
  else
    if fields[#fields].name == nil then --if back button is not assigned yet, means there is no field yet.
      return
    end
  end

  if event == EVT_VIRTUAL_EXIT then
    if edit then
      edit = nil
      reloadCurField()
    elseif currentFolderId == nil then
      returnToDashboard()
    else
      fieldBackExec(fields[#fields])
    end
  elseif event == EVT_VIRTUAL_ENTER then -- toggle editing/selecting current field
    if elrsFlags > 0x1F then
      elrsFlags = 0
      crossfireTelemetryPush(0x2D, { deviceId, handsetId, 0x2E, 0x00 })
    else
      local field = getField(lineIndex)
      if field and field.name and not field.rejected and (field.ready or field.type == 11 or field.type == 14) then
        -- Editable fields
        if not field.grey and field.type < 10 then
          edit = not edit
          if not edit then
            reloadRelatedFields(field)
          end
        end
        if not edit then
          if functions[field.type+1].save then
            functions[field.type+1].save(field)
          end
        end
      end
    end
  elseif edit then
    if event == EVT_VIRTUAL_NEXT then
      incrField(1)
    elseif event == EVT_VIRTUAL_PREV then
      incrField(-1)
    end
  else
    if event == EVT_VIRTUAL_NEXT then
      selectField(1)
    elseif event == EVT_VIRTUAL_PREV then
      selectField(-1)
    end
  end
end

-- Main
local function runDevicePage(event)
  handleDevicePageEvent(event)

  -- Cache only the visible rows and quick commands. Keep the full field index.
  for i = 1, fields_count do
    local field = fields[i]
    if field.ready and not keepField(field) then fields[i] = discardDetails(field) end
  end
  for row = (fieldPopup and 0 or maxLineIndex + 1), 1, -1 do
    local field = getField(pageOffset + row)
    if field and field.id and not field.ready and not field.rejected and field.type ~= 11 then
      local queued = false
      for i = 1, #loadQ do
        if loadQ[i] == field.id then queued = true; break end
      end
      if not queued or event ~= 0 then reloadField(field) end
    end
  end

  lcd_title()

  if elrsFlags > 0x1F then
    lcd_warn()
  else
    for y = 1, maxLineIndex+1 do
      local field = getField(pageOffset+y)
      if not field then
        break
      elseif field.name ~= nil then
        local attr = lineIndex == (pageOffset+y)
          and ((edit and BLINK or 0) + INVERS)
          or 0
        if field.type < 11 or field.type == 12 then -- if not folder, command, or back
          lcd.drawText(COL1, y*textSize+textYoffset, field.name, 0)
        end
        if field.rejected then
          lcd.drawText(COL2, y*textSize+textYoffset, "ERR", attr)
        elseif (field.ready or field.type == 11 or field.type == 14) and functions[field.type+1] and functions[field.type+1].display then
          functions[field.type+1].display(field, y*textSize+textYoffset, attr)
        elseif field.cachedValue ~= nil then
          lcd.drawText(COL2, y*textSize+textYoffset, field.cachedValue, attr)
        end
      end
    end
  end
  if uiNotice and getTime() < uiNoticeUntil then
    lcd.drawFilledRectangle(0, 55, LCD_W, 9, GREY_DEFAULT)
    lcd.drawText(LCD_W / 2, 56, uiNotice, CENTER + INVERS)
  end
end

local function runPopupPage(event)
  if event == EVT_VIRTUAL_EXIT then
    crossfireTelemetryPush(0x2D, { deviceId, handsetId, fieldPopup.id, 5 }) -- lcsCancel
    fieldTimeout = getTime() + 200 -- 2s
  end

  if fieldPopup.status == 0 and fieldPopup.lastStatus ~= 0 then -- stopped
      popupConfirmation(fieldPopup.info, "Stopped!", event)
      reloadAllField()
      fieldPopup = nil
  elseif fieldPopup.status == 3 then -- confirmation required
    if fieldPopup.autoConfirm then
      crossfireTelemetryPush(0x2D, { deviceId, handsetId, fieldPopup.id, 4 }) -- lcsConfirmed
      fieldTimeout = getTime() + fieldPopup.timeout -- we are expecting an immediate response
      fieldPopup.status = 4
    else
      local result = popupConfirmation(fieldPopup.info, "PRESS [OK] to confirm", event)
      fieldPopup.lastStatus = fieldPopup.status
      if result == "OK" then
        crossfireTelemetryPush(0x2D, { deviceId, handsetId, fieldPopup.id, 4 }) -- lcsConfirmed
        fieldTimeout = getTime() + fieldPopup.timeout -- we are expecting an immediate response
        fieldPopup.status = 4
      elseif result == "CANCEL" then
        fieldPopup = nil
      end
    end
  elseif fieldPopup.status == 2 then -- running
    if fieldChunk == 0 then
      commandRunningIndicator = (commandRunningIndicator % 4) + 1
    end
    local result = popupConfirmation(fieldPopup.info .. " [" .. string.sub("|/-\\", commandRunningIndicator, commandRunningIndicator) .. "]", "Press [RTN] to exit", event)
    fieldPopup.lastStatus = fieldPopup.status
    if result == "CANCEL" then
      crossfireTelemetryPush(0x2D, { deviceId, handsetId, fieldPopup.id, 5 }) -- lcsCancel
      fieldTimeout = getTime() + fieldPopup.timeout -- we are expecting an immediate response
      fieldPopup = nil
    end
  end
end

-- Init
local function init()
  stopActionPulses()
  devices = {}
  page = PAGE_DASHBOARD
  dashboardCursor = 1
  dashboardDeviceOffset = 0
  rfSelected = true
  usbSelected = true
  joystickReady = isUsbJoystickReady() == true
  loadRadioSettings()
  exitscript = 0
  devicesRefreshTimeout = 0
  txRefreshAt = 0
  crsfCheckAt = 0
  updateCrsfModuleState()
end

-- Main
local function run(event)
  if event == nil then
    stopActionPulses()
    return 2
  end

  updateCrsfModuleState()
  local usbChanged = updateJoystickReady()
  updateActionPulses()
  -- If ENTER pressed, skip any pushing this loop to reserve queue for the save command
  local forceRedraw = refreshNext(event == EVT_VIRTUAL_ENTER)
  forceRedraw = forceRedraw or usbChanged

  if fieldPopup ~= nil then
    runPopupPage(event)
  elseif quickConfirm then
    runQuickConfirmation(event)
  elseif page == PAGE_DASHBOARD then
    if event ~= 0 or forceRedraw or getTime() >= dashboardRedrawAt then
      runDashboard(event)
      dashboardRedrawAt = getTime() + 10
    end
  elseif page == PAGE_VRX then
    if event ~= 0 or forceRedraw or scanPulseEnd > 0 or
       getTime() >= dashboardRedrawAt then
      runVrxPage(event)
      dashboardRedrawAt = getTime() + 10
    end
  elseif event ~= 0 or forceRedraw or edit or getTime() >= dashboardRedrawAt then
    runDevicePage(event)
    dashboardRedrawAt = getTime() + 10
  end

  return exitscript
end

return { init=init, run=run }
