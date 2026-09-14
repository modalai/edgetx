-- TNS|Joystick|TNE

local activeMode = nil
local activeEndTime = 0
local spinIdx = 1
local cursorIdx = 0

local COL1, textSize, textYoffset

-- Bind/Unbind/VRX Scan map to sticky switches L1/L3/L2 respectively
-- (model mixes wire L1->BIND, L2->VRXSCAN, L3->UNBIND)
local STICKY_IDX = { [0] = 0, [1] = 2, [2] = 1 }

local function lcd_title()
  if lcd.RGB then
    lcd.clear()
    local EGREEN = lcd.RGB(0x9f, 0xc7, 0x6f)
    lcd.setColor(CUSTOM_COLOR, EGREEN)
    lcd.drawRectangle(0, 0, LCD_W, LCD_H, CUSTOM_COLOR)
    lcd.drawRectangle(1, 0, LCD_W - 2, LCD_H - 1, CUSTOM_COLOR)
    lcd.drawFilledRectangle(0, 0, LCD_W, 30, CUSTOM_COLOR)
    lcd.setColor(CUSTOM_COLOR, BLACK)
    lcd.drawText(COL1 + 1, 4, "Joystick", CUSTOM_COLOR)
  else
    lcd.clear()
    lcd.drawFilledRectangle(0, 0, LCD_W, 9, GREY_DEFAULT)
    lcd.drawText(COL1, 1, "Joystick", INVERS)
  end
end

local function init()
  setStickySwitch(0, false)
  setStickySwitch(1, false)
  setStickySwitch(2, false)
  activeMode = nil
  cursorIdx = 0
  if LCD_W == 480 then
    COL1 = 3; textSize = 22; textYoffset = 10
  elseif LCD_W == 320 then
    COL1 = 3; textSize = 22; textYoffset = 10
  else
    COL1 = 0; textSize = 8; textYoffset = 3
  end
end

local function run(event)
  if event == EVT_VIRTUAL_EXIT then
    setStickySwitch(0, false)
    setStickySwitch(1, false)
    setStickySwitch(2, false)
    return 2
  end

  if activeMode == nil then
    if event == EVT_VIRTUAL_NEXT or event == EVT_VIRTUAL_NEXT_REPT then
      cursorIdx = math.min(cursorIdx + 1, 2)
    elseif event == EVT_VIRTUAL_PREV or event == EVT_VIRTUAL_PREV_REPT then
      cursorIdx = math.max(cursorIdx - 1, 0)
    elseif event == EVT_VIRTUAL_ENTER then
      activeMode = cursorIdx
      activeEndTime = getTime() + 300
      spinIdx = 1
      setStickySwitch(STICKY_IDX[cursorIdx], true)
    end
  end

  if activeMode ~= nil and getTime() >= activeEndTime then
    setStickySwitch(STICKY_IDX[activeMode], false)
    activeMode = nil
  end

  lcd_title()

  if activeMode ~= nil then
    spinIdx = (spinIdx % 4) + 1
    local spin = string.sub("|/-\\", spinIdx, spinIdx)
    local labels = { [0] = "Binding", [1] = "Unbinding", [2] = "VRX Scanning" }
    popupConfirmation(labels[activeMode] .. "... [" .. spin .. "]", event)
  else
    local y1 = textSize + textYoffset
    local y2 = y1 + textSize + 4
    local y3 = y2 + textSize + 4
    local bindFlags = BOLD + (cursorIdx == 0 and INVERS or 0)
    local unbindFlags = BOLD + (cursorIdx == 1 and INVERS or 0)
    local scanFlags = BOLD + (cursorIdx == 2 and INVERS or 0)
    lcd.drawText(COL1 + 10, y1, "[Bind]", bindFlags)
    lcd.drawText(COL1 + 10, y2, "[Unbind]", unbindFlags)
    lcd.drawText(COL1 + 10, y3, "[VRX Scan]", scanFlags)
  end

  return 0
end

return { init=init, run=run }
