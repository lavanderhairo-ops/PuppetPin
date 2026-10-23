--[[
Puppet Pin - Add Room

On the Fusion page Puppet Pin's picture is exactly as big as the image plugged into it, so an arm or a
leg pulled past the edge of that image is cut off. This script gives the selected Puppet Pin node a
bigger, see-through picture to work in (a transparent Background and a Merge in front of it) and tells
Puppet Pin where the original image went, so every pin, keyframe, path and expression keeps pointing at
the same spot. The result looks the same until you pull something past the old edge.

Use: right-click the Puppet Pin node > Scripts > Puppet Pin - Add Room. Undo with Ctrl+Z.
Running it again adds more room.
]]

local ROOM = 1.6  -- the new picture is at least this many times the old one (and at least the comp frame)

local t = tool or comp.ActiveTool
if not t or t:GetAttrs().TOOLS_RegID ~= "ofx.com.puppetpin.PuppetPin" then
    print("Puppet Pin - Add Room: select a Puppet Pin node first (right-click it > Scripts).")
    return
end
local out = t.Source:GetConnectedOutput()
if not out then
    print("Puppet Pin - Add Room: connect an image to the Puppet Pin node first.")
    return
end
local time = comp.CurrentTime
local img = out:GetValue(time)
if not img then
    print("Puppet Pin - Add Room: the image going into Puppet Pin could not be read.")
    return
end
local W, H = img.Width, img.Height
local FW = tonumber(comp:GetPrefs("Comp.FrameFormat.Width")) or W
local FH = tonumber(comp:GetPrefs("Comp.FrameFormat.Height")) or H

-- the new size: room on every side, an even difference so the old image lands on whole pixels
local function grow(n, frame)
    local m = math.max(frame, math.ceil(n * ROOM))
    if (m - n) % 2 == 1 then m = m + 1 end
    return m
end
local NW, NH = grow(W, FW), grow(H, FH)
local ox, oy = (NW - W) / 2, (NH - H) / 2

-- New nodes get explicit positions next to Puppet Pin: without them Fusion would wire each new node in
-- after whatever node is active.
local px, py = 0, 0
pcall(function()
    local x, y = comp.CurrentFrame.FlowView:GetPos(t)
    if x and y then px, py = x, y end
end)

comp:Lock()
comp:StartUndo("Puppet Pin - Add Room")

local bg = comp:AddTool("Background", false, px - 2, py - 1)
bg.UseFrameFormatSettings = 0
bg.Width = NW
bg.Height = NH
bg.TopLeftRed = 0
bg.TopLeftGreen = 0
bg.TopLeftBlue = 0
bg.TopLeftAlpha = 0

local mg = comp:AddTool("Merge", false, px - 1, py - 1)
mg.Background = bg.Output
mg.Foreground = out
t.Source = mg.Output

-- where the old picture went (pixels); add to an earlier "Add Room"
local size = t:GetInput("pinFrameSize", time)
local origin = t:GetInput("pinFrameOrigin", time)
if size and size[1] and size[1] > 0 and size[2] > 0 then
    t:SetInput("pinFrameOrigin", { origin[1] + ox, origin[2] + oy })
else
    t:SetInput("pinFrameOrigin", { ox, oy })
    t:SetInput("pinFrameSize", { W, H })
end

-- Fusion's Merge centres the bigger picture where the old one was, so a Merge after Puppet Pin places it
-- the same. A Transform right after Puppet Pin moves by a fraction of the picture's width: scale its
-- move so it still moves the same number of pixels.
local fixed = 0
for _, inp in ipairs(t.Output:GetConnectedInputs() or {}) do
    local d = inp:GetTool()
    if d and d:GetAttrs().TOOLS_RegID == "Transform" then
        for _, id in ipairs({ "Center", "Pivot" }) do
            local i = d[id]
            if i and not i:GetConnectedOutput() then
                local c = d:GetInput(id, time)
                d:SetInput(id, { 0.5 + (c[1] - 0.5) * W / NW, 0.5 + (c[2] - 0.5) * H / NH })
                fixed = fixed + 1
            end
        end
    end
end

comp:EndUndo(true)
comp:Unlock()
print(string.format("Puppet Pin - Add Room: %dx%d -> %dx%d (Background %s, Merge %s)%s", W, H, NW, NH,
    bg.Name, mg.Name, fixed > 0 and string.format(", adjusted %d Transform setting(s)", fixed) or ""))
