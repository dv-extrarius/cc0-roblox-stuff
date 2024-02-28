--[[
Copyright Extrarius 2024.
This work is marked with CC0 1.0 Universal.
To view a copy of this license, visit http://creativecommons.org/publicdomain/zero/1.0
--
This is a collection of miscellaneous functions I use to visualize debug information.
--]]
--!strict
--!optimize 2
local DebugHelp = {}


--
function DebugHelp.SaveInsances(t: {Instance}, ...:Instance)
    for _, inst in {...} do
        table.insert(t, inst)
    end
end


--
function DebugHelp.MakeBall(pos: Vector3, params: {[string]: any}?): Part
    local params: {[string]: any} = params or {}
    local radius = params.Radius or 1
    local color = params.Color or Color3.new(1, 0, 0)
    local transparency = params.Transparency or 0

    local ball: Part = Instance.new("Part")
    ball.Archivable = false
    ball.Shape = Enum.PartType.Ball
    ball.CastShadow = false
    ball.Size = Vector3.new(radius, radius, radius)
    ball.BottomSurface = Enum.SurfaceType.Smooth
    ball.TopSurface = Enum.SurfaceType.Smooth
    ball.Position = pos
    ball.Anchored = true
    ball.CanCollide = false
    ball.CanQuery = false
    ball.CanTouch = false
    ball.Color = color
    ball.Transparency = transparency
    ball.Parent = workspace.Terrain
    return ball
end


--
function DebugHelp.MakeBeam(start: Vector3, finish: Vector3, params: {[string]: any}?): (Attachment, Attachment, Beam)
    local params: {[string]: any} = params or {}
    local transparency0 = params.Transparency0 or params.Transparency or 0.5
    local transparency1 = params.Transparency1 or params.Transparency or 0.5
    local color0 = params.Color0 or params.Color or Color3.new(0.5, 1, 0.5)
    local color1 = params.Color1 or params.Color  or Color3.new(1, 0.5, 1)
    local size0 = params.Size0 or params.Size or 1
    local size1 = params.Size1 or params.Size or 1
    local faceCamera = if params.FaceCamera ~= nil then (params.FaceCamera and true or false) else true

    local a0 = Instance.new("Attachment")
    local a1 = Instance.new("Attachment")
    a0.Archivable = false
    a1.Archivable = false

    --Orient the attachments such that the beam will be visible from above
    local delta = (finish - start).Unit
    local yAxis = delta:Cross(Vector3.yAxis)
    a0.CFrame = CFrame.lookAlong(start, -delta, yAxis)
    a1.CFrame = CFrame.lookAlong(finish, delta, yAxis)

    a0.Parent = workspace.Terrain
    a1.Parent = workspace.Terrain

    local beam = Instance.new("Beam")
    beam.Archivable = false
    beam.Attachment0 = a0
    beam.Attachment1 = a1
    beam.FaceCamera = faceCamera
    beam.Width0 = size0
    beam.Width1 = size1
    beam.Color = ColorSequence.new(color0, color1)
    beam.Transparency = NumberSequence.new(transparency0, transparency1)
    beam.Parent = workspace.Terrain
    return a0, a1, beam
end


--
function DebugHelp.MakeLabel(center: Vector3, text: any, params: {[string]: any}?): (Part)
    local params: {[string]: any} = params or {}

    local part = Instance.new("Part")
    part.Archivable = false
    part.Transparency = 1
    part.CFrame = CFrame.new(center)
    part.CastShadow = false
    part.CanCollide = false
    part.CanQuery = false
    part.CanTouch = false
    part.Anchored = true
    part.Size = Vector3.new(1, 0, 1)

    local gui = Instance.new("SurfaceGui")
    gui.Archivable = false
    gui.LightInfluence = 0
    gui.ClipsDescendants = false
    gui.Face = Enum.NormalId.Top
    gui.MaxDistance = 1000
    gui.ZIndexBehavior = Enum.ZIndexBehavior.Sibling
    gui.SizingMode = Enum.SurfaceGuiSizingMode.PixelsPerStud
    gui.PixelsPerStud = 50
    gui.ZOffset = 1

    local textbox = Instance.new("TextLabel")
    textbox.Archivable = false
    textbox.AutomaticSize = Enum.AutomaticSize.XY
    textbox.AnchorPoint = Vector2.new(0.5, 0.5)
    textbox.BorderSizePixel = 1
    textbox.Position = UDim2.fromScale(0.5, 0.5)
    textbox.FontFace = Font.new("rbxasset://fonts/families/Roboto.json", Enum.FontWeight.Bold, Enum.FontStyle.Normal)
    textbox.Text = tostring(text)
    textbox.TextSize = 36
    textbox.BackgroundColor3 = Color3.new(1, 1, 1)
    textbox.BorderColor3 = Color3.new(0, 0, 0)

    local padding = Instance.new("UIPadding")
    padding.Archivable = false
    padding.PaddingBottom = UDim.new(0, 5)
    padding.PaddingLeft = UDim.new(0, 5)
    padding.PaddingRight = UDim.new(0, 5)
    padding.PaddingTop = UDim.new(0, 5)

    padding.Parent = textbox
    textbox.Parent = gui
    gui.Parent = part
    part.Parent = workspace.Terrain
    return part
end


--Sums a sequence in a way that reduces precision loss when values differ in magnitude
local function KahanSum(values: {number}): number
    local sum = 0
    local compensator = 0
    for _, value in values do
        local y = value - compensator
        local t = sum + y
        compensator = (t - sum) - y
        sum = t
    end
    return sum
end
DebugHelp.Sum = KahanSum


--
function DebugHelp.HistogramString(values: {number}): string
    local histogram = {}
    values = table.clone(values)
    table.sort(values)

    local count = #values
    local range = (count - 1)

    --Getting the number magnitude from the middle seems to give the best results
    local digitIndex = range // 2 + 1
    while digitIndex < count and values[digitIndex] == 0 do
        digitIndex = count - ((count - digitIndex) // 2)
    end

    if values[digitIndex] == 0 then
        return string.format("[%5d](    0.000); [Total =    0.0000000000]", count)
    end

    local numDigits = (math.log10(values[digitIndex]) // 3) * 3
    numDigits = math.clamp(numDigits, -12, 12)

    local suffix = ""
    if numDigits < 0 then
        suffix = ({"m", "μ", "n", "p"})[numDigits / -3]
    elseif numDigits > 0 then
        suffix = ({"K", "M", "G", "T"})[numDigits / 3]
    end
    local multiplier = math.pow(10, -numDigits)

    local steps: {number}
    if count >= 250 then
        steps = {0, 5, 10, 25, 35, 50, 65, 75, 90, 95, 100}
    elseif count >= 50 then
        steps = {0, 10, 25, 50, 75, 90, 100}
    else
        steps = {0, 25, 50, 75, 100}
    end
    for _, s in steps do
        local index = ((s / 100) * range) + 1
        local low = math.floor(index)
        local high = math.ceil(index)
        local blend = index - low
        if blend <= 0.0001 then
            table.insert(histogram, string.format("%#8.3f%s", values[low] * multiplier, suffix))
        else
            table.insert(histogram, string.format("%#8.3f%s", (values[low] * (1 - blend) + values[high] * blend) * multiplier, suffix))
        end
    end
    local suffixExplanation = ""
    if suffix ~= "" then
        suffixExplanation = string.format("[%s=10^%d]", suffix, numDigits)
    end
    return string.format("[%5d](%s)%s; [Total = %#15.10f]", count, table.concat(histogram, ", "), suffixExplanation, KahanSum(values))
end


--
return table.freeze(DebugHelp)