--[[
Copyright Extrarius 2023, 2024
This work is marked with CC0 1.0 Universal.
To view a copy of this license, visit http://creativecommons.org/publicdomain/zero/1.0
--
BoxSAT - Test box-box intersection using the Separating Axis Theory
]]
--!strict
--!optimize 2

local BoxSAT = {}

--Types and values used for results
export type BoxDisposition = "Separated" | "Touching" | "Intersecting"
local DispositionSeparated: BoxDisposition = "Separated"
local DispositionTouching: BoxDisposition = "Touching"
local DispositionIntersecting: BoxDisposition = "Intersecting"

BoxSAT.Disposition = table.freeze({
    Separated = DispositionSeparated,
    Touching = DispositionTouching,
    Intersecting = DispositionIntersecting
})

export type BoxTestResult = {
    Disposition: BoxDisposition,
    PlaneNormal: Vector3,
    PlaneDistance: number,
    SeparationDistance: number,
}

local Vector3Dot = Vector3.one.Dot

--Check whether a plane with a given normal divides the two sets of points. (Disposition, PlaneDistance, SeparationDistance)
local function DoesPlaneSeparate(box1Points: {Vector3}, box2Points: {Vector3}, normal: Vector3): (BoxDisposition, number, number)
    local low1 = math.huge
    local high1 = -math.huge
    for _, point in box1Points do
        local value = Vector3Dot(point, normal)
        if value < low1 then
            low1 = value
        end
        if value > high1 then
            high1 = value
        end
    end
    local low2 = math.huge
    local high2 = -math.huge
    for _, point in box2Points do
        local value = Vector3Dot(point, normal)
        if value < low2 then
            low2 = value
        end
        if value > high2 then
            high2 = value
        end
    end
    if (low1 > high2) or (low2 > high1) then
        local planeDistance
        local separationDistance
        if low1 > high2 then
            planeDistance = (low1 + high2) * 0.5
            separationDistance = (low1 - high2)
        else
            planeDistance = (low2 + high1) * 0.5
            separationDistance = (low2 - high1)
        end
        return DispositionSeparated, planeDistance, separationDistance
    elseif (low1 == high2) or (low2 == high1) then
        local midpoint
        if low1 == high2 then
            midpoint = low1
        else
            midpoint = low2
        end
        return DispositionTouching, midpoint, 0
    else
        return DispositionIntersecting, 0, -1
    end
end

local BOX_POINT_1 = Vector3.new(0.5, 0.5, 0.5)
local BOX_POINT_2 = Vector3.new(0.5, 0.5, -0.5)
local BOX_POINT_3 = Vector3.new(0.5, -0.5, 0.5)
local BOX_POINT_4 = Vector3.new(0.5, -0.5, -0.5)
local BOX_POINT_5 = Vector3.new(-0.5, 0.5, 0.5)
local BOX_POINT_6 = Vector3.new(-0.5, 0.5, -0.5)
local BOX_POINT_7 = Vector3.new(-0.5, -0.5, 0.5)
local BOX_POINT_8 = Vector3.new(-0.5, -0.5, -0.5)

local CFramePointToWorldSpace = (CFrame.new().PointToWorldSpace) :: ((CFrame, ...Vector3) -> ...Vector3)

--Test whether two boxes intersect, touch, or are separated
local function Test(box1CFrame: CFrame, box1Size: Vector3, box2CFrame: CFrame, box2Size: Vector3): BoxTestResult
    local box1Points = {CFramePointToWorldSpace(box1CFrame, box1Size*BOX_POINT_1, box1Size*BOX_POINT_2, box1Size*BOX_POINT_3, box1Size*BOX_POINT_4, box1Size*BOX_POINT_5, box1Size*BOX_POINT_6, box1Size*BOX_POINT_7, box1Size*BOX_POINT_8)}
    local box2Points = {CFramePointToWorldSpace(box2CFrame, box2Size*BOX_POINT_1, box2Size*BOX_POINT_2, box2Size*BOX_POINT_3, box2Size*BOX_POINT_4, box2Size*BOX_POINT_5, box2Size*BOX_POINT_6, box2Size*BOX_POINT_7, box2Size*BOX_POINT_8)}

    --Cube faces and edges lie along the axis adjusted for the rotation
    local box1Axes = {box1CFrame.XVector, box1CFrame.YVector, box1CFrame.ZVector}
    local box2Axes = {box2CFrame.XVector, box2CFrame.YVector, box2CFrame.ZVector}

    local bestDisposition: BoxDisposition = DispositionIntersecting
    local bestPlaneNormal = Vector3.new(0,0,0)
    local bestPlaneDistance = 0
    local bestSeparationDistance = -1

    --Test each set of axes
    for _, axes in {box1Axes, box2Axes} do
        for _, normal in axes do
            local disposition: BoxDisposition, planeDistance, separationDistance = DoesPlaneSeparate(box1Points, box2Points, normal)
            if disposition == DispositionSeparated then
                if (bestDisposition ~= DispositionSeparated) or (bestSeparationDistance < separationDistance) then
                    bestDisposition = DispositionSeparated
                    bestPlaneNormal = normal
                    bestPlaneDistance = planeDistance
                    bestSeparationDistance = separationDistance
                end
            elseif disposition == DispositionTouching then
                if (bestDisposition ~= DispositionSeparated) then
                    bestDisposition = DispositionTouching
                    bestPlaneNormal = normal
                    bestPlaneDistance = planeDistance
                    bestSeparationDistance = separationDistance
                end
            end
        end
    end
    --Test extra planes defined by cross products of edges to catch edge-edge separations
    for _, axis1 in box1Axes do
        for _, axis2 in box2Axes do
            local normal = axis1:Cross(axis2)
            if normal.Magnitude == 0 then
                continue
            else
                normal = normal.Unit
            end
            local disposition: BoxDisposition, planeDistance, separationDistance = DoesPlaneSeparate(box1Points, box2Points, normal)
            if disposition == DispositionSeparated then
                if (bestDisposition ~= DispositionSeparated) or (bestSeparationDistance < separationDistance) then
                    bestDisposition = DispositionSeparated
                    bestPlaneNormal = normal
                    bestPlaneDistance = planeDistance
                    bestSeparationDistance = separationDistance
                end
            elseif disposition == DispositionTouching then
                if (bestDisposition ~= DispositionSeparated) then
                    bestDisposition = DispositionTouching
                    bestPlaneNormal = normal
                    bestPlaneDistance = planeDistance
                    bestSeparationDistance = separationDistance
                end
            end
        end
    end
    return {
        Disposition = bestDisposition,
        PlaneNormal = bestPlaneNormal,
        PlaneDistance = bestPlaneDistance,
        SeparationDistance = bestSeparationDistance
    }
end
BoxSAT.Test = Test

return table.freeze(BoxSAT)
