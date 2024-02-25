--[[
Copyright Extrarius 2024.
This work is marked with CC0 1.0 Universal.
To view a copy of this license, visit http://creativecommons.org/publicdomain/zero/1.0
--
Functions to convert between SRGB, OkLab, and OkLCH color spaces.
Based on code by Björn Ottosson that he placed in the public domain.
See https://bottosson.github.io/posts/oklab/ for details of OkLAb
--
To produce RGB colors that don't clamp too badly, the Lab/LCH components should only use part of the range.
The following limits were determined experimentally and aren't perfect, but you'll get better results.

Lab range suggestion:
L:  0.0000 to 1.0000
a: -0.2339 to 0.2763
b: -0.3116 to 0.1986

LCH range suggestion:
L: 0 to 1.0000
C: 0 to 0.3225
H: 0 to 360 (or -180 to 180 if you prefer)

--]]
--!strict
--!optimize 2
local OkLab = {}


--Convert a component of linear SRGB to regular SRGB
local function FromLinear(component: number): number
    if component >= 0.0031308 then
        return (1.055) * component ^ (1.0/2.4) - 0.055
    else
        return 12.92 * component
    end
end


--Convert a component of regular SRGB to linear SRGB
local function ToLinear(component: number): number
    if component >= 0.04045 then
        return ((component + 0.055)/(1.055))^2.4
    else
        return component / 12.92
    end
end

--Convert a roblox color to an oklab color. Output: (X=L, Y=a, Z=b)
local function RobloxToLab(color: Color3): Vector3
    local r = ToLinear(color.R)
    local g = ToLinear(color.G)
    local b = ToLinear(color.B)

    local l = 0.4122214708 * r + 0.5363325363 * g + 0.0514459929 * b
    local m = 0.2119034982 * r + 0.6806995451 * g + 0.1073969566 * b
    local s = 0.0883024619 * r + 0.2817188376 * g + 0.6299787005 * b

    local lRoot = l ^ (1.0/3.0)
    local mRoot = m ^ (1.0/3.0)
    local sRoot = s ^ (1.0/3.0)

    return Vector3.new(
        0.2104542553 * lRoot + 0.7936177850 * mRoot - 0.0040720468 * sRoot,
        1.9779984951 * lRoot - 2.4285922050 * mRoot + 0.4505937099 * sRoot,
        0.0259040371 * lRoot + 0.7827717662 * mRoot - 0.8086757660 * sRoot
    )
end
OkLab.RobloxToLab = RobloxToLab


--Convert an oklab color to a roblox color. Input: (X=L, Y=a, Z=b)
local function LabToRoblox(color: Vector3, fullRange: boolean?): Color3
    local L = color.X
    local a = color.Y
    local b = color.Z

    local lRoot = L + 0.3963377774 * a + 0.2158037573 * b
    local mRoot = L - 0.1055613458 * a - 0.0638541728 * b
    local sRoot = L - 0.0894841775 * a - 1.2914855480 * b

    local l = lRoot^3
    local m = mRoot^3
    local s = sRoot^3

    local r = FromLinear( 4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s)
    local g = FromLinear(-1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s)
    local b = FromLinear(-0.0041960863 * l - 0.7034186147 * m + 1.7076147010 * s)

    if not fullRange then
        r = math.clamp(r, 0, 1)
        g = math.clamp(g, 0, 1)
        b = math.clamp(b, 0, 1)
    end
    return Color3.new(r, g, b)
end
OkLab.LabToRoblox = LabToRoblox


--Conversion from an oklab color to oklab-based LCH color. Input: (X=L, Y=a, Z=b); Output: (X=L, Y=C, Z=H in degrees)
local function LabToLCH(color: Vector3): Vector3
    local C = math.sqrt(color.Y^2 + color.Z^2)
    local H = math.deg(math.atan2(color.Z, color.Y))
    return Vector3.new(color.X, C, H)
end
OkLab.LabToLCH = LabToLCH


--Conversion from an oklab color to oklab-based LCH color. Input: (X=L, Y=C, Z=H in degrees); Output: (X=L, Y=a, Z=b)
local function LCHToLab(color: Vector3): Vector3
    local rad = math.rad(color.Z)
    local mag = color.Y
    local a = mag * math.cos(rad)
    local b = mag * math.sin(rad)
    return Vector3.new(color.X, a, b)
end
OkLab.LCHToLab = LCHToLab


--Conversion from a roblox color to oklab-based LCH color. Output: (X=L, Y=C, Z=H in degrees)
local function RobloxToLCH(color: Color3): Vector3
    return LabToLCH(RobloxToLab(color))
end
OkLab.RobloxToLCH = RobloxToLCH


--Conversion from an oklab-based LCH color to a roblox color. Input: (X=L, Y=C, Z=H in degrees)
local function LCHToRoblox(color: Vector3, fullRange: boolean?): Color3
    return LabToRoblox(LCHToLab(color), fullRange)
end
OkLab.LCHToRoblox = LCHToRoblox


--
return table.freeze(OkLab)